#!/usr/bin/env bash
# NAT traversal lab: a master and a slave behind Linux NATs that meet through
# a rendezvous server, all in network namespaces on one Linux host. Needs
# root, iproute2, iptables and conntrack (for the report); run it in a
# privileged container on other systems:
#
#   docker run --rm -it --privileged -v "$PWD:/src" ubuntu:24.04 bash
#   apt-get install g++ cmake pkg-config libsodium-dev iproute2 iptables conntrack
#   cmake -S /src -B /tmp/b && cmake --build /tmp/b && /src/tools/nat_lab.sh --build /tmp/b
#
# Topology. Both homes use the same private prefix, as homes often do, so the
# host candidates of one end name the other end's own LAN. Between the two
# NATs is a chain of routers, r1 to rN (--wan-hops N); the server hangs off
# the middle one:
#
#   master 10.0.0.2 --[natA 10.0.0.1 | 198.51.100.2]-- r1 -- ... -- rN --+
#   slave  10.0.0.2 --[natB 10.0.0.1 | 203.0.113.2 ]---------------------+
#                                       server 192.0.2.10 -- r(N+1)/2
#
# A datagram from the slave passes natB (hop 1) and the routers (hops 2 to
# N+1) and reaches natA at hop N+2; one that the slave sends with a hop
# limit of 3 therefore dies at r2 if N >= 2, and reaches natA itself, with
# one hop to live, if N = 1.
#
# NAT behaviour (RFC 4787 terms). Both filter by address and port: a datagram
# from outside is forwarded only on a mapping it is a reply to (conntrack).
#   cone       MASQUERADE: an inside port keeps its outside port for every
#              destination (endpoint-independent mapping)
#   symmetric  MASQUERADE --random-fully: a new random outside port for every
#              destination (endpoint-dependent mapping)
#
# Options:
#   --build DIR            build directory (./build)
#   --nat-a KIND           the master's NAT, cone or symmetric (cone)
#   --nat-b KIND           the slave's NAT (cone)
#   --same-lan             the slave sits next to the master behind natA
#   --wan-hops N           routers between the NATs (3)
#   --wan-delay MS         one-way delay of each NAT's and the server's link
#                          to the routers (5)
#   --slave-lan-delay MS   one-way delay between the slave and its NAT (0);
#                          a Wi-Fi client in power save can take 100 ms
#   --master-lan-delay MS  the same between the master and its NAT (0)
#   --nat-b-input POLICY   what natB does with unsolicited datagrams to itself:
#                          drop (a firewall, the usual) or accept (no firewall:
#                          the kernel answers port unreachable) (drop)
#   --nat-a-input POLICY   the same for natA (drop)
#   --master-first         start the master before the slave
#   --prime-hops N         passed to both ends (their default, 3)
#   --keep DIR             keep the logs in DIR
#
# Prints one line: PASS or FAIL, the scenario, and the time from the master's
# start to the end of its run; the logs and both NATs' conntrack tables are
# in the log directory.

set -euo pipefail

BUILD=./build
NAT_A=cone
NAT_B=cone
SAME_LAN=0
WAN_HOPS=3
WAN_DELAY=5
PRIME_HOPS=
SLAVE_LAN_DELAY=0
MASTER_LAN_DELAY=0
NAT_A_INPUT=drop
NAT_B_INPUT=drop
MASTER_FIRST=0
KEEP=
while [ $# -gt 0 ]; do
  case "$1" in
    --build) BUILD=$2; shift ;;
    --nat-a) NAT_A=$2; shift ;;
    --nat-b) NAT_B=$2; shift ;;
    --same-lan) SAME_LAN=1 ;;
    --wan-hops) WAN_HOPS=$2; shift ;;
    --wan-delay) WAN_DELAY=$2; shift ;;
    --prime-hops) PRIME_HOPS=$2; shift ;;
    --slave-lan-delay) SLAVE_LAN_DELAY=$2; shift ;;
    --master-lan-delay) MASTER_LAN_DELAY=$2; shift ;;
    --nat-b-input) NAT_B_INPUT=$2; shift ;;
    --nat-a-input) NAT_A_INPUT=$2; shift ;;
    --master-first) MASTER_FIRST=1 ;;
    --keep) KEEP=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 1 ;;
  esac
  shift
done

TOOL=$(cd "$BUILD" && pwd)/tools/tuushin-teleop
[ -x "$TOOL" ] || { echo "no $TOOL" >&2; exit 1; }
LOG=${KEEP:-$(mktemp -d)}
mkdir -p "$LOG"
[ "$WAN_HOPS" -ge 1 ] || { echo "--wan-hops must be at least 1" >&2; exit 1; }
ROUTERS=
for i in $(seq 1 "$WAN_HOPS"); do ROUTERS="$ROUTERS r$i"; done
NS="master slave natA natB server$ROUTERS"

cleanup() {
  for ns in $NS; do
    for p in $(ip netns pids "$ns" 2>/dev/null); do kill "$p" 2>/dev/null || true; done
  done
  wait 2>/dev/null || true
  for ns in $NS; do ip netns del "$ns" 2>/dev/null || true; done
  [ -n "$KEEP" ] || rm -rf "$LOG"
}
trap cleanup EXIT
for ns in $NS; do ip netns del "$ns" 2>/dev/null || true; done

nsx() { local ns=$1; shift; ip netns exec "$ns" "$@"; }

for ns in $NS; do
  ip netns add "$ns"
  ip -n "$ns" link set lo up
done

# veth NS1 IF1 NS2 IF2
veth() {
  ip link add "$2" netns "$1" type veth peer name "$4" netns "$3"
  ip -n "$1" link set "$2" up
  ip -n "$3" link set "$4" up
}
# delay NS IF MS: netem on the egress of one end of a link
delay() {
  [ "$3" = 0 ] || nsx "$1" tc qdisc add dev "$2" root netem delay "$3ms" limit 10000
}

# The internet: a chain of routers r1..rN, r(i) and r(i+1) on 172.16.i.0/24
# (.1 on the left, .2 on the right), natA on r1, natB on rN, the server on
# the middle one; each router routes the prefixes it does not have towards
# the end of the chain that has them.
N=$WAN_HOPS
MID=$(((N + 1) / 2))
for i in $(seq 1 "$N"); do nsx "r$i" sysctl -qw net.ipv4.ip_forward=1; done
for i in $(seq 1 $((N - 1))); do
  veth "r$i" right "r$((i + 1))" left
  nsx "r$i" ip addr add "172.16.$i.1/24" dev right
  nsx "r$((i + 1))" ip addr add "172.16.$i.2/24" dev left
done
veth r1 w-a natA a-w
veth "r$N" w-b natB b-w
veth "r$MID" w-s server s-w
nsx r1 ip addr add 198.51.100.1/24 dev w-a
nsx "r$N" ip addr add 203.0.113.1/24 dev w-b
nsx "r$MID" ip addr add 192.0.2.1/24 dev w-s
for i in $(seq 1 "$N"); do
  [ "$i" = 1 ] || nsx "r$i" ip route add 198.51.100.0/24 via "172.16.$((i - 1)).1"
  [ "$i" = "$N" ] || nsx "r$i" ip route add 203.0.113.0/24 via "172.16.$i.2"
  if [ "$i" -lt "$MID" ]; then
    nsx "r$i" ip route add 192.0.2.0/24 via "172.16.$i.2"
  elif [ "$i" -gt "$MID" ]; then
    nsx "r$i" ip route add 192.0.2.0/24 via "172.16.$((i - 1)).1"
  fi
done
nsx server ip addr add 192.0.2.10/24 dev s-w
nsx server ip route add default via 192.0.2.1
for l in "r1 w-a" "natA a-w" "r$N w-b" "natB b-w" "r$MID w-s" "server s-w"; do
  delay $l "$WAN_DELAY"
done

# nat NS WAN_IF WAN_ADDR GATEWAY KIND INPUT
nat() {
  local ns=$1 wif=$2 kind=$5 input=$6
  nsx "$ns" ip addr add "$3/24" dev "$wif"
  nsx "$ns" ip route add default via "$4"
  nsx "$ns" ip link add lan type bridge
  nsx "$ns" ip link set lan up
  nsx "$ns" ip addr add 10.0.0.1/24 dev lan
  nsx "$ns" sysctl -qw net.ipv4.ip_forward=1
  local random=
  [ "$kind" = symmetric ] && random=--random-fully
  nsx "$ns" iptables -t nat -A POSTROUTING -o "$wif" -j MASQUERADE $random
  nsx "$ns" iptables -A FORWARD -i "$wif" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
  nsx "$ns" iptables -A FORWARD -i "$wif" -j DROP
  if [ "$input" = drop ]; then
    nsx "$ns" iptables -A INPUT -i "$wif" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
    nsx "$ns" iptables -A INPUT -i "$wif" -j DROP
  fi
}
nat natA a-w 198.51.100.2 198.51.100.1 "$NAT_A" "$NAT_A_INPUT"
nat natB b-w 203.0.113.2 203.0.113.1 "$NAT_B" "$NAT_B_INPUT"

# host NS IF NAT NAT_IF ADDR
host() {
  veth "$3" "$4" "$1" "$2"
  nsx "$3" ip link set "$4" master lan
  nsx "$1" ip addr add "$5/24" dev "$2"
  nsx "$1" ip route add default via 10.0.0.1
}
host master m-a natA a-m 10.0.0.2
delay natA a-m "$MASTER_LAN_DELAY"
delay master m-a "$MASTER_LAN_DELAY"
if [ "$SAME_LAN" = 1 ]; then
  host slave s-a natA a-s 10.0.0.3
  delay natA a-s "$SLAVE_LAN_DELAY"
  delay slave s-a "$SLAVE_LAN_DELAY"
else
  host slave s-b natB b-s 10.0.0.2
  delay natB b-s "$SLAVE_LAN_DELAY"
  delay slave s-b "$SLAVE_LAN_DELAY"
fi

KEY=$LOG/key
rm -f "$KEY"
"$TOOL" keygen "$KEY" 2>/dev/null
nsx server "$TOOL" rendezvous --port 47200 >"$LOG/server.out" 2>"$LOG/server.err" &
sleep 0.2

PRIME=
[ -z "$PRIME_HOPS" ] || PRIME="--prime-hops $PRIME_HOPS"
start_slave() {
  # shellcheck disable=SC2086
  nsx slave "$TOOL" slave --once --no-rt --key-file "$KEY" --rendezvous 192.0.2.10 $PRIME \
    >"$LOG/slave.out" 2>"$LOG/slave.err" &
}
DURATION=2
start_master() {
  local t0 t1 rc=0
  t0=$(date +%s%N)
  # shellcheck disable=SC2086
  nsx master "$TOOL" master --no-rt --key-file "$KEY" --rendezvous 192.0.2.10 $PRIME \
    --duration "$DURATION" >"$LOG/master.out" 2>"$LOG/master.err" || rc=$?
  t1=$(date +%s%N)
  ELAPSED_MS=$(((t1 - t0) / 1000000))
  return $rc
}

RC=0
if [ "$MASTER_FIRST" = 1 ]; then
  # The slave comes half a second after the master.
  (sleep 0.5; start_slave) &
  start_master || RC=$?
else
  start_slave
  sleep 0.5
  start_master || RC=$?
fi

for ns in natA natB; do
  nsx "$ns" conntrack -L -p udp >"$LOG/conntrack-$ns.txt" 2>/dev/null || true
done

SCENARIO="nat-a=$NAT_A nat-b=$NAT_B same-lan=$SAME_LAN wan=${WAN_HOPS}x${WAN_DELAY}ms"
SCENARIO="$SCENARIO lan-delay=${MASTER_LAN_DELAY}/${SLAVE_LAN_DELAY}ms"
SCENARIO="$SCENARIO input=$NAT_A_INPUT/$NAT_B_INPUT"
SCENARIO="$SCENARIO master-first=$MASTER_FIRST prime-hops=${PRIME_HOPS:-default}"
if [ "$RC" = 0 ] && grep -q "^authenticated" "$LOG/master.out"; then
  printf 'PASS %s  %d ms  %s\n' "$SCENARIO" "$ELAPSED_MS" \
    "$(grep -o 'reached the slave at .*' "$LOG/master.out")"
else
  printf 'FAIL %s  %d ms  %s\n' "$SCENARIO" "$ELAPSED_MS" \
    "$(grep -E 'failed|announced no|no answer' "$LOG/master.out" | head -1)"
fi
[ -z "$KEEP" ] || echo "logs in $LOG"

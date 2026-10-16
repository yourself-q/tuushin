#!/usr/bin/env bash
# NAT traversal lab: a master and a slave behind Linux NATs that meet through
# a rendezvous server, all in network namespaces on one Linux host. Needs
# root, iproute2, iptables and conntrack (for the report), and coturn for
# --turn; run it in a privileged container on other systems:
#
#   docker run --rm -it --privileged -v "$PWD:/src" ubuntu:24.04 bash
#   apt-get install g++ cmake pkg-config libsodium-dev iproute2 iptables conntrack coturn
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
# With --cgnat-a, a carrier-grade NAT (cgnA) stands between natA and r1:
# natA's outside address is then 100.64.0.2 in the shared space of RFC 6598,
# cgnA's inside one 100.64.0.1, and 198.51.100.2 is cgnA's. --cgnat-b does
# the same for natB (cgnB, 203.0.113.2). A CGNAT maps endpoint-independently,
# as RFC 6888 requires (REQ-1, through RFC 4787's REQ-1), and here filters
# by address and port, which it allows.
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
#                          drop (a firewall, the usual), reject (a firewall
#                          that answers port unreachable, OpenWrt's default
#                          for its WAN zone) or accept (no firewall: the
#                          kernel answers port unreachable, and conntrack
#                          keeps the datagram's flow) (drop)
#   --nat-a-input POLICY   the same for natA (drop)
#   --master-first         start the master before the slave
#   --prime-hops N         passed to both ends (their default: auto, measured)
#   --cgnat-a, --cgnat-b   a carrier-grade NAT behind natA, natB (above)
#   --silent-cgnat         the CGNATs send no time-exceeded errors, as some
#                          carrier routers do not
#   --nat-b-errors POLICY  pass (the default) or drop: whether natB passes
#                          the ICMP errors for its mappings back inside
#                          (RFC 5508, REQ-4), which cheap home routers may
#                          not; the slave's measurement then waits for its
#                          timeout
#   --nat-a-errors POLICY  the same for natA
#   --nat-b-source WHERE   inside (the default) or outside: the address natB
#                          (and cgnB) sends its own ICMP errors from. Linux
#                          takes the one it would send anything to the
#                          error's destination from, the inside address;
#                          many home routers use their outside one, the
#                          address the rendezvous server sees
#   --nat-a-source WHERE   the same for natA (and cgnA)
#   --rebind-at S          S seconds after the master starts, natB (cgnB with
#                          --cgnat-b) moves the slave to a new outside
#                          address (203.0.113.3, as after a new DHCP lease or
#                          a carrier-grade NAT that forgot it) and forgets
#                          its mappings; the session
#                          must survive it (the master's link follows the
#                          slave, and the slave is ended by the master)
#   --turn                 run a TURN server (coturn) next to the rendezvous
#                          server, on 192.0.2.10:3478, and give the slave a
#                          relay there (--turn); the PASS line says whether
#                          the session ran through it
#   --open-at S            with --turn: r1 drops what the two NATs send each
#                          other until S seconds after the master starts, so
#                          that the session starts on the relay; it must
#                          then move to the direct path (the PASS line says
#                          when it did, and how many samples went missing)
#   --one-way              with --open-at: the opening lets through what natB
#                          sends to natA only; the session must stay on the
#                          relay, and lose nothing to the path
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
REBIND_AT=
OPEN_AT=
ONE_WAY=0
TURN=0
CGNAT_A=0
CGNAT_B=0
SILENT_CGNAT=0
NAT_A_ERRORS=pass
NAT_B_ERRORS=pass
NAT_A_SOURCE=inside
NAT_B_SOURCE=inside
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
    --rebind-at) REBIND_AT=$2; shift ;;
    --turn) TURN=1 ;;
    --open-at) OPEN_AT=$2; shift ;;
    --one-way) ONE_WAY=1 ;;
    --cgnat-a) CGNAT_A=1 ;;
    --cgnat-b) CGNAT_B=1 ;;
    --silent-cgnat) SILENT_CGNAT=1 ;;
    --nat-a-errors) NAT_A_ERRORS=$2; shift ;;
    --nat-b-errors) NAT_B_ERRORS=$2; shift ;;
    --nat-a-source) NAT_A_SOURCE=$2; shift ;;
    --nat-b-source) NAT_B_SOURCE=$2; shift ;;
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
if [ -n "$REBIND_AT" ] && [ "$SAME_LAN" = 1 ]; then
  echo "--rebind-at moves the slave behind natB; it has no meaning with --same-lan" >&2
  exit 1
fi
if [ -n "$OPEN_AT" ] && { [ "$TURN" = 0 ] || [ "$SAME_LAN" = 1 ]; }; then
  echo "--open-at needs --turn, and the NATs apart (no --same-lan)" >&2
  exit 1
fi
if [ "$ONE_WAY" = 1 ] && [ -z "$OPEN_AT" ]; then
  echo "--one-way needs --open-at" >&2
  exit 1
fi
ROUTERS=
for i in $(seq 1 "$WAN_HOPS"); do ROUTERS="$ROUTERS r$i"; done
NS="master slave natA natB server$ROUTERS"
[ "$CGNAT_A" = 0 ] || NS="$NS cgnA"
[ "$CGNAT_B" = 0 ] || NS="$NS cgnB"
for e in "$NAT_A_ERRORS" "$NAT_B_ERRORS"; do
  case "$e" in pass | drop) ;; *) echo "--nat-a-errors, --nat-b-errors: pass or drop" >&2; exit 1 ;; esac
done
for e in "$NAT_A_SOURCE" "$NAT_B_SOURCE"; do
  case "$e" in inside | outside) ;; *) echo "--nat-a-source, --nat-b-source: inside or outside" >&2; exit 1 ;; esac
done

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
# The home NATs' outside links: to the routers, or to a CGNAT's inside.
A_SIDE=natA
B_SIDE=natB
[ "$CGNAT_A" = 0 ] || A_SIDE=cgnA
[ "$CGNAT_B" = 0 ] || B_SIDE=cgnB
veth r1 w-a "$A_SIDE" a-w
veth "r$N" w-b "$B_SIDE" b-w
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
for l in "r1 w-a" "$A_SIDE a-w" "r$N w-b" "$B_SIDE b-w" "r$MID w-s" "server s-w"; do
  delay $l "$WAN_DELAY"
done

# err_source NS IF PREFIX OUT_ADDR: unless NS sends its errors from inside,
# what it sends into IF's PREFIX, its ICMP errors among it, leaves from
# OUT_ADDR.
err_source() {
  local where=$NAT_A_SOURCE
  case "$1" in natB | cgnB) where=$NAT_B_SOURCE ;; esac
  [ "$where" = inside ] || nsx "$1" ip route replace "$3" dev "$2" proto kernel scope link src "$4"
}
# nat NS WAN_IF WAN_ADDR GATEWAY KIND INPUT ERRORS
nat() {
  local ns=$1 wif=$2 kind=$5 input=$6 errors=$7
  nsx "$ns" ip addr add "$3/24" dev "$wif"
  nsx "$ns" ip route add default via "$4"
  nsx "$ns" ip link add lan type bridge
  nsx "$ns" ip link set lan up
  nsx "$ns" ip addr add 10.0.0.1/24 dev lan
  err_source "$ns" lan 10.0.0.0/24 "$3"
  nsx "$ns" sysctl -qw net.ipv4.ip_forward=1
  local random=
  [ "$kind" = symmetric ] && random=--random-fully
  nsx "$ns" iptables -t nat -A POSTROUTING -o "$wif" -j MASQUERADE $random
  if [ "$errors" = drop ]; then
    nsx "$ns" iptables -A FORWARD -i "$wif" -p icmp -j DROP
  fi
  nsx "$ns" iptables -A FORWARD -i "$wif" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
  nsx "$ns" iptables -A FORWARD -i "$wif" -j DROP
  case "$input" in
    drop | reject)
      nsx "$ns" iptables -A INPUT -i "$wif" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
      if [ "$input" = drop ]; then
        nsx "$ns" iptables -A INPUT -i "$wif" -j DROP
      else
        nsx "$ns" iptables -A INPUT -i "$wif" -p udp -j REJECT --reject-with icmp-port-unreachable
      fi
      ;;
    accept) ;;
    *) echo "unknown input policy $input" >&2; exit 1 ;;
  esac
}
# cgnat NS HOME_NS OUT_ADDR GATEWAY: a CGNAT on the outside of HOME_NS's
# link a-w (or b-w), with its own link to HOME_NS on 100.64.0.0/24.
cgnat() {
  local ns=$1 home=$2 wif=$5
  veth "$ns" in "$home" "$wif"
  nsx "$ns" ip addr add "$3/24" dev "$wif"
  nsx "$ns" ip route add default via "$4"
  nsx "$ns" ip addr add 100.64.0.1/24 dev in
  err_source "$ns" in 100.64.0.0/24 "$3"
  nsx "$ns" sysctl -qw net.ipv4.ip_forward=1
  nsx "$ns" iptables -t nat -A POSTROUTING -o "$wif" -j MASQUERADE
  nsx "$ns" iptables -A FORWARD -i "$wif" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
  nsx "$ns" iptables -A FORWARD -i "$wif" -j DROP
  nsx "$ns" iptables -A INPUT -i "$wif" -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
  nsx "$ns" iptables -A INPUT -i "$wif" -j DROP
  if [ "$SILENT_CGNAT" = 1 ]; then
    nsx "$ns" iptables -A OUTPUT -p icmp --icmp-type time-exceeded -j DROP
  fi
  delay "$ns" in "$WAN_DELAY"
  delay "$home" "$wif" "$WAN_DELAY"
}
if [ "$CGNAT_A" = 1 ]; then
  cgnat cgnA natA 198.51.100.2 198.51.100.1 a-w
  nat natA a-w 100.64.0.2 100.64.0.1 "$NAT_A" "$NAT_A_INPUT" "$NAT_A_ERRORS"
else
  nat natA a-w 198.51.100.2 198.51.100.1 "$NAT_A" "$NAT_A_INPUT" "$NAT_A_ERRORS"
fi
if [ "$CGNAT_B" = 1 ]; then
  cgnat cgnB natB 203.0.113.2 203.0.113.1 b-w
  nat natB b-w 100.64.0.2 100.64.0.1 "$NAT_B" "$NAT_B_INPUT" "$NAT_B_ERRORS"
else
  nat natB b-w 203.0.113.2 203.0.113.1 "$NAT_B" "$NAT_B_INPUT" "$NAT_B_ERRORS"
fi
[ -z "$REBIND_AT" ] || nsx "$B_SIDE" ip addr add 203.0.113.3/24 dev b-w

# The rebinding: new flows leave natB (or cgnB) from its second address, and
# the old mappings go, so the slave's next datagram to anyone opens a new
# one (the rule first, so that no datagram in between maps to the old
# address again).
rebind() {
  local random=
  [ "$NAT_B" = symmetric ] && [ "$CGNAT_B" = 0 ] && random=--random-fully
  sleep "$REBIND_AT"
  # shellcheck disable=SC2086
  nsx "$B_SIDE" iptables -t nat -R POSTROUTING 1 -o b-w -j SNAT --to-source 203.0.113.3 $random
  nsx "$B_SIDE" conntrack -D -p udp >/dev/null 2>&1 || true
  echo "rebound at $(date +%s%N)" >"$LOG/rebind.txt"
}

# The direct path between the NATs is shut at r1 until the opening (both
# outside prefixes: a CGNAT's address, or the rebinding's, is in them too).
if [ -n "$OPEN_AT" ]; then
  nsx r1 iptables -A FORWARD -s 198.51.100.0/24 -d 203.0.113.0/24 -j DROP
  nsx r1 iptables -A FORWARD -s 203.0.113.0/24 -d 198.51.100.0/24 -j DROP
fi
open_path() {
  sleep "$OPEN_AT"
  if [ "$ONE_WAY" = 1 ]; then
    nsx r1 iptables -D FORWARD -s 203.0.113.0/24 -d 198.51.100.0/24 -j DROP
  else
    nsx r1 iptables -F FORWARD
  fi
  echo "opened at $(date +%s%N)" >"$LOG/open.txt"
}

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
RELAY=
if [ "$TURN" = 1 ]; then
  command -v turnserver >/dev/null || { echo "--turn needs coturn's turnserver" >&2; exit 1; }
  printf 'lab-secret\n' >"$LOG/turn-pass"
  chmod 600 "$LOG/turn-pass"
  # Long-term credentials, UDP only, relayed addresses on the server's own.
  nsx server turnserver -n --listening-ip=192.0.2.10 --relay-ip=192.0.2.10 \
    --listening-port=3478 --lt-cred-mech --user=lab:lab-secret --realm=tuushin.lab \
    --fingerprint --no-tcp --no-tls --no-dtls --no-cli --no-tcp-relay \
    --log-file=stdout --simple-log >"$LOG/turn.out" 2>&1 &
  RELAY="--turn 192.0.2.10 --turn-user lab --turn-pass-file $LOG/turn-pass"
  sleep 0.3
fi
sleep 0.2

PRIME=
[ -z "$PRIME_HOPS" ] || PRIME="--prime-hops $PRIME_HOPS"
start_slave() {
  # shellcheck disable=SC2086
  nsx slave "$TOOL" slave --once --no-rt --key-file "$KEY" --rendezvous 192.0.2.10 $PRIME \
    $RELAY >"$LOG/slave.out" 2>"$LOG/slave.err" &
}
DURATION=2
# Enough session left after the rebinding whenever the meeting took.
[ -z "$REBIND_AT" ] || DURATION=$(awk "BEGIN { print $REBIND_AT + 3 }")
[ -z "$OPEN_AT" ] || DURATION=$(awk "BEGIN { print $OPEN_AT + 3 }")
start_master() {
  local t0 t1 rc=0
  t0=$(date +%s%N)
  [ -z "$REBIND_AT" ] || rebind &
  [ -z "$OPEN_AT" ] || open_path &
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

for ns in natA natB cgnA cgnB; do
  case " $NS " in *" $ns "*) ;; *) continue ;; esac
  nsx "$ns" conntrack -L -p udp >"$LOG/conntrack-$ns.txt" 2>/dev/null || true
done

SCENARIO="nat-a=$NAT_A nat-b=$NAT_B same-lan=$SAME_LAN wan=${WAN_HOPS}x${WAN_DELAY}ms"
SCENARIO="$SCENARIO lan-delay=${MASTER_LAN_DELAY}/${SLAVE_LAN_DELAY}ms"
SCENARIO="$SCENARIO input=$NAT_A_INPUT/$NAT_B_INPUT"
SCENARIO="$SCENARIO master-first=$MASTER_FIRST prime-hops=${PRIME_HOPS:-default}"
[ -z "$REBIND_AT" ] || SCENARIO="$SCENARIO rebind-at=${REBIND_AT}s"
[ "$TURN" = 0 ] || SCENARIO="$SCENARIO turn"
[ -z "$OPEN_AT" ] || SCENARIO="$SCENARIO open-at=${OPEN_AT}s"
[ "$ONE_WAY" = 0 ] || SCENARIO="$SCENARIO one-way"
[ "$CGNAT_A$CGNAT_B" = 00 ] || SCENARIO="$SCENARIO cgnat=$CGNAT_A/$CGNAT_B"
[ "$SILENT_CGNAT" = 0 ] || SCENARIO="$SCENARIO silent-cgnat"
[ "$NAT_A_ERRORS$NAT_B_ERRORS" = passpass ] || SCENARIO="$SCENARIO nat-errors=$NAT_A_ERRORS/$NAT_B_ERRORS"
[ "$NAT_A_SOURCE$NAT_B_SOURCE" = insideinside ] || SCENARIO="$SCENARIO nat-source=$NAT_A_SOURCE/$NAT_B_SOURCE"
OK=0
if [ "$RC" = 0 ] && grep -q "^authenticated" "$LOG/master.out"; then
  OK=1
  if [ -n "$REBIND_AT" ]; then
    # Wait for the slave to report; it ends with the master's final samples.
    for _ in $(seq 1 30); do
      grep -q "^session " "$LOG/slave.out" 2>/dev/null && break
      sleep 0.1
    done
    grep -q "the other end moved [1-9]" "$LOG/master.out" || OK=0
    grep -q "ended by the master" "$LOG/slave.out" || OK=0
  fi
  if [ -n "$OPEN_AT" ]; then
    for _ in $(seq 1 30); do
      grep -q "^session " "$LOG/slave.out" 2>/dev/null && break
      sleep 0.1
    done
    if [ "$ONE_WAY" = 1 ]; then
      # Stayed, and the master's samples all reached the slave.
      ! grep -q "^left the relay" "$LOG/master.out" "$LOG/slave.out" || OK=0
      grep -q "missing 0 " "$LOG/slave.out" || OK=0
    else
      grep -q "^left the relay" "$LOG/master.out" || OK=0
      grep -q "^left the relay" "$LOG/slave.out" || OK=0
    fi
  fi
fi
# missing N from E: what the master's samples and the slave's lost
missing() {
  printf 'missing %s from the slave, %s from the master' \
    "$(grep -o 'missing [0-9]*' "$LOG/master.out" | head -1 | cut -d' ' -f2)" \
    "$(grep -o 'missing [0-9]*' "$LOG/slave.out" | head -1 | cut -d' ' -f2)"
}
if [ "$OK" = 1 ]; then
  if [ -n "$OPEN_AT" ]; then
    printf 'PASS %s  %d ms  %s; %s; %s\n' "$SCENARIO" "$ELAPSED_MS" \
      "$(grep -o 'reached the slave at .*' "$LOG/master.out")" \
      "$(grep '^left the relay' "$LOG/master.out" || echo "stayed on the relay")" "$(missing)"
  else
    printf 'PASS %s  %d ms  %s\n' "$SCENARIO" "$ELAPSED_MS" \
      "$(grep -o 'reached the slave at .*' "$LOG/master.out")"
  fi
elif [ -n "$OPEN_AT" ] && [ "$RC" = 0 ] && grep -q "^authenticated" "$LOG/master.out"; then
  printf 'FAIL %s  %d ms  %s\n' "$SCENARIO" "$ELAPSED_MS" \
    "$(grep '^left the relay' "$LOG/master.out" || echo "stayed on the relay"); $(missing)"
elif [ "$RC" = 0 ] && grep -q "^authenticated" "$LOG/master.out"; then
  printf 'FAIL %s  %d ms  %s\n' "$SCENARIO" "$ELAPSED_MS" \
    "the session did not survive the rebinding: $(grep -o 'received [0-9]* from the slave' "$LOG/master.out")"
else
  printf 'FAIL %s  %d ms  %s\n' "$SCENARIO" "$ELAPSED_MS" \
    "$(grep -E 'failed|announced no|no answer' "$LOG/master.out" | head -1)"
fi
[ -z "$KEEP" ] || echo "logs in $LOG"

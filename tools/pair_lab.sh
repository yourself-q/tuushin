#!/usr/bin/env bash
# Pair lab: checks tuushin-probe pair, which measures each direction of a
# path between two interfaces of one host on one clock, in network
# namespaces on one Linux host. Each interface sits behind its own NAT, the
# two NATs meet in a router that also runs a STUN server (coturn), and the
# two directions get different constant delays (netem on the router), which
# pair must find as they are: a round trip halved, as measure's clock model
# does, would put their mean on each. Needs root, iproute2, iptables and
# coturn; run it in a privileged container on other systems:
#
#   docker run --rm -it --privileged -v "$PWD:/src" ubuntu:24.04 bash
#   apt-get install g++ cmake pkg-config libsodium-dev iproute2 iptables coturn
#   cmake -S /src -B /tmp/b && cmake --build /tmp/b && /src/tools/pair_lab.sh --build /tmp/b
#
# Topology:
#
#        a0 10.1.0.2 --[natA 10.1.0.1 | 198.51.100.2]--+
#   h                                                   [inet] STUN 192.0.2.10
#        b0 10.2.0.2 --[natB 10.2.0.1 | 203.0.113.2 ]--+
#
# h has a default route through each interface, a0's preferred, and loose
# reverse-path filtering: with strict filtering (rp_filter 1) Linux drops
# what arrives on b0 from addresses it would reach through a0, which is
# everything. A real host with two networks needs the same.
#
# NAT behaviour as in nat_lab.sh: both filter by address and port; cone is
# MASQUERADE (endpoint-independent mapping), symmetric is MASQUERADE
# --random-fully (a new outside port for every destination). As both
# filter by port, a symmetric NAT on either side leaves no way through: the
# other end can only send to the port the STUN server saw, which lets in
# nothing but the server's replies. pair then says so, and the lab FAILs.
#
# Options:
#   --build DIR     build directory (./build)
#   --ab MS         one-way delay from a0 to b0, added on inet's link to natB (20)
#   --ba MS         one-way delay from b0 to a0, on inet's link to natA (35)
#   --nat-a KIND    cone or symmetric (cone)
#   --nat-b KIND    cone or symmetric (cone)
#   --rate HZ       probes per second each way (1000)
#   --duration S    seconds of probing (10)
#   --keep DIR      keep the traces and logs in DIR
#
# Prints pair's report, then PASS or FAIL: PASS if pair got through and its
# median delay each way lies between the delay set and 1 ms above it.

set -euo pipefail

BUILD=./build
AB=20
BA=35
NAT_A=cone
NAT_B=cone
RATE=1000
DURATION=10
KEEP=
while [ $# -gt 0 ]; do
  case "$1" in
    --build) BUILD=$2; shift ;;
    --ab) AB=$2; shift ;;
    --ba) BA=$2; shift ;;
    --nat-a) NAT_A=$2; shift ;;
    --nat-b) NAT_B=$2; shift ;;
    --rate) RATE=$2; shift ;;
    --duration) DURATION=$2; shift ;;
    --keep) KEEP=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 1 ;;
  esac
  shift
done
for k in "$NAT_A" "$NAT_B"; do
  case "$k" in cone|symmetric) ;; *) echo "a NAT is cone or symmetric, not $k" >&2; exit 1 ;; esac
done

PROBE=$(cd "$BUILD" && pwd)/tools/tuushin-probe
[ -x "$PROBE" ] || { echo "no $PROBE" >&2; exit 1; }
command -v turnserver >/dev/null || { echo "needs coturn's turnserver" >&2; exit 1; }
LOG=${KEEP:-$(mktemp -d)}
mkdir -p "$LOG"
LOG=$(cd "$LOG" && pwd)
NS="plH plNA plNB plI"

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
ip link add a0 netns plH type veth peer name lan netns plNA
ip link add b0 netns plH type veth peer name lan netns plNB
ip link add wan netns plNA type veth peer name ia netns plI
ip link add wan netns plNB type veth peer name ib netns plI
for x in "plH a0" "plH b0" "plNA lan" "plNA wan" "plNB lan" "plNB wan" "plI ia" "plI ib"; do
  set -- $x
  ip -n "$1" link set "$2" up
done

ip -n plI addr add 198.51.100.1/24 dev ia
ip -n plI addr add 203.0.113.1/24 dev ib
ip -n plI addr add 192.0.2.10/32 dev lo
nsx plI sysctl -qw net.ipv4.ip_forward=1

# nat NS LAN_PREFIX WAN_ADDR GATEWAY KIND
nat() {
  local ns=$1 random=
  ip -n "$ns" addr add "$2.1/24" dev lan
  ip -n "$ns" addr add "$3/24" dev wan
  ip -n "$ns" route add default via "$4"
  nsx "$ns" sysctl -qw net.ipv4.ip_forward=1
  [ "$5" = symmetric ] && random=--random-fully
  nsx "$ns" iptables -t nat -A POSTROUTING -o wan -j MASQUERADE $random
  nsx "$ns" iptables -A FORWARD -i wan -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
  nsx "$ns" iptables -A FORWARD -i wan -j DROP
  nsx "$ns" iptables -A INPUT -i wan -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
  nsx "$ns" iptables -A INPUT -i wan -j DROP
}
nat plNA 10.1.0 198.51.100.2 198.51.100.1 "$NAT_A"
nat plNB 10.2.0 203.0.113.2 203.0.113.1 "$NAT_B"

ip -n plH addr add 10.1.0.2/24 dev a0
ip -n plH addr add 10.2.0.2/24 dev b0
ip -n plH route add default via 10.1.0.1 dev a0 metric 100
ip -n plH route add default via 10.2.0.1 dev b0 metric 200
for c in all default a0 b0; do nsx plH sysctl -qw "net.ipv4.conf.$c.rp_filter=2"; done

nsx plI tc qdisc add dev ib root netem delay "${AB}ms" limit 100000
nsx plI tc qdisc add dev ia root netem delay "${BA}ms" limit 100000

nsx plI turnserver -n --stun-only --listening-ip=192.0.2.10 --listening-port=3478 \
  --no-tcp --no-tls --no-dtls --no-cli --log-file=stdout --simple-log >"$LOG/stun.log" 2>&1 &
sleep 0.3

echo "== pair, $AB ms a0 to b0 and $BA ms back, NATs $NAT_A and $NAT_B"
set +e
nsx plH "$PROBE" pair --a a0 --b b0 --stun 192.0.2.10 --rate "$RATE" --duration "$DURATION" \
  --out "$LOG/pair" >"$LOG/pair.out" 2>"$LOG/pair.err"
STATUS=$?
set -e
grep -v '^ ' "$LOG/pair.err" || true
cat "$LOG/pair.out"

# p50 of the line whose first field is NAME (ns, from ms).
median() { awk -v name="$1" '$1 == name { for (i = 2; i < NF; i++) if ($i == "p50") print $(i + 1) }' "$LOG/pair.out"; }
VERDICT=FAIL
if [ "$STATUS" = 0 ]; then
  MAB=$(median a0-\>b0)
  MBA=$(median b0-\>a0)
  if awk -v m="$MAB" -v d="$AB" -v n="$MBA" -v e="$BA" \
      'BEGIN { exit !(m >= d && m <= d + 1 && n >= e && n <= e + 1) }'; then
    VERDICT=PASS
  fi
fi
echo "$VERDICT  pair $NAT_A/$NAT_B, set $AB/$BA ms, measured p50 ${MAB:-none}/${MBA:-none} ms"
[ -z "$KEEP" ] || echo "logs in $LOG"
[ "$VERDICT" = PASS ]

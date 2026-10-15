#!/usr/bin/env bash
# Replay lab: checks tuushin-netreplay against the traces it replays, in
# network namespaces on one Linux host, where both ends share one clock and
# a datagram's one-way delay is known exactly. Needs root, iproute2 and a
# build with tuushin-netreplay (clang and libbpf); run it in a privileged
# container on other systems:
#
#   docker run --rm -it --privileged -v "$PWD:/src" ubuntu:24.04 bash
#   apt-get install g++ cmake pkg-config libsodium-dev clang libbpf-dev iproute2
#   cmake -S /src -B /tmp/b && cmake --build /tmp/b && /src/tools/netreplay_lab.sh --build /tmp/b
#
# Topology. The path lives in a router between the two ends, as it would on
# a switch port in Mininet, and each direction is replayed on the router's
# egress towards its receiver:
#
#   a 10.9.0.1 --- 10.9.0.254 [ r ] 10.9.1.254 --- b 10.9.1.1
#         bwd trace on r's egress to a     fwd trace on r's egress to b
#
# Stamped datagrams go both ways at once (a to b on port 47100, b to a on
# 47101), and what each direction met is compared with its trace: losses
# where the trace lost, losses elsewhere, deliveries where the trace lost,
# and the delay minus the trace's. Without --fwd and --bwd, a synthetic pair
# is replayed: a base delay that wanders, a queue that fills for 300 ms
# every few seconds and drains at 0.8 ms per ms, and losses in bursts.
#
# What to expect. The replay releases a datagram when the kernel's timer
# fires, so the delay comes out late by that timer's latency: on a virtual
# machine on a busy laptop (colima, Linux 6.8) about 0.15 to 0.2 ms at the
# median and a few ms at the 99.9th percentile, the same as netem's on the
# same machine (--netem). A datagram due just as another arrives leaves
# with it, sooner: a constant delay that is a whole number of the sending
# interval comes out late by only about 0.06 ms there. A host that stalls
# its network processing for tens of ms delays the datagrams behind it as
# much, and judges them by the trace at the moment they reach the router:
# some are dropped by a loss that came after they were sent.
#
# Options:
#   --build DIR     build directory (./build)
#   --fwd FILE      the trace from a to b (synthetic)
#   --bwd FILE      the trace from b to a (synthetic)
#   --rate HZ       datagrams per second each way (1000)
#   --duration S    seconds of sending (20)
#   --netem MS      no replay: netem with a constant delay of MS each way,
#                   compared with a constant trace, for reference
#   --at-sender     replay the fwd trace on a's own egress instead of the
#                   router's: the held datagrams then count against the
#                   sending socket's buffer, and a 1 kHz flow of 100-byte
#                   datagrams held 300 ms had 21 % of its sends refused
#                   (500 ms: 52 %) with Linux's default 208 KiB
#   --probe         then measure the replayed path with tuushin-probe
#   --keep DIR      keep the traces, recordings and logs in DIR

set -euo pipefail

BUILD=./build
FWD=
BWD=
RATE=1000
DURATION=20
NETEM=
AT_SENDER=0
PROBE=0
KEEP=
while [ $# -gt 0 ]; do
  case "$1" in
    --build) BUILD=$2; shift ;;
    --fwd) FWD=$2; shift ;;
    --bwd) BWD=$2; shift ;;
    --rate) RATE=$2; shift ;;
    --duration) DURATION=$2; shift ;;
    --netem) NETEM=$2; shift ;;
    --at-sender) AT_SENDER=1 ;;
    --probe) PROBE=1 ;;
    --keep) KEEP=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 1 ;;
  esac
  shift
done

TOOLS=$(cd "$BUILD" && pwd)/tools
NR=$TOOLS/tuushin-netreplay
[ -x "$NR" ] || { echo "no $NR (it needs clang and libbpf at build time)" >&2; exit 1; }
[ "$PROBE" = 0 ] || [ -x "$TOOLS/tuushin-probe" ] || { echo "no $TOOLS/tuushin-probe" >&2; exit 1; }
if [ -n "$NETEM" ] && [ "$AT_SENDER" = 1 ]; then
  echo "--at-sender replays a trace; it has no meaning with --netem" >&2
  exit 1
fi
LOG=${KEEP:-$(mktemp -d)}
mkdir -p "$LOG"
LOG=$(cd "$LOG" && pwd)
NS="nrA nrR nrB"

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
ip link add a0 netns nrA type veth peer name ra netns nrR
ip link add b0 netns nrB type veth peer name rb netns nrR
ip -n nrA addr add 10.9.0.1/24 dev a0
ip -n nrB addr add 10.9.1.1/24 dev b0
ip -n nrR addr add 10.9.0.254/24 dev ra
ip -n nrR addr add 10.9.1.254/24 dev rb
for x in "nrA a0" "nrB b0" "nrR ra" "nrR rb"; do
  set -- $x
  ip -n "$1" link set "$2" up
done
ip -n nrA route add default via 10.9.0.254
ip -n nrB route add default via 10.9.1.254
nsx nrR sysctl -qw net.ipv4.ip_forward=1

# synth SEED BASE_MS PERIOD_S: a trace of DURATION + 10 s at 1 kHz.
synth() {
  awk -v seed="$1" -v base="$2" -v period="$3" -v n=$(((DURATION + 10) * 1000)) 'BEGIN {
    srand(seed)
    print "# synthetic, netreplay_lab.sh: base " base " ms, wander period " period " s"
    print "send_ns,delay_ns"
    q = 0; bad = 0
    for (i = 0; i < n; i++) {
      # A queue fills by 25 ms over 300 ms every 4 s or so, then drains at
      # 0.8 ms per ms: every packet still arrives after the one before.
      if (i % 4000 >= 2000 && i % 4000 < 2300) q += 25 / 300
      else q = q > 0.8 ? q - 0.8 : 0
      d = base + 3 * sin(6.2831853 * i / (period * 1000)) + q
      # Gilbert-Elliott losses: bursts of a few packets, 0.3 % overall.
      if (bad) { if (rand() < 0.3) bad = 0 } else if (rand() < 0.002) bad = 1
      lost = bad && rand() < 0.5
      printf "%d,%d\n", i * 1000000, lost ? -1 : d * 1e6
    }
  }'
}
# constant MS: a trace of DURATION + 10 s with a constant delay.
constant() {
  awk -v d="$1" -v n=$(((DURATION + 10) * 1000)) 'BEGIN {
    print "send_ns,delay_ns"
    for (i = 0; i < n; i++) printf "%d,%d\n", i * 1000000, d * 1e6
  }'
}
if [ -n "$NETEM" ]; then
  constant "$NETEM" > "$LOG/fwd.csv"
  cp "$LOG/fwd.csv" "$LOG/bwd.csv"
else
  if [ -n "$FWD" ]; then cp "$FWD" "$LOG/fwd.csv"; else synth 1 15 9 > "$LOG/fwd.csv"; fi
  if [ -n "$BWD" ]; then cp "$BWD" "$LOG/bwd.csv"; else synth 2 12 13 > "$LOG/bwd.csv"; fi
fi

START=$(($("$NR" clock) + 1000000000))
REPLAYS=
if [ -n "$NETEM" ]; then
  nsx nrR tc qdisc add dev rb root netem delay "${NETEM}ms" limit 100000
  nsx nrR tc qdisc add dev ra root netem delay "${NETEM}ms" limit 100000
else
  if [ "$AT_SENDER" = 1 ]; then FWD_AT="nrA a0"; else FWD_AT="nrR rb"; fi
  # Not through nsx: $! must be the tool itself, which `ip netns exec`
  # becomes, for the kill below to reach it.
  set -- $FWD_AT
  ip netns exec "$1" "$NR" run --dev "$2" --trace "$LOG/fwd.csv" --start "$START" > "$LOG/replay.fwd.log" 2>&1 &
  REPLAYS="$!"
  ip netns exec nrR "$NR" run --dev ra --trace "$LOG/bwd.csv" --start "$START" > "$LOG/replay.bwd.log" 2>&1 &
  REPLAYS="$REPLAYS $!"
fi
nsx nrB "$NR" recv --port 47100 --out "$LOG/fwd.rec.csv" --start "$START" --idle 2 > "$LOG/recv.fwd.log" 2>&1 &
RECV_FWD=$!
nsx nrA "$NR" recv --port 47101 --out "$LOG/bwd.rec.csv" --start "$START" --idle 2 > "$LOG/recv.bwd.log" 2>&1 &
RECV_BWD=$!

# Send from just after the start, when both replays have their points.
NOW=$("$NR" clock)
sleep "$(awk -v ns=$((START - NOW + 100000000)) 'BEGIN { printf "%.3f", (ns > 0 ? ns / 1e9 : 0) }')"
nsx nrA "$NR" send 10.9.1.1 --port 47100 --rate "$RATE" --duration "$DURATION" > "$LOG/send.fwd.log" 2>&1 &
SEND_FWD=$!
nsx nrB "$NR" send 10.9.0.1 --port 47101 --rate "$RATE" --duration "$DURATION" > "$LOG/send.bwd.log" 2>&1 &
SEND_BWD=$!
wait "$SEND_FWD" "$SEND_BWD" "$RECV_FWD" "$RECV_BWD"

if [ "$PROBE" = 1 ]; then
  ip netns exec nrB "$TOOLS/tuushin-probe" reflect --port 47000 > "$LOG/reflect.log" 2>&1 &
  REFLECT=$!
  sleep 0.5
  echo "== tuushin-probe through the replayed path"
  nsx nrA "$TOOLS/tuushin-probe" measure 10.9.1.1 --port 47000 --rate 1000 \
    --duration "$(( DURATION < 10 ? DURATION : 10 ))" --out "$LOG/probe" 2>&1
  kill "$REFLECT"
  wait "$REFLECT" || true
fi

for p in $REPLAYS; do kill "$p"; done
for p in $REPLAYS; do wait "$p" || true; done

if [ -n "$NETEM" ]; then echo "== netem, ${NETEM} ms each way"; fi
for dir in fwd bwd; do
  echo "== $dir"
  [ -n "$NETEM" ] || tail -1 "$LOG/replay.$dir.log"
  cat "$LOG/send.$dir.log" "$LOG/recv.$dir.log"
  "$NR" compare --trace "$LOG/$dir.csv" --recorded "$LOG/$dir.rec.csv"
done
[ -z "$KEEP" ] || echo "logs in $LOG"

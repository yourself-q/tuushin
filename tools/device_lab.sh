#!/bin/sh
# The teleoperator end to end on this machine with the firmware in the
# loop: two simulated knobs (tuushin-devsim), one turned by a hand at the
# master, one running into an object at the slave, and tuushin-teleop's
# master and slave driving them over TCP, talking UDP to each other.
#
#   tools/device_lab.sh [BUILD_DIR] [SECONDS] [-- extra tuushin-teleop options for both ends]
#
# DEVICE_OPTS is appended to both --device specs (",damping=0"), and
# SLAVE_ARGS goes to the slave alone ("--kp 300").
#
# Prints both ends' reports; exits non-zero if either end did.
set -eu

build=${1:-build}
seconds=${2:-10}
shift $(( $# < 2 ? $# : 2 ))
[ "${1:-}" = "--" ] && shift
out=$(mktemp -d "${TMPDIR:-/tmp}/device_lab.XXXXXX")
pids=""
cleanup() {
  for p in $pids; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

life=$((seconds + 40))
"$build/tools/tuushin-devsim" --tcp 47300 --hand --duration "$life" --quiet 2>"$out/devsim-master.txt" &
pids="$pids $!"
"$build/tools/tuushin-devsim" --tcp 47301 --stop 0.5 --duration "$life" --quiet 2>"$out/devsim-slave.txt" &
pids="$pids $!"
# Each device prints its banner once it listens.
i=0
until grep -q "^devsim: tcp" "$out/devsim-master.txt" 2>/dev/null &&
  grep -q "^devsim: tcp" "$out/devsim-slave.txt" 2>/dev/null; do
  i=$((i + 1))
  if [ $i -gt 100 ]; then
    cat "$out/devsim-master.txt" "$out/devsim-slave.txt"
    exit 1
  fi
  sleep 0.1
done

"$build/tools/tuushin-teleop" slave --once --port 47110 --device "tcp:127.0.0.1:47301${DEVICE_OPTS:-}" ${SLAVE_ARGS:-} "$@" \
  >"$out/slave.txt" 2>&1 &
slave=$!
pids="$pids $slave"
# The slave connects to its device and calibrates it before it listens.
i=0
until grep -q "^slave on" "$out/slave.txt" 2>/dev/null; do
  i=$((i + 1))
  if [ $i -gt 300 ] || ! kill -0 "$slave" 2>/dev/null; then
    cat "$out/slave.txt"
    exit 1
  fi
  sleep 0.1
done

rc=0
"$build/tools/tuushin-teleop" master 127.0.0.1 --port 47110 --duration "$seconds" \
  --device "tcp:127.0.0.1:47300${DEVICE_OPTS:-}" "$@" >"$out/master.txt" 2>&1 || rc=$?
wait "$slave" || rc=$?

echo "== master"; cat "$out/master.txt"
echo "== slave"; cat "$out/slave.txt"
echo "== devices"; cat "$out/devsim-master.txt" "$out/devsim-slave.txt"
echo "(logs in $out)"
exit $rc

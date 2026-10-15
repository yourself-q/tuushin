#!/bin/sh
# The teleoperator end to end with the ESP32 firmware itself at the slave:
# the emulated board (firmware/esp32/qemu.sh, built with
# CONFIG_TUUSHIN_EMULATED_STOP=y) running into an object, a simulated knob
# turned by a hand (tuushin-devsim) at the master, and tuushin-teleop's
# master and slave driving them, talking UDP to each other.
#
#   tools/emu_lab.sh DEVICE_HOST [BUILD_DIR] [SECONDS] [-- extra tuushin-teleop options for both ends]
#
# DEVICE_HOST is where QEMU listens (tcp:DEVICE_HOST:5555). DEVICE_OPTS is
# appended to both --device specs, and SLAVE_ARGS goes to the slave alone,
# as in device_lab.sh.
#
# Prints both ends' reports; exits non-zero if either end did.
set -eu

dev=$1
build=${2:-build}
seconds=${3:-10}
shift $(( $# < 3 ? $# : 3 ))
[ "${1:-}" = "--" ] && shift
out=$(mktemp -d "${TMPDIR:-/tmp}/emu_lab.XXXXXX")
pids=""
cleanup() {
  for p in $pids; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$build/tools/tuushin-devsim" --tcp 47300 --hand --duration $((seconds + 120)) --quiet \
  2>"$out/devsim-master.txt" &
pids="$pids $!"
i=0
until grep -q "^devsim: tcp" "$out/devsim-master.txt" 2>/dev/null; do
  i=$((i + 1))
  if [ $i -gt 100 ]; then
    cat "$out/devsim-master.txt"
    exit 1
  fi
  sleep 0.1
done

"$build/tools/tuushin-teleop" slave --once --port 47110 --device "tcp:$dev:5555${DEVICE_OPTS:-}" \
  ${SLAVE_ARGS:-} "$@" >"$out/slave.txt" 2>&1 &
slave=$!
pids="$pids $slave"
# The slave calibrates its device before it listens, which under QEMU
# takes some seconds.
i=0
until grep -q "^slave on" "$out/slave.txt" 2>/dev/null; do
  i=$((i + 1))
  if [ $i -gt 1200 ] || ! kill -0 "$slave" 2>/dev/null; then
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
echo "== device at the master"; cat "$out/devsim-master.txt"
echo "(logs in $out; the emulated device's own reports are in its console, BUILD_DIR/uart1.log)"
exit $rc

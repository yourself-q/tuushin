#!/bin/sh
# Builds the emulated board (sdkconfig.emulated) and runs it in Espressif's
# QEMU, with the host link on a TCP port and the emulator's reports on a
# console file.
#
#   firmware/esp32/qemu.sh BUILD_DIR [SECONDS] [EXTRA_DEFAULTS]
#
# Run in an ESP-IDF 5.x environment (export.sh sourced) with Espressif's
# QEMU installed (idf_tools.py install qemu-xtensa). BUILD_DIR is relative
# to this directory; EXTRA_DEFAULTS is one more sdkconfig defaults file,
# such as one with CONFIG_TUUSHIN_EMULATED_STOP=y for a slave. SECONDS
# (default 0) ends the run after that long; 0 runs until killed. The
# console is BUILD_DIR/uart1.log, printed at the end.
#
# A host reaches the device as tcp:HOST:5555 (PORT overrides it), one
# host at a time.
set -eu

cd "$(dirname "$0")"
build=$1
seconds=${2:-0}
extra=${3:+;$3}
port=${PORT:-5555}

# The defaults only seed a new sdkconfig: delete BUILD_DIR/sdkconfig after
# changing them.
idf.py -B "$build" -D SDKCONFIG="$build/sdkconfig" -D IDF_TARGET=esp32 \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.emulated$extra" build >"$build.log" 2>&1 || {
  grep -E "error|Error" "$build.log" | head -30
  exit 1
}
cd "$build"
flash_size=$(grep '^CONFIG_ESPTOOLPY_FLASHSIZE=' sdkconfig | cut -d'"' -f2)
python -m esptool --chip=esp32 merge_bin --output=qemu_flash.bin --fill-flash-size="$flash_size" @flash_args >/dev/null
# The eFuses of an ESP32 of chip revision 3 (as Espressif's QEMU notes
# give them); QEMU keeps writes to them in this file.
[ -f qemu_efuse.bin ] || python3 -c "
import binascii
open('qemu_efuse.bin', 'wb').write(binascii.unhexlify(
    '00000000000000000000000000800000000000000000100000000000000000000000000000000000'
    + '0' * 160 + '00000000'))"
rm -f uart1.log

# - No strap_mode global: 0x0f is download mode, and the ROM would wait for
#   esptool instead of booting the flash.
# - The watchdogs off: QEMU runs the chip at no steady speed, and a timer
#   group's watchdog would reset it when the host is slow.
# - The first -serial is UART0, the link; nodelay, since the host's small
#   frames otherwise wait on its delayed acknowledgements (40 ms on Linux)
#   whenever QEMU has an unacknowledged segment out.
# - The second -serial is UART1, the emulator's console.
set -- qemu-system-xtensa -M esp32 -m 4M \
  -drive file=qemu_flash.bin,if=mtd,format=raw \
  -drive file=qemu_efuse.bin,if=none,format=raw,id=efuse \
  -global driver=nvram.esp32.efuse,property=drive,value=efuse \
  -global driver=timer.esp32.timg,property=wdt_disable,value=true \
  -nic none -nographic -monitor none \
  -serial "tcp::$port,server=on,wait=off,nodelay=on" -serial file:uart1.log
set +e
if [ "$seconds" -gt 0 ]; then
  timeout "$seconds" "$@" >qemu.out 2>&1
else
  "$@" >qemu.out 2>&1
fi
cat uart1.log

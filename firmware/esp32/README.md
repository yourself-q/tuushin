# The firmware on an ESP32

The servo loop of `firmware/core` (`fw::DeviceCore`) on an ESP32, ESP32-S3
or ESP32-C3, under ESP-IDF 5.x: one axis, a gimbal motor under
field-oriented control, a magnetic encoder, and the host link of
`include/tuushin/device/proto.hpp` on a serial port. On the host,
`tuushin-teleop --device SPEC` drives it as the controllers' `Device1D`
(`device::RemoteDevice`).

**Status: built, not yet run on a chip.** Every target builds without
warnings with ESP-IDF 5.5.5, and the firmware runs end to end at the slave of
a teleoperator session in Espressif's QEMU (below), on a simulated motor.
Nothing here has driven a real motor yet: the timings on the chip, the
encoders' drivers and the ADC are unmeasured.

## What it does

A general-purpose timer raises an alarm every tick (200 us by default,
`CONFIG_TUUSHIN_TICK_US`); the servo task, at the highest priority and on
a core of its own where the chip has two, runs one tick of the core:
read the encoder, take the host's messages, run the checks, set the
three duty cycles, and send a State when one is due (every millisecond by
default; the host sets it). The core's safety rules are in
`firmware/core/include/tuushin/fw/core.hpp`: the driver is off until the
host enables it, a watchdog cuts the torque when commands stop (2 ms by
default), the driver goes off when the host is gone for 500 ms, and an
encoder that stops answering or jumps, or a speed over the limit, latches
a fault.

The torque is set by voltage, not by a current loop: the boards in mind
have no current sensing. For a torque `T` the core applies
`u_q = R T / kt + kt omega / 1.5` (the back-EMF of the speed it measures),
so the torque the shaft gets is only as right as the phase resistance `R`
and the torque constant `kt` it is given, and `R` grows with the windings'
temperature (copper, about 0.4 % a kelvin). The work each State reports is
the commanded torque times the displacement, which the host's energy
books use: an error in `kt / R` is an error in them. See First power-on
for measuring it.

The calibration finds the motor's electrical zero: it holds the rotor on
electrical angle 0 with `CONFIG_TUUSHIN_ALIGN_MV` on the d axis, turns the
field one electrical revolution forward and back, and judges where the
shaft settled. How far it turned checks the pole pairs (within 8 %, the
tolerance SimpleFOC uses), and the direction it turned gives the sign. It
takes 3.1 s, needs the shaft free, and is stored in NVS, so a board that
has been calibrated starts calibrated. The host calibrates a device that
is not when it connects (`calibrate=auto`, the default).

## Hardware

What the defaults are written for, for a first knob:

| part | for instance | notes |
|---|---|---|
| board | ESP32-S3-DevKitC-1 | Its native USB port (USB Serial/JTAG) is the host link: no UART bridge, no baud rate. An ESP32 DevKitC (UART through its USB bridge at 921600 baud) or an ESP32-C3 board also work. |
| driver | a DRV8313 board (the SimpleFOC Mini, for instance) | Three half-bridges with one PWM input each and enables that put them in high impedance; 8 to 60 V, 2.5 A peak, 0.24 ohm a FET at 25 C, overcurrent trip at 3 A or more ([datasheet](https://www.ti.com/lit/ds/symlink/drv8313.pdf)). Tie the three ENx together to the enable GPIO. |
| motor | a gimbal motor of 7 pole pairs (12N14P), the iPower GM2804 for instance | High resistance (a few ohms a phase), so that the supply's voltage sets a small current. A hollow shaft, or a shaft end free for the magnet. |
| encoder | MT6701 (SSI, 14 bit) or AS5047P (SPI, 14 bit) | Over the shaft's end, with a diametrically magnetized magnet on the shaft. The AS5600 (I2C, 12 bit) is supported, but a 12-bit reading is coarse for a knob (Coarse encoders in the top README) and takes some 50 us of the tick at 1 MHz. |
| supply | 12 V, 2 A | Above the DRV8313's 8 V minimum. A divider of 100 kohm over 10 kohm to the ADC pin lets the firmware measure it; without one it assumes `CONFIG_TUUSHIN_SUPPLY_MV`. |

Not supported: drivers with six inputs (a high and a low side per phase),
such as the TMC6300 that the SmartKnob uses, which runs from 2 to 11 V and
could take its power from USB; the bridge here drives three PWM pins and
one enable.

A gimbal motor's cogging is felt through a knob: the SmartKnob's notes
find most cheap gimbal motors moderately to severely cogging, and nothing
here compensates for it yet.

### Pins

The defaults (`idf.py menuconfig`, "tuushin device"; -1 means none):

| signal | ESP32 | ESP32-S3 | ESP32-C3 |
|---|---|---|---|
| PWM A, B, C | 25, 26, 27 | 4, 5, 6 | 3, 4, 5 |
| driver enable | 14 | 7 | 10 |
| SPI SCLK, MISO, MOSI, CS | 18, 19, 23, 5 | 12, 13, 11, 10 | 6, 1, 7, 0 |
| I2C SDA, SCL (AS5600) | 21, 22 | 8, 9 | 1, 0 |
| supply ADC | 36 | 1 | none |
| host link | UART0 (the USB bridge) | USB Serial/JTAG | USB Serial/JTAG |

The MT6701 has no data input, so it leaves MOSI unconnected (-1). They
keep off the chips' strapping pins where they can, the ESP32-S3's octal
flash and PSRAM pins, and the USB pins.

**The enable pin matters.** Without one (`CONFIG_TUUSHIN_DRIVER_EN_GPIO`
-1), "off" can only pull the three phases low, which shorts the windings:
the motor then brakes, with a torque of `kt^2 / R` times the speed, the
moment the host lets go or a fault trips. With it, off leaves the windings
open and the knob free.

### Heat

The torque ceiling (`CONFIG_TUUSHIN_MAX_TORQUE_UNM`, 0.1 N m) bounds the
current, and the current the heat: held at a torque `T`, the windings
dissipate `1.5 R (T / kt)^2`. With 4.5 ohm a phase and `kt` 0.05 N m/A,
0.1 N m is 2 A and the supply cannot give it (at 12 V the q-axis voltage
tops out at `12 / sqrt(3)` = 6.9 V, 1.5 A), but 1.5 A is 15 W, too much
for a small gimbal motor to hold for long. Nothing here models the
windings' temperature: for long contacts, lower the ceiling or the
voltage limit (`CONFIG_TUUSHIN_VOLTAGE_LIMIT_MV`, which caps the stall
current at `u / R`).

## Build

In an ESP-IDF 5.x environment (`. $IDF_PATH/export.sh`, or Espressif's
`espressif/idf:v5.5.5` container):

```sh
cd firmware/esp32
idf.py -B build-s3 -D IDF_TARGET=esp32s3 menuconfig   # "tuushin device": motor, pins, encoder
idf.py -B build-s3 -D IDF_TARGET=esp32s3 build
idf.py -B build-s3 -p PORT flash
```

`sdkconfig.defaults` (no console and no logs on the link's port, a 1 kHz
FreeRTOS tick, optimized for speed) and `sdkconfig.defaults.TARGET` (the
CPU clock) seed a new build directory's `sdkconfig`; once there, the
build keeps its `sdkconfig` and changes go through menuconfig. The motor's
parameters to set, from its data or a measurement:

- pole pairs: half the number of magnets (7 for 12N14P, 11 for 24N22P);
  the calibration checks it;
- phase resistance, phase to neutral: half of what an ohmmeter reads
  between two of its terminals;
- the torque constant, amplitude-invariant (N m per ampere of q-axis
  current), or its KV (rpm per volt of line-to-line peak back-EMF) with
  the torque constant 0: `kt = 1.5 / (sqrt(3) KV 2 pi / 60)`. A listed no-load
  speed is only a rough KV: iFlight lists 1602 to 1771 rpm at 10 V for
  the GM2804, which would be KV 170 and `kt` 0.05 N m/A;
- the inertia of the rotor and the knob, which the host uses to judge
  the coupling it can afford.

## First power-on

`tuushin-device` (one of the host's tools, `build/tools/tuushin-device`)
runs a device on its own, without a session, at the shaft: angles in
radians, torques in N m, positive the way the encoder counts up. Its first
argument is the same device spec as `tuushin-teleop --device`: the
ESP32-S3's USB port is `serial:/dev/cu.usbmodem...` on macOS and
`serial:/dev/ttyACM0` on Linux; a DevKitC's UART bridge is
`/dev/cu.usbserial-...` or `/dev/ttyUSB0`, at 921600 baud, the default.
Ctrl-C stops any command and switches the driver off.

1. **The link and the encoder.** Supply off or the motor unplugged:

   ```sh
   tuushin-device serial:PORT info
   tuushin-device serial:PORT watch --for 30
   ```

   `info` prints the device's Info (name, counts a turn, pole pairs,
   torque ceiling, tick, firmware), whether it is calibrated, the round
   trip while connecting, and the States of a second: about a thousand, none
   lost. Neither command calibrates or drives. During `watch`, turn the
   knob one full turn by hand: the angle should move 6.28 rad (360 deg),
   smoothly; a fault `encoder` or `encoder jump` means the encoder does
   not answer or reads noise (the magnet's gap and centring, the wiring).

2. **The calibration.** Supply on, the motor connected, nothing touching
   the knob:

   ```sh
   tuushin-device serial:PORT calibrate
   ```

   The knob turns a fraction of a turn each way for 3.1 s, and the device
   logs "calibrated: direction D, zero Z rad, off by E rad", E being how
   far one electrical turn missed a full one, in electrical radians (0.5
   at most). "calibration failed: pole pairs look like N" means the pole
   pairs are wrong, the align voltage (`CONFIG_TUUSHIN_ALIGN_MV`) is too
   low to beat the cogging and the friction, or the encoder misreads. The
   result is stored; later connects skip it.

3. **The torque's scale**, before trusting the energy books. First the
   resistance: half of what an ohmmeter reads between two of the motor's
   terminals, at room temperature, into menuconfig. Then a lever of known
   length clamped on the knob, its end resting on a kitchen scale, tared
   with the driver off:

   ```sh
   tuushin-device serial:PORT hold --torque 0.02 --lever 0.05 --for 5
   ```

   with the sign that presses the lever onto the scale (the other way the
   knob swings free and spins up, to the 30 rad/s fault at most). The tool
   prints what the scale should read, `T / L` (0.4 N, 40.8 g here). Held
   still there is no back-EMF, and the torque comes out times the true
   `kt / R` over the configured one: with `R` measured, set the torque
   constant to itself times the reading over the expected. The cogging
   adds a torque that depends on the angle, and the friction one that
   depends on the way the lever last moved: take the mean of a few angles,
   pressing from both sides. At 0.02 N m and `kt` 0.05 N m/A, the 0.4 A
   heats a 4.5 ohm motor by 1.1 W.

4. **A spring to feel:**

   ```sh
   tuushin-device serial:PORT spring --for 20
   ```

   A spring about the angle where it starts, as stiff as the device's
   damping (1 mN m s/rad on its ticks, unless the spec sets `damping=`)
   and the loop's delay allow, chosen as `tuushin-teleop`'s slave chooses
   its coupling. Turn the knob away and let go. At the end it prints the
   energy the loop's delay made, which the device's damping takes back.
   `--k` over that bound shows why it is one: the knob buzzes or swings
   up.

5. **The encoder's noise.** The spring changes the torque on nearly every
   tick, and with `CONFIG_TUUSHIN_ENC_REREAD` (the default for the MT6701
   and the AS5047P) the device reads the encoder again after each change.
   After 4096 of those, under a second of turning the knob, it logs
   "reread: indep I, error E rms, set A-B counts". I near 1 says the
   second reading is a conversion of its own, and E is the readings' error
   in counts, the rounding's included; near 0, the encoder returned its
   last conversion again, and the noise tax cannot be used: turn the
   reread off. With I near 1, set the noise range in menuconfig
   (`CONFIG_TUUSHIN_ENC_NOISE_LO_MILLICOUNTS` and `_HI_`, the noise before
   the rounding, sqrt(E^2 - 1/12) counts) with a margin each way, so that
   the logged A-B brackets E: the device then taxes each change of the
   torque by the noise's bound (`control/noise_tax.hpp`) rather than by
   the count's width, which bounds only noiseless readings. Unset, it
   keeps the count's width.

Then the session: `tuushin-teleop slave --device serial:PORT,radius=R`
(top README), with `R` the radius where the hand holds the knob.

## Under QEMU

`CONFIG_TUUSHIN_EMULATED` replaces the bridge, the encoder and the supply
with the simulated gimbal motor of `firmware/core` (`fw/plant.hpp`, the one
`tuushin-devsim` runs); the timer, the tasks, the servo loop and the host
link are the firmware's own, on Espressif's QEMU of the ESP32. The link is
UART0, which QEMU puts on a TCP port, and the emulator reports on UART1.

```sh
# in the ESP-IDF environment, with QEMU (python $IDF_PATH/tools/idf_tools.py install qemu-xtensa)
firmware/esp32/qemu.sh build-slave 0 sdkconfig.emulated.slave
# on a host that reaches it: a simulated hand at the master, the emulated board at the slave
tools/emu_lab.sh QEMU_HOST build 10
```

`sdkconfig.emulated.slave` gives the knob an object half a radian on once
it is calibrated; `sdkconfig.emulated.master` a hand that turns it to and
fro. The console (`BUILD_DIR/uart1.log`) shows, once a second, the motor's
angle, torque and work, and how the servo loop keeps up.

A session of 10 s, the emulated board at the slave on QEMU 9.2.2 in a
container on an Apple M4 Mac, the master a simulated hand:

| | |
|---|---|
| the link's round trip | 0.98 ms while connecting; 1.31 ms (p50) from a command to the State applying it |
| States lost | 0 |
| coupling the slave chose | 631 N/m |
| energy | master 0.159 J, slave -0.158 J |
| servo ticks | about 950 a second, 1 a second missed |
| the simulation's load | 25 % of the motor time it covers while driving |

Runs differ with the load on the host. Another, after States moved onto a
lattice of their period, connected with a round trip of 0.81 ms, coupled
at 691 N/m, carried 0.137 J, and lost no State through one stall of 74 ms
in the virtual machine.

What it took, and what differs from a chip:

- **The tick is 1 ms**, not 200 us: QEMU runs the chip at no steady speed
  and cannot keep up with 5 kHz. The plant's integration step is 250 us
  (`CONFIG_TUUSHIN_EMULATED_STEP_US`): pressed into a stop, its angle
  strays 0.47 mrad rms from a 5 us step, against 0.37 for 100 us and 4.3
  for 500 us.
- **The forces are computed in float** (`PlantParams::float_forces`):
  under QEMU a double `sin` costs 4 to 5 us and a float one under 1.3,
  since the ESP32 has no double-precision unit. The state, the time and the
  work stay in double; the knob strays 0.022 mrad at worst from the double
  plant.
- **The simulation gives up on time it falls behind**: more than four
  ticks behind the clock, the motor stands still for the rest and the
  report counts it as lost (105 ms in the run above, mostly at start and
  right after the calibration). Without that, a slow tick made the next
  one slower and the loop never caught up.
- **The timer is re-armed when its alarms stop.** QEMU's timer drops an
  alarm set to a count already passed, where the chip raises it at once;
  the servo task restarts the count when no alarm has come for a few
  ticks ("timer restarts" in the report).
- **The link's chardev runs with `nodelay`.** QEMU's TCP chardev has
  Nagle's algorithm on; with Linux's delayed acknowledgements, the round
  trip was 40 ms and the slave's coupling fell to 30 N/m.
- **The reread check reads low at the slave** ("reread: indep 0.42,
  error 1.30 rms" in a run, for an emulated error of 0.58). The device
  measures the readings' error from their second differences, which take
  the shaft's acceleration for error times the tick's square: the knob
  rings against its object, on ticks of 1 ms that QEMU stretches to
  several. The simulated hand's device, at 200 us, logged 1.03 and 0.57
  in the same run; at 1 ms against the object `tuushin-devsim` logs 0.57
  to 0.65 and 0.73 to 0.77, at 200 us 0.99 to 1.05 and 0.58.
- The watchdogs are off, and `qemu.sh` writes the eFuses of a chip of
  revision 3.
- The worst tick, 32 ms, comes right after the calibration. Writing the
  calibration to NVS stops the flash, and with it the code running from
  it, which is the likely cause; it has not been confirmed, and the
  driver is off at that moment.

## Not yet done

- Running on a chip: the tick's time at 5 kHz, the encoders' drivers
  (the MT6701's SSI frame and CRC, the AS5047P's parity, the AS5600's
  magnet status), the ADC, and the USB Serial/JTAG link's latency.
- The encoders' noise, and whether a second transfer is a conversion of
  its own (step 5 above). The noise tax's cost in the tick: computed in
  double precision, in software on these chips, some 5000 cycles a
  change on an ESP32-S3 by Espressif's per-operation figures, about 20 us
  of a 200 us tick; on an ESP32-C3, which has no FPU at all, more.
- Measuring the friction and the cogging (`hold` measures the torque
  constant, by hand).
- Cogging compensation, and a temperature model of the windings.
- Six-input drivers, and current sensing.

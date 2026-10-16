# tuushin

An open C++ stack for haptic teleoperation over the ordinary internet: a
master device (the operator's hand) and a slave robot, kilometres apart,
exchanging positions and forces at 1 kHz over UDP, with the operator feeling
what the robot touches and the loop staying stable whatever delay, jitter
and loss the path adds.

Force feedback across a network is stable on a laboratory LAN or a dedicated
link. On the public internet it meets tens of milliseconds of delay that
varies, packets that arrive late, out of order or not at all, clocks that are
not synchronized, and NATs. tuushin aims to be a shared, measured base for
that case: the passivity control that keeps the loop stable, the transport
that carries it, and tools that measure real paths and replay them.

**Status: early.** One degree of freedom, measured on loopback and in a
lab of Linux NATs so far; simulated devices, and firmware for a haptic
knob on an ESP32 that has run under emulation but not yet on a chip.
POSIX C++17, developed on macOS and also tested on Linux (Ubuntu 24.04,
GCC 13.3). MIT licensed.

## What is in it

| module | what it does |
|---|---|
| `control/` | Time Domain Passivity Approach (TDPA): passivity observers and controllers in the conventional two-channel form, and the variant that books energy reflection at a slave-side position coupling in one store (Panzirsch, Ryu and Ferre 2019, "Approach 3"). The passivity controllers predict the work a zero-order-held force will do on a light device instead of assuming constant velocity. |
| `codec/` | Perceptual deadband data reduction (Weber's law, Hinterseer et al. 2008). |
| `teleop/` | Master and slave controllers, the device interface (`Device1D`) with simulated devices, `HapticLink` (one end of a session over UDP), and real-time master and slave nodes. |
| `net/` | UDP sockets with kernel receive timestamps and hop-limited sends, clock-skew estimation from four-timestamp exchanges, channel models (fixed delay, jitter, Gilbert-Elliott loss, bottleneck queue, recorded traces), path measurement with probes and a reflector, a rendezvous server and client for ends behind NATs, a measurement of how many hops an end's own NATs reach, and STUN messages and a TURN client (RFC 8656) for a relayed path where no direct one opens, reached over UDP, TCP or TLS (see NAT traversal below). |
| `wire/` | The datagram format: a 32-byte header whose echo fields let each end measure the round trip from ordinary traffic, as RTCP does. |
| `crypto/` | A Noise_NNpsk0 handshake and ChaCha20-Poly1305 on libsodium: a pre-shared key becomes fresh keys for every session, and every datagram is authenticated and encrypted (see Authentication below). SHA-1, HMAC-SHA1 and MD5, only because STUN and TURN require them. |
| `device/` | `RemoteDevice`: a device running tuushin's firmware, over a serial port or TCP, as a `Device1D`, and the link's protocol, which the firmware compiles too (see A device in the loop below). |
| `firmware/` | The device's servo loop (`firmware/core`: field-oriented control of a gimbal motor, calibration, a watchdog and faults, and a simulated motor to test it on) and its port to the ESP32 family under ESP-IDF (`firmware/esp32`). |
| `sim/` | A deterministic 1-DoF teleoperation simulator built on the same controllers and devices. |
| `tools/` | `tuushin-sim`, `tuushin-teleop` (master, slave and rendezvous server as separate processes), `tuushin-probe` (measure a path, write replayable traces; `pair` measures between two networks of one host on one clock), `tuushin-netreplay` (Linux: replay a trace on an interface, datagram by datagram, for network namespaces, Mininet and testbeds built on them), `nat_lab.sh` (master and slave behind Linux NATs in network namespaces, with coturn as a relay over UDP, TCP or TLS), `netreplay_lab.sh` (checks the replay against its traces), `pair_lab.sh` (checks `pair` behind two NATs, with coturn as the STUN server), `fetch_starlink.sh` (public Starlink one-way delays as traces), `tuushin-devsim` (the firmware on a simulated motor, behind a pty or a TCP port), `tuushin-device` (bring up one device on its own), `device_lab.sh` and `emu_lab.sh` (the teleoperator end to end with devices running the firmware, simulated or under QEMU). `tests/sim_nat.hpp`, `tests/sim_turn.hpp` and `tests/sim_tls.hpp` are a NAT, a TURN server and a TLS front for it in user space for tests on any system. |

## Build and test

```sh
cmake -S . -B build -G Ninja
cmake --build build
./build/tests/tuushin_tests          # all tests; pass a name to run a subset
```

Needs a C++17 compiler, CMake and libsodium (ISC licence):
`brew install libsodium`, or `apt install libsodium-dev pkg-config`.
`tuushin-netreplay` is built on Linux where clang and libbpf (used under
its BSD-2-Clause licence) are found: `apt install clang libbpf-dev`.
TURN over TLS uses OpenSSL 3 (Apache License 2.0) where it is found:
`brew install openssl@3`, or `apt install libssl-dev`. Without it, or with
`-DTUUSHIN_TLS=OFF`, everything else builds, and a relay is reached over
UDP or TCP only.

## Quick start

Simulate a teleoperator touching a wall with 25 ms one-way delay:

```sh
./build/tools/tuushin-sim --passivity reflect --delay-ms 25
./build/tools/tuushin-sim --passivity channel --sweep 0,10,25,50,100
```

Run the master and the slave as separate processes over UDP (simulated
devices here; `--device` drives one running the firmware, A device in the
loop below). On loopback, `--emulate-*` impairs each end's outgoing
datagrams in-process, so the measurements see the impairment as if the
network had caused it:

```sh
./build/tools/tuushin-teleop slave --port 47100 --emulate-delay 25
./build/tools/tuushin-teleop master 127.0.0.1 --port 47100 --duration 10 --emulate-delay 25
```

Each simulated device cuts its force when no command has come for two
periods, as haptic devices do (`--watchdog MS` changes it, 0 disables it;
see Loop stalls below).

Across a network the slave should accept only its own master. Make a key,
copy it to the other machine over a channel you trust (scp, say), and pass
it to both ends:

```sh
./build/tools/tuushin-teleop keygen haptic.key       # mode 0600, never overwrites
./build/tools/tuushin-teleop slave --port 47100 --key-file haptic.key
./build/tools/tuushin-teleop master HOST --port 47100 --key-file haptic.key
```

When both ends are behind NATs, they meet through a rendezvous server on
a host both can reach, and need no forwarded port:

```sh
./build/tools/tuushin-teleop rendezvous --port 47200          # reachable host
./build/tools/tuushin-teleop slave --key-file haptic.key --rendezvous SERVER
./build/tools/tuushin-teleop master --key-file haptic.key --rendezvous SERVER
```

Where both NATs give every destination its own port (symmetric NATs), no
direct path opens. Give the slave a relay on a TURN server, coturn for
instance, with long-term credentials; the password is read from a file so
that it does not show in `ps`:

```sh
./build/tools/tuushin-teleop slave --key-file haptic.key --rendezvous SERVER \
    --turn TURNSERVER --turn-user NAME --turn-pass-file turn.pass
```

The master needs nothing more: the rendezvous server tells it of the
relay, and it tries the relay when its direct hellos go unanswered.

Where the slave's network lets out TCP, or only TLS, but no UDP beyond
the rendezvous server, reach the TURN server over TCP or TLS; the relay
still talks to the master over UDP (Through a relay over TCP or TLS,
below):

```sh
./build/tools/tuushin-teleop slave --key-file haptic.key --rendezvous SERVER \
    --turn 'turn:TURNSERVER?transport=tcp' --turn-user NAME --turn-pass-file turn.pass
./build/tools/tuushin-teleop slave --key-file haptic.key --rendezvous SERVER \
    --turn turns:TURNSERVER --turn-user NAME --turn-pass-file turn.pass  # TLS, port 5349
```

Over TLS the server's certificate is checked against the CAs OpenSSL
trusts by default, or against those in `--turn-ca FILE`.

Measure a real path and replay it:

```sh
# on a reachable host
./build/tools/tuushin-probe reflect --port 47000
# on the other end: 60 s of 1 kHz probes, both directions, skew-corrected
./build/tools/tuushin-probe measure HOST --out path
./build/tools/tuushin-sim --trace-fwd path.fwd.csv --trace-bwd path.bwd.csv
```

or, on a host with two networks (Wi-Fi and a phone tethered by cable, say),
measure each direction between them on the host's one clock, with no
reflector and no assumption about the split (Both directions on one clock,
below):

```sh
./build/tools/tuushin-probe pair --a en0 --b en7 --stun stun.l.google.com:19302 --out wifi-lte
./build/tools/tuushin-sim --trace-fwd wifi-lte.ab.csv --trace-bwd wifi-lte.ba.csv
```

or replay it under real network stacks, on a Linux interface that forwards
the traffic (a router namespace, a switch port in Mininet), as root:

```sh
./build/tools/tuushin-netreplay run --dev s1-eth2 --trace path.fwd.csv
```

## Results so far

### Simulation

`tuushin-sim --sweep`: a 0.2 kg master held by a simulated hand (a
spring-damper pulling it along a 4 cm, 0.5 Hz reference), a 1 kg slave in a
position coupling (2000 N/m, 60 N s/m) touching a 5000 N/m wall, 1 kHz, 10 s,
20 N actuator limit. *Margin* is the minimum over time of the energy the
teleoperator has absorbed between the two device ports; a negative margin
means it generated energy. *Displayed stiffness* is the stiffness the operator
feels while pressing into the wall; statically, the coupling and the wall in
series give about 1430 N/m.

| one-way delay | no passivity control: margin | conventional TDPA: stiffness / margin | TDPA with energy reflection: stiffness / margin |
|---|---|---|---|
| 0 ms | 0 J | 1326 N/m / 0 J | 1492 N/m / 0 J |
| 5 ms | −19 J | 252–656 / −2.1e−4–0 J | 1262–1263 / 0 J |
| 10 ms | −66 J | 88–98 / −1.1e−3 J | 1026–1100 / 0 J |
| 25 ms | −410 J | 38–66 / −0.071–−0.058 J | 391 / 0 J |
| 50 ms | −140 J | 85–142 / −0.30–−0.20 J | 106 / 0 J |
| 100 ms | −461 J | 62–100 / −0.26–−0.15 J | 77–83 / 0 J |
| 200 ms | −322 J | 70–71 / −0.25 J | 74–81 / 0 J |

The ranges are over twelve runs whose hand motion differs by one part in
10^12. The conventional scheme's numbers are chaotic: every sample its PC
cuts changes the command the slave integrates, so a rounding difference
grows into a different trajectory (at 50 ms the final position drift lands
anywhere between −4 and 14 mm). With energy reflection the PCs only scale
forces, and the same runs agreed to the digits shown until the model error
margin below: a margin that keeps the largest recent miss makes the force
a discontinuous function of the history, and at 10, 100 and 200 ms the
stiffness now spreads by up to a tenth.

Without passivity control the loop generates hundreds of joules and runs
into the force limit from 5 ms on. The conventional TDPA keeps each channel
direction passive (its channel margins stay above −2e−6 J), but not the
two-port between the devices, because the slave's coupling controller sits
outside it: energy the coupling spring stored and gives back looks generated
in the backward channel and is dissipated, the conservatism Panzirsch et al.
describe, and the displayed stiffness falls to tens of N/m. Booking the
coupling inside one store keeps several times the stiffness up to 25 ms,
and from 50 ms on about the same stiffness without generating energy.

Its residual margins, under 0.1 mJ over 10 s at 25 to 200 ms, came from the
predictive PCs (see *Coarse encoders* and *Model mismatch* below) trusting
a prediction the device did not follow. A margin for the model's recent
misses removed them, at a price in stiffness on this scenario: 7 % at 5 ms,
half at 50 ms, a fifth at 100 ms. Over bands of delays the cost is smaller
and not one-sided: over 8 runs at each delay in 4–20, 25–60 and 70–120 ms the
median stiffness went from 1129, 208 and 159 N/m to 1090, 249 and 85 N/m,
and the RMS error of the displayed against the contact force fell in all
three bands (1.56, 2.98 and 5.90 N to 1.47, 2.61 and 5.61 N). From 70 ms on
the master's PC acts in more than 95 % of the samples and the stiffness
of single runs ranges from 37 to 270 N/m.

### Real time, two processes on loopback

`tuushin-teleop`, same plant and controller as above, energy-reflection
scheme, 10 s sessions on macOS with the Mach time-constraint policy:

| | no emulated delay | 25 ms each way (emulated) | simulator, 25 ms |
|---|---|---|---|
| round trip, p50 / p99.9 | 0.041 / 0.090 ms | 50.04 / 50.09 ms | — |
| master tick lateness, p50 / max | 6 / 26 µs | 7 / 24 µs | — |
| samples lost | 0 | 0 | — |
| max master position | 0.0231 m (simulator: 0.0231 m) | 0.0289 m | 0.0288 m |
| max displayed force | 5.081 N (simulator: 5.080 N) | 4.89 N | 5.20 N |
| master / slave PC active | 5 % / 81 % | 90 % / 80 % | 90 % / 80 % |
| min margin | 0 J | 0 J | 0 J |

With the same delay the real-time loop and the simulator agree to within
1 % in position and in how often the passivity controllers act, and
neither generates energy. The peak displayed force at 25 ms differs by
6 % (in four sessions the real-time loop's stayed within 4.89–4.90 N);
in the simulator alone it ranges from 5.2 to 5.6 N between 24 and 27 ms,
so it is sensitive to the exact delay. The one-way delays come from
the echo fields and a clock fit, with no clock synchronization between the
ends.

### Loop stalls

A control loop that is descheduled leaves its last force on the actuator
for the whole stall, and that force does work no passivity controller
checks. tuushin handles this in three parts:

- The device cuts a force that has not been renewed within two periods,
  the horizon the predictive PC already plans over (`--watchdog`). The
  servo loops of haptic devices have such a watchdog (Force Dimension's
  `dhdSetWatchdog`, for one); `Device1D::force_cut_after()` reports it.
- A force that was cut is booked as the output the controller's device
  model predicts for the time it acted, never as input.
- After a sample further than half a period from nominal, or one in which
  the force was cut, the PC applies no force until two regular samples have
  refilled the device model (2 ms at 1 kHz). Samples within half a period
  are modelled with their actual length: a loop 0.3 ms late makes the next
  sample 0.7 ms, and read as 1 ms it understates the device's velocity by
  30 %, and with it the energy a large force will draw.

A watchdog alone is not enough: with the force cut but the books not told,
they credited the operator's push during the stall as input and the
controller paid it out: after 50 to 100 ms stalls the margin fell to between
−74 and −82 mJ, worse than without a watchdog.

Worst physical margin (minus the work both actuators did on their devices)
in a virtual-time replica of the two nodes (`tests/test_teleop_stall.cpp`;
5 ms each way, energy-reflection scheme, 3 s), over 40 stall start times at
the master, the slave or both:

| | 25 ms stall | 50 ms | 100 ms | random stalls at both ends |
|---|---|---|---|---|
| no watchdog | −8.5e−3 J | −0.057 J | −0.065 J | −0.039 J |
| watchdog, two periods | 0 J | −7e−7 J | −6e−7 J | 0 J |

Random stalls: 30 per second at each end, 0.2 to 12 ms long, 200 runs; with
the watchdog the energy books, which are the more conservative of the two,
stay above −2e−8 J. Before the model error margin (see *Model mismatch*)
the watchdog row read −5e−5, −6e−5, −4e−5 and 0 J, and the row without
it −8e−6, −0.019, −0.025 and −0.006 J. That row grew through the
trajectory, not the controller: in the worst case the force held when the
master stalled happened to be 5.9 N instead of 4.1 N, and its work during
the stall, which no controller checks, was 64 mJ instead of 26 mJ, while
what the PC let go after the stall stayed about the same (5 against
6 mJ). In real time, with both nodes as threads of one process
without real-time priority on a laptop loaded with 10 busy processes (the
setup of the test `nodes_run_a_session_over_loopback`), 2 of 48 sessions
without the watchdog fell below −1 mJ (the worst to −6 mJ); with it the
worst of 48 sessions was −1e−5 J, and of 60 under 24 busy processes
−2e−5 J. (These real-time sessions ran before the model error margin.)

### Coarse encoders

Everything above reads the device positions exactly. A real encoder rounds
them, to about 55 µm on a 3D Systems Touch, and that breaks the books: work
booked from readings is off by every change of the force times a position
error of up to half a count, and nothing makes those errors cancel. Abbott
and Okamura (IEEE T-RO 2005) show the same mechanism making a sampled
virtual wall non-passive. Told the resolution, the controllers

- estimate the device's velocity and the other forces on it with a Kalman
  filter on the readings instead of differencing them twice, which turns
  one count into 22 N of apparent force on a 0.2 kg device at 1 kHz;
- book the worst case of the misattributed work as an output, a *tax* of
  q |Δφ| on every rise of the port force, which makes the books a lower
  bound on the true work for any reading error within one count;
- let a rise repay its tax, and any deficit of the books, from the input
  predicted over the next four samples instead of the coming one. Paid
  within the sample, the tax of a force starting from zero is more than a
  master moving less than a count per sample (q/dt, 5.5 cm/s on a Touch at
  1 kHz) can take in, so the force never starts: the master displays
  nothing, takes in nothing, and never earns a budget. Three of the nine
  motions below are that slow, and the operator felt nothing for the whole
  20 s. Over four samples the threshold is about q/(4 dt), 1.4 cm/s;
- predict with the lower bound of one standard deviation of the filter's
  own uncertainty, and let the filter tune how fast it expects the hand's
  push to change from its innovations (covariance matching, Mehra, IEEE
  T-AC 1970). Told 20 N/s, the filter misses the device's displacement by
  about four of its standard deviations, told 5 N/s by fifteen, and a
  four-sample horizon that trusts it let the actuators generate up to
  7 mJ at 50 ms. Adapted, it misses by about one. The bound also takes in
  what the innovations show beyond the filter's spread (see *Model
  mismatch*), so a filter that keeps the rate it was told is safe too;
- let a rise of the master's force go only as far as the expected motion
  could keep paying for over the next 64 samples. Without that the force
  chatters at the count rate with every constraint met: a count transition
  shows the hand moving into the force, the force rises and pays its tax,
  and by the next transition it has decayed again, so each cycle pays the
  tax anew. With exact readings, the filter and the tax unchanged, the
  chatter is a fifth, so the readings cause it, not the tax. The held force
  may stay; only rises that would have to be taken back are declined.
  Passivity does not rest on this, so it uses the mean prediction: with
  the lower bound the master conveyed 11 % at 50 ms instead of 16 %. The
  slave does not use it: there the displayed share rose, but the slave
  pushed the wall about a third less hard and tracked worse at 10 ms
  (4.8 mm against 3.0 mm).

And every port pays its own excess first: when its output has run past
what the other end granted, its next input covers that before any of it is
reported to the other end, which would otherwise spend it a second time.

Reflective scheme, 55 µm master and 1 µm slave encoders, 20 s over nine
hand motions (0.4 to 0.6 Hz, 3 to 5 cm; `tests/test_teleop1d.cpp` checks
four of them, `tuushin-sim --master-encoder 55e-6 --slave-encoder 1e-6`
runs the default one, `--pc-rise 0` drops the rise hold, and
`--fixed-force-rate --pc-confidence 0 --pc-tax 0 --pc-rise 0` prepays the
tax as before):

| one-way delay | worst true margin: resolution known | not known | exact encoders | force displayed: known | no rise hold | tax prepaid | exact encoders |
|---|---|---|---|---|---|---|---|
| 10 ms | 0 J | −7.5e−6 J | 0 J | 92 % | 28 % | 32 % | 84 % |
| 50 ms | 0 J | −7.5e−6 J | 0 J | 16 % | 13 % | 12 % | 26 % |
| 100 ms | 0 J | −4.0e−6 J | 0 J | 11 % | 7 % | 4 % | 12 % |

*True margin* is minus the work the actuators really did, which the books
see only through the encoders. *Force displayed* is the force on the master
in contact as a share of the contact force one delay earlier.

Treating the readings as exact, the predictor differentiates quantization
noise. Before the model error margin the teleoperator then generated up
to a joule (−4.8e−4, −0.79 and −1.14 J); now the noise shows as model
error, the master's first force generates a few microjoules once, and
after it the master displays nothing: the wall is not felt, but nothing
is generated either. Knowing the resolution, it generates nothing, also
under the loop stalls above (`tests/test_teleop_stall.cpp`), with the
filter told a fifth of the default rate, and on the misdescribed devices
below. The price is transparency at long delays. At 10 ms the operator
feels more of the contact force than with exact encoders, at 50 ms 65 %
of it, at 100 ms 80 %. On the six motions fast enough to pay a prepaid
tax the master conveys 90, 18 and 9 %, against 47, 18 and 6 % with the
tax prepaid. The displayed force still chatters more than with exact
encoders at 50 and 100 ms: its total variation is 1.8 and 2.3 times
theirs (and half of it at 10 ms), down from 3.5 and 4.0 times without the
rise hold, which removes 43 to 62 % of it. Slower hands pay more for the
resolution: on motions of 0.15 to 0.3 Hz the master conveys 63, 14 and
12 %, against 111, 74 and 46 % with exact encoders. The rise hold is not
what costs them: it more than triples the share at 10 ms, leaves it at
50 ms and raises it by a quarter at 100 ms (before the model error margin
it lowered it by a sixth to a quarter at 50 and 100 ms). The books, which pay the worst case of
every rise, fall at most 0.4 mJ below zero, against 7.8 mJ with the
prepaid tax. (The columns other than *known* were measured before the
controllers estimated the reading noise, below; with it the known column
moved from 92, 17 and 10 % to 92, 16 and 11 %.)

**Encoders that dither.** A magnetic encoder (MT6701, AS5047P) does not
only round: its readings also wander by about half a count or more. The
filter and the predictor took a reading's error to be the quantization's,
uniform over a count (q²/12); told less than the truth, the filter trusts
each reading too much, its innovations come out larger than it expects,
covariance matching takes that for a hand pushing faster and raises the
force rate to its bound, and the predictions spread too wide for the PCs
to let any force through. So the controllers estimate the variance of a
reading's error online (`control/reading_noise.hpp`) from second
differences of the readings, the difference-based estimators of
nonparametric regression (Rice, Ann. Statist. 1984; Gasser, Sroka and
Jennen-Steinmetz, Biometrika 1986), with two changes. A quantized sawtooth
makes second differences of one count without any noise, 1.4 times q²/12
by their mean and 4.4 times by their median, so only what lies beyond a
count is counted, and the estimate is the variance whose Gaussian
truncated second moment explains it; quantization alone then gives
q²/12 exactly. And the change of velocity the device model expects from
the forces it knows is subtracted first, since on a micrometre encoder a
hand's acceleration alone makes two counts. On readings with 0.1 to 2
counts of noise it comes out within 6 % of the truth on average.

Nine motions as above with half a count of Gaussian noise added before
the rounding at both ends (`tuushin-sim --master-encoder-noise 0.5
--slave-encoder-noise 0.5`, and `--fixed-reading-noise` for q²/12):

| one-way delay | worst true margin: noise estimated | taken as q²/12 | force displayed: estimated | taken as q²/12 | without the noise |
|---|---|---|---|---|---|
| 10 ms | −7.4e−9 J | −1.5e−7 J | 20 % | 0 % | 92 % |
| 50 ms | −6.7e−9 J | −1.9e−8 J | 9 % | 0 % | 16 % |
| 100 ms | −6.7e−9 J | −3.1e−7 J | 6 % | 0 % | 11 % |

The tax bounds the misattributed work for errors within a count, which
noise beyond a count breaks; here by nanojoules. The noise costs most at
short delays, where the coarse encoders cost least: at 10 ms the master
conveys a fifth of the contact force instead of nine tenths.

### Model mismatch

The predictive PCs trust a model of each device, a rigid mass under a
slowly changing push, over the next few samples. A real device is not
one: its links bend, which puts part of its mass behind a resonance; it
has friction; its mass is known only roughly. Where the device moves
otherwise than predicted, the PC spends energy on inputs that do not
come, and the longer its horizon, the more.

The PCs now widen each prediction by what the model has lately missed,
grown as n² over n samples as an unmodeled force would:

- with coarse encoders, by what the filter's innovations show beyond its
  own spread, ν² − S averaged over 50 ms (Mehra's covariance matching
  again, applied to the residual). The misses hide among the reading
  errors, so they have to be averaged out;
- with exact readings, by the envelope of the squared misses, decaying
  over 50 ms. Each miss is known exactly after its sample, and there is
  nothing to average.

With exact readings too, a rise of the force must then repay any deficit
of the books over the tax horizon, so that a deficit the margin came too
late to prevent is not added to.

Worst true margin over nine hand motions, 20 s each, 10, 50 and 100 ms
each way (reflective scheme; coarse: 55 µm master, 1 µm slave), before
and after the margin:

| device | coarse encoders, before | after | exact readings, before | after |
|---|---|---|---|---|
| as modelled | 0 J | 0 J | −1.4e−4 J | 0 J |
| master, model twice its mass | 0 J | 0 J | −0.64 J | 0 J |
| master, half its mass beyond 100 Hz | −77 J | 0 J | −0.78 J | −2e−8 J |
| master, 80 % of its mass beyond 50 Hz | −67 J | 0 J | −3.9 J | −1.9e−4 J |
| master, 1 N Coulomb friction | −2.2e−4 J | 0 J | −3.5e−4 J | −2.3e−7 J |
| slave, model twice its mass | −0.018 J | 0 J | −0.016 J | −5.9e−5 J |
| slave, half its mass beyond 50 Hz | −0.014 J | 0 J | −0.018 J | −8.5e−6 J |
| slave, half its mass beyond 20 Hz | −1.8e−4 J | 0 J | −0.034 J | −1.1e−4 J |
| slave, 10 N Coulomb friction | −4.7e−4 J | 0 J | −4.2e−4 J | −1.3e−5 J |

The mass rows halve the device's true mass under an unchanged model.
`tuushin-sim --master-compliance 0.8,50`, `--master-friction 1` and their
`--slave-*` counterparts set up the others, and `--model-mass 0.4` the
same ratio on the default master; `tests/test_teleop1d.cpp` checks all
eight on one motion. Resonances are damped at 5 % of critical.

With coarse encoders nothing is generated on any of them. With exact
readings the margin covers misses like those it has seen, not one larger
than any before it, so a little remains, at most 0.19 mJ where it was
3.9 J. Before the margin the force on the worst devices was mostly
chatter: with exact readings and 80 % of the master's mass beyond 50 Hz
its total variation was 3100 to 4600 N/s and its RMS error 4 to 11 times
the contact force; with the margin 44 to 96 N/s and under 1. On the
devices as modelled, with exact readings, the master conveys 84, 26 and
12 % of the contact force at 10, 50 and 100 ms (77, 29 and 13 % before)
and its force varies a little less. The cost shows in the stiffness on
the default scenario (Simulation, above). A margin kept per direction of
the force recovered that stiffness but let the 80 % master generate
eight times as much; shorter memories of 20, 10 or 5 ms did not recover
it.

### Actuation delay

A device need not apply a force when it is commanded: a servo loop picks
commands up on its own ticks, a bus carries them, an amplifier has its
own loop. With a delay d the force held over a sample is partly the one
before, and a new force acts over a sample that starts d later. A
controller that takes every force to act at once credits each force with
the displacement the one before it caused, and its books run ahead of the
actuators. Told d, the controllers now keep the commands in flight
(`teleop::ActuatorSchedule`). The device model takes a sample whose force
changed within it exactly, as a mass; the books split the sample's
displacement where the force changed, by the model's prediction for each
piece; and the PC first pays for the forces in flight, from their
predicted work less its uncertainty, then predicts the new force from
where they leave the device.

Worst true margin over nine hand motions, 20 s each, 10, 50 and 100 ms
each way (reflective scheme; coarse: 55 µm master, 1 µm slave), with the
controllers unaware of the delay and told it:

| delay | coarse, unaware | told | exact, unaware | told |
|---|---|---|---|---|
| master 0.25 ms | 0 J | 0 J | −1.4e−3 J | −3.6e−6 J |
| master 1 ms | −3.3e−3 J | 0 J | −0.14 J | −2.6e−6 J |
| master 2 ms | −0.079 J | 0 J | −1.2 J | −1.2e−4 J |
| slave 0.25 ms | 0 J | 0 J | −3.8e−3 J | −2.3e−6 J |
| slave 1 ms | −0.15 J | 0 J | −0.14 J | −1.4e−8 J |
| slave 2 ms | −0.63 J | 0 J | −0.51 J | −1.8e−8 J |

`tuushin-sim --master-actuation-delay-ms 1` sets up the second row, and
`--actuation-delay-unknown` hides the delay from the controllers. With
coarse encoders nothing is generated, also with both ends delayed and
the loops up to 0.2 ms off their grid. With exact readings what is left
is model error: the split misplaces a few microjoules against what the
model leaves out within a sample, the hand's impedance and the device's
damping (up to 14 µJ with both ends at 1 ms and the loops off their
grid), and with 2 ms at the master a prediction four samples ahead
overshoots by up to 0.12 mJ. Unaware, the master's force also chatters:
with exact readings and 1 ms it varies 350, 190 and 210 N/s, told 130,
20 and 16 N/s, against 107, 19 and 15 without a delay. Told, the master
conveys as much of the contact force as without a delay with exact
readings (87, 26 and 11 %, against 84, 26 and 12 %), and with a coarse
encoder and 2 ms 78, 14 and 9 % instead of 93, 17 and 10 %.

The delay has to be known. Believed 0.1 ms off (0.9 or 1.1 ms for 1 ms)
the teleoperator generated up to 0.9 mJ with exact readings; half a
millisecond off, 8 mJ (master 1 ms believed 0.5) and 35 mJ (slave 2 ms
believed 1.5). Half a millisecond too long generated less, up to 4 mJ,
but cost transparency: for 2 ms believed 2.5 the master conveyed 48
instead of 88 % of the contact force at 10 ms. Coarse encoders generated
nothing in all of these but one, 11 mJ for the slave at 2 ms believed
1.5 ms at 100 ms.

Or estimated. Given the longest delay a device may have
(`MasterConfig::actuation_delay_max`, the same for the slave), a
controller runs a bank of device models, one per candidate delay
62.5 µs apart, fed the same readings and commands, and scores each by
how well it predicts the readings (`teleop::DeviceModel`: the prediction
error method, run as multiple-model adaptive estimation). Two things
stood in the way. A hand's damping answers a force at once, so a model
of the device as a bare mass takes it for an earlier actuation: from a
55 µm encoder 2 ms looked like 1.4 ms, 1 ms like 0.6 ms. Each candidate
therefore estimates the damping and stiffness holding the device along
with H (`control::HeldDeviceFilter`, an extended Kalman filter). And a
delay taken too short costs far more than one taken too long: held at a
fixed belief, a coarse master with 2 ms at 10 ms generated energy
believing 1 ms or less, and none believing 1.25, 1.8–2.2 or 3–4 ms. So
the controller uses not the best-scoring delay but the longest the
readings cannot tell from it (a likelihood ratio test at 95 %). It
starts from the maximum, takes a longer delay at once and a shorter one
only after the readings have kept to it for 0.1 s: without that, a
contact dropped a slave's 2 ms to 0 for 20 ms, and the teleoperator
generated 2 mJ.

The same runs with both controllers estimating, from 0 to 4 ms
(`tuushin-sim --estimate-actuation-delay 4`):

| delay | coarse: worst true margin | estimate at the end | exact: worst true margin | estimate |
|---|---|---|---|---|
| none | — | — | 0 J | 0 ms |
| master 1 ms | 0 J | 0.75–1.44 ms | 0 J | 1 ms |
| master 2 ms | 0 J | 1.81–2.5 ms | −7.3e−5 J | 2 ms |
| slave 1 ms | 0 J | 0.94–4 ms | 0 J | 1 ms |
| slave 2 ms | 0 J | 1.94–4 ms | 0 J | 2 ms |

With exact readings the estimates are the delays, and what is left is
the model error of a told delay (−1.2e−4 J for the master at 2 ms).
Coarse estimates land up to 0.25 ms short and 0.5 ms long, and in some
motions at 10 and 50 ms the slave's stayed at 4 ms, its readings never
telling the candidates apart: nothing was generated. The long side
costs a little transparency: a coarse master with 2 ms conveyed 66, 12
and 8 % of the contact force instead of the 78, 14 and 9 % it conveys
told the delay.

A device that says when it applied each command needs none of this. The
firmware behind `device::RemoteDevice` does, in every State, and the
controllers take the delay from it (`Device1D::actuation_delay`), with
one model and one schedule whose delay follows the measurement: a
command never takes effect before the one sent before it, whatever the
delays (A device in the loop, below). The estimate is for the devices
that do not.

### Model-mediated teleoperation

The schemes above send forces over the channel, so every force the
operator feels comes a round trip late, and the PCs that keep the
teleoperator passive take away whatever the delay would generate: at
100 ms the reflective scheme's master PC acts in 91 % of the samples. In
the model-mediated scheme (`tuushin-sim --passivity model`) no force
crosses the channel. The slave fits a spring to its contact force over its
position (`control/wall_model.hpp`: stiffness, surface and damping, each
accepted only within a stated standard error) and sends the model every
sample; the master renders it through an impedance-type PC on its own
books, with no delay inside the operator's haptic loop; the slave follows
the master's position with the force the master displays as a
feedforward. Only the master's device port is passive by construction:
the slave's actuator is a position servo with its own supply, which nothing
books, so the margin across both ports (`min_true_margin`) is about
−0.18 J on every run below, and the master's (`min_master_true_margin`)
is the one to compare.

Twelve hand motions, 0.3 to 0.8 Hz at 4 and 4.6 cm, 20 s each, exact
readings:

| one-way delay | model-mediated: stiffness | force error | conveyed | reflective: stiffness | force error | conveyed |
|---|---|---|---|---|---|---|
| 10 ms | 3483 N/m | 0.45 N | 100 % | 1094 N/m | 1.56 N | 101 % |
| 50 ms | 1086 N/m | 0.67 N | 100 % | 192 N/m | 3.57 N | 41 % |
| 100 ms | 523 N/m | 1.05 N | 101 % | 206 N/m | 5.38 N | 56 % |

The force error is the RMS of the displayed against the contact force,
delay included. The stiffness falls with the delay for one reason, the
first touch: the master learns of the wall a round trip after the slave
touches it, by when the hand is 11 mm deeper at 100 ms (its speed times
the round trip). From the second touch on, the operator feels the wall
at its own stiffness: on the 0.45 Hz motion at 100 ms each way, 5180 to
5420 N/m against the wall's 5000 N/m, with the surface within 0.07 mm of
where it is. No
passivity controller can do anything about the first touch; sensing the
wall before touching it could. The master's port gave out at most
6.1e−8 J over these runs, the overrun of a one-sample prediction.

With 55 µm encoders at both ends the PC error (the displayed force less
what the model asks for) is 0.57 N at 100 ms against 0.25 N with exact
readings, and the hand meets 1.15 force episodes per cycle against 1.01:
the window of readings bounds where in its count the device lies and the
taxes are refunded as the readings narrow it (`DevicePredictor::
solve_window`). Before those bounds it was 5.68 N and 3.59 episodes.

**Encoders that dither, again.** Half a count of noise breaks the
quantization tax here too: it bounds nothing once the error is
unbounded, the PC error is 14 N at every delay, and a held contact breaks
into about four force episodes. `control/noise_tax.hpp` bounds the books' error
instead with a stated probability δ (10⁻³), at every reading at once: the
error splits into a bias of the device's place in its count, bounded from
the exact distribution of the rounded Gaussian, and a martingale, bounded
by Robbins' normal mixture and Ville's inequality, which grows as the
square root of the sum of the squared force changes, not their sum. It
holds when each change is booked by a reading it was not decided on:
with a whole sample of actuation lead, or, with none, by reading the
encoder again once the command has taken effect (`--reread`,
`MasterConfig::reread`), which costs no delay. Half a count, 55 µm, the
same motions:

| one-way delay | quantization tax: PC error | episodes per cycle | noise tax, read again: PC error | episodes per cycle | exact readings: PC error |
|---|---|---|---|---|---|
| 10 ms | 13.96 N | 4.07 | 2.35 N | 2.34 | 0.04 N |
| 50 ms | 13.63 N | 3.85 | 2.06 N | 1.94 | 0.19 N |
| 100 ms | 14.27 N | 3.61 | 2.09 N | 2.08 | 0.25 N |

The master's true margin stayed at 0 J in all of them. With a lead of
one sample at both ends instead of the second reading, 4.69 N and 2.66
at 100 ms.

What is left, at 100 ms: books that knew the true work would reach
0.99 N and 1.27 episodes, and the same encoders without noise 0.57 N. So
about half the rest is the books and half the predictions made from noisy
readings. The tax's confidence is not what binds: δ = 0.5 gives 1.75 N,
δ = 10⁻⁶ 2.62 N. Neither is the noise level near half a count: a quarter
count gives 2.05 N, where the bias the noise no longer smooths takes
over; a whole count 6.71 N. In the runs the PC is engaged through the
whole of a held contact and the displayed force swings by several newtons
from sample to sample, 131 N/s of variation against 23 N/s with exact
readings; as the bound grows with the squared changes, the chatter adds
to what limits the PC. Rendering from the filtered position instead of
the reading cut the chatter by a quarter but not the error.

**The limit cycle, and a surface that yields.** The chatter was a limit
cycle. Held in the wall, the master oscillated by about a millimetre at
33 Hz: the PC held the force back while the hand pushed in, so that it
rose behind what the model asked for, and let it through while the hand
came back out, until the books ran dry. More force on the way out than on
the way in is negative damping, which kept the oscillation going, and
the large changes it made raised the noise bound, which tightened the PC
further. What the PC cut against was the trouble: a cut left the
rendered surface where it was, and the next request, from that surface,
tried to catch up. The master now moves the rendered surface in where
its PC cuts, until the wall gives what was displayed
(`WallRenderer::yield`), as passive set-position modulation moves a
spring's set position where the energy available allows (D. Lee and
K. Huang, IEEE T-RO 26(2), 2010). The wall stays a spring of the hand's
position, and what the move releases, the energy the PC dissipated, is
not spent on moving it back out. The PC error is then near zero by
construction (the PC errors and episodes above are with `--no-yield`), so
what follows compares the feel instead: for each contact after the
first, a line through the displayed force over the position, whose
slope is the stiffness felt and whose zero is the surface felt.

The slave's fit fell short under the noise too. The positions' error
spreads them without spreading the forces, which attenuates a
least-squares slope, and the model put the wall at 3000 to 4600 N/m,
with its surface 0.7 mm towards the master to make up the force. The fit
now takes the error's spread out of the positions' (the method of
moments for a known error variance, W. A. Fuller, *Measurement Error
Models*, 1987), with the slave's own estimate of its reading noise
(31.7 µm for half a count and the rounding, 31.8 µm in theory). And a
reading's error, rendered, is the stiffness times it, 0.16 N on every
reading for 31.7 µm at 5000 N/m: the displayed force varied by 79 N/s
against 23 N/s with exact readings. A master with an encoder resolution
now renders at its Kalman filter's position instead (32 N/s; the books
still go by the readings, and `--render-readings` turns it off). The same
twelve motions, medians with the 10th to 90th percentiles:

| one-way delay | readings | contacts | stiffness felt | surface felt, beyond the wall |
|---|---|---|---|---|
| 10 ms | exact | 114 | 5056 N/m (5008 to 5080) | 0.012 mm (0.002 to 0.020) |
| 10 ms | half a count | 120 | 4978 N/m (4599 to 5325) | 0.087 mm (0.00 to 0.36) |
| 100 ms | exact | 114 | 5308 N/m (4916 to 5453) | 0.09 mm (0.03 to 0.13) |
| 100 ms | half a count | 120 | 4953 N/m (4574 to 5272) | 0.16 mm (0.07 to 0.56) |
| 100 ms | half a count, `--no-yield` | 244 | 5207 N/m (2435 to 10631) | 0.38 mm (0.04 to 3.50) |

The motions make 113 presses after their first; with the surface
yielding the hand meets the wall 1.06 times per press, and the PC acts
in 2 % of the samples rather than 22 %. The force error against the
contact is 0.51 N at 10 ms and 1.09 N at 100 ms, against 0.45 and 1.05 N
with exact readings, and the master's true margin stays at 0 J. What the
noise still costs is a surface felt 0.07 mm deeper than with exact
readings, where the PC's cuts left it, a stiffness that varies by about
a tenth from contact to contact, near the Weber fraction the model's
updates are held to, and a displayed force that varies by 32 N/s against
12 to 23 N/s.

On the reflective scheme the second reading does not help: at 100 ms its
PC acts in 91 % of the samples with exact readings too, and with the
noise tax and the second reading its force error grows from 1.63 to
3.11 N at 100 ms and from 2.38 to 3.16 N at 10 ms.

The second reading has to be a conversion of its own, with noise
independent of the first's, sampled close to the change: the motion in
between is booked with the old force and not covered (1e−6 J per newton
of change at 0.1 m/s and 10 µs, against the tax's 3e−5 J). The noise has
to be Gaussian, white at the sampling rate and independent of the motion,
of a standard deviation within the range given (`--noise-tolerance`).
And the newest reading's noise times the force now is not charged: it
does not accumulate, but it is outside the bound's time-uniformity, and
on the reflective scheme leaves −2.7e−9 J. Whether an MT6701's noise is
white, and whether two of its conversions are independent, has not been
measured.

## Authentication

Without a key, anyone who can send datagrams to the slave's port can drive
the robot, and one forged datagram is enough to stall a session: claiming
a sequence number far ahead, it makes every genuine sample look stale
(`tests/test_teleop_auth.cpp` shows both). With `--key-file`:

- The master opens with a Noise_NNpsk0_25519_ChaChaPoly_SHA256 handshake
  (Perrin, *The Noise Protocol Framework*, rev. 34), the pattern for two
  ends that share only a symmetric key. Its prologue binds the session id
  and the passivity scheme. Both ends get two new keys, one per direction,
  from the pre-shared key and two ephemeral X25519 keys, so every session
  has its own keys, and a pre-shared key stolen later does not decrypt a
  recorded session (forward secrecy).
- Every haptic datagram is then sealed with ChaCha20-Poly1305: the sequence
  number is the nonce, and the header, which stays readable, is associated
  data. A datagram is authenticated before its sequence number reaches the
  replay window, so a forged one changes nothing.
- The first handshake message can be replayed: the slave cannot tell a
  recorded hello from a new one. So it starts a session only once a sample
  sealed under that handshake's keys arrives, which only the master that
  holds the ephemeral key can produce. And, as in WireGuard's initiation
  (Donenfeld, NDSS 2017), the hello carries the master's wall-clock time
  inside its encryption: the slave answers only a hello newer than the
  newest it has answered, so once a newer one has arrived a recorded hello
  costs it neither a key exchange nor a reply. A copy of the current hello, even one that arrives ahead
  of the genuine one from another address, gets the very same reply, which
  only the master can use. With `--hello-state FILE` the slave keeps that
  time on disk, written and synced before each reply goes out, so that a
  restart does not make the hellos recorded before it new again.

What it costs, measured on an Apple M4: 16 bytes per datagram (84 instead
of 68 for one degree of freedom), 0.39 µs to seal and 0.39 µs to open one,
about 72 µs per end for the handshake, once per session. Over loopback the
round trip did not change measurably (p50 0.041 to 0.044 ms sealed, 0.041
to 0.047 ms open, five sessions each). The hello reply (80 bytes) is
smaller than the hello (88 bytes), so the slave cannot be used to amplify
traffic.

## NAT traversal

A NAT forwards a datagram from outside only to an inside socket that has
recently sent to that sender, so two ends behind NATs cannot reach each
other by address alone. With `--rendezvous`, both register at a server
that both can reach, from the socket they will use for the session. The
server sees the outside address each NAT gave that socket and tells each
end the other's, and then both send to each other: the first datagram of
each end opens its own NAT for the other's address, so the other end's
next one gets through. This is UDP hole punching (Ford, Srisuresh and
Kegel 2005; RFC 5128) and the core of ICE (RFC 8445), simplified:

- The candidates are the address the server saw and the host's own
  addresses (for two ends on one LAN, whose router may not hairpin), and,
  with `--turn`, the slave's relayed address (Through a relay, below).
- The connectivity check is the haptic handshake. The master sends its
  hello to every candidate of the slave and takes the first reply that
  authenticates; the slave sends small punch datagrams to the master's
  candidates only to open its own NAT. This is why `--rendezvous` needs
  `--key-file`: authentication is what tells the slave's reply from
  anything else at a candidate address.
- The server learns an identifier both ends derive from their key (not the
  key), their addresses and when they meet, and carries no haptic traffic.
  Its replies are never larger than the registrations, so a forged source
  address cannot make it amplify.

**Sending at the same time is not enough; the first datagrams must be
ordered.** When a datagram from the other end reaches a NAT before the
inside socket has sent to that address, the NAT may keep a state of its own
for the pair: Linux's conntrack does when the NAT host accepts datagrams
addressed to itself, as it does without a firewall of its own. When the
inside socket then sends to the same address, the NAT finds the pair taken
and gives the socket another outside port. The address the server
announced is then not the one the datagrams come from, and the other NAT
drops them. In the lab below this broke every meeting in which one end was
100 ms behind a NAT that accepts datagrams to itself (a Wi-Fi client in
power save can be that far behind): the other end's datagrams came first.
Trautwein et al. (2025), who measured 4.4 million hole-punching attempts in
the IPFS network and found 70 % of them succeed, propose sending low-TTL
packets first (section 7.2). tuushin does two things:

- **Priming.** On learning the other end's candidates, each end first sends
  them datagrams with a small hop limit, measured at start (below). They
  open its own NAT and expire on the way, before they reach the other NAT.
- **Ready.** Each end then tells the server it is ready, and sends
  datagrams that go the whole way (hellos, punches) only once the server
  has passed on the other end's ready, or after 2 s without it. The server
  forgets an end's ready when the other end moves or registers anew, and
  tells the end so.

**The hop limit.** A datagram sent with a hop limit of t is forwarded by
the first t − 1 routers and dropped by the t-th. It has to pass every NAT
in front of this end (a home router, and often a carrier-grade NAT behind
it) and must not reach the other end's, which takes a datagram addressed to
itself whatever hop limit it has left. Guha and Francis (2005), on the same
method for TCP, name choosing it as a problem and leave it open. tuushin
measures it the way traceroute finds a path: while it registers, each end
sends small datagrams towards the rendezvous server with hop limits 1 to
10, each from a socket of its own, and reads the routers' time-exceeded
errors from the sockets' error queues (`IP_RECVERR`, Linux). A NAT must
send such errors and pass back those for its mappings (RFC 5508). Linux
sends them from its inside address, which is private (RFC 1918) or, at a
carrier-grade NAT, shared (RFC 6598); the first router that answers from a
public address is past this end's NATs, and its hop count is the limit. A
NAT that answers from its outside address instead, as many home routers
do, answers from the address the server sees this end at, and the limit is
one past it. Silent routers are skipped: once the first public router has
answered, the silent hops before it get one more probe and twice its round
trip to answer. The measurement runs alongside the registration, which
takes a round trip to the server, further away, so it delays little.

Each end prints what it found (`rendezvous: priming with hop limit 2
(measured: 10.0.0.1, 198.51.100.1)`). Where nothing can be measured the
limit is 3, which passes a home router and the next hop and stops before
the other end's NAT wherever that is more than 3 hops away: on systems
other than Linux, with a server at an IPv6 or private address, and behind
a NAT that passes no errors back, after waiting 0.5 s for them.
`--prime-hops N` sets the limit instead of measuring it. With 0, or on a
system that cannot limit the hop count, nothing is primed: the master
still waits for the slave's ready, but the slave, which must then go
first, punches at once.

A limit too large reaches the other NAT. One too small opens nothing,
and then the ready alone orders the datagrams, which fails when the other
end becomes ready later than this one by more than its datagrams take to
reach this end's NAT: the late end knows at once that this one is ready
and sends, while this end hears of the late ready only through the server.
An end is late when its route to the server is longer, or its measurement
waits; the difference is most likely to exceed the route between the ends
when they are close to each other and far from the server.

`tools/nat_lab.sh` builds a master and a slave behind Linux NATs that
filter by address and port, a rendezvous server and a chain of routers
between the NATs, in network namespaces (in a privileged container on
other systems; the script says how). Three routers, 20 ms round trip
between the NATs; *time* is from the master's start to the end of a 2 s
session:

| scenario | result | time |
|---|---|---|
| cone NATs (endpoint-independent mapping) | connects; measured limit 2 | 2.14 s |
| the master starts 0.5 s before the slave | connects | 2.66 s |
| both behind one NAT | connects, over the LAN | 2.09 s |
| the slave 100 ms behind its NAT | connects | 2.75 s |
| the master 100 ms behind its NAT | connects | 2.98 s |
| both NATs accept datagrams to themselves | connects | 2.14 s |
| the slave 100 ms behind a NAT that accepts them | connects (failed without priming and ready) | 2.77 s |
| the master 100 ms behind a NAT that accepts them | connects (failed without priming and ready) | 2.97 s |
| both 100 ms behind NATs that accept them | connects | 3.57 s |
| the slave 100 ms behind a NAT that accepts them, `--prime-hops 0` | connects: ready alone orders it | 2.75 s |
| the master 100 ms behind a NAT that accepts them, `--prime-hops 0` | fails: the NAT moved the master to another port | — |
| the same, primed, with one router between the NATs | connects; measured limit 2 | 2.97 s |
| the same with `--prime-hops 3` (the earlier default) | fails: the primes reach the other NAT, with one hop to live | — |
| a carrier-grade NAT behind one home router, or behind both | connects; measured limit 3 there | 2.19 s, 2.22 s |
| the carrier-grade NATs send no time-exceeded errors | connects; limit 3, past the silent hop | 2.25 s |
| home routers that answer from their outside address | connects; limit 2, one past the NAT | 2.15 s |
| home routers that pass no errors back | connects; not measured, limit 3 after 0.5 s | 2.59 s |
| the slave's NAT passes no errors back, so the slave is ready 0.5 s late; the master starts first, behind a NAT that accepts datagrams to itself and answers from its outside address | connects; limit 2 (with 1, the first public answer, which opens nothing, it failed) | 3.12 s |
| a symmetric NAT (a new port per destination) at either end or both | fails without a relay (Through a relay, below) | — |

The slave punches until its session starts, so an end 100 ms behind its
NAT still receives the last few punches during the session; they are
dropped before the link and not counted as `foreign`.

### Through a relay

Behind a NAT that gives every destination its own outside port
(endpoint-dependent mapping, a "symmetric" NAT), the port the server saw
is not the one the other end would be let in on, and what that end's
datagrams come from is a port nobody was told of; the other NAT filters
them. No ordering helps (the table's last row). With `--turn`, the slave
allocates an address on a TURN server (RFC 8656) from a second socket and
announces it through the rendezvous server as one more candidate. The TURN
server passes on to the slave what reaches that relayed address from any
IP address the slave has given a permission, and sends from it what the
slave asks it to (`net/turn.hpp`, `net/paths.hpp`):

- **One relay crosses both NATs.** To the master the relayed address is an
  ordinary public one: its hello goes there from whatever port its NAT
  picks, and the slave's answer comes back from the address the hello went
  to, which that NAT lets in. Permissions are per IP address, without the
  port, which is what makes this work against a symmetric NAT at the
  master: the port is unknown in advance, but the address is the one the
  rendezvous server saw. The slave's own NAT sees a single destination,
  the TURN server. So only the slave takes a relay, and the master has no
  TURN client.
- **Direct first.** The master sends its hellos to the slave's own
  candidates first and to the relay 1 s later
  (`RendezvousConfig::relay_delay`); a direct path that opens answers well
  within that. It takes the first reply that authenticates, and the session
  starts on that path; one that starts on the relay moves to a direct path
  that opens later (below).
- **The relay carries only sealed datagrams.** TURN authenticates the
  slave's requests with its long-term credentials (MESSAGE-INTEGRITY, RFC
  8489, 9.2), and the client accepts a response to them only with a valid
  MAC. It does not protect the data, which the session's keys seal anyway:
  the TURN server learns the ends' addresses and when they talk, as the
  rendezvous server does, and its operator never needs the haptic key.
- **It costs a detour and 4 bytes.** Datagrams go in ChannelData messages
  (4 bytes more each) once a channel to the master is bound, in Send
  indications (36 bytes) before. The round trip is the master's to the
  relay plus the relay's to the slave. The relay's socket carries the
  direct socket's DSCP mark.
- **It keeps the allocation alive.** A Refresh every 15 s keeps the slave's
  NAT mapping to the TURN server (a NAT may forget it after 30 s of
  silence) and shows when the allocation is lost: the server answers 437
  when it no longer knows the slave's address. The slave then allocates
  again and announces the new relayed address. When the master falls
  silent, the slave checks at once rather than at the next Refresh.

The client is tested against coturn 4.6.1 in the lab (`nat_lab.sh --turn`
runs one, with long-term credentials, next to the rendezvous server) and
against `tests/sim_turn.hpp` in the unit tests; its STUN messages against
the test vectors of RFC 5769. In the same lab, with 2 s sessions; the TURN
server sits by the rendezvous server, on the middle router's own 5 ms
link, which a relayed datagram crosses twice:

| scenario | path | time | round trip, p50 |
|---|---|---|---|
| cone NATs | direct | 2.14 s | 20.1 ms |
| cone NATs, the slave with a relay | direct: the relay is not used | 2.14 s | 20.1 ms |
| both behind one NAT, the slave with a relay | direct, over the LAN | 2.09 s | 0.02 ms |
| the slave's NAT symmetric, no relay | fails | — | — |
| the slave's NAT symmetric | relayed | 3.19 s | 40.5 ms |
| the master's NAT symmetric | relayed | 3.20 s | 40.4 ms |
| both symmetric | relayed | 3.19 s | 40.4 ms |
| both symmetric, the master starts 0.5 s before the slave | relayed | 3.69 s | 40.4 ms |

No sample was lost in any of them. A relayed session starts about 1 s
later than a direct one: the master waits `relay_delay` before it tries
the relay.

A direct path can also open after the session has started on the relay: a
firewall that lets the other end in only later, a NAT whose mapping was
not ready when the hellos came, an end that moves to a network without a
symmetric NAT. While a session runs through the slave's relay, both ends
check for one every second (`RendezvousConfig::direct_check_interval`):

- **A copy races the relay.** Right after its link sends, an end sends a
  copy of that datagram to the other end's own candidates. The copy has
  the original's sequence number, so whichever arrives first is the newest
  datagram there, and the link follows it (When an end moves, below); the
  other is a duplicate. A copy that wins moves the other end's link to the
  direct path, the other end's samples then come that way, and this end's
  link follows them. A path that is shut, or slower than the relay, loses
  every race and moves nothing. The checks are the session's own sealed
  datagrams, so they need no STUN of their own; a WebRTC agent likewise
  keeps checking the candidate pairs it does not use, and may switch to a
  better one.
- **The relay carries copies for one more second (make before break).**
  After its link has left the relay, an end also sends a copy of every
  datagram along it, for one interval. A path may be open one way only:
  the master's NAT lets the slave in, since the master has sent to it,
  while the slave's still drops the master. The slave's copy then moves
  the master's link to a path on which the slave hears nothing; the copies
  along the relay still reach the slave, whose next sample through the
  relay brings the master back. Without them, each check would cost the
  master's samples of a period and the relay's delay (about 20 at 1 kHz in
  the unit tests).
- **The end that moves first stays off the relay for half a second.** The
  samples the other end sent along the relay before it heard of the move
  are newer than the copy that won, and would pull the link straight back.
  The two ends then go there and back, the stretch on the direct path
  growing by only 2(R - D) a round trip, R being the relay's round trip
  and D the direct path's: with the slave 100 ms behind its NAT, the lab's
  session left the relay only 3.99 s in, after 19 and 23 moves (2.22 s
  and one each now, below). For half an interval after it left, the link
  takes what arrives along the relay without following it
  (`HapticLink::stay_off`), which covers a direct round trip of up to
  500 ms.

In the lab, `--turn --open-at 3` makes the middle router drop what the
two NATs send each other until 3 s after the master starts, about 2 s
into the relayed session (6 s sessions; the times are the master's, from
the start of its session):

| scenario | result | moves, master / slave | round trip, p50 |
|---|---|---|---|
| cone NATs | left the relay 2.022 s in | 1 / 1 | 20.3 ms (about 41 on the relay) |
| the slave 100 ms behind its NAT | left it 2.222 s in | 1 / 1 | 220.4 ms (about 241) |
| the master 100 ms behind its NAT | left it 2.222 s in | 1 / 1 | 220.5 ms (about 241) |
| a carrier-grade NAT behind each NAT | left it 2.043 s in | 1 / 1 | 40.7 ms (about 62) |
| open from the slave to the master only (`--one-way`) | stayed on the relay | 8 / 0 | 40.7 ms |
| the slave's NAT symmetric | stayed on the relay: nothing opens | 0 / 0 | 40.8 ms |
| both NATs symmetric | the same | 0 / 0 | 40.6 ms |

No sample was lost in any of them. Each move to the direct path costs a
second of the relay carrying everything twice (1000 duplicates at each
end). In the unit tests (`tests/sim_nat.hpp` and `tests/sim_turn.hpp`),
with NATs that hold 25 ms each way and a relay 10 ms, so that the direct
round trip is 100 ms and the relay saves 20, each end moves once; without
`stay_off`, 11 and 9 times, and the session settles half a second later.

### Through a relay over TCP or TLS

Some networks let no UDP out, only TCP, or only what looks like HTTPS.
With `--turn turn:HOST?transport=tcp` the slave reaches the TURN server
over TCP (port 3478 unless one is given), and with `--turn turns:HOST`
over TLS (5349), as RFC 8656 allows. The relay still talks to the master
over UDP: only the slave's leg changes (`net/tcp.hpp`, `net/tls.hpp`,
`net/paths.hpp`).

- **A connection is an allocation.** The server knows the allocation by
  the connection (RFC 8656, 3.2). When the connection drops, the slave
  connects again at once, allocates anew and announces the new relayed
  address, as when an allocation is lost over UDP. A connection that does
  not open within 5 s, fails before it is allocated, or drops twice
  within 10 s is tried again after 10 s (`TurnClient::kRetryDelay`).
- **Messages follow each other on the connection.** STUN messages and
  ChannelData (padded to 4 bytes on a stream, RFC 8656, 12.5) are cut
  apart by the lengths in their headers. A request goes once, and a
  connection on which one is unanswered for 5 s is given up
  (`kStreamTimeout`), since TCP would go on resending for minutes. Bytes
  that are not TURN, such as a TLS alert from a TLS port reached with
  `turn:`, close the connection at once.
- **Nothing old is queued.** Nagle's algorithm is off, so a datagram
  leaves when it is written. What the kernel does not take yet waits in
  the stream, up to 64 KiB (`Stream::kMaxBacklog`); a datagram that would
  pass that is dropped whole, as a full UDP socket would drop it, since on
  a connection backed up that far it would be stale before it left.
- **TLS checks the server.** Its certificate must chain to a CA that
  OpenSSL trusts by default (`openssl version -d` names the directory),
  or to one in `--turn-ca FILE` (PEM), and name the host in the URI: a DNS
  name, which is also sent by SNI, or an IP address. TLS 1.2 or later,
  offering `stun.turn` by ALPN (RFC 7443). A certificate that does not
  check out is reported with OpenSSL's reason ("hostname mismatch",
  "self-signed certificate") and tried again after 10 s.

In the lab, `nat_lab.sh --turn --turn-over tcp` (or `tls`) runs coturn
4.6.1 on that transport only, over TLS with a certificate for its
address made for the run and given to the slave with `--turn-ca`; and
`--block-udp` makes the slave's NAT drop every UDP datagram it would send
on, but those to the rendezvous server. In 2 s sessions, or 6 s with the
direct path opened 3 s after the master starts (`--open-at 3`, above):

| scenario | relay | result | round trip, p50 |
|---|---|---|---|
| both NATs symmetric | UDP | relayed | 40.4 ms |
| both NATs symmetric | TCP | relayed | 40.5 ms |
| both NATs symmetric | TLS | relayed | 40.5 ms |
| UDP blocked at the slave's NAT (cone NATs) | none | fails | — |
| the same | UDP | fails | — |
| the same | TCP | relayed | 40.5 ms |
| the same | TLS | relayed | 40.5 ms |
| UDP blocked, both NATs symmetric | TCP | relayed | 40.5 ms |
| UDP blocked, a carrier-grade NAT behind the slave's NAT | TLS | relayed | 50.4 ms |
| UDP blocked, carrier-grade NATs behind both, the slave's NAT symmetric | TLS | relayed | 60.6 ms |
| cone NATs, the direct path opened 3 s in | TCP | left the relay 2.023 s in | 20.2 ms |
| the same | TLS | left it 2.021 s in | 20.2 ms |
| UDP blocked, the direct path opened 3 s in | TCP | stayed on the relay | 40.4 ms |
| the same | TLS | stayed on the relay | 40.5 ms |

No sample was lost in any of them. A session relayed over TCP or TLS
starts when one over UDP does, 3.18 to 3.26 s after the master: the
connection and the handshake fit within `relay_delay`. Leaving the relay
went as over UDP, each end moving once.

The lab loses nothing, so its tails are what the transports add on their
own. In four more runs of each of the first three rows, the round trip's
p99 was 40.7 to 40.9 ms over UDP, 40.9 to 41.5 over TCP and 41.0 to 42.3
over TLS, and the largest single one 41.5, 43.5 and 46.6 ms. Larger ones,
up to 67 ms in some runs, came with both processes stalled 13 to 24 ms at
the same moment (their `tick late`): the lab's virtual machine, not the
relay.

On a path that loses packets, TCP costs more than UDP. A lost segment
holds back every datagram behind it until it is sent again: about a round
trip of the slave's leg later when the segments after it bring duplicate
acknowledgements, as they do at 1 kHz, and after a retransmission
timeout, 200 ms or more on Linux, when the retransmission is lost too.
The datagrams then arrive late and together rather than not at all. This
has not been measured on a lossy path yet.

### When an end moves

A session can lose its path without either end doing anything: a NAT
drops a mapping (a reboot, a timeout, a carrier-grade NAT under load) or an
end gets a new address (a new DHCP lease, Wi-Fi to mobile). The moved end
keeps sending to the right address from a wrong one, and the other end
keeps sending to an address that no longer leads anywhere. Two mechanisms
repair this:

- **The link follows an authentic peer** (`HapticLink`, as WireGuard does,
  section 2.1 of its paper). A sealed sample that authenticates and is
  newer than any before it moves the link's peer to the address it came
  from; a replayed, late or forged one does not. Open links never move,
  since anyone could redirect them. Reports count the moves.
- **Both ends rejoin when they fall silent.** Behind a NAT that filters by
  address and port, the link cannot follow: the other end's NAT drops the
  moved end's datagrams, as it has never sent to the new address. Neither
  end can tell which of them moved, since both just hear nothing; so after
  100 ms without an authentic sample each registers with the rendezvous
  server again (then every 0.5 s while the silence lasts). The server sees
  the moved end at its new address and announces it to the other end, which
  punches towards it (for a second, and while the silence lasts); its NAT
  now lets the moved end's datagrams in, and its link follows them. This
  needs the rendezvous server; between two fixed addresses the link alone
  follows.
- **The punches are copies of the last sample.** A rejoining end punches
  with copies of the last datagram its link sent, sealed like any other.
  They open its own NAT as a punch would, and they also authenticate, so
  the other end's link follows the punching end to the port its NAT sent
  them from. That matters where that NAT gives every destination its own
  port: the punch towards the moved end leaves from a port nobody was told
  of, and the moved end would otherwise keep answering to the old one. A
  copy has the sequence number of the sample it copies, so where the old
  path still works it is a duplicate, and nothing moves.

`tests/sim_nat.hpp` is a NAT in user space for loopback tests (with RFC
4787's mapping and filtering behaviours): when the slave's NAT forgets its
mappings mid-session, the master loses about 100 samples and the session
goes on; without the rejoin it is lost. In the lab, `--rebind-at 1.5`
moves the slave behind natB to a second outside address, 203.0.113.3, and
flushes natB's mappings 1.5 s after the master starts (a 4.5 s session):

| scenario | result | samples lost, master / slave |
|---|---|---|
| cone NATs | the master's link follows, the master ends the session | 111 / 132 |
| the slave 100 ms behind its NAT | the same | 309 / 330 |
| the master 100 ms behind its NAT | the same | 309 / 525 |
| the slave's NAT accepts datagrams to itself | the same | 110 / 131 |
| both NATs reject them (port unreachable from the firewall, as OpenWrt's WAN zone does) | the same | 107 / 129 |
| the master's NAT accepts them | lost: the moved slave's datagrams take the master's port first | — |

The last row is the collision above, and nothing an end does can order it
away: the moved end's first datagrams reach the other NAT before anyone
knows it moved, and conntrack keeps their flow while they keep coming.
A firewall that drops or rejects them, as nearly every router has, keeps
conntrack from recording them.

Through a relay (`--turn --rebind-at 2`, since a relayed session starts
about 1.2 s after the master), natB's flush takes the slave's mapping to the TURN server with the
others. The slave, hearing nothing, refreshes at once; the TURN server
answers 437 to the new address; the slave allocates again and announces
the new relayed address, the master punches towards it, and the links
follow:

| scenario | result | samples lost, master / slave |
|---|---|---|
| the slave's NAT symmetric | the master's link follows the slave to the new relay | 173 / 214 |
| the master's NAT symmetric | both links follow: the master's copies come from a new port | 216 / 256 |
| both NATs symmetric | the same | 213 / 252 |
| cone NATs (the relay unused) | as without a relay | 112 / 131 |

The new allocation costs 60 to 100 samples more than a direct move. With
both NATs symmetric the session was lost before the punches were copies:
the slave kept sending to the master's port for the old relay.

## Replaying a path on Linux

`tuushin-sim` replays a trace inside the simulator. Testbeds that run real
network stacks, such as Mininet and those built on it, need the path on
an interface. netem, the usual tool, gives a delay drawn from a
distribution, not one that follows a recording, and reorders datagrams
whenever its jitter does. `tuushin-netreplay` replays a trace instead:

- An eBPF program on the interface's egress (tc, direct action) looks up
  each UDP datagram's send time in the trace and sets its earliest
  departure time to now plus the delay recorded then; the fq qdisc, made
  the interface's root, holds it until then, as it does for TCP's pacing.
  Becker et al. (2022) emulated delay this way, and Tian et al. (2024)
  replayed satellite traces; neither's code has a licence, so this is
  written anew.
- The lookup is by time, not by datagram number, so traffic at any rate
  meets the path as it was at that moment, and the trace loops.
- Between the trace's points the delay is interpolated linearly. A
  datagram sent at t leaves at t + d(t), which keeps the datagrams sent
  between two points in order if the points' own packets arrived in order;
  holding each point's delay as a step reorders a datagram sent late in a
  bin behind the next one's first whenever the delay falls (in a test, a
  queue that drains at 0.95 ms per ms, 27 kHz of datagrams: 7770 of 81000
  reordered in 3 s with steps, none interpolated).
- Losses are dropped at the same place, from a datagram sent where the
  trace lost its packet to the next point.

`tools/netreplay_lab.sh` puts it between two network namespaces, where
both ends share a clock and a datagram's one-way delay is known exactly,
sends 1 kHz of stamped datagrams each way and compares what they met with
the traces. On a virtual machine on a laptop (colima, Linux 6.8, 4 vCPUs):

| path | losses | delay minus the trace's (median, p99, max) |
|---|---|---|
| synthetic pair: wander, a filling queue every 4 s, 0.3 % lost in bursts; 15 s | 56 and 57 of 15000, all where the trace lost: none elsewhere, none delivered where it lost; none reordered | 0.12 ms, 1.1 ms, 30 ms |
| constant 20.5 ms, replayed | none | 0.19 ms, 0.51 ms, 2.2 ms |
| the same with netem | none | 0.18 ms, 0.47 ms, 4.5 ms |
| constant 20 ms, replayed | none | 0.07 ms, 0.53 ms, 9.1 ms |
| the same with netem | none | 0.04 ms, 0.38 ms, 11 ms |

The datagrams come out late by the latency of the kernel's timer, the
same for both tools: here 0.15 to 0.2 ms. A delay that is a whole number
of sending intervals lets each datagram leave as the next one arrives,
before the timer fires, which is why the constant 20 ms looks better. The
maxima are this virtual machine stalling its network processing for up to
tens of ms; behind a stall, datagrams are judged by the trace at the
moment they reach the router, and in one run 42 in 10000 were dropped by
losses that came after they were sent.

Two things to know when attaching it:

- **On a hop that forwards, not on the sender.** A datagram held in the
  sender's own qdisc counts against its socket's send buffer. 1 kHz of
  100-byte datagrams held 300 ms at their sender had 21 % of their sends
  refused with Linux's default 208 KiB (500 ms: 52 %); held at a router
  they have no socket.
- **It replaces the root qdisc.** Give a replayed link no tc parameters of
  its own (in Mininet, no `bw` or `delay` on its `TCLink`).

## A real path: Starlink

Garcia, Sundberg and Brunstrom (ACM LEONET 2025) published the one-way
delays of 1437 runs over ten days of April 2025 at an unobstructed Starlink
terminal in Karlstad, Sweden: 50 s each, both directions, a UDP probe every
266 µs, sent and received on one hardware-timestamping NIC, so both ends
read one clock. The data is licensed CC BY 4.0 and is not in this
repository: `tools/fetch_starlink.sh` downloads the runs asked for, about
2 MB each through Zenodo's API, and converts them to traces that carry the
credit.

```sh
tools/fetch_starlink.sh --list | head
tools/fetch_starlink.sh --out traces 2025-04-11-15-16-16
./build/tools/tuushin-sim --passivity reflect \
    --trace-fwd traces/starlink-2025-04-11-15-16-16.dl.csv \
    --trace-bwd traces/starlink-2025-04-11-15-16-16.ul.csv
```

The downlink is the forward direction here: the slave sits behind the
terminal, as a robot in the field would. Twenty runs, two a day twelve
hours apart:

| | downlink | uplink |
|---|---|---|
| median | 24.6–31.5 ms | 29.1–39.4 ms |
| p99 | 31–64 ms | 46–130 ms |
| p99.9 | 49–90 ms | 85–243 ms |
| max | 67–111 ms | 91–264 ms |
| probes missing | 0.07–2.0 % | 0.11–2.1 % |

Within a run the delay falls by 0.99 ms per ms and jumps back up by 6 to
7 ms every 5 to 13 ms (median 7.7 ms down, 5.6 ms up), so 92 to 98 % of
the probes arrive within 20 µs of the one before: the path delivers in
bunches. In the runs looked at, the delay also spikes every 15 s, with the
network's reconfigurations, and the downlink then misses 37 to 76 probes
in a row (10 to 20 ms); the uplink's missing probes come one or two at a
time, spread evenly.

The reflective scheme on each run, nine hand motions of 20 s as under
*Coarse encoders*, against constant delays at the run's medians, 720
simulations in all:

| encoders | path | worst true margin | force displayed | total variation | displayed stiffness | diverged |
|---|---|---|---|---|---|---|
| 55 µm and 1 µm, known | Starlink | 0 J | 24 % | 35 N/s | 215 N/m | none |
| | constant | 0 J | 23 % | 37 N/s | 207 N/m | none |
| exact | Starlink | −4.9e−15 J | 38 % | 59 N/s | 357 N/m | none |
| | constant | −1.6e−15 J | 37 % | 35 N/s | 337 N/m | none |

The margins below zero are the predictor's misses of a few femtojoules,
not rounding (Both directions on one clock). With encoders whose resolution the
controllers know, the path costs nothing beyond its median delays: the
same share of the contact force, the same chatter (on one run also with
1, 5 and 20 µm master encoders). With exact readings, which nothing
filters, the displayed force varies 1.7 times as much, and the bunches
cause most of it. On one run, holding each probe's delay at the largest
of the last 15 ms, which spaces the arrivals like the sends and which a
receiver could do knowing delays only up to its clock offset, brought the
variation from 57 to 40 N/s, against 32 at a constant delay of the
result's mean (34 ms); a playout buffer at the run's p99 (34 and 52 ms)
brought it to 23, as at a constant 43 ms, and halved the stiffness (386 to
198 N/m). There is no playout buffer: with real encoders it would buy no
smoothness (38 N/s before and after on that run) and cost half the
stiffness (287 to 143 N/m).

The conventional TDPA, on the default scenario (`tuushin-sim`, exact
encoders, 10 s), generated 0.28 J over a run at the median and 0.41 J at
worst, against 0.11 and 0.18 J at a constant delay halfway between each
run's two medians (the simulator takes one delay for both directions);
the reflective scheme generated nothing on any run.

Replayed by `tuushin-netreplay` in the lab above (one run, 20 s of 1 kHz
each way, on the virtual machine at a load of about 7.6), the datagrams
sent where the trace lost were all lost on the downlink (12) and 24 of 28
on the uplink; 2 and 8 more were lost elsewhere. The delay minus the
trace's had a median of 0.11 and 0.09 ms, a p99 of 7.0 and 7.2 ms and a
maximum of 57 and 58 ms, all of it late and at the same moments in both
directions: the virtual machine stalling. 2 and 5 datagrams arrived out
of order, each within 63 ns of its neighbour: released in one bunch and
stamped by the receiver in another order.

## Both directions on one clock

`measure` gets one-way delays from two clocks, the client's and the
reflector's, and must model their offset: it fits it to the minimum
delays, which assumes that the two directions' minima are equal. Where
they are not, a difference a moves a/2 from one trace to the other, and no
measurement between unsynchronized clocks can tell (Known issues). Access
networks are asymmetric by design (a cable or mobile uplink is narrower
and scheduled differently from its downlink). The twenty Starlink runs
above, taken on one clock with hardware stamps, had uplink minima 1.7 to
8.1 ms below the downlink's (and medians 2.5 to 12 ms above them): `measure`
would have moved up to 4 ms of delay from one direction to the other.

`pair` puts both ends on one host instead, on two of its networks: each
probe leaves through one interface, crosses the internet and comes back in
through the other, and its delay is a receive time minus a send time on
the same clock. That is the path a teleoperator would meet between an
operator on the first network and a robot on the second: the first
network's uplink, the internet between them and the second's downlink,
and the reverse.

- Each socket is tied to its interface (`SO_BINDTODEVICE` on Linux,
  `IP_BOUND_IF` on macOS) and bound to its address, so the datagrams leave
  through it whatever the routing table prefers.
- A STUN server (RFC 8489 Binding) tells each end its public address. Two
  ends with the same one share their last router, and a datagram between
  them could turn round inside it without crossing the internet: pair
  refuses.
- Each end sends hellos to the other's public address every 20 ms until
  both have heard from the other. An end that hears from an address other
  than the one the server saw (a NAT that maps by destination) answers
  there: a peer-reflexive address, in ICE's terms.
- Then both directions are probed at the same rate and stamped by the
  kernel on arrival; `PREFIX.ab.csv` and `PREFIX.ba.csv` are traces for
  `tuushin-sim` and `tuushin-netreplay`. They record no address.
- A probe whose send call fails never left, so it counts as neither sent
  nor lost. When every send through one end fails for a second, the end
  has gone away (a phone unplugged takes its interface with it), and the
  run ends where the failures began: what follows is not the path's loss,
  nor are the losses just before it, back to the longest delay seen, as
  those probes may have been on their way to the end when it went.
  `measure` does the same where its own sends fail, leaving out the
  unanswered probes back to the longest round trip seen.

`tools/pair_lab.sh` puts each of two interfaces of one network namespace
behind its own NAT, the NATs meet in a router that runs coturn as the STUN
server and delays each direction differently with netem. With 20 ms from
a to b and 35 ms back (5 s of 1 kHz each way, the virtual machine above),
pair measured medians of 20.049 and 35.041 ms, minima of 20.011 and
35.009 ms, nothing lost or reordered, and a difference of -14.999 ms at
the minimum, where halving the round trip, as `measure` does, gives
27.51 ms each way. Behind a NAT that maps each destination to a new port
(here, `MASQUERADE --random-fully`) on either side, the hellos found no
way through, as they cannot when the other NAT also filters by port, and
pair said so after 5 s.

On a real host: on macOS, an iPhone tethered by USB is an interface of
its own (`networksetup -listallhardwareports` names it), next to Wi-Fi.
On Linux, strict reverse-path filtering (`rp_filter` 1) drops what
arrives on the second interface from addresses the first one routes to,
which is all of them; loose filtering (2) lets it in. A Linux host also
needs a default route through each interface.

A real path: a Mac on home Wi-Fi (5 GHz) and, tethered to it by USB, an
iPhone on mobile data (5G NSA when its log last said), probed at 1 kHz
each way for 15 minutes, of which the first 820 s count (the phone was
then unplugged; pair now ends a run there by itself):

| | min | p50 | p99 | p99.9 | lost |
|---|---|---|---|---|---|
| Wi-Fi to phone (home uplink, mobile downlink) | 8.744 ms | 17.339 ms | 82.846 ms | 105.967 ms | 0.023 % |
| phone to Wi-Fi (mobile uplink, home downlink) | 13.886 ms | 30.349 ms | 69.592 ms | 153.467 ms | 0.005 % |

The way out through the phone is 5.1 ms slower at the minimum and 13.0 ms
at the median, as a mobile uplink waits for a grant before it may send.
Halving the round trip would have put 11.3 ms on each direction at the
minimum and 23.8 ms at the median: off by 2.6 and 6.5 ms. Each minute's
median stayed within 16.4 to 18.3 ms one way and 29.9 to 31.6 ms the
other. What remains in the tail is, on the way to the phone, about 20 a
minute above 90 ms, each a pause and a drain (after the peak the delay
falls 0.98 ms per ms of send time) of 40 to 110 ms over the delay just
before (most 70 to 90), with no loss before it and next to nothing in the
other direction at the same moment. So they are not at the Mac's Wi-Fi, which both directions cross,
but on one direction's own stretch: the mobile downlink most likely,
though this run cannot tell it from the home uplink.

Two shorter runs told the rest. One, of 120 s, ran beside the phone's
baseband log: the phone was on 5G NSA for the first 20 s and on LTE after,
and the stalls went on, 38 a minute on 5G and 69 on LTE. In the other, of
60 s, each interface also sent ICMP echoes at 100 Hz to one public anycast
resolver. From the Wi-Fi, over the home line alone, none of 6000 came back
more than 16 ms over the median (22.9 ms at most); from the phone, 159
stalls rose 40 ms over it, and 160 of the 178 on the way to the phone fell
within 50 ms of one of them, where 29 % would by chance (the two sets of
stamps aligned at the best offset). The stalls are the mobile downlink's,
on LTE as on 5G.

The reflective scheme on these traces, cut into 20 s windows (41 from the
long run, 6 from each 120 s run, 1 from a 30 s run), nine hand motions on
each as under *Coarse encoders*, forward to the phone (the robot behind
it, as with Starlink), against constant delays at each window's medians:

| encoders | path | worst true margin | force displayed | total variation | displayed stiffness | diverged |
|---|---|---|---|---|---|---|
| 55 µm and 1 µm, known | home and phone | 0 J | 27 % | 38 N/s | 241 N/m | none |
| | delays capped at 33 and 44 ms | 0 J | 27 % | 38 N/s | 260 N/m | none |
| | constant | 0 J | 28 % | 39 N/s | 274 N/m | none |
| exact | home and phone | −2.5e−15 J | 41 % | 69 N/s | 397 N/m | none |
| | delays capped at 33 and 44 ms | −1.7e−15 J | 43 % | 67 N/s | 453 N/m | none |
| | constant | 0 J | 46 % | 48 N/s | 525 N/m | none |

Unlike Starlink, this path costs something beyond its medians: 12 % of
the displayed stiffness with real encoders and 24 % with exact ones.
Capping each delay at the long run's 90th percentile (33 ms to the phone,
44 ms back) gave back about three fifths of it with real encoders and two
fifths with exact ones, and next to none of the extra chatter (69 to
67 N/s, against 48): the stalls soften the wall, and the chatter comes
from the jitter below them. With the directions swapped (the robot on the
Wi-Fi) it was the same, 248 against 273 N/m and 404 against 525. The
margins below zero are not rounding. Each of the eight looked at (the
largest here and seven from the Starlink runs) was one sample's overshoot
at one end, seven of them at the master: the work the predictor let
through (`predictive_pc.hpp`) went past what was left by tens to tens of
thousands of times the last bit of the books. That is model error, which
the margin covers only once it has been seen (Known issues). The largest,
−6.5e−13 J in one of the 3888 runs (swapped, exact encoders), was 0.3 % of
that sample's 0.2 nJ.

Two things on the Mac got in the way. macOS did not bring the phone's USB
interface up by itself (`sudo ifconfig en6 up`, with en6 the phone's
port). And AWDL, the link behind AirDrop and Sidecar, takes the Wi-Fi off
its channel to visit its own: a 10 s trial showed stalls of about 100 ms
every 512 TU (524.3 ms) in both directions, the Wi-Fi-to-phone median at
47.8 ms, and the same stall meeting the other direction 20 to 35 ms of
send time earlier, as it would at the Mac's end. The period is that of
the two AWDL channel visits per 16-window period in Stute et al. (USENIX
Security 2019, figure 1). `sudo ifconfig awdl0 down` removed it (until
`up` or a restart). A Mac measuring a path, or running an end of a
teleoperator, wants AWDL off.

## A device in the loop

The controllers drive a `teleop::Device1D`. Besides the simulated ones,
`device::RemoteDevice` is a device running tuushin's own firmware: a
haptic knob of one axis, a gimbal motor under field-oriented control with
a magnetic encoder, whose servo loop runs on a microcontroller and talks
to the host over a serial port (or TCP, for the simulation and the
emulator).

| part | what it is |
|---|---|
| `include/tuushin/device/proto.hpp` | The link, compiled by both sides: messages in COBS frames with a CRC-16. The device streams a State every millisecond on its own clock (the encoder count, the speed, the torque applied, the work it booked, and which command was in effect since when); the host sends a torque whenever it has one, which the device applies on its next tick. |
| `firmware/core` | `fw::DeviceCore`, the servo loop in portable C++ that does not allocate: field-oriented control by voltage, the calibration of the motor's electrical zero and of its windings' L / R, the safety rules (the driver is off until the host enables it, the torque is cut 2 ms after the last command and the driver switched off after 500 ms, a failing encoder or a runaway latches a fault), and the work cap below. `fw::Plant` is a simulated gimbal motor (back-EMF, cogging, friction, a hand that turns it, an object it runs into) for the core to drive. |
| `firmware/esp32` | The core on an ESP32, ESP32-S3 or ESP32-C3 under ESP-IDF, with the parts, the wiring and the bring-up: [firmware/esp32/README.md](firmware/esp32/README.md). Built, and run end to end under QEMU; not yet run on a chip. |
| `tuushin-devsim` | The core driving the plant in real time behind a pseudo-terminal or a TCP port: a device to develop against with no hardware. |
| `tuushin-device` | One device on its own, for bringing it up: its Info, its encoder, its calibration, a steady torque to weigh on a scale, a spring to feel. |
| `device_lab.sh`, `emu_lab.sh` | The teleoperator end to end with two simulated knobs, or with the ESP32 firmware under QEMU at the slave. |

```sh
./build/tools/tuushin-devsim --tcp 47300 --hand &       # a knob that a hand turns to and fro
./build/tools/tuushin-device tcp:127.0.0.1:47300 info
./build/tools/tuushin-teleop master HOST --device tcp:127.0.0.1:47300
./build/tools/tuushin-teleop slave --device serial:/dev/cu.usbmodem1101,radius=0.025
tools/device_lab.sh build 10
```

`--device` takes `serial:PATH` (or `PATH`) or `tcp:HOST:PORT`, then
`,option=value`: `radius` (0.02 m; the controllers think in metres and
newtons at the rim, the device in radians and N m at the shaft),
`reverse`, `baud`, `torque-limit`, `damping` (on the device's own ticks),
`velocity-limit`, `state-us`, `calibrate=auto|always|never` and `nozero`.

What the host takes from the device:

- **Its model**, from its Info: the rotor's inertia over `r^2` as a mass
  (17.5 g at the rim for the simulated knob), the encoder's count as the
  quantization, and the torque ceiling as the largest force.
- **The slave's coupling**, from the device's damping and the loop's
  delay rather than a fixed gain. Damping on the device's ticks acts with
  no link in between, and a sampled spring `K` stays passive while that
  damping `b` exceeds `K T / 2` (Colgate and Schenkel 1997); here the
  force also waits for the link and the State's age, which count as a
  longer hold, and the slave takes half the stiffness that allows: 524 N/m
  for the simulated knob (2.5 N s/m at the rim, 2.39 ms), where a slave on
  a simulated device couples with 2000.
- **The energy books**, from the work the device measures. The firmware
  sums the torque times the displacement over its ticks, and each State
  carries the total. The torque is the one the windings' current makes,
  which follows the voltage over their L / R from where the PWM's next
  period starts: the firmware keeps it as an interval and books its
  middle, for an L / R it measures in calibration (Measuring the
  windings, below).
  The controllers' own estimate, which splits a sample's displacement by
  a model when the force changes within it, missed by hundreds of
  micrometres on a slave pressed into the object. With the device's
  damping off, the books matched the simulated motor's true work: −9.51
  against −9.5 mJ at the master, +7.81 against +7.6 mJ at the slave; with
  damping, they differ by what the damping dissipates, outside the books.
- **The actuation delay**, measured command by command. Each State
  carries the sequence number of the command in effect and how long it
  has been (`cmd_seq`, `cmd_age_us`); the host knows when it sent that
  command, and places the device's time on its own clock as it places the
  positions. The mapping's error, from the link's asymmetry or the least
  delay's, is the positions' too, so the force is still timed right
  against them. The controllers take the longest of the last 64
  (`Device1D::actuation_delay`), and until those have come back, the
  longest of eight commands `connect` sends with the driver off, and a
  tick.

The estimate this replaces (Actuation delay above) was wrong both ways
on these devices. In two runs where the controllers still estimated it,
the slave's commands took 0.67 and 1.60 ms to act (at a 200 µs and a
1 ms tick) and the estimate said 0.375 and 1.125 ms, short by 0.29 and
0.48 ms, the side that costs energy; the master's took 0.71 and 1.09 ms
and the estimate said 1.19 and 2.13 ms, which holds the force back.
Timed by the measurement, the master's passivity controller acted on
fewer ticks and the operator felt more of the slave's force, in five
runs of the lab's set-up below and three of each other, against as many
runs before:

| set-up | ticks the master's PC acted on | displayed force rms over the slave's |
|---|---|---|
| the table's | 92 → 81–89 % | 0.23–0.24 → 0.27–0.33 |
| no damping on the devices, 300 N/m | 83–85 → 53–78 % | 0.49 → 0.53–0.71 |
| 1 ms tick | 95–97 → 94 % | 0.10–0.126 → 0.133–0.136 |

The delay is not one number for long. The device picks commands up on
its own ticks, and its clock is not the host's: with the simulated
device's clock 100 ppm fast or slow (`tuushin-devsim --ppm`), its ticks
drift through the host's and the delay swept a whole tick, 0.61 to
0.84 ms at a 200 µs tick and 1.01 to 2.02 ms at 1 ms.

What the host gives the device besides the torque is **a cap on its
work**. The books learn of the work a State and a link's latency after
the device did it, and a light knob can turn round within that (below).
So each command carries the most work the host's torque may have done by
then, counted as the books count it: at the master, the work booked plus
the port's balance, which puts the cap below the work while the port is
in debt; at the slave, the work plus the store's grant less the
quantization tax and a share of what the store holds beyond, or less its
deficit. The firmware keeps to the cap tick by tick
(`DeviceCore::keep_to_cap`). It predicts the coming tick's work from a
velocity observer that feeds the motor's torque forward (Lorenz and Van
Patten 1991), taking the worse of a rotor alone and of one held by a
hand of any inertia, and lowers the torque until the work fits what is
left. Once the cap is spent it holds the torque while the torque does no
work, and lets it rise again only where the work the rise takes out
repays the tax it costs. The device taxes every change of the torque it
applies, the host's, the cap's, the voltage limit's, as it applies it,
and reports the tax with the work (`State::tax`; `kInfoTaxesChanges` in
its Info), so that the host's books take none of their own. A rise it
holds back then costs nothing: when the host taxed its commands, a slave
on the emulated board, at rest against the object while its host raised
the force twice, left the store 0.09 mJ down for as long as it stayed
there, more than half of that run's lowest margin, −0.166 mJ, until the
device gave back the tax of what it held back. With the encoder's noise
configured, the tax is the noise tax (Encoders that dither, again, under
Model-mediated teleoperation above), and each change is booked by a
second reading taken once the new duties are out, which nothing decided
on (`Hal::reread_encoder`; "Reading again" in
`firmware/core/src/core.cpp`). On the simulated motor at half a count,
the noise tax takes 3.4e−5 J where the interval tax takes 6.9e−5 J for
the same motions under a cap far off, and 3.0e−5 against 2.1e−4 J on a
stiff spring, with the true work never above the books; a first rise
from rest costs about 3.6 times the interval tax. The device checks that
the second reading is a conversion of its own: the two readings'
difference, less the motion the velocity predicts, has twice the
readings' variance for independent noise and none for the same error
read again (0.98 and 0 on the simulated motor), and it logs the measure
after 4096 rereads. What the device cannot foresee as it keeps to the
cap, the books then owe. A shaft at a count's edge steps the torque's
way unannounced, the torque times a count: 38 µJ at the ceiling. And the
velocity estimate lags a stiff contact's push as it changes, which the
torque turns into work at τ·dt for each rad/s it misses: where the cap
is spent, a knob thrown off the object meets a torque the prediction
lets turn it round within the tick for no work, and it comes back faster
by what the estimate missed. Each device reports the most it overran its
cap (`DeviceCore::worst_overrun`, in `tuushin-devsim`'s last lines and
the ESP32's report): in 16 runs of the lab below, 7 to 34 µJ at the
master and 7 to 70 µJ at the slave; with the emulated board's 1 ms tick
and the coupling it gets under QEMU (`DEVSIM_ARGS="--tick-us 1000
--latency 0.1" SLAVE_ARGS="--kp 700"`), up to 115 µJ at the slave in 12
runs, half of them under 33 µJ. A margin for the estimate's spread does
not pay: it cut a torque braking a hand's slow turn and let it back on
at every count, at the tax each time.

Ten seconds of `device_lab.sh` on an Apple M4 Mac, a hand at the master,
an object half a radian on at the slave:

| | master | slave |
|---|---|---|
| States, lost | 10034, 0 | 13179, 0 |
| device round trip p50 (a command to the State applying it) | 1.40 ms | 1.34 ms |
| actuation delay p50 (a command to the tick applying it) | 0.70 ms | 0.69 ms |
| coupling | | 524 N/m |
| largest force | 4.63 N displayed | 10.9 N |
| energy | 0.0281 J | −0.0270 J |
| work cap overrun at most | 6.9 µJ | 25 µJ |
| margin at the end, and at its lowest | +1.02 mJ, −0.015 mJ | |

Before the work cap the dip was the teleoperator's, not the books'. In a
run followed sample by sample (no damping on the devices, a coupling of
300 N/m, a dip of 1.57 mJ), the slave's passivity controller switched
the force between nothing and 3 to 4 N from one sample to the next as
the slave left the object, and the 17.5 g knob bounced off it: with
States 0.8 to 1.8 ms old and a command acting 0.6 to 0.8 ms after it is
sent, the speed reversed within a sample, and that one sample did 0.7 mJ
of work; the master made the other 0.6 mJ. Under QEMU, the ESP32 firmware
at the slave dipped 0.35, 1.27 and 2.1 mJ in three runs. With the cap,
and the tax of the rises it holds back given back, that set-up
(`DEVICE_OPTS=",damping=0" SLAVE_ARGS="--kp 300"`) dipped 0.013 to 0.039
mJ in three runs, the table's 0.019 to 0.038 mJ in five, at a 1 ms tick
0.002 to 0.032 mJ in three, and under QEMU 0.008 to 0.019 mJ in three,
where the ESP32 overran its cap by 1.8 to 4.8 µJ. With the forces also
timed by the measured delay, 0.013 to 0.031, 0.015 to 0.021, 0.001 to
0.038 and 0.012 to 0.025 mJ, the ESP32 overrunning its cap by 0.4 to
27 µJ (the 27 in a second in which the emulated board missed 96 of its
ticks); with the simulated device's clock 100 ppm fast or slow, 0.010
and 0.038 mJ at the table's tick and 0.002 and 0.094 mJ at 1 ms.

What is left is what the devices cannot foresee, and the books in
flight. Mostly it is the master's overrun, which the margin meets where
the slave's store is spent. At a 1 ms tick the slave raised the force on
a knob moving into the object at about 0.16 m/s, which the object
stopped within the tick, in one run of three before the delay was
measured and one after, and in one of two with the device's clock
100 ppm off, each time at the same point of the hand's motion: the rise
took out none of the 0.066 to 0.085 mJ of tax it cost, and pushed the
knob back with 0.038 to 0.11 mJ; the master's balance covered all but
0.031 to 0.094 mJ of the two. The tax of a rise is in the books from
the command on, the input that is to repay it only from the States that
report it, within the four samples the rise was granted on (above): in
a 10 s run the slave's store fell below zero this way 6 to 26 times, at
the table's tick mostly for two samples and by up to 0.089 mJ, at 1 ms
for four and by up to 0.131 mJ. The master's balance covered nearly all
of it; it was 0.005 and 0.008 mJ of two runs' lowest margins, and all of
one under QEMU, 0.025 mJ. And the tax of a rise the device holds back
stays in the books until the State that gives it back arrives, a round
trip and a State's age after the command: 0.018 mJ at the start
of one run, whose slave's host raised the force on a knob at rest every
few milliseconds, and the device held each rise back.

### Measuring the windings

The books take the torque of the windings' current, which lags the
voltage by their L / R, and L / R is the number of a gimbal motor least
known. The listings of the GM2804 give its resistance, 9 Ω at iFlight
and 5.57 Ω at iPower, and no inductance; the one measurement found, a
SimpleFOC user's of a GBM2804-100T, is 6.3 Ω and 2.0 mH on the q axis,
1.5 on the d axis. ODrive's calibration finds gimbal motors' inductance
outside the range it accepts. The ways to measure L and R at the drive,
ODrive's and SimpleFOC's calibrations and TI's InstaSPIN motor
identification, all measure the current, which the boards in mind have
no sensor for; a search of the literature and of patents found none that
does it from the shaft's angle alone. A device whose L / R lies outside its bounds
books work the current did not do, unbounded (Known issues).

So the calibration measures it from the encoder
(`fw::WindingIdent`, `firmware/core/src/windings.cpp`). Once the
electrical zero is found, it puts a sinusoidal q voltage on the windings
at six frequencies, from 625 Hz, or a quarter of the tick's rate if that
is less, down to a fifth of that, half a second each, sized to move the
shaft 8 mrad within the 5 V limit and ramped up and down over five
periods so that the shaft ends where it began (within 1 mrad, 9 on the
way; it wandered 0.15 rad when the voltage stopped at once). A lock-in
on the readings gives the angle per volt at each frequency, which the motor's
equations make

    G(s) = kt e^(−sD) / ((L s + R)(J s² + b s + k) + kt ke s)

for a shaft of inertia J, damping b and a spring k (the cogging about
its detent): the ratio of its cubic and quadratic coefficients is L / R,
less b L / (R J), 1e−4 for a free knob. The fit weighs each point by the
noise the lock-in left, takes in Coulomb friction by its describing
function and the delay D, and takes out the tick's hold and the images
it folds back. It rejects a shaft that is not free: friction or cogging
too large against the inertia's torque, an inertia not within a factor 2
of the one configured, or damping that would take more than 2 % off L /
R, which is a hand on the knob.

On the simulated motor of `tests/test_fw_windings.cpp` (1.5 mH, 5 Ω,
7e−6 kg m², Coulomb friction of 1e−3 N m and cogging of 1.5e−3, the
encoder's noise half a count, a 50 µs PWM period, a 200 µs tick; 4.1 s
a run, 8 seeds each):

| | L / R off, on average | spread | σ the fit gives |
|---|---|---|---|
| as above | −0.4 % | 1.1 % | 1.3 % |
| friction 4e−3 N m | +0.9 % | | |
| cogging 1.2e−2 N m | +0.4 % | | |
| both at 4e−3 N m | +0.7 % | | |
| the encoder's noise 2 counts | −1.3 % | 3.6 % | 4.4 % |
| 0.5 mH (32 seeds) | −0.5 % | 5 % | 5.7 % |
| 3 mH | +0.3 % | 0.7 % | |
| PWM period 0, 100 µs | −0.3, 0.0 % | | |
| a hand on the knob (1e−5 kg m², 2e−3 N m s/rad) | rejected 8 of 8 | | |
| a 1 ms tick | rejected 8 of 8, friction | | |

At a 1 ms tick, QEMU's, the frequencies come down to 50 to 250 Hz, where
a gimbal motor's friction is 0.23 of the inertia's torque; L / R came
out 1 to 5 % high and the fit's check rejects it.

The device then takes L / R within three of the fit's standard
deviations, with 1 % for what the model leaves out, and widens that by
the configured spread for the windings' heating (20 %,
`current_lag_spread`). The bounds are no narrower than the configured
ones, which are the same spread about a configured L / R: wider, by the
measurement's own spread (237 to 370 µs for 297, against 250 to 360 for
300 configured). What the measurement buys is where they lie, about the
motor's L / R rather than a listing's or an LCR meter's; narrowing them
takes following the windings' temperature, which is not done. When the
configured L / R lies outside the fit's three standard deviations, the
device says so ("L/R configured as 600 us: check L and R"). The fit's D
is logged but does not narrow the delay the books allow, any within the
PWM's period: the tick and the PWM run on clocks of their own, so where
a write lands in the period varies (on the simulated motor at a 200 µs
tick every write lands on a period's edge, and D came out at the
period). The bounds are stored with the calibration, and a device
calibrated before keeps the configured ones until it is calibrated
again; a fit that fails, or a hand that grips the knob during it, leaves
the configured ones too, and the calibration succeeds.

```sh
./build/tools/tuushin-devsim --tcp 47311 --inductance 1.5 --pwm-us 50 --identify-windings &
./build/tools/tuushin-device tcp:127.0.0.1:47311 calibrate
```

logged "windings: L/R 301 us +- 1.3 %, delay 46 us, J 7.00e-06" and
"current lag 239 to 379 us", the calibration 6.9 s in all. The ESP32
firmware measures by default (`CONFIG_TUUSHIN_IDENTIFY_WINDINGS`), except
with the AS5600, whose filter lags the readings by more than the
frequencies' periods. It has not been run on a chip.

## Known issues

- **Devices need a watchdog.** The stall handling above relies on the
  device cutting a stale force by itself; a host process cannot do it while
  it is descheduled. Behind a device without one, a stalled loop leaves the
  last force on (the first row of the table above).
- **Authentication is opt-in and needs a key handed over out of band.**
  Without `--key-file` nothing is authenticated (the tools say so at start).
  The key belongs to one master-slave pair, like a WireGuard preshared key;
  there are no public-key identities, and the key changes only when you
  make a new one.
- **A flood of forged hellos costs the slave CPU** while it waits for a
  session: about 10 µs to reject each one. There is no cookie mechanism
  (WireGuard's answer to this under load).
- Without `--hello-state`, the slave forgets the newest hello time when it
  restarts: until the master's next hello, every hello recorded before can
  be answered once more, oldest first (still without starting a session),
  each holding one of the 16 waiting slots for up to 5 s. A master whose
  wall clock is set back is refused until it passes the newest time the
  slave has seen; with `--hello-state`, across restarts too, so delete the
  file if a clock ran far ahead.
- **Coarse encoders cost transparency.** With a 55 µm master encoder the
  teleoperator stays passive and at 10 ms conveys at least as much of the
  contact force as with exact encoders, but at 50 and 100 ms only 65 and
  80 % of what they convey, and the force chatters about twice as much
  (Coarse encoders above). Slower hands lose more: at 0.15 to 0.3 Hz the
  master conveys a fifth to three fifths of what it would with exact
  encoders. A hand pressing into the wall slower than about q/(4 dt),
  1.4 cm/s on a Touch, still never starts a force. A longer tax horizon
  lowers that threshold (eight samples started a force on a hand peaking
  at 0.94 cm/s) but in our runs let the actuators generate about a
  microjoule. Encoders that dither cost far more: with half a count of
  noise the master conveys a fifth of the contact force at 10 ms, and
  under a tenth at 50 and 100 ms (Encoders that dither, above). The
  model-mediated scheme with the noise tax, a second reading, a surface
  that yields where the PC cuts and a fit corrected for the noise meets
  the wall once per press, at a stiffness felt within a tenth of the
  wall's, but about 0.07 mm deeper than with exact readings, and with
  a displayed force that varies by 32 N/s against 12 to 23 N/s
  (Model-mediated teleoperation, above).
- **Exact readings of a misdescribed device are covered nearly, not
  entirely.** The model error margin covers misses like those already
  seen; a miss larger than any before it is not, and over nine hand
  motions a master with 80 % of its mass beyond a 50 Hz resonance still
  generated 0.19 mJ (Model mismatch above). Coarse encoders, whose
  misses show among the innovations, generated nothing on the same
  devices.
- **A device that does not measure its actuation delay has it estimated,
  and only on simulated hands has the estimate been tried.** Their
  impedance is constant; a real grip changes, and the filters let the
  damping and stiffness drift at a rate (a grip that changes over
  seconds) that no simulation has tested. The estimate needs the device
  to move under changing forces: until then it stays at the maximum,
  which holds the forces back. A delay that varies from command to
  command is not modelled (Actuation delay above). A device that
  measures it is timed by the longest of its last 64 commands, which
  lags the device twice over. When a drifting clock carries one of the
  device's ticks past the moment the commands arrive, the next command
  waits a tick longer than those before it, and the controllers learn of
  it a round trip later. And after a stall the controllers hold the
  forces back as if the next 64 commands still took that long (up to
  22 ms under QEMU, whose emulated board stalls now and then).
- **No real device yet, one degree of freedom.** The firmware has run on
  a simulated motor, as a process and under QEMU, but not on a chip:
  its timing at 5 kHz, the encoders' drivers and the torque's true scale
  are unmeasured (firmware/esp32/README.md). The torque is set by voltage,
  without current sensing, so the energy books are as right as the
  motor's resistance and torque constant, and the resistance drifts with
  the windings' temperature.
- **A device's noise tax rests on a second conversion, on time it has
  not been measured to have, and on the change's instant.** Whether an
  MT6701 or an AS5047P converts anew for a second transfer some 10 µs
  after the first, and whether its noise is white at the tick, is
  unmeasured: their datasheets do not give the conversion's timing or
  filter, and the device's own check (`reread: indep` in its log) is
  what will tell. The tax is computed in double precision, which these
  chips do in software: with one logarithm and one square root a change
  and some forty other operations, about 5000 cycles on an ESP32-S3 by
  Espressif's figures (1619 for a double `cos`, 70 for an addition),
  some 20 µs of the 200 µs tick, also unmeasured.
- **A device's books bound the current's lag by a measurement checked in
  simulation only.** The current follows the voltage over the windings'
  L / R, some 0.3 ms on a gimbal motor, from where the PWM's next period
  starts: a spring the device renders lags by that much, which makes
  energy the torque set does not, about k ω² (L / R), 0.3 mW for
  1 N m/rad at 1 rad/s. The firmware keeps the current's torque as an
  interval, for an L / R within the bounds it measured in calibration
  (Measuring the windings, above), or within 20 % of the one configured
  where it did not measure, and any delay within the PWM's period, books
  its middle, taxes half its width on the travel either way
  (`fw::CurrentLag`), and keeps room under the work cap for what the
  current flowing will still do. On the simulated motor at 1.5 mH, two
  seconds of a hand pushing into a host's wall at 2 N m/rad made 1.68 mJ
  that books taking the torque set at once missed, ten times their noise
  tax of 0.18 mJ; the interval's books came 0.13 to 0.34 mJ above the
  true work, and its tax took 0.50 to 1.16 mJ besides, with the PWM's
  period at 0 and 50 µs and the tick at 200 µs and 1 ms (with L / R as
  configured). Under a cap of 100 µJ a knob leaving a stop did 37 to
  60 µJ of true work, against 48 to 73 µJ without the inductance, and
  the device overran the cap by at most 0.001 µJ. The tax is the width
  of what the device does not know: L / R's spread, which the
  measurement puts about the motor's L / R but does not narrow, since the
  windings warm by an amount nobody measures; and where in the PWM's
  period a change lands, which the tick's clock does not know. The
  measurement's simulated motor is the model the fit assumes, with
  friction and cogging, so a real motor's departures from it (the
  iron's saturation, an inductance that varies with the angle, a
  coupling between the motor and the knob) are untested, and at a 1 ms
  tick the fit rejects even the simulated motor and the configured L / R
  stands. What the model leaves out is not bounded: an L / R outside the
  bounds, the bridge's dead time, the back-EMF's coupling of the axes
  through the electrical speed times L.
- **A device keeps to its work cap closely, not exactly.** What its
  readings cannot foresee, a count's step and the velocity estimate's
  lag behind a stiff contact's push, overruns the cap by up to 70 µJ at
  a 5 kHz tick and 115 µJ at 1 kHz in the lab. The teleoperator's margin
  still dips, by 0.001 to 0.094 mJ in the lab and under QEMU (under 0.04
  in all but one run), from those overruns and from the books in flight:
  the tax of a rise is booked up to four samples before the input that
  repays it, and that of a rise the device holds back is spent in the
  host's books until the device gives it back, a round trip later (A
  device in the loop, above). The prediction assumes a rigid shaft; a
  compliant coupling between the motor and the knob is not modelled.
- **Only the slave takes a relay.** One relay is enough to cross both
  NATs, but a slave without one cannot be reached through a symmetric
  NAT, and a master cannot offer a relay of its own: a master on a network
  that lets no UDP out cannot be reached at all. The credentials are
  long-term ones; for coturn's time-limited credentials (its
  `use-auth-secret`), compute the user name and password outside and pass
  them in.
- **Over TCP or TLS, only the slave's leg to the relay changes.** The
  rendezvous server is reached over UDP, so a network that lets no UDP out
  at all, not even to it, keeps the session from starting. There is no
  DTLS (`turns:` with `?transport=udp`) and no way through an HTTP proxy:
  the slave connects to the TURN server itself. A lost segment holds back
  what follows it (Through a relay over TCP or TLS, above), and a relayed
  datagram's arrival time is when it was read from the connection, not
  when the kernel took it, so the one-way delays and the clock skew
  measured through such a relay include the time each waited to be read.
  A reply that begins like ChannelData, as an HTTP response does, is taken
  for one until the connection closes or 5 s pass.
- **A relayed session checks for a direct path once a second.** A path
  open one way only costs a detour at every check: the end that hears it
  follows it for half a second, the other end hearing from it only
  through the copies along the relay, which carries everything twice for
  a second. A
  direct round trip longer than half the interval (500 ms) outlasts
  `stay_off`, and the ends go there and back before they settle. Nothing
  moves a session from a direct path back to a relay that has become
  faster. Between symmetric NATs no check succeeds, as at the start. Every
  master the rendezvous server announces gets a permission on the relay,
  and keeps it until the next allocation.
- **A move during the handshake is not followed.** The rejoin starts with
  the session, and a relayed session starts more than a second after the
  master (it waits `relay_delay`): the lab's `--turn --rebind-at 1` loses
  it.
- **The hop limit is measured only on Linux, over IPv4, and once.**
  Elsewhere, and behind a NAT that passes no errors back, it is 3, which
  is too large for two ends a few hops apart (the lab's one-router case).
  It is measured at start and not again when an end moves. A carrier-grade
  NAT that answers from a pool address other than the one the server sees
  is taken for a router past it, so the limit stops inside it.
- **A move costs about 100 ms plus a round trip to the server** (When an end
  moves, above), about 200 ms through a relay, during which the ends hear
  nothing from each other, as in any outage. Behind a NAT that
  accepts datagrams addressed to itself, the end that did not move cannot
  take the moved one back at all; its session ends.
- **Host candidates are taken once.** An end that changes network keeps
  announcing the LAN addresses it started with; only the address the
  server sees follows it.
- **`measure`'s one-way delays assume equal minimum delays in both
  directions**, which no measurement between unsynchronized clocks can
  avoid. `pair` needs no such assumption, but it measures only between two
  networks one host can join at once, over IPv4, and finds no way through
  a NAT that maps by destination facing one that filters by port. It has
  been checked in the lab only, not yet on two real networks.
- **The replay is checked on a virtual machine only**, where its timing is
  that of the machine's timers (Replaying a path on Linux, above); bare
  metal should do better and has not been measured. It replays UDP over
  IPv4 and IPv6 on Ethernet-framed interfaces; with a port filter, IPv4
  fragments after the first pass undelayed. The two directions loop with
  their own traces' lengths, so they drift apart over many loops.
- Sequence numbers do not wrap (49 days at 1 kHz).

## Next

- Traces of real paths (fibre, 4G and 5G in Japan) recorded with
  `tuushin-probe`, published so that any simulator or testbed can replay
  them (`tuushin-netreplay` on Linux), alongside public sets such as the
  Starlink one above.
- Host candidates taken again when an end moves; the rendezvous server
  over TCP or TLS, and a relay for the master, for networks that let no
  UDP out; then QUIC datagrams as a transport.
- The firmware on a chip and a real knob, measured: its timing, its
  encoder's noise, its windings' L / R against an LCR meter, and the work
  cap under a real hand; then the windings' temperature followed, to
  narrow the current's lag.
- Deadbands that adapt to the measured delay; more degrees of freedom.

## References

The code cites its sources where it uses them. The main ones:

- J.-H. Ryu and C. Preusche, "Stable bilateral control of teleoperators under
  time-varying communication delay: time domain passivity approach", ICRA 2007.
- M. Panzirsch, J.-H. Ryu and M. Ferre, "Reducing the conservatism of the time
  domain passivity approach through consideration of energy reflection in
  delayed coupled network systems", Mechatronics 58, 2019.
- J.-H. Ryu, Y. S. Kim and B. Hannaford, "Sampled- and continuous-time
  passivity and stability of virtual environments", IEEE T-RA 20(4), 2004.
- J. J. Abbott and A. M. Okamura, "Effects of position quantization and
  sampling rate on virtual-wall passivity", IEEE T-RO 21(5), 2005, for why
  coarse encoders break the books; P. R. Belanger, P. Dobrovolny,
  A. Helmy and X. Zhang, "Estimation of angular velocity and acceleration
  from shaft-encoder measurements", IJRR 17(11), 1998, for the Kalman
  filter on encoder readings; R. K. Mehra, "On the identification of
  variances and adaptive Kalman filtering", IEEE T-AC 15(2), 1970, for
  covariance matching, which tunes the filter and sizes the model error
  margin.
- J. E. Colgate and G. G. Schenkel, "Passivity of a class of sampled-data
  systems: application to haptic interfaces", Journal of Robotic Systems
  14(1), 1997, for the damping a sampled spring needs, which sets the
  coupling a device can take.
- ODrive's motor calibration, SimpleFOC's `characteriseMotor`
  (docs.simplefoc.com/motor_characterisation) and TI's InstaSPIN-FOC
  and InstaSPIN-MOTION User's Guide (SPRUHJ1), for the ways of measuring
  L and R that measure the current; the ODrive forum's "Gimbal motors:
  inductance out of range" and the SimpleFOC forum's "Measuring gimbal
  resistance, inductance, and KV using SimpleFOC", for what is known of
  a gimbal motor's windings.
- R. D. Lorenz and K. W. Van Patten, "High-resolution velocity estimation
  for all-digital, AC servo drives", IEEE T-IA 27(4), 1991, for the
  velocity observer with the torque fed forward that the work cap
  predicts with.
- P. Hinterseer, S. Hirche, S. Chaudhuri, E. Steinbach and M. Buss,
  "Perception-based data reduction and transmission of haptic data in
  telepresence and teleaction systems", IEEE Trans. Signal Processing 56(2), 2008.
- M. Franken, S. Stramigioli, S. Misra, C. Secchi and A. Macchelli, "Bilateral
  telemanipulation with time delays: a two-layer approach combining passivity
  and transparency", IEEE T-RO 27(4), 2011.
- H. Schulzrinne et al., RFC 3550 (RTP), for the jitter estimate and the echo
  fields; S. B. Moon, P. Skelly and D. Towsley, and V. Paxson, for clock-skew
  estimation from one-way delays.
- T. Perrin, "The Noise Protocol Framework", revision 34, 2018, for the
  handshake; checked against the cacophony test vectors.
- J. A. Donenfeld, "WireGuard: next generation kernel network tunnel", NDSS
  2017, for the timestamped first message (section 5.1) and the counter
  nonce with a replay window.
- B. Ford, P. Srisuresh and D. Kegel, "Peer-to-peer communication across
  network address translators", USENIX ATC 2005, and RFC 5128, for UDP hole
  punching; A. Keranen, C. Holmberg and J. Rosenberg, RFC 8445 (ICE), for
  candidates and connectivity checks; F. Audet and C. Jennings, RFC 4787,
  for the terms of NAT behaviour.
- T. Reddy, A. Johnston, P. Matthews and J. Rosenberg, RFC 8656 (TURN), and
  M. Petit-Huguenin, G. Salgueiro, J. Rosenberg, D. Wing, R. Mahy and
  P. Matthews, RFC 8489 (STUN), for the relay and its long-term
  credentials; R. Denis-Courmont, RFC 5769, for STUN's test vectors;
  P. Patil, T. Reddy, G. Salgueiro and M. Petit-Huguenin, RFC 7443, for
  the ALPN label of TURN over TLS.
- S. Becker, T. Pfandzelter, N. Japke, D. Bermbach and O. Kao, "Network
  emulation in large-scale virtual edge testbeds: a note of caution and
  the way forward", TDIS 2022, arXiv:2208.05862, and W. Tian, Y. Li,
  J. Zhao, S. Wu and J. Pan, "An eBPF-based trace-driven emulation method
  for satellite networks", IEEE Networking Letters, 2024,
  arXiv:2408.15581, for replaying a path with departure times and fq.
- J. Garcia, S. Sundberg and A. Brunstrom, "A detailed characterization of
  Starlink one-way delay", ACM LEONET 2025, and its dataset,
  doi:10.5281/zenodo.16275284 (CC BY 4.0), for the Starlink traces.
- D. Trautwein, C. Ihle, M. Schubotz and B. Gipp, "Challenging tribal
  knowledge: large scale measurement campaign on decentralized NAT
  traversal", arXiv:2510.27500, 2025, for priming with low-TTL datagrams
  (section 7.2).
- S. Guha and P. Francis, "Characterization and measurement of TCP
  traversal through NATs and firewalls", IMC 2005, for the low-TTL methods
  and their open question, the hop limit; P. Srisuresh, B. Ford,
  S. Sivakumar and S. Guha, RFC 5508, for what NATs do with ICMP errors;
  J. Weil et al., RFC 6598, for the shared address space, and
  S. Perreault et al., RFC 6888, for the carrier-grade NATs that use it.

## License

MIT, see `LICENSE`.

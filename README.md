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

**Status: early.** One degree of freedom, simulated devices, measured on
loopback and in a lab of Linux NATs so far. POSIX C++17, developed on macOS
and also tested on Linux (Ubuntu 24.04, GCC 13.3). MIT licensed.

## What is in it

| module | what it does |
|---|---|
| `control/` | Time Domain Passivity Approach (TDPA): passivity observers and controllers in the conventional two-channel form, and the variant that books energy reflection at a slave-side position coupling in one store (Panzirsch, Ryu and Ferre 2019, "Approach 3"). The passivity controllers predict the work a zero-order-held force will do on a light device instead of assuming constant velocity. |
| `codec/` | Perceptual deadband data reduction (Weber's law, Hinterseer et al. 2008). |
| `teleop/` | Master and slave controllers, the device interface (`Device1D`) with simulated devices, `HapticLink` (one end of a session over UDP), and real-time master and slave nodes. |
| `net/` | UDP sockets with kernel receive timestamps and hop-limited sends, clock-skew estimation from four-timestamp exchanges, channel models (fixed delay, jitter, Gilbert-Elliott loss, bottleneck queue, recorded traces), path measurement with probes and a reflector, and a rendezvous server and client for ends behind NATs (see NAT traversal below). |
| `wire/` | The datagram format: a 32-byte header whose echo fields let each end measure the round trip from ordinary traffic, as RTCP does. |
| `crypto/` | A Noise_NNpsk0 handshake and ChaCha20-Poly1305 on libsodium: a pre-shared key becomes fresh keys for every session, and every datagram is authenticated and encrypted (see Authentication below). |
| `sim/` | A deterministic 1-DoF teleoperation simulator built on the same controllers and devices. |
| `tools/` | `tuushin-sim`, `tuushin-teleop` (master, slave and rendezvous server as separate processes), `tuushin-probe` (measure a path, write replayable traces), `nat_lab.sh` (master and slave behind Linux NATs in network namespaces). |

## Build and test

```sh
cmake -S . -B build -G Ninja
cmake --build build
./build/tests/tuushin_tests          # all tests; pass a name to run a subset
```

Needs a C++17 compiler, CMake and libsodium (ISC licence):
`brew install libsodium`, or `apt install libsodium-dev pkg-config`.

## Quick start

Simulate a teleoperator touching a wall with 25 ms one-way delay:

```sh
./build/tools/tuushin-sim --passivity reflect --delay-ms 25
./build/tools/tuushin-sim --passivity channel --sweep 0,10,25,50,100
```

Run the master and the slave as separate processes over UDP (simulated
devices for now). On loopback, `--emulate-*` impairs each end's outgoing
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

Measure a real path and replay it:

```sh
# on a reachable host
./build/tools/tuushin-probe reflect --port 47000
# on the other end: 60 s of 1 kHz probes, both directions, skew-corrected
./build/tools/tuushin-probe measure HOST --out path
./build/tools/tuushin-sim --trace-fwd path.fwd.csv --trace-bwd path.bwd.csv
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
| 0 ms | 0 J | 1318 N/m / 0 J | 1470 N/m / 0 J |
| 5 ms | −19 J | 256–508 / −1.3e−4–0 J | 1358 / 0 J |
| 10 ms | −66 J | 97–107 / −1.2e−3 J | 1129 / 0 J |
| 25 ms | −410 J | 38–87 / −0.055–−0.044 J | 438 / −3.9e−5 J |
| 50 ms | −140 J | 53–106 / −0.24–−0.13 J | 202 / −3.7e−5 J |
| 100 ms | −461 J | 75–78 / −0.17 J | 99–102 / −6.4e−5 J |
| 200 ms | −322 J | 77 / −0.16 J | 87 / −9.3e−5 J |

The ranges are over twelve runs whose hand motion differs by one part in
10^12. The conventional scheme's numbers are chaotic: every sample its PC
cuts changes the command the slave integrates, so a rounding difference
grows into a different trajectory (at 50 ms the final position drift lands
anywhere between −4 and 14 mm). With energy reflection the PCs only scale
forces and the same runs agree to the digits shown.

Without passivity control the loop generates hundreds of joules and runs
into the force limit from 5 ms on. The conventional TDPA keeps each channel
direction passive (its channel margins stay above −2e−6 J), but not the
two-port between the devices, because the slave's coupling controller sits
outside it: energy the coupling spring stored and gives back looks generated
in the backward channel and is dissipated, the conservatism Panzirsch et al.
describe, and the displayed stiffness falls to tens of N/m. Booking the coupling inside one store
keeps several times the stiffness up to 100 ms. Its small residual margins
(under 0.2 mJ over 10 s) are not yet characterized.

### Real time, two processes on loopback

`tuushin-teleop`, same plant and controller as above, energy-reflection
scheme, 10 s sessions on macOS with the Mach time-constraint policy:

| | no emulated delay | 25 ms each way (emulated) | simulator, 25 ms |
|---|---|---|---|
| round trip, p50 / p99.9 | 0.041 / 0.107 ms | 50.04 / 50.11 ms | — |
| master tick lateness, p50 / max | 7 / 23 µs | 7 / 27 µs | — |
| samples lost | 0 | 0 | — |
| max master position | 0.0233 m | 0.0288 m | 0.0286 m |
| max displayed force | 5.017 N (simulator: 5.017 N) | 5.288 N | 5.398 N |
| master / slave PC active | 3 % / 72 % | 89 % / 75 % | 89 % / 75 % |
| min margin | 0 J | −3.6e−5 J | −3.9e−5 J |

With the same delay the real-time loop and the simulator agree to within
2 % in position, force and how often the passivity controllers act, and
their residual margins are of the same order. The one-way delays come from
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
| no watchdog | −8e−6 J | −0.019 J | −0.025 J | −0.006 J |
| watchdog, two periods | −5e−5 J | −6e−5 J | −4e−5 J | 0 J |

Random stalls: 30 per second at each end, 0.2 to 12 ms long, 200 runs; with
the watchdog the energy books, which are the more conservative of the two,
stay above −2e−5 J. In real time, with both nodes as threads of one process
without real-time priority on a laptop loaded with 10 busy processes (the
setup of the test `nodes_run_a_session_over_loopback`), 2 of 48 sessions
without the watchdog fell below −1 mJ (the worst to −6 mJ); with it the
worst of 48 sessions was −1e−5 J, and of 60 under 24 busy processes
−2e−5 J.

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
  bound on the true work for any reading error within one count.

And every port pays its own excess first: when its output has run past
what the other end granted, its next input covers that before any of it is
reported to the other end, which would otherwise spend it a second time.

Reflective scheme, 55 µm master and 1 µm slave encoders, 20 s over nine
hand motions (0.4 to 0.6 Hz, 3 to 5 cm; `tests/test_teleop1d.cpp` checks
four of them, `tuushin-sim --master-encoder 55e-6 --slave-encoder 1e-6`
runs the default one):

| one-way delay | worst true margin: resolution known | not known | exact encoders | force displayed: known | exact encoders |
|---|---|---|---|---|---|
| 10 ms | 0 J | −4.8e−4 J | 0 J | 32 % | 77 % |
| 50 ms | 0 J | −0.79 J | −1.4e−4 J | 12 % | 29 % |
| 100 ms | 0 J | −1.14 J | −1.3e−4 J | 4 % | 13 % |

*True margin* is minus the work the actuators really did, which the books
see only through the encoders. *Force displayed* is the force on the master
in contact as a share of the contact force one delay earlier.

Treating the readings as exact, the predictor differentiates quantization
noise and the teleoperator generates up to a joule. Knowing the
resolution, it generates nothing, also under the loop stalls above
(`tests/test_teleop_stall.cpp`). The price is transparency: less than half
the force reaches the operator, on the default motion at 50 ms the
displayed stiffness falls from 202 N/m with exact encoders to 88 N/m, and
the books, which pay the worst case of every rise of the force, end a few
millijoules below the true margin.

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
  only the master can use.

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
  addresses (for two ends on one LAN, whose router may not hairpin). There
  are no relayed candidates (TURN).
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
  them datagrams with a small hop limit (`--prime-hops`, 3 by default).
  They open its own NAT and expire on the way, before they reach the other
  NAT.
- **Ready.** Each end then tells the server it is ready, and sends
  datagrams that go the whole way (hellos, punches) only once the server
  has passed on the other end's ready, or after 2 s without it. The server
  forgets an end's ready when the other end moves or registers anew, and
  tells the end so.

Choosing `--prime-hops`: a datagram sent with a hop limit of t is
forwarded by the first t − 1 routers and dropped by the t-th. It has to
pass this end's NATs and must not reach the other end's, which takes a
datagram addressed to itself whatever hop limit it has left. 3 passes a
home router and the next hop (often a carrier-grade NAT) and stops before
the other end's NAT wherever that is more than 3 hops away, as it nearly
always is for two homes (traceroute shows the count). Two ends close
together on one network may need 2. With 0, or on a system that cannot
limit the hop count, nothing is primed: the master still waits for the
slave's ready, but the slave, which must then go first, punches at once.

`tools/nat_lab.sh` builds a master and a slave behind Linux NATs that
filter by address and port, a rendezvous server and a chain of routers
between the NATs, in network namespaces (in a privileged container on
other systems; the script says how). Three routers, 20 ms round trip
between the NATs; *time* is from the master's start to the end of a 2 s
session:

| scenario | result | time |
|---|---|---|
| cone NATs (endpoint-independent mapping) | connects | 2.15 s |
| the master starts 0.5 s before the slave | connects | 2.65 s |
| both behind one NAT | connects, over the LAN | 2.09 s |
| the slave 100 ms behind its NAT | connects | 2.76 s |
| the master 100 ms behind its NAT | connects | 2.97 s |
| both NATs accept datagrams to themselves | connects | 2.14 s |
| the slave 100 ms behind a NAT that accepts them | connects (failed without priming and ready) | 2.75 s |
| the master 100 ms behind a NAT that accepts them | connects (failed without priming and ready) | 2.98 s |
| both 100 ms behind NATs that accept them | connects | 3.58 s |
| the slave 100 ms behind a NAT that accepts them, `--prime-hops 0` | connects: ready alone orders it | 2.76 s |
| the master 100 ms behind a NAT that accepts them, `--prime-hops 0` | fails: the NAT moved the master to another port | — |
| the same, primed, with one router between the NATs | fails: the primes reach the other NAT, with one hop to live | — |
| the same with `--prime-hops 2` | connects | 2.96 s |
| a symmetric NAT (a new port per destination) at either end or both | fails | — |

The slave punches until its session starts, so with an end 100 ms behind
its NAT the master counts the last few punches as `foreign` datagrams (5 to
10 in the lab); they do no harm.

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
- The slave forgets the newest hello time when it restarts, so one recorded
  hello can be answered once more afterwards (still without starting a
  session). A master whose wall clock is set back is refused until it
  passes the newest time the slave has seen.
- **Coarse encoders cost transparency.** With a 55 µm master encoder the
  teleoperator stays passive, but at 50 ms less than half the force reaches
  the operator (Coarse encoders above). The tax is the worst case, and a
  master pressing an almost still handle into the wall pays it on every
  rise of the force that the filter's velocity noise allows, while the
  input it predicted mostly never comes.
- **Simulated devices only, one degree of freedom.** `teleop::Device1D` is
  the interface a real device will implement.
- **No relay.** Hole punching fails where a NAT gives every destination its
  own port (a symmetric NAT, at either end, in the lab), and there is no
  relayed path (TURN) to fall back to: the master reports that no
  authenticated reply came. In the IPFS network, Trautwein et al. found
  about 30 % of hole-punching attempts fail.
- **`--prime-hops` is a guess about the path.** Two ends a few hops apart
  need a smaller value, or the primes reach the other NAT (the lab's one-
  router case); nothing measures the hop count yet.
- **A session does not follow an end whose address changes.** If a NAT
  moves an end to another port mid-session, or an end changes network, its
  datagrams come from an address the other end ignores (counted as
  `foreign`); the slave returns to idle after a second of silence, and
  there is no reconnection.
- One-way delays assume equal minimum delays in both directions, which no
  measurement between unsynchronized clocks can avoid.
- Sequence numbers do not wrap (49 days at 1 kHz).

## Next

- Traces of real paths (fibre, 4G and 5G in Japan) recorded with
  `tuushin-probe`, published so that any simulator or testbed can replay them.
- A relay for ends behind symmetric NATs, a measured hop count for priming,
  and sessions that survive an address change; then QUIC datagrams as a
  transport.
- Deadbands that adapt to the measured delay; more degrees of freedom; a real
  device behind `Device1D`.

## References

The code cites its sources where it uses them. The main ones:

- J.-H. Ryu and C. Preusche, "Stable bilateral control of teleoperators under
  time-varying communication delay: time domain passivity approach", ICRA 2007.
- M. Panzirsch, J.-H. Ryu and M. Ferre, "Reducing the conservatism of the time
  domain passivity approach through consideration of energy reflection in
  delayed coupled network systems", Mechatronics 58, 2019.
- J.-H. Ryu, Y. S. Kim and B. Hannaford, "Sampled- and continuous-time
  passivity and stability of virtual environments", IEEE T-RA 20(4), 2004.
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
- D. Trautwein, C. Ihle, M. Schubotz and B. Gipp, "Challenging tribal
  knowledge: large scale measurement campaign on decentralized NAT
  traversal", arXiv:2510.27500, 2025, for priming with low-TTL datagrams
  (section 7.2).

## License

MIT, see `LICENSE`.

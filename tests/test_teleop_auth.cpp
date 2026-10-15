#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "test.hpp"
#include "tuushin/crypto/noise.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/teleop/link.hpp"
#include "tuushin/teleop/node.hpp"
#include "tuushin/wire/packet.hpp"

using namespace tuushin;
using namespace tuushin::teleop;

namespace {

net::UdpSocket bind_loopback() { return net::UdpSocket(*net::Endpoint::resolve("127.0.0.1", 0)); }

net::Endpoint peer_of(const net::UdpSocket& s) {
  return *net::Endpoint::resolve("127.0.0.1", s.local_endpoint().port());
}

struct KeyPair {
  crypto::SessionKeys init, resp;
};

KeyPair handshake(const crypto::Key& psk) {
  const std::uint8_t none = 0;
  crypto::NoiseNNpsk0 i(crypto::NoiseNNpsk0::Role::kInitiator, psk, nullptr, 0);
  crypto::NoiseNNpsk0 r(crypto::NoiseNNpsk0::Role::kResponder, psk, nullptr, 0);
  const auto m1 = i.write_message(&none, 0);
  r.read_message(m1.data(), m1.size());
  const auto m2 = r.write_message(&none, 0);
  i.read_message(m2.data(), m2.size());
  return KeyPair{i.split(), r.split()};
}

struct Datagram {
  std::vector<std::uint8_t> bytes;
  net::Endpoint from;
  Nanos rx = 0;
};

std::optional<Datagram> receive(net::UdpSocket& sock, Nanos timeout = kNanosPerSecond) {
  if (!sock.wait_readable(timeout)) return std::nullopt;
  Datagram d;
  d.bytes.resize(wire::kMaxDatagram + 1);
  const auto n = sock.recv_from(d.bytes.data(), d.bytes.size(), &d.from, &d.rx);
  if (!n) return std::nullopt;
  d.bytes.resize(*n);
  return d;
}

std::optional<HapticLink::Received> feed(HapticLink& link, const Datagram& d) {
  return link.on_datagram(d.bytes.data(), d.bytes.size(), d.from, d.rx);
}

// Rewrites the sequence number of a datagram in place (offset 8).
void set_seq(std::vector<std::uint8_t>& b, std::uint32_t seq) {
  for (int i = 0; i < 4; ++i) b[8 + i] = static_cast<std::uint8_t>(seq >> (8 * i));
}

}  // namespace

TEST_CASE(sealed_link_carries_samples_and_hides_them) {
  net::UdpSocket sa = bind_loopback(), sb = bind_loopback();
  const auto keys = handshake(crypto::random_key());
  HapticLink a(sa, 21, peer_of(sb), Passivity::kReflective);
  HapticLink b(sb, 21, peer_of(sa), Passivity::kReflective);
  a.set_keys(keys.init);
  b.set_keys(keys.resp);
  CHECK(a.sealed() && b.sealed());

  const double v = 0.123456789;
  a.send(monotonic_now(), Sample{v, -1.5, 0.003}, 0.002);
  const auto d = receive(sb);
  REQUIRE(d);
  const auto h = wire::decode_header(d->bytes.data(), d->bytes.size());
  REQUIRE(h);
  CHECK((h->flags & wire::kFlagSealed) != 0);
  CHECK(d->bytes.size() == wire::kHeaderSize + 4 + 8 * 4 + crypto::kTagSize);
  // The velocity's bytes do not appear in the clear.
  std::uint8_t pattern[8];
  std::memcpy(pattern, &v, 8);
  bool found = false;
  for (std::size_t i = 0; i + 8 <= d->bytes.size(); ++i) {
    if (std::memcmp(d->bytes.data() + i, pattern, 8) == 0) found = true;
  }
  CHECK(!found);
  const auto r = feed(b, *d);
  REQUIRE(r);
  CHECK(r->sample.value == v && r->sample.aux == -1.5 && r->sample.energy == 0.003);
  CHECK(r->monitor == 0.002);

  // The answer is sealed the other way, and its echo gives a round trip.
  b.send(monotonic_now(), Sample{4.0, 0.0, 0.0}, 0.0, true);
  const auto d2 = receive(sa);
  REQUIRE(d2);
  const auto r2 = feed(a, *d2);
  REQUIRE(r2);
  CHECK(r2->final && r2->sample.value == 4.0);
  CHECK(a.rtts().size() == 1);
  CHECK(a.counters().unauthentic == 0 && b.counters().unauthentic == 0);
}

TEST_CASE(sealed_link_rejects_unsealed_altered_and_foreign_keyed_datagrams) {
  net::UdpSocket sa = bind_loopback(), sb = bind_loopback();
  const crypto::Key psk = crypto::random_key();
  const auto keys = handshake(psk);
  HapticLink b(sb, 33, peer_of(sa), Passivity::kChannel);
  b.set_keys(keys.resp);

  // From the right address and session, but not sealed.
  HapticLink open_link(sa, 33, peer_of(sb), Passivity::kChannel);
  open_link.send(monotonic_now(), Sample{1.0, 0.0, 0.0}, 0.0);
  auto d = receive(sb);
  REQUIRE(d);
  CHECK(!feed(b, *d));
  CHECK(b.counters().unauthentic == 1);

  // Sealed, but with the keys of another handshake under the same psk.
  HapticLink other(sa, 33, peer_of(sb), Passivity::kChannel);
  other.set_keys(handshake(psk).init);
  other.send(monotonic_now(), Sample{1.0, 0.0, 0.0}, 0.0);
  d = receive(sb);
  REQUIRE(d);
  CHECK(!feed(b, *d));
  CHECK(b.counters().unauthentic == 2);

  // The genuine sender. Every altered byte that still parses as this
  // session's datagram fails authentication: header fields are associated
  // data, the payload is under the tag.
  HapticLink a(sa, 33, peer_of(sb), Passivity::kChannel);
  a.set_keys(keys.init);
  a.send(monotonic_now(), Sample{2.0, 0.5, 0.25}, 0.1);
  d = receive(sb);
  REQUIRE(d);
  int rejected = 0;
  for (std::size_t i = 0; i < d->bytes.size(); ++i) {
    Datagram bad = *d;
    bad.bytes[i] ^= 0x01;
    if (!feed(b, bad)) ++rejected;
  }
  CHECK(rejected == static_cast<int>(d->bytes.size()));
  CHECK(b.counters().received == 0);  // none of them reached the window
  // The genuine datagram still gets through, once.
  CHECK(feed(b, *d).has_value());
  CHECK(!feed(b, *d).has_value());
  CHECK(b.counters().received == 1);
  CHECK(b.counters().duplicates == 1);
}

TEST_CASE(sealed_link_follows_its_peer_to_a_new_address) {
  // The master sends to `mid`, and the test forwards each datagram from one
  // of three sockets: a NAT that moves the master to another port mid-
  // session, as far as the slave can tell.
  net::UdpSocket sa = bind_loopback(), mid = bind_loopback(), sb = bind_loopback();
  net::UdpSocket n1 = bind_loopback(), n2 = bind_loopback(), n3 = bind_loopback();
  const auto keys = handshake(crypto::random_key());
  HapticLink a(sa, 55, peer_of(mid), Passivity::kReflective);
  HapticLink b(sb, 55, peer_of(n1), Passivity::kReflective);
  a.set_keys(keys.init);
  b.set_keys(keys.resp);
  std::vector<Datagram> sent;  // the master's datagrams, in order
  auto next = [&] {
    a.send(monotonic_now(), Sample{static_cast<double>(sent.size()), 0.0, 0.0}, 0.0);
    const auto d = receive(mid);
    REQUIRE(d);
    sent.push_back(*d);
  };
  auto via = [&](net::UdpSocket& nat, const Datagram& d) -> std::optional<HapticLink::Received> {
    CHECK(nat.send_to(peer_of(sb), d.bytes.data(), d.bytes.size()));
    const auto got = receive(sb);
    CHECK(got.has_value());
    if (!got) return std::nullopt;
    return feed(b, *got);
  };
  for (int k = 0; k < 6; ++k) next();

  CHECK(via(n1, sent[0]).has_value());
  CHECK(b.peer() == peer_of(n1));
  // The NAT moves the master: the newest authentic datagram moves the link,
  // and the slave's next sample goes to the new address.
  CHECK(via(n2, sent[1]).has_value());
  CHECK(b.peer() == peer_of(n2));
  CHECK(b.counters().moved == 1);
  b.send(monotonic_now(), Sample{9.0, 0.0, 0.0}, 0.0);
  CHECK(receive(n2).has_value());
  CHECK(!n1.wait_readable(20 * kNanosPerMilli));

  // A replay from the old address or a third one moves nothing.
  CHECK(!via(n1, sent[1]).has_value());
  CHECK(!via(n3, sent[0]).has_value());
  CHECK(b.peer() == peer_of(n2));
  // A late datagram still in flight from the old address is a sample, but
  // the link stays where the newest came from.
  CHECK(via(n2, sent[3]).has_value());
  const auto late = via(n1, sent[2]);
  REQUIRE(late);
  CHECK(late->verdict == SeqWindow::Verdict::kLate);
  CHECK(b.peer() == peer_of(n2));
  // An altered datagram from elsewhere is someone else's: no sample, no
  // move, counted as foreign rather than as an unauthentic one of the peer.
  Datagram bad = sent[4];
  bad.bytes.back() ^= 0x01;
  const auto foreign = b.counters().foreign;
  CHECK(!via(n3, bad).has_value());
  CHECK(b.counters().foreign == foreign + 1);
  CHECK(b.counters().unauthentic == 0);
  CHECK(b.peer() == peer_of(n2));
  CHECK(b.counters().moved == 1);
  // And the genuine one gets through, from there too.
  CHECK(via(n3, sent[4]).has_value());
  CHECK(b.peer() == peer_of(n3));
  CHECK(b.counters().moved == 2);
}

TEST_CASE(open_link_stays_with_its_address) {
  net::UdpSocket sa = bind_loopback(), sb = bind_loopback(), other = bind_loopback();
  HapticLink a(sa, 56, peer_of(sb), Passivity::kReflective);
  HapticLink b(sb, 56, peer_of(sa), Passivity::kReflective);
  a.send(monotonic_now(), Sample{1.0, 0.0, 0.0}, 0.0);
  auto d = receive(sb);
  REQUIRE(d);
  // The same datagram from another address: anyone could have sent it.
  REQUIRE(other.send_to(peer_of(sb), d->bytes.data(), d->bytes.size()));
  const auto copy = receive(sb);
  REQUIRE(copy);
  CHECK(!feed(b, *copy).has_value());
  CHECK(b.counters().foreign == 1);
  CHECK(b.peer() == peer_of(sa));
  CHECK(feed(b, *d).has_value());
  CHECK(b.counters().moved == 0);
}

TEST_CASE(forged_high_seq_stalls_an_open_link_but_not_a_sealed_one) {
  // The attack: one datagram claiming a sequence number far ahead. An open
  // link takes it as the newest and from then on treats every genuine
  // sample as stale; a sealed link drops it because it does not
  // authenticate.
  for (const bool sealed : {false, true}) {
    net::UdpSocket sa = bind_loopback(), sb = bind_loopback();
    const auto keys = handshake(crypto::random_key());
    HapticLink a(sa, 44, peer_of(sb), Passivity::kReflective);
    HapticLink b(sb, 44, peer_of(sa), Passivity::kReflective);
    if (sealed) {
      a.set_keys(keys.init);
      b.set_keys(keys.resp);
    }
    a.send(monotonic_now(), Sample{1.0, 0.0, 0.0}, 0.0);
    auto d = receive(sb);
    REQUIRE(d);
    REQUIRE(feed(b, *d));
    // The forgery: a copy of a genuine datagram with seq 2^31, as anyone
    // on the path could make (the address can be spoofed as well).
    Datagram forged = *d;
    set_seq(forged.bytes, 0x80000000u);
    const bool forged_accepted = feed(b, forged).has_value();
    CHECK(forged_accepted == !sealed);
    // The next genuine samples.
    int delivered = 0;
    for (int k = 0; k < 5; ++k) {
      a.send(monotonic_now(), Sample{2.0 + k, 0.0, 0.0}, 0.0);
      d = receive(sb);
      REQUIRE(d);
      if (feed(b, *d)) ++delivered;
    }
    if (sealed) {
      CHECK(delivered == 5);
      CHECK(b.counters().unauthentic == 1);
    } else {
      CHECK(delivered == 0);  // the vulnerability the keys remove
    }
  }
}

namespace {

struct NodePair {
  MasterNodeReport master;
  std::vector<SlaveSessionReport> slave;
  std::vector<std::string> refusals;
};

// A short session over loopback between a master and a slave with the given
// keys. `before` runs once the slave is up, before the master starts.
// `via`, if set, is where the master sends instead of the slave's socket.
// `tweak`, if set, changes the slave's configuration.
template <class Before>
NodePair run_pair(std::optional<crypto::Key> master_psk, std::optional<crypto::Key> slave_psk,
                  Nanos duration, Nanos handshake_timeout, net::UdpSocket& slave_sock,
                  Before&& before, const net::Endpoint* via = nullptr,
                  void (*tweak)(SlaveNodeConfig&) = nullptr) {
  net::UdpSocket master_sock = bind_loopback();
  const net::Endpoint slave_ep = via ? *via : peer_of(slave_sock);
  const Nanos period = kNanosPerMilli;
  WallParams wall;
  SimulatedSlave slave_dev(1.0, 2.0, wall, period, 10, monotonic_now(), 0.0);
  slave_dev.set_watchdog(2 * period);
  SlaveNodeConfig scfg;
  scfg.control.dt = period;
  scfg.control.hold_after = 150 * kNanosPerMilli;
  scfg.once = true;
  scfg.psk = slave_psk;
  scfg.handshake_timeout = handshake_timeout;
  if (tweak) tweak(scfg);
  NodePair out;
  std::mutex mu;
  SlaveNodeHooks hooks;
  hooks.on_refused = [&](const std::string& why) {
    std::lock_guard<std::mutex> lock(mu);
    out.refusals.push_back(why);
  };
  std::atomic<bool> stop{false};
  std::thread slave([&] { out.slave = run_slave_node(slave_sock, slave_dev, scfg, hooks, &stop); });
  before();

  OperatorParams op;
  SimulatedMaster master_dev(0.2, 0.5, op, period, 10, monotonic_now());
  master_dev.set_watchdog(2 * period);
  MasterNodeConfig mcfg;
  mcfg.control.dt = period;
  mcfg.control.model_mass = 0.2;
  mcfg.duration = duration;
  mcfg.psk = master_psk;
  mcfg.handshake_timeout = handshake_timeout;
  mcfg.hello_interval = 50 * kNanosPerMilli;
  out.master = run_master_node(master_sock, slave_ep, master_dev, mcfg);
  // Give a refusal on the slave's side time to be reported.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  stop.store(true);
  slave.join();
  return out;
}

bool any_contains(const std::vector<std::string>& v, const char* what) {
  for (const auto& s : v) {
    if (s.find(what) != std::string::npos) return true;
  }
  return false;
}

// A hello as a master with `psk` would send it, with the given payload.
std::vector<std::uint8_t> make_hello(const crypto::Key& psk, std::uint32_t session,
                                     const std::vector<std::uint8_t>& payload) {
  const auto prologue = handshake_prologue(session, Passivity::kReflective);
  crypto::NoiseNNpsk0 hs(crypto::NoiseNNpsk0::Role::kInitiator, psk, prologue.data(),
                         prologue.size());
  const auto msg1 = hs.write_message(payload.data(), payload.size());
  wire::Header h;
  h.type = wire::PacketType::kHello;
  h.session = session;
  h.seq = 1;
  h.flags = static_cast<std::uint16_t>(static_cast<unsigned>(Passivity::kReflective)
                                       << wire::kSchemeShift);
  std::vector<std::uint8_t> out;
  wire::encode_raw(h, msg1.data(), msg1.size(), out);
  return out;
}

bool is_reply(const std::optional<Datagram>& d) {
  if (!d) return false;
  const auto h = wire::decode_header(d->bytes.data(), d->bytes.size());
  return h && h->type == wire::PacketType::kHelloReply;
}

}  // namespace

TEST_CASE(nodes_authenticate_and_run_a_sealed_session) {
  net::UdpSocket slave_sock = bind_loopback();
  const crypto::Key psk = crypto::random_key();
  const auto r = run_pair(psk, psk, 600 * kNanosPerMilli, kNanosPerSecond, slave_sock, [] {});
  CHECK(r.master.authenticated);
  CHECK(!r.master.handshake_failed);
  CHECK(r.master.slave_answered && r.master.slave_final);
  CHECK(r.master.link.received > 400);
  CHECK(r.master.link.unauthentic == 0);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].authenticated);
  CHECK(r.slave[0].session == r.master.session);
  CHECK(r.slave[0].ended_by_final);
  CHECK(r.slave[0].link.received > 400);
  CHECK(r.slave[0].link.unauthentic == 0);
  CHECK(r.refusals.empty());
  CHECK(r.master.max_position > 0.002);
}

TEST_CASE(slave_reporting_contact_force_refuses_energy_reflection) {
  // Energy reflection needs the coupling force (sim::SlaveParams). A slave
  // that adopts the master's scheme refuses such a session; one that would
  // run it itself does not start.
  net::UdpSocket slave_sock = bind_loopback();
  const crypto::Key psk = crypto::random_key();
  const auto r = run_pair(psk, psk, 300 * kNanosPerMilli, 300 * kNanosPerMilli, slave_sock, [] {},
                          nullptr, [](SlaveNodeConfig& c) {
                            c.control.passivity = Passivity::kChannel;
                            c.control.report_contact_force = true;
                            c.adopt_scheme = true;
                          });
  CHECK(!r.master.authenticated && !r.master.slave_answered);
  CHECK(r.slave.empty());
  CHECK(any_contains(r.refusals, "reports the contact force"));

  SimulatedSlave slave_dev(1.0, 2.0, WallParams{}, kNanosPerMilli, 10, monotonic_now(), 0.0);
  SlaveNodeConfig scfg;
  scfg.control.passivity = Passivity::kReflective;
  scfg.control.report_contact_force = true;
  bool threw = false;
  try {
    run_slave_node(slave_sock, slave_dev, scfg);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE(nodes_with_different_keys_never_start_a_session) {
  net::UdpSocket slave_sock = bind_loopback();
  const auto r = run_pair(crypto::random_key(), crypto::random_key(), 300 * kNanosPerMilli,
                          300 * kNanosPerMilli, slave_sock, [] {});
  CHECK(r.master.handshake_failed);
  CHECK(!r.master.authenticated && !r.master.slave_answered);
  CHECK(r.master.ticks == 0);
  CHECK(r.slave.empty());
  CHECK(any_contains(r.refusals, "does not authenticate"));
}

TEST_CASE(nodes_refuse_a_key_on_one_end_only) {
  {
    // A master with a key at a slave without one: the slave cannot answer
    // the hello, and says why.
    net::UdpSocket slave_sock = bind_loopback();
    const auto r = run_pair(crypto::random_key(), std::nullopt, 300 * kNanosPerMilli,
                            300 * kNanosPerMilli, slave_sock, [] {});
    CHECK(r.master.handshake_failed);
    CHECK(r.slave.empty());
    CHECK(any_contains(r.refusals, "wants authentication"));
  }
  {
    // A master without a key at a slave with one: its samples are refused.
    net::UdpSocket slave_sock = bind_loopback();
    const auto r = run_pair(std::nullopt, crypto::random_key(), 300 * kNanosPerMilli,
                            300 * kNanosPerMilli, slave_sock, [] {});
    CHECK(!r.master.handshake_failed);
    CHECK(!r.master.slave_answered);
    CHECK(r.slave.empty());
    CHECK(any_contains(r.refusals, "unauthenticated session"));
  }
}

TEST_CASE(replayed_hello_does_not_start_a_session) {
  // An attacker who recorded a genuine hello sends it again from its own
  // address, gets an answer it cannot use, and follows up with samples it
  // cannot seal. The slave (serving one session) must still be free for the
  // real master afterwards.
  net::UdpSocket slave_sock = bind_loopback();
  const crypto::Key psk = crypto::random_key();
  bool answered = false;
  bool unsealed_answered = false;
  const auto r = run_pair(psk, psk, 300 * kNanosPerMilli, kNanosPerSecond, slave_sock, [&] {
    net::UdpSocket attacker = bind_loopback();
    const net::Endpoint slave_ep = peer_of(slave_sock);
    // The "recording": a hello made by someone who knew the psk, older than
    // anything the master will send, but the slave has just started and has
    // not seen a newer one.
    const std::uint32_t session = 0x5eed;
    const auto out = make_hello(psk, session, hello_payload(1000));
    attacker.send_to(slave_ep, out.data(), out.size());
    answered = is_reply(receive(attacker));
    // Samples for that session: one unsealed, one "sealed" without the keys,
    // which only the recorded master's ephemeral key could have given.
    HapticLink open_link(attacker, session, slave_ep, Passivity::kReflective);
    open_link.send(monotonic_now(), Sample{5.0, 0.0, 0.0}, 0.0);
    HapticLink guess(attacker, session, slave_ep, Passivity::kReflective);
    guess.set_keys(handshake(psk).init);
    for (int k = 0; k < 5; ++k) guess.send(monotonic_now(), Sample{5.0, 0.0, 0.0}, 0.0);
    unsealed_answered = receive(attacker, 100 * kNanosPerMilli).has_value();
  });
  CHECK(answered);             // the slave cannot tell a replayed hello
  CHECK(!unsealed_answered);   // but no session started for it
  CHECK(r.master.authenticated);
  CHECK(r.master.slave_answered);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].session == r.master.session);
  CHECK(r.slave[0].authenticated);
}

TEST_CASE(hellos_must_be_newer_and_copies_share_one_reply) {
  net::UdpSocket slave_sock = bind_loopback();
  const crypto::Key psk = crypto::random_key();
  bool copy_same = false, older = true, equal = true, newer = false, timeless = true;
  const auto r = run_pair(psk, psk, 300 * kNanosPerMilli, kNanosPerSecond, slave_sock, [&] {
    net::UdpSocket a = bind_loopback(), b = bind_loopback();
    const net::Endpoint slave_ep = peer_of(slave_sock);
    const Nanos quiet = 100 * kNanosPerMilli;
    // A hello, and a copy of it from elsewhere: the copy gets the very same
    // reply (the same ephemeral key, so no second key exchange was made).
    const auto h1 = make_hello(psk, 0x101, hello_payload(2000));
    a.send_to(slave_ep, h1.data(), h1.size());
    const auto r1 = receive(a);
    b.send_to(slave_ep, h1.data(), h1.size());
    const auto r2 = receive(b);
    copy_same = is_reply(r1) && is_reply(r2) && r1->bytes == r2->bytes;
    // Fresh hellos at an older and at the same time are not answered, a
    // newer one is.
    const auto h2 = make_hello(psk, 0x102, hello_payload(1000));
    a.send_to(slave_ep, h2.data(), h2.size());
    older = is_reply(receive(a, quiet));
    const auto h3 = make_hello(psk, 0x103, hello_payload(2000));
    a.send_to(slave_ep, h3.data(), h3.size());
    equal = is_reply(receive(a, quiet));
    const auto h4 = make_hello(psk, 0x104, hello_payload(3000));
    a.send_to(slave_ep, h4.data(), h4.size());
    newer = is_reply(receive(a));
    // A hello that authenticates but carries no time is refused.
    const auto h5 = make_hello(psk, 0x105, {});
    a.send_to(slave_ep, h5.data(), h5.size());
    timeless = is_reply(receive(a, quiet));
  });
  CHECK(copy_same);
  CHECK(!older);
  CHECK(!equal);
  CHECK(newer);
  CHECK(!timeless);
  CHECK(any_contains(r.refusals, "not newer"));
  CHECK(any_contains(r.refusals, "carries no time"));
  // The master's clock is far ahead of these times, so its session runs.
  CHECK(r.master.authenticated);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].authenticated);
}

TEST_CASE(a_copied_hello_ahead_of_the_master_does_not_take_its_place) {
  // An attacker on the path copies the master's hello and gets copies to
  // the slave first, from 20 addresses. Were a hello tied to the address it
  // came from, the copies would take the slots and the genuine hello would
  // then be refused as not newer; the session must run regardless.
  net::UdpSocket slave_sock = bind_loopback();
  net::UdpSocket relay = bind_loopback();
  const net::Endpoint slave_ep = peer_of(slave_sock), relay_ep = peer_of(relay);
  std::vector<net::UdpSocket> attackers;
  for (int i = 0; i < 20; ++i) attackers.push_back(bind_loopback());
  std::atomic<bool> stop{false};
  std::atomic<int> copies_answered{0};
  std::thread tap([&] {
    std::optional<net::Endpoint> master;
    bool copied = false;
    std::vector<std::uint8_t> buf(wire::kMaxDatagram + 1);
    while (!stop.load()) {
      relay.wait_readable(10 * kNanosPerMilli);
      for (;;) {
        net::Endpoint from;
        const auto n = relay.recv_from(buf.data(), buf.size(), &from);
        if (!n) break;
        if (from == slave_ep) {
          if (master) relay.send_to(*master, buf.data(), *n);
          continue;
        }
        master = from;
        const auto h = wire::decode_header(buf.data(), *n);
        if (h && h->type == wire::PacketType::kHello && !copied) {
          copied = true;
          for (auto& a : attackers) a.send_to(slave_ep, buf.data(), *n);
          // Let the copies arrive first.
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
          for (auto& a : attackers) copies_answered += is_reply(receive(a, kNanosPerMilli));
        }
        relay.send_to(slave_ep, buf.data(), *n);
      }
    }
  });
  const crypto::Key psk = crypto::random_key();
  const auto r =
      run_pair(psk, psk, 400 * kNanosPerMilli, kNanosPerSecond, slave_sock, [] {}, &relay_ep);
  stop.store(true);
  tap.join();
  CHECK(copies_answered.load() == 20);  // the slave cannot tell a copy
  CHECK(r.master.authenticated);
  CHECK(!r.master.handshake_failed);
  CHECK(r.master.slave_answered);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].authenticated);
  CHECK(r.slave[0].session == r.master.session);
  CHECK(r.slave[0].master == relay_ep);
}

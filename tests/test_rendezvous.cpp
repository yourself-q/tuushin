#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "test.hpp"
#include "tuushin/crypto/noise.hpp"
#include "tuushin/net/rendezvous.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/teleop/node.hpp"

using namespace tuushin;
using namespace tuushin::net;

namespace {

Endpoint ep(const char* host, std::uint16_t port) { return *Endpoint::resolve(host, port); }

RendezvousId id_of(std::uint8_t b) {
  RendezvousId id{};
  id.fill(b);
  return id;
}

RendezvousMessage registration(const RendezvousId& id, RendezvousRole role, std::uint64_t txid,
                               std::vector<Endpoint> host = {}) {
  RendezvousMessage m;
  m.type = RendezvousMessage::Type::kRegister;
  m.role = role;
  m.id = id;
  m.txid = txid;
  m.candidates = std::move(host);
  return m;
}

std::vector<RendezvousServer::Out> feed(RendezvousServer& s, const RendezvousMessage& m,
                                        const Endpoint& from, Nanos now) {
  std::vector<RendezvousServer::Out> out;
  const auto bytes = encode_rendezvous(m);
  s.on_datagram(bytes.data(), bytes.size(), from, now, out);
  return out;
}

RendezvousMessage decoded(const RendezvousServer::Out& o) {
  return *decode_rendezvous(o.bytes.data(), o.bytes.size());
}

}  // namespace

TEST_CASE(rendezvous_messages_round_trip_and_reject_malformed) {
  RendezvousMessage m;
  m.type = RendezvousMessage::Type::kPeer;
  m.role = RendezvousRole::kSlave;
  m.id = id_of(0xa5);
  m.txid = 0x0102030405060708ull;
  m.observed = ep("203.0.113.7", 40000);
  m.candidates = {ep("198.51.100.2", 1), ep("2001:db8::1", 65535), ep("192.168.1.20", 47100)};
  const auto b = encode_rendezvous(m);
  CHECK(b.size() == kRendezvousMessageSize);
  const auto d = decode_rendezvous(b.data(), b.size());
  REQUIRE(d);
  CHECK(d->type == m.type && d->role == m.role && d->id == m.id && d->txid == m.txid);
  CHECK(d->observed == m.observed);
  CHECK(d->candidates == m.candidates);

  // At most kRendezvousMaxCandidates are carried.
  RendezvousMessage many = m;
  for (int i = 0; i < 12; ++i) many.candidates.push_back(ep("10.0.0.1", 1000 + i));
  CHECK(decode_rendezvous(encode_rendezvous(many).data(), kRendezvousMessageSize)
            ->candidates.size() == kRendezvousMaxCandidates);

  auto mutated = [&](std::size_t at, std::uint8_t v) {
    auto x = b;
    x[at] = v;
    return decode_rendezvous(x.data(), x.size()).has_value();
  };
  CHECK(!decode_rendezvous(b.data(), b.size() - 1));
  CHECK(!mutated(0, 'X'));            // magic
  CHECK(!mutated(4, 2));              // version
  CHECK(!mutated(5, 0) && !mutated(5, 4));  // type
  CHECK(!mutated(6, 3));              // role
  CHECK(!mutated(7, 9));              // count beyond the slots
  CHECK(!mutated(7, 4));              // a slot counted but empty
  CHECK(!mutated(52, 5));             // address family
  CHECK(!mutated(53, 1));             // reserved byte
  CHECK(!mutated(52 + 4 + 4, 1));     // IPv4 address with a non-zero tail
  CHECK(mutated(7, 2));               // fewer candidates is still well-formed
  CHECK(!decode_rendezvous(nullptr, 0));
}

TEST_CASE(rendezvous_server_introduces_the_two_roles_of_a_meeting) {
  RendezvousServer server(30 * kNanosPerSecond);
  const auto id = id_of(1);
  const Endpoint a = ep("203.0.113.1", 1111), b = ep("198.51.100.2", 2222);
  const Endpoint lan = ep("192.168.1.20", 2222);

  // The master is first: it is told to wait, and where it was seen.
  auto out = feed(server, registration(id, RendezvousRole::kMaster, 11), a, 0);
  REQUIRE(out.size() == 1);
  CHECK(out[0].to == a);
  auto w = decoded(out[0]);
  CHECK(w.type == RendezvousMessage::Type::kWaiting && w.txid == 11 && w.observed == a);

  // The slave comes: it learns the master, and the master is told of it.
  out = feed(server, registration(id, RendezvousRole::kSlave, 22, {lan}), b, kNanosPerSecond);
  REQUIRE(out.size() == 2);
  CHECK(out[0].to == b);
  auto p = decoded(out[0]);
  CHECK(p.type == RendezvousMessage::Type::kPeer && p.role == RendezvousRole::kMaster);
  CHECK(p.txid == 22 && p.observed == b);
  CHECK(p.candidates == std::vector<Endpoint>{a});
  CHECK(out[1].to == a);
  p = decoded(out[1]);
  CHECK(p.type == RendezvousMessage::Type::kPeer && p.role == RendezvousRole::kSlave);
  CHECK(p.txid == 11 && p.observed == a);
  CHECK((p.candidates == std::vector<Endpoint>{b, lan}));
  // Nothing the server sends is larger than what it received.
  for (const auto& o : out) CHECK(o.bytes.size() == kRendezvousMessageSize);

  // A refresh with the same addresses is answered, nothing more.
  out = feed(server, registration(id, RendezvousRole::kSlave, 22, {lan}), b, 2 * kNanosPerSecond);
  CHECK(out.size() == 1 && out[0].to == b);
  // A slave whose NAT moved it: the master is told again.
  const Endpoint b2 = ep("198.51.100.2", 3333);
  out = feed(server, registration(id, RendezvousRole::kSlave, 22, {lan}), b2, 3 * kNanosPerSecond);
  REQUIRE(out.size() == 2);
  CHECK(out[1].to == a && decoded(out[1]).candidates.front() == b2);

  // Another meeting does not see this one.
  out = feed(server, registration(id_of(2), RendezvousRole::kSlave, 33), b, 4 * kNanosPerSecond);
  CHECK(out.size() == 1 && decoded(out[0]).type == RendezvousMessage::Type::kWaiting);
  CHECK(server.meetings() == 2);

  // The master stops refreshing; 30 s later it is gone.
  out = feed(server, registration(id, RendezvousRole::kSlave, 22, {lan}), b2,
             31 * kNanosPerSecond);
  CHECK(out.size() == 1 && decoded(out[0]).type == RendezvousMessage::Type::kWaiting);

  // Requests the server does not serve get no answer at all.
  auto bogus = registration(id, RendezvousRole::kMaster, 1);
  bogus.type = RendezvousMessage::Type::kPeer;
  CHECK(feed(server, bogus, a, 32 * kNanosPerSecond).empty());
  std::vector<RendezvousServer::Out> none;
  const std::uint8_t junk[8] = {};
  server.on_datagram(junk, sizeof junk, a, 0, none);
  CHECK(none.empty());
}

TEST_CASE(rendezvous_server_makes_room_by_dropping_the_oldest_meeting) {
  RendezvousServer server(30 * kNanosPerSecond, 2);
  const Endpoint a = ep("203.0.113.1", 1), b = ep("203.0.113.2", 2);
  feed(server, registration(id_of(1), RendezvousRole::kMaster, 1), a, 0);
  feed(server, registration(id_of(2), RendezvousRole::kMaster, 2), a, 1);
  feed(server, registration(id_of(1), RendezvousRole::kMaster, 1), a, 2);  // refreshed
  feed(server, registration(id_of(3), RendezvousRole::kMaster, 3), a, 3);  // evicts 2
  CHECK(server.meetings() == 2);
  auto out = feed(server, registration(id_of(1), RendezvousRole::kSlave, 4), b, 4);
  CHECK(decoded(out[0]).type == RendezvousMessage::Type::kPeer);
  out = feed(server, registration(id_of(2), RendezvousRole::kSlave, 5), b, 5);
  CHECK(decoded(out[0]).type == RendezvousMessage::Type::kWaiting);
}

TEST_CASE(rendezvous_client_registers_retries_and_learns_the_peer) {
  const Endpoint server = ep("203.0.113.9", 47200);
  const Endpoint lan = ep("192.168.1.5", 47100);
  RendezvousClient c(server, id_of(7), RendezvousRole::kMaster, {lan});
  const auto reg = c.poll(0);
  REQUIRE(reg);
  const auto m = decode_rendezvous(reg->data(), reg->size());
  REQUIRE(m);
  CHECK(m->type == RendezvousMessage::Type::kRegister && m->role == RendezvousRole::kMaster);
  CHECK(m->candidates == std::vector<Endpoint>{lan});
  CHECK(!c.poll(100 * kNanosPerMilli));
  CHECK(c.poll(RendezvousClient::kRetryInterval));  // unanswered: retried
  CHECK(!c.answered());

  auto reply = [&](RendezvousMessage::Type type, RendezvousRole role, std::uint64_t txid,
                   std::vector<Endpoint> cands) {
    RendezvousMessage r;
    r.type = type;
    r.role = role;
    r.id = id_of(7);
    r.txid = txid;
    r.observed = ep("198.51.100.1", 6000);
    r.candidates = std::move(cands);
    return encode_rendezvous(r);
  };
  const Nanos t = kNanosPerSecond;
  auto w = reply(RendezvousMessage::Type::kWaiting, RendezvousRole::kMaster, m->txid, {});
  CHECK(!c.on_datagram(w.data(), w.size(), ep("203.0.113.10", 47200), t));  // not the server
  auto wrong = reply(RendezvousMessage::Type::kWaiting, RendezvousRole::kMaster, m->txid + 1, {});
  CHECK(!c.on_datagram(wrong.data(), wrong.size(), server, t));  // not our registration
  CHECK(c.on_datagram(w.data(), w.size(), server, t));
  CHECK(c.answered() && c.observed() == ep("198.51.100.1", 6000));
  CHECK(c.peer().empty() && c.peer_changes() == 0);
  CHECK(c.next_due() == t + RendezvousClient::kRefreshInterval);

  const std::vector<Endpoint> slave{ep("198.51.100.2", 7000), ep("10.1.1.1", 7000)};
  auto own_role = reply(RendezvousMessage::Type::kPeer, RendezvousRole::kMaster, m->txid, slave);
  CHECK(!c.on_datagram(own_role.data(), own_role.size(), server, t));
  auto p = reply(RendezvousMessage::Type::kPeer, RendezvousRole::kSlave, m->txid, slave);
  CHECK(c.on_datagram(p.data(), p.size(), server, t));
  CHECK(c.peer() == slave && c.peer_changes() == 1);
  CHECK(c.on_datagram(p.data(), p.size(), server, t));
  CHECK(c.peer_changes() == 1);  // the same announcement again
}

TEST_CASE(rendezvous_id_depends_only_on_the_key) {
  const auto k1 = crypto::random_key(), k2 = crypto::random_key();
  CHECK(crypto::rendezvous_id(k1) == crypto::rendezvous_id(k1));
  CHECK(crypto::rendezvous_id(k1) != crypto::rendezvous_id(k2));
  const auto id = crypto::rendezvous_id(k1);
  CHECK(std::memcmp(id.data(), k1.data(), id.size()) != 0);
}

// ---------------------------------------------------------------- nodes

namespace {

using namespace tuushin::teleop;

struct Meeting {
  MasterNodeReport master;
  std::vector<SlaveSessionReport> slave;
  std::vector<std::string> slave_events;
  std::vector<std::string> server_log;
};

// A master and a slave that know only the server, all on loopback. The
// master starts `master_lead` before the slave (negative: after it).
Meeting meet(UdpSocket& master_sock, Nanos master_lead, Nanos duration, Nanos wait,
             bool slave_present = true) {
  UdpSocket server_sock(ep("127.0.0.1", 0));
  const Endpoint server = ep("127.0.0.1", server_sock.local_endpoint().port());
  const crypto::Key psk = crypto::random_key();
  Meeting out;
  std::mutex mu;
  std::atomic<bool> stop_server{false}, stop_slave{false};
  std::thread srv([&] {
    run_rendezvous_server(server_sock, &stop_server, [&](const std::string& line) {
      std::lock_guard<std::mutex> lock(mu);
      out.server_log.push_back(line);
    });
  });

  const Nanos period = kNanosPerMilli;
  RendezvousConfig rc;
  rc.server = server;
  rc.id = crypto::rendezvous_id(psk);
  rc.wait = wait;

  UdpSocket slave_sock(ep("127.0.0.1", 0));
  SimulatedSlave slave_dev(1.0, 2.0, WallParams{}, period, 10, monotonic_now(), 0.0);
  slave_dev.set_watchdog(2 * period);
  SlaveNodeConfig scfg;
  scfg.control.dt = period;
  scfg.control.hold_after = 150 * kNanosPerMilli;
  scfg.once = true;
  scfg.psk = psk;
  scfg.rendezvous = rc;
  SlaveNodeHooks hooks;
  hooks.on_rendezvous = [&](const std::string& what) {
    std::lock_guard<std::mutex> lock(mu);
    out.slave_events.push_back(what);
  };
  std::thread slave;
  auto start_slave = [&] {
    if (!slave_present) return;
    slave = std::thread([&] { out.slave = run_slave_node(slave_sock, slave_dev, scfg, hooks, &stop_slave); });
  };
  if (master_lead <= 0) {
    start_slave();
    std::this_thread::sleep_for(std::chrono::nanoseconds(-master_lead));
  }

  SimulatedMaster master_dev(0.2, 0.5, OperatorParams{}, period, 10, monotonic_now());
  master_dev.set_watchdog(2 * period);
  MasterNodeConfig mcfg;
  mcfg.control.dt = period;
  mcfg.control.model_mass = 0.2;
  mcfg.duration = duration;
  mcfg.psk = psk;
  mcfg.hello_interval = 50 * kNanosPerMilli;
  mcfg.rendezvous = rc;
  std::thread late_slave;
  if (master_lead > 0) {
    late_slave = std::thread([&] {
      std::this_thread::sleep_for(std::chrono::nanoseconds(master_lead));
      start_slave();
    });
  }
  out.master = run_master_node(master_sock, Endpoint{}, master_dev, mcfg);
  if (late_slave.joinable()) late_slave.join();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_slave.store(true);
  if (slave.joinable()) slave.join();
  stop_server.store(true);
  srv.join();
  return out;
}

bool any_contains(const std::vector<std::string>& v, const char* what) {
  for (const auto& s : v) {
    if (s.find(what) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

TEST_CASE(nodes_meet_through_a_rendezvous_server) {
  // The slave registers first and waits; the master learns it from the
  // server's answer, and the slave learns the master from the server's push.
  UdpSocket master_sock(ep("127.0.0.1", 0));
  const auto r = meet(master_sock, -200 * kNanosPerMilli, 600 * kNanosPerMilli, kNanosPerSecond);
  CHECK(!r.master.rendezvous_failed && !r.master.handshake_failed);
  CHECK(r.master.authenticated && r.master.slave_answered && r.master.slave_final);
  CHECK(r.master.link.received > 400);
  CHECK(r.master.observed == ep("127.0.0.1", master_sock.local_endpoint().port()));
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].authenticated && r.slave[0].ended_by_final);
  CHECK(r.slave[0].link.received > 400);
  CHECK(r.slave[0].master == ep("127.0.0.1", master_sock.local_endpoint().port()));
  REQUIRE(!r.master.candidates.empty());
  CHECK(r.master.slave == r.master.candidates.front());
  CHECK(any_contains(r.slave_events, "registered at"));
  CHECK(any_contains(r.slave_events, "master announced at 127.0.0.1:"));
  CHECK(any_contains(r.server_log, "slave at 127.0.0.1:"));
  CHECK(any_contains(r.server_log, "peer announced"));
}

TEST_CASE(nodes_meet_when_the_master_is_first) {
  // Now the master waits at the server, and the slave's registration makes
  // the server push it to the master.
  UdpSocket master_sock(ep("127.0.0.1", 0));
  const auto r = meet(master_sock, 300 * kNanosPerMilli, 400 * kNanosPerMilli, 2 * kNanosPerSecond);
  CHECK(r.master.authenticated && r.master.slave_answered);
  CHECK(r.master.link.received > 250);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].authenticated);
}

TEST_CASE(nodes_meet_from_a_dual_stack_socket) {
  // A master on an IPv6 wildcard socket sends to the IPv4 server and slave
  // in the mapped form, and recognizes their answers in it.
  std::optional<UdpSocket> sock;
  try {
    sock.emplace(ep("::", 0));
  } catch (const std::exception&) {
    return;  // no IPv6 on this host
  }
  const auto r = meet(*sock, -200 * kNanosPerMilli, 400 * kNanosPerMilli, kNanosPerSecond);
  CHECK(r.master.authenticated && r.master.slave_answered);
  CHECK(r.master.link.received > 250);
  CHECK(r.master.slave.canonical() == r.master.candidates.front());
  CHECK(r.master.observed == ep("127.0.0.1", sock->local_endpoint().port()));
}

TEST_CASE(master_gives_up_when_the_server_announces_no_slave) {
  UdpSocket master_sock(ep("127.0.0.1", 0));
  const auto t0 = monotonic_now();
  const auto r = meet(master_sock, 0, 400 * kNanosPerMilli, 300 * kNanosPerMilli, false);
  CHECK(r.master.rendezvous_failed && r.master.handshake_failed);
  CHECK(!r.master.slave_answered);
  CHECK(r.master.observed == ep("127.0.0.1", master_sock.local_endpoint().port()));
  CHECK(monotonic_now() - t0 < 2 * kNanosPerSecond);
}

TEST_CASE(rendezvous_requires_a_key_and_a_master_needs_a_slave_or_a_server) {
  // Without authentication a slave could not tell the master's hello from
  // anything else arriving at its candidates, nor the master the slave's reply.
  UdpSocket sock(ep("127.0.0.1", 0));
  const Nanos period = kNanosPerMilli;
  SimulatedMaster master_dev(0.2, 0.5, OperatorParams{}, period, 10, monotonic_now());
  MasterNodeConfig mcfg;
  mcfg.control.dt = period;
  RendezvousConfig rc;
  rc.server = ep("127.0.0.1", 9);
  mcfg.rendezvous = rc;
  bool threw = false;
  try {
    run_master_node(sock, Endpoint{}, master_dev, mcfg);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  mcfg.rendezvous.reset();
  mcfg.psk = crypto::random_key();
  threw = false;
  try {
    run_master_node(sock, Endpoint{}, master_dev, mcfg);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  SimulatedSlave slave_dev(1.0, 2.0, WallParams{}, period, 10, monotonic_now(), 0.0);
  SlaveNodeConfig scfg;
  scfg.control.dt = period;
  scfg.rendezvous = rc;
  std::atomic<bool> stop{true};
  threw = false;
  try {
    run_slave_node(sock, slave_dev, scfg, SlaveNodeHooks{}, &stop);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE(endpoints_convert_between_families) {
  const Endpoint v4 = ep("192.0.2.1", 5000);
  const Endpoint mapped = *v4.for_family(true);
  CHECK(mapped.is_ipv6());
  CHECK(mapped.to_string().find("::ffff:192.0.2.1") != std::string::npos);
  CHECK(mapped.canonical() == v4 && v4.canonical() == v4);
  CHECK(*mapped.for_family(false) == v4 && *v4.for_family(false) == v4);
  CHECK(mapped.address().size() == 4 && v4.address().size() == 4);
  const Endpoint v6 = ep("2001:db8::7", 6000);
  CHECK(v6.address().size() == 16 && v6.canonical() == v6);
  CHECK(!v6.for_family(false));  // an IPv4 socket cannot reach it
  CHECK(*v6.for_family(true) == v6);
  const auto a = v4.address();
  CHECK(*Endpoint::from_address(a.data(), a.size(), 5000) == v4);
  CHECK(!Endpoint::from_address(a.data(), 3, 5000));
  CHECK(ep("127.0.0.1", 1).is_local_only() && ep("::1", 1).is_local_only());
  CHECK(ep("fe80::1", 1).is_local_only() && ep("169.254.1.1", 1).is_local_only());
  CHECK(!v4.is_local_only() && !ep("10.0.0.1", 1).is_local_only());
}

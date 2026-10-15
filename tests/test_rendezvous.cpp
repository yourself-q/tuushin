#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "sim_nat.hpp"
#include "sim_turn.hpp"
#include "test.hpp"
#include "tuushin/crypto/noise.hpp"
#include "tuushin/net/rendezvous.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/teleop/device.hpp"
#include "tuushin/teleop/node.hpp"
#include "tuushin/wire/packet.hpp"

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
  m.ready = true;
  m.id = id_of(0xa5);
  m.txid = 0x0102030405060708ull;
  m.observed = ep("203.0.113.7", 40000);
  m.candidates = {ep("198.51.100.2", 1), ep("2001:db8::1", 65535), ep("192.168.1.20", 47100)};
  const auto b = encode_rendezvous(m);
  CHECK(b.size() == kRendezvousMessageSize);
  const auto d = decode_rendezvous(b.data(), b.size());
  REQUIRE(d);
  CHECK(d->type == m.type && d->role == m.role && d->id == m.id && d->txid == m.txid);
  CHECK(d->ready);
  CHECK(d->observed == m.observed);
  CHECK(d->candidates == m.candidates);

  // At most kRendezvousMaxCandidates are carried.
  RendezvousMessage many = m;
  for (int i = 0; i < 12; ++i) many.candidates.push_back(ep("10.0.0.1", 1000 + i));
  CHECK(decode_rendezvous(encode_rendezvous(many).data(), kRendezvousMessageSize)
            ->candidates.size() == kRendezvousMaxCandidates);

  // Relayed addresses follow the end's own, marked as such, and take
  // their slots before the end's own beyond what fits.
  RendezvousMessage relayed = m;
  relayed.relays = {ep("192.0.2.50", 49152)};
  const auto rb = encode_rendezvous(relayed);
  const auto rd = decode_rendezvous(rb.data(), rb.size());
  REQUIRE(rd);
  CHECK(rd->candidates == m.candidates && rd->relays == relayed.relays);
  CHECK(rb[7] == 4 && rb[56 + 20 * 3 + 1] == 1);
  many.relays = {ep("192.0.2.50", 49152), ep("192.0.2.51", 49153), ep("192.0.2.52", 49154)};
  const auto mr = decode_rendezvous(encode_rendezvous(many).data(), kRendezvousMessageSize);
  REQUIRE(mr);
  CHECK(mr->relays.size() == kRendezvousMaxRelays);
  CHECK(mr->candidates.size() == kRendezvousMaxCandidates - kRendezvousMaxRelays);
  CHECK(mr->candidates[0] == m.candidates[0]);

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
  CHECK(!mutated(8, 4));              // an undefined flag
  CHECK(!mutated(9, 1) && !mutated(11, 1));  // reserved bytes
  CHECK(!mutated(56, 5));             // address family
  CHECK(!mutated(57, 2));             // an undefined kind
  CHECK(!mutated(57, 1));             // a relayed address before the end's own
  CHECK(!mutated(37, 1));             // the observed address marked relayed
  CHECK(!mutated(56 + 4 + 4, 1));     // IPv4 address with a non-zero tail
  CHECK(!mutated(7, 2));              // a slot not counted, but not empty
  CHECK(!mutated(56 + 20 * 5 + 1, 1));  // an empty slot with a kind
  CHECK(mutated(56 + 20 * 2 + 1, 1));   // the last address relayed instead
  auto unready = b;
  unready[8] = 0;
  const auto u = decode_rendezvous(unready.data(), unready.size());
  CHECK(u && !u->ready);
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

TEST_CASE(rendezvous_server_passes_the_ready_flag_on) {
  RendezvousServer server(30 * kNanosPerSecond);
  const auto id = id_of(1);
  const Endpoint a = ep("203.0.113.1", 1111), b = ep("198.51.100.2", 2222);
  feed(server, registration(id, RendezvousRole::kMaster, 11), a, 0);
  auto out = feed(server, registration(id, RendezvousRole::kSlave, 22), b, 1);
  REQUIRE(out.size() == 2);
  CHECK(!decoded(out[0]).ready && !decoded(out[1]).ready);

  // The slave becomes ready: the master is told at once.
  auto ready = registration(id, RendezvousRole::kSlave, 22);
  ready.ready = true;
  out = feed(server, ready, b, 2);
  REQUIRE(out.size() == 2);
  CHECK(out[1].to == a && decoded(out[1]).ready && !decoded(out[1]).recipient_ready);
  // The slave hears of the master's ready, and that its own is held.
  CHECK(!decoded(out[0]).ready && decoded(out[0]).recipient_ready);
  // A refresh that changes nothing reaches only the slave.
  CHECK(feed(server, ready, b, 3).size() == 1);
  CHECK(decoded(feed(server, registration(id, RendezvousRole::kMaster, 11), a, 4)[0]).ready);

  // The master moves: the slave's ready was about the old address.
  const Endpoint a2 = ep("203.0.113.1", 1112);
  out = feed(server, registration(id, RendezvousRole::kMaster, 11), a2, 5);
  REQUIRE(out.size() == 2);
  CHECK(!decoded(out[0]).ready);
  CHECK(out[1].to == b && decoded(out[1]).candidates.front() == a2);
  CHECK(!decoded(out[1]).recipient_ready);
  // Until the slave says it again.
  out = feed(server, ready, b, 6);
  REQUIRE(out.size() == 2);
  CHECK(out[1].to == a2 && decoded(out[1]).ready);

  // A master that registers anew from the same address (it restarted)
  // clears the slave's ready too.
  out = feed(server, registration(id, RendezvousRole::kMaster, 12), a2, 7);
  REQUIRE(out.size() == 2);
  CHECK(!decoded(out[0]).ready && decoded(out[0]).txid == 12);
  CHECK(out[1].to == b && !decoded(out[1]).recipient_ready);
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
  CHECK(!c.peer_ready());

  // Becoming ready registers again at once, with the flag, and keeps
  // registering until the server confirms it.
  CHECK(!c.poll(t + 1));
  c.set_ready(true);
  CHECK(c.ready());
  const auto again = c.poll(t + 1);
  REQUIRE(again);
  CHECK(decode_rendezvous(again->data(), again->size())->ready);
  CHECK(!c.poll(t + 2));
  CHECK(c.next_due() == t + 1 + RendezvousClient::kRetryInterval);
  // An announcement that crossed that registration does not hold this end
  // ready yet; that is no reason to open the NAT again.
  auto pr = *decode_rendezvous(p.data(), p.size());
  auto bytes = encode_rendezvous(pr);
  CHECK(c.on_datagram(bytes.data(), bytes.size(), server, t + 2));
  CHECK(c.ready() && c.peer_changes() == 1);
  CHECK(c.next_due() == t + 1 + RendezvousClient::kRetryInterval);
  pr.ready = true;            // the peer is ready too
  pr.recipient_ready = true;  // and the server holds this end ready
  bytes = encode_rendezvous(pr);
  CHECK(c.on_datagram(bytes.data(), bytes.size(), server, t + 3));
  CHECK(c.peer_ready() && c.ready() && c.peer_changes() == 1);
  CHECK(c.next_due() == t + 3 + RendezvousClient::kRefreshInterval);

  // The peer registered anew: the server no longer holds this end ready,
  // and this end has to open its NAT again.
  pr.ready = false;
  pr.recipient_ready = false;
  bytes = encode_rendezvous(pr);
  CHECK(c.on_datagram(bytes.data(), bytes.size(), server, t + 4));
  CHECK(c.peer_changes() == 2 && !c.peer_ready() && !c.ready());
  c.set_ready(true);
  CHECK(c.poll(t + 5));
  pr.recipient_ready = true;
  bytes = encode_rendezvous(pr);
  CHECK(c.on_datagram(bytes.data(), bytes.size(), server, t + 6));
  CHECK(c.ready() && c.peer_changes() == 2);

  // So does a peer at a new address.
  pr.candidates = {ep("198.51.100.2", 7001)};
  pr.recipient_ready = false;
  bytes = encode_rendezvous(pr);
  CHECK(c.on_datagram(bytes.data(), bytes.size(), server, t + 7));
  CHECK(c.peer_changes() == 3 && !c.ready() && c.peer() == pr.candidates);

  // And a peer with a new relay.
  pr.relays = {ep("192.0.2.50", 49152)};
  bytes = encode_rendezvous(pr);
  CHECK(c.on_datagram(bytes.data(), bytes.size(), server, t + 8));
  CHECK(c.peer_changes() == 4 && c.peer_relays() == pr.relays && c.peer() == pr.candidates);
}

TEST_CASE(rendezvous_carries_relayed_addresses) {
  // A slave that allocated a relay registers it; the server announces it
  // to the master apart from the slave's own addresses, and announces it
  // again when it changes, as it does a move.
  RendezvousServer server(30 * kNanosPerSecond);
  const auto id = id_of(3);
  const Endpoint a = ep("203.0.113.1", 1111), b = ep("198.51.100.2", 2222);
  const Endpoint relay = ep("192.0.2.50", 49152), relay2 = ep("192.0.2.50", 49153);
  feed(server, registration(id, RendezvousRole::kMaster, 11), a, 0);
  const Endpoint srv = ep("203.0.113.9", 47200);
  RendezvousClient slave(srv, id, RendezvousRole::kSlave, {});
  slave.set_relays({relay});
  CHECK(slave.relays() == std::vector<Endpoint>{relay});
  const auto reg = slave.poll(0);
  REQUIRE(reg);
  std::vector<RendezvousServer::Out> out;
  CHECK(server.on_datagram(reg->data(), reg->size(), b, kNanosPerSecond, out));
  REQUIRE(out.size() == 2 && out[1].to == a);
  auto p = decoded(out[1]);
  CHECK(p.candidates == std::vector<Endpoint>{b});
  CHECK(p.relays == std::vector<Endpoint>{relay});
  CHECK(slave.on_datagram(out[0].bytes.data(), out[0].bytes.size(), srv, kNanosPerSecond));
  CHECK(slave.peer_relays().empty());  // the master has none

  // The same registration again changes nothing.
  out.clear();
  CHECK(!server.on_datagram(reg->data(), reg->size(), b, 2 * kNanosPerSecond, out));
  CHECK(out.size() == 1);

  // A new relay registers at once, and the master hears of it.
  CHECK(!slave.poll(2 * kNanosPerSecond));
  slave.set_relays({relay2});
  const auto reg2 = slave.poll(2 * kNanosPerSecond);
  REQUIRE(reg2);
  out.clear();
  CHECK(server.on_datagram(reg2->data(), reg2->size(), b, 3 * kNanosPerSecond, out));
  REQUIRE(out.size() == 2);
  CHECK(decoded(out[1]).relays == std::vector<Endpoint>{relay2});
  // And so does the loss of it.
  slave.set_relays({});
  const auto reg3 = slave.poll(3 * kNanosPerSecond);
  REQUIRE(reg3);
  out.clear();
  CHECK(server.on_datagram(reg3->data(), reg3->size(), b, 4 * kNanosPerSecond, out));
  REQUIRE(out.size() == 2);
  CHECK(decoded(out[1]).relays.empty());
  // Setting the same relays again is no news.
  slave.set_relays({});
  CHECK(!slave.poll(4 * kNanosPerSecond));
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

namespace {

struct Approach {
  bool punch_first = false;  // the master's first datagram was a punch
  Nanos first_hello = -1;    // after the announcement; -1 for none
};

// How a master approaches a slave that the server announced: the slave is
// played by a bare socket that registers, ready at once or never.
Approach approach(bool slave_ready, Nanos ready_wait) {
  UdpSocket server_sock(ep("127.0.0.1", 0));
  const Endpoint server = ep("127.0.0.1", server_sock.local_endpoint().port());
  std::atomic<bool> stop_server{false};
  std::thread srv([&] { run_rendezvous_server(server_sock, &stop_server); });
  const crypto::Key psk = crypto::random_key();
  RendezvousConfig rc;
  rc.server = server;
  rc.id = crypto::rendezvous_id(psk);
  rc.ready_wait = ready_wait;

  const Nanos period = kNanosPerMilli;
  UdpSocket master_sock(ep("127.0.0.1", 0));
  SimulatedMaster dev(0.2, 0.5, OperatorParams{}, period, 10, monotonic_now());
  dev.set_watchdog(2 * period);
  MasterNodeConfig mcfg;
  mcfg.control.dt = period;
  mcfg.psk = psk;
  mcfg.hello_interval = 50 * kNanosPerMilli;
  mcfg.handshake_timeout = ready_wait + 300 * kNanosPerMilli;
  mcfg.rendezvous = rc;
  std::atomic<bool> master_done{false};
  std::thread master([&] {
    run_master_node(master_sock, Endpoint{}, dev, mcfg);
    master_done.store(true);
  });

  UdpSocket sock(ep("127.0.0.1", 0));
  RendezvousClient rv(server, rc.id, RendezvousRole::kSlave, {});
  Approach out;
  Nanos announced = -1;
  bool any = false;
  std::uint8_t buf[wire::kMaxDatagram + 1];
  while (!master_done.load()) {
    if (const auto reg = rv.poll(monotonic_now())) sock.send_to(server, reg->data(), reg->size());
    sock.wait_readable(5 * kNanosPerMilli);
    Endpoint from;
    while (const auto n = sock.recv_from(buf, sizeof buf, &from)) {
      const Nanos now = monotonic_now();
      if (rv.on_datagram(buf, *n, from, now)) {
        if (announced < 0 && rv.peer_changes() > 0) {
          announced = now;
          rv.set_ready(slave_ready);
        }
        continue;
      }
      const auto h = wire::decode_header(buf, *n);
      if (!h) continue;
      if (!any) out.punch_first = h->type == wire::PacketType::kPunch;
      any = true;
      if (h->type == wire::PacketType::kHello && out.first_hello < 0 && announced >= 0) {
        out.first_hello = now - announced;
      }
    }
  }
  master.join();
  stop_server.store(true);
  srv.join();
  return out;
}

}  // namespace

TEST_CASE(master_primes_and_sends_hellos_once_the_slave_is_ready) {
  // Ready at once: the hellos follow the slave's ready, a round trip to the
  // server after the announcement.
  const Nanos wait = 600 * kNanosPerMilli;
  const auto ready = approach(true, wait);
  CHECK(ready.punch_first);
  CHECK(ready.first_hello >= 0 && ready.first_hello < wait / 2);
  // Never ready: the hellos wait ready_wait.
  const auto unready = approach(false, wait);
  CHECK(unready.punch_first);
  CHECK(unready.first_hello >= wait - 50 * kNanosPerMilli);
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

namespace {

struct Rebound {
  MasterNodeReport master;
  std::vector<SlaveSessionReport> slave;
  std::uint64_t filtered = 0;  // by the master's NAT, after the rebinding
};

// A master and a slave, each behind a NAT that maps independently of the
// destination and filters by address and port (the common home router).
// Once the session runs, the slave's NAT forgets its mappings, so the slave
// sends from a new outside port that the master's NAT does not let in, and
// the master keeps sending to the old one.
Rebound rebound(Nanos rejoin_after) {
  using test::SimNat;
  UdpSocket server_sock(ep("127.0.0.1", 0));
  const Endpoint server = ep("127.0.0.1", server_sock.local_endpoint().port());
  std::atomic<bool> stop_server{false}, stop_slave{false};
  std::thread srv([&] { run_rendezvous_server(server_sock, &stop_server); });

  UdpSocket master_sock(ep("127.0.0.1", 0)), slave_sock(ep("127.0.0.1", 0));
  SimNat master_nat(server, SimNat::Mapping::kEndpointIndependent,
                    SimNat::Filtering::kAddressPortDependent);
  SimNat slave_nat(server, SimNat::Mapping::kEndpointIndependent,
                   SimNat::Filtering::kAddressPortDependent);
  const crypto::Key psk = crypto::random_key();
  RendezvousConfig rc;
  rc.id = crypto::rendezvous_id(psk);
  rc.wait = 2 * kNanosPerSecond;
  rc.rejoin_after = rejoin_after;
  RendezvousConfig master_rc = rc, slave_rc = rc;
  master_rc.server = master_nat.proxy(server);
  slave_rc.server = slave_nat.proxy(server);
  master_nat.start();
  slave_nat.start();

  const Nanos period = kNanosPerMilli;
  SimulatedSlave slave_dev(1.0, 2.0, WallParams{}, period, 10, monotonic_now(), 0.0);
  slave_dev.set_watchdog(2 * period);
  SlaveNodeConfig scfg;
  scfg.control.dt = period;
  scfg.control.hold_after = 150 * kNanosPerMilli;
  scfg.once = true;
  scfg.psk = psk;
  scfg.rendezvous = slave_rc;
  Rebound out;
  std::thread slave([&] { out.slave = run_slave_node(slave_sock, slave_dev, scfg, {}, &stop_slave); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Rebind once the master's samples have run for a while.
  std::atomic<bool> master_done{false};
  std::uint64_t filtered_before = 0;
  std::thread rebinder([&] {
    while (!master_done.load() && slave_nat.delivered() < 300) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    filtered_before = master_nat.filtered();
    slave_nat.rebind();
  });

  SimulatedMaster master_dev(0.2, 0.5, OperatorParams{}, period, 10, monotonic_now());
  master_dev.set_watchdog(2 * period);
  MasterNodeConfig mcfg;
  mcfg.control.dt = period;
  mcfg.control.model_mass = 0.2;
  mcfg.duration = kNanosPerSecond;
  mcfg.psk = psk;
  mcfg.hello_interval = 50 * kNanosPerMilli;
  mcfg.rendezvous = master_rc;
  out.master = run_master_node(master_sock, Endpoint{}, master_dev, mcfg);
  master_done.store(true);
  rebinder.join();
  out.filtered = master_nat.filtered() - filtered_before;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_slave.store(true);
  slave.join();
  stop_server.store(true);
  srv.join();
  return out;
}

}  // namespace

TEST_CASE(session_survives_a_nat_that_rebinds) {
  // Both ends fall silent; both register anew; the server announces the
  // slave's new address to the master, whose punch lets the slave's samples
  // through its NAT, and the master's link follows them there.
  const auto r = rebound(100 * kNanosPerMilli);
  CHECK(r.master.authenticated && r.master.slave_answered);
  CHECK(r.filtered > 0);
  CHECK(r.master.rejoins >= 1);
  CHECK(r.master.link.moved == 1);
  CHECK(r.master.link.received > 700);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].ended_by_final);
  CHECK(r.slave[0].rejoins == 0);  // the master did not move
  CHECK(r.slave[0].link.moved == 0);
  // The master's punches towards the slave's new address are not foreign.
  CHECK(r.slave[0].link.foreign == 0 && r.master.link.foreign == 0);
}

TEST_CASE(session_is_lost_to_a_rebinding_nat_without_the_rejoin) {
  // The control: the same rebinding, with the ends waiting for their next
  // refresh (10 s) to register again. The slave's samples stay filtered, so
  // the session ends there.
  const auto r = rebound(3600 * kNanosPerSecond);
  CHECK(r.master.authenticated && r.master.slave_answered);
  CHECK(r.filtered > 100);
  CHECK(r.master.rejoins == 0 && r.master.link.moved == 0);
  CHECK(r.master.link.received < 600);
  REQUIRE(r.slave.size() == 1);
  CHECK(!r.slave[0].ended_by_final);
}

namespace {

struct Relayed {
  MasterNodeReport master;
  std::vector<SlaveSessionReport> slave;
  std::vector<std::string> slave_events;
  std::uint64_t relayed_datagrams = 0;  // passed on by the TURN server
  std::uint64_t allocations = 0;
};

// A master behind a NAT with `master_mapping` and a slave behind one that
// picks a new outside port for every destination ("symmetric"), both
// filtering by address and port: the server announces to each end the port
// the other's NAT uses towards the server, which lets no one else in, and
// a symmetric NAT sends to anyone else from a port nobody was told of, so
// no direct path opens. With `turn`, the slave allocates a relay on a TURN
// server outside both NATs, and the server announces its address to the
// master. With `rebind`, the slave's NAT forgets its mappings once the
// session runs, the relay's included, which the TURN server then no
// longer knows.
Relayed relayed_session(test::SimNat::Mapping master_mapping, bool turn, bool rebind = false) {
  using test::SimNat;
  UdpSocket server_sock(ep("127.0.0.1", 0));
  const Endpoint server = ep("127.0.0.1", server_sock.local_endpoint().port());
  std::atomic<bool> stop_server{false}, stop_slave{false};
  std::thread srv([&] { run_rendezvous_server(server_sock, &stop_server); });
  test::SimTurn turn_server("tuushin", "relay-secret");
  turn_server.start();

  UdpSocket master_sock(ep("127.0.0.1", 0)), slave_sock(ep("127.0.0.1", 0));
  SimNat master_nat(server, master_mapping, SimNat::Filtering::kAddressPortDependent);
  SimNat slave_nat(server, SimNat::Mapping::kAddressPortDependent,
                   SimNat::Filtering::kAddressPortDependent);
  const crypto::Key psk = crypto::random_key();
  RendezvousConfig rc;
  rc.id = crypto::rendezvous_id(psk);
  rc.wait = 2 * kNanosPerSecond;
  rc.ready_wait = 200 * kNanosPerMilli;
  rc.relay_delay = 300 * kNanosPerMilli;
  RendezvousConfig master_rc = rc, slave_rc = rc;
  master_rc.server = master_nat.proxy(server);
  slave_rc.server = slave_nat.proxy(server);
  const Endpoint turn_at = slave_nat.proxy(turn_server.endpoint());
  master_nat.start();
  slave_nat.start();

  Relayed out;
  std::mutex mu;
  const Nanos period = kNanosPerMilli;
  SimulatedSlave slave_dev(1.0, 2.0, WallParams{}, period, 10, monotonic_now(), 0.0);
  slave_dev.set_watchdog(2 * period);
  SlaveNodeConfig scfg;
  scfg.control.dt = period;
  scfg.control.hold_after = 150 * kNanosPerMilli;
  scfg.once = true;
  scfg.psk = psk;
  scfg.rendezvous = slave_rc;
  if (turn) scfg.relay = RelayConfig{turn_at, "tuushin", "relay-secret"};
  SlaveNodeHooks hooks;
  hooks.on_rendezvous = [&](const std::string& what) {
    std::lock_guard<std::mutex> lock(mu);
    out.slave_events.push_back(what);
  };
  std::thread slave([&] { out.slave = run_slave_node(slave_sock, slave_dev, scfg, hooks, &stop_slave); });
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::atomic<bool> master_done{false};
  std::thread rebinder([&] {
    if (!rebind) return;
    while (!master_done.load() && slave_nat.delivered() < 300) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    slave_nat.rebind();
  });

  SimulatedMaster master_dev(0.2, 0.5, OperatorParams{}, period, 10, monotonic_now());
  master_dev.set_watchdog(2 * period);
  MasterNodeConfig mcfg;
  mcfg.control.dt = period;
  mcfg.control.model_mass = 0.2;
  mcfg.duration = rebind ? kNanosPerSecond : 600 * kNanosPerMilli;
  mcfg.psk = psk;
  mcfg.hello_interval = 50 * kNanosPerMilli;
  mcfg.handshake_timeout = 1500 * kNanosPerMilli;
  mcfg.rendezvous = master_rc;
  out.master = run_master_node(master_sock, Endpoint{}, master_dev, mcfg);
  master_done.store(true);
  rebinder.join();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_slave.store(true);
  slave.join();
  out.relayed_datagrams = turn_server.data_indications() + turn_server.channel_data();
  out.allocations = turn_server.allocations();
  stop_server.store(true);
  srv.join();
  return out;
}

}  // namespace

TEST_CASE(session_runs_through_the_relay_between_symmetric_nats) {
  const auto r = relayed_session(test::SimNat::Mapping::kAddressPortDependent, true);
  REQUIRE(!r.master.rendezvous_failed && !r.master.handshake_failed);
  CHECK(any_contains(r.slave_events, "relay allocated at"));
  REQUIRE(r.master.relays.size() == 1);
  CHECK(r.master.relayed && r.master.slave == r.master.relays[0]);
  CHECK(r.master.authenticated && r.master.slave_answered && r.master.slave_final);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].relayed && r.slave[0].authenticated && r.slave[0].ended_by_final);
  // The session ran its 600 ms, and the relay carried what each end sent
  // (counted against what was sent: a loaded machine skips ticks, so fewer
  // samples go out, but none may be lost on the way).
  CHECK(r.master.ticks + r.master.skipped > 550);
  CHECK(r.master.link.received > 0.95 * static_cast<double>(r.slave[0].link.sent));
  CHECK(r.slave[0].link.received > 0.95 * static_cast<double>(r.master.link.sent));
  CHECK(r.relayed_datagrams >= r.slave[0].link.received);
  CHECK(r.slave[0].link.foreign == 0 && r.master.link.foreign == 0);
}

TEST_CASE(symmetric_nats_meet_but_cannot_open_a_path_without_a_relay) {
  // The control: the same NATs and no relay. Both ends learn each other,
  // and every hello stops at the slave's NAT.
  const auto r = relayed_session(test::SimNat::Mapping::kAddressPortDependent, false);
  CHECK(!r.master.rendezvous_failed);
  CHECK(r.master.candidates.size() == 1 && r.master.relays.empty());
  CHECK(r.master.handshake_failed && !r.master.authenticated);
  CHECK(r.slave.empty());
}

TEST_CASE(relayed_session_survives_a_nat_that_rebinds) {
  // The slave's NAT forgets the relay's mapping with the others. The
  // slave's silence makes its TURN client refresh at once, which the
  // server answers with 437 from the new address; the client allocates
  // again, the slave announces the new relay, the master punches towards
  // it, and the master's link follows the slave's samples there. (The
  // master's NAT maps independently of the destination: one that does not
  // would give the punch a port the slave's samples are not sent to.)
  const auto r = relayed_session(test::SimNat::Mapping::kEndpointIndependent, true, true);
  REQUIRE(r.master.authenticated && r.master.relayed);
  CHECK(r.allocations == 2);
  CHECK(any_contains(r.slave_events, "relay lost (437"));
  CHECK(r.master.rejoins >= 1);
  CHECK(r.master.link.moved == 1);
  CHECK(r.master.link.received > 700);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].ended_by_final);
  CHECK(r.slave[0].link.foreign == 0 && r.master.link.foreign == 0);
}

TEST_CASE(relayed_session_survives_a_rebinding_nat_between_symmetric_nats) {
  // As above, with a master behind a NAT that gives every destination its
  // own port: its punches towards the new relay leave from a port the
  // slave was never told of. They are copies of the master's last sample,
  // so the slave's link follows the master there, and the slave's samples
  // through the new relay go to that port, which lets them in.
  const auto r = relayed_session(test::SimNat::Mapping::kAddressPortDependent, true, true);
  REQUIRE(r.master.authenticated && r.master.relayed);
  CHECK(r.allocations == 2);
  CHECK(any_contains(r.slave_events, "relay lost (437"));
  CHECK(r.master.rejoins >= 1);
  CHECK(r.master.link.moved == 1);
  CHECK(r.master.link.received > 700);
  REQUIRE(r.slave.size() == 1);
  CHECK(r.slave[0].ended_by_final);
  CHECK(r.slave[0].link.moved == 1);
  CHECK(r.slave[0].link.foreign == 0 && r.master.link.foreign == 0);
}

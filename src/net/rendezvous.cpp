#include "tuushin/net/rendezvous.hpp"

#include <algorithm>
#include <cstring>
#include <random>
#include <utility>

namespace tuushin::net {

namespace {

constexpr std::uint8_t kMagic[4] = {'T', 'S', 'R', 'V'};
constexpr std::uint8_t kVersion = 1;
constexpr std::size_t kAddressSize = 20;
constexpr std::size_t kObservedAt = 32;
constexpr std::size_t kCandidatesAt = 52;

void put_address(std::uint8_t* p, const Endpoint& e) {
  std::memset(p, 0, kAddressSize);
  const auto a = e.address();
  if (a.size() != 4 && a.size() != 16) return;
  p[0] = a.size() == 4 ? 4 : 6;
  p[2] = static_cast<std::uint8_t>(e.port());
  p[3] = static_cast<std::uint8_t>(e.port() >> 8);
  std::memcpy(p + 4, a.data(), a.size());
}

// nullopt if malformed; an invalid Endpoint for an empty slot.
std::optional<Endpoint> get_address(const std::uint8_t* p) {
  if (p[1] != 0) return std::nullopt;
  const std::uint16_t port = static_cast<std::uint16_t>(p[2] | (p[3] << 8));
  if (p[0] == 0) return Endpoint{};
  if (p[0] == 4) {
    for (int i = 8; i < 20; ++i) {
      if (p[i] != 0) return std::nullopt;
    }
    return Endpoint::from_address(p + 4, 4, port);
  }
  if (p[0] == 6) return Endpoint::from_address(p + 4, 16, port);
  return std::nullopt;
}

std::uint64_t random_txid() {
  std::random_device rd;
  return (static_cast<std::uint64_t>(rd()) << 32) | rd();
}

int index_of(RendezvousRole r) { return r == RendezvousRole::kMaster ? 0 : 1; }

RendezvousRole other(RendezvousRole r) {
  return r == RendezvousRole::kMaster ? RendezvousRole::kSlave : RendezvousRole::kMaster;
}

}  // namespace

std::vector<std::uint8_t> encode_rendezvous(const RendezvousMessage& m) {
  std::vector<std::uint8_t> out(kRendezvousMessageSize, 0);
  std::memcpy(out.data(), kMagic, 4);
  out[4] = kVersion;
  out[5] = static_cast<std::uint8_t>(m.type);
  out[6] = static_cast<std::uint8_t>(m.role);
  std::size_t n = 0;
  for (const auto& c : m.candidates) {
    if (n == kRendezvousMaxCandidates) break;
    if (!c.valid()) continue;
    put_address(out.data() + kCandidatesAt + kAddressSize * n, c);
    ++n;
  }
  out[7] = static_cast<std::uint8_t>(n);
  std::memcpy(out.data() + 8, m.id.data(), m.id.size());
  for (int i = 0; i < 8; ++i) out[24 + i] = static_cast<std::uint8_t>(m.txid >> (8 * i));
  if (m.observed.valid()) put_address(out.data() + kObservedAt, m.observed);
  return out;
}

std::optional<RendezvousMessage> decode_rendezvous(const std::uint8_t* data, std::size_t size) {
  if (data == nullptr || size != kRendezvousMessageSize) return std::nullopt;
  if (std::memcmp(data, kMagic, 4) != 0 || data[4] != kVersion) return std::nullopt;
  RendezvousMessage m;
  if (data[5] < 1 || data[5] > 3 || data[6] < 1 || data[6] > 2) return std::nullopt;
  m.type = static_cast<RendezvousMessage::Type>(data[5]);
  m.role = static_cast<RendezvousRole>(data[6]);
  const std::size_t n = data[7];
  if (n > kRendezvousMaxCandidates) return std::nullopt;
  std::memcpy(m.id.data(), data + 8, m.id.size());
  for (int i = 0; i < 8; ++i) m.txid |= static_cast<std::uint64_t>(data[24 + i]) << (8 * i);
  const auto observed = get_address(data + kObservedAt);
  if (!observed) return std::nullopt;
  m.observed = *observed;
  for (std::size_t i = 0; i < n; ++i) {
    const auto c = get_address(data + kCandidatesAt + kAddressSize * i);
    if (!c || !c->valid()) return std::nullopt;
    m.candidates.push_back(*c);
  }
  return m;
}

// ---------------------------------------------------------------- server

RendezvousServer::RendezvousServer(Nanos expiry, std::size_t capacity)
    : expiry_(expiry), capacity_(capacity > 0 ? capacity : 1) {}

RendezvousMessage RendezvousServer::peer_message(const RendezvousId& id, const End& to,
                                                 const End& peer, RendezvousRole peer_role) {
  RendezvousMessage m;
  m.type = RendezvousMessage::Type::kPeer;
  m.role = peer_role;
  m.id = id;
  m.txid = to.txid;
  m.observed = to.from;
  m.candidates.push_back(peer.from);
  for (const auto& h : peer.host) {
    if (m.candidates.size() == kRendezvousMaxCandidates) break;
    if (h != peer.from) m.candidates.push_back(h);
  }
  return m;
}

bool RendezvousServer::on_datagram(const std::uint8_t* data, std::size_t size,
                                   const Endpoint& from, Nanos now, std::vector<Out>& out) {
  const auto m = decode_rendezvous(data, size);
  if (!m || m->type != RendezvousMessage::Type::kRegister || !from.valid()) return false;

  auto it = meetings_.find(m->id);
  if (it == meetings_.end()) {
    if (meetings_.size() >= capacity_) {
      auto oldest = std::min_element(
          meetings_.begin(), meetings_.end(),
          [](const auto& a, const auto& b) { return a.second.seen < b.second.seen; });
      meetings_.erase(oldest);
    }
    it = meetings_.emplace(m->id, Meeting{}).first;
  }
  Meeting& mt = it->second;
  mt.seen = now;
  End& me = mt.ends[index_of(m->role)];
  End& peer = mt.ends[index_of(other(m->role))];
  if (peer.present && now - peer.seen > expiry_) peer = End{};

  const Endpoint source = from.canonical();
  std::vector<Endpoint> host;
  for (const auto& c : m->candidates) {
    const Endpoint h = c.canonical();
    if (std::find(host.begin(), host.end(), h) == host.end()) host.push_back(h);
  }
  const bool changed = !me.present || me.from != source || me.host != host;
  me.present = true;
  me.from = source;
  me.host = std::move(host);
  me.txid = m->txid;
  me.seen = now;

  if (peer.present) {
    out.push_back(Out{from, encode_rendezvous(peer_message(m->id, me, peer, other(m->role)))});
    if (changed) {
      out.push_back(Out{peer.from, encode_rendezvous(peer_message(m->id, peer, me, m->role))});
    }
  } else {
    RendezvousMessage w;
    w.type = RendezvousMessage::Type::kWaiting;
    w.role = m->role;
    w.id = m->id;
    w.txid = m->txid;
    w.observed = source;
    out.push_back(Out{from, encode_rendezvous(w)});
  }
  return changed;
}

// ---------------------------------------------------------------- client

RendezvousClient::RendezvousClient(const Endpoint& server, const RendezvousId& id,
                                   RendezvousRole role, std::vector<Endpoint> host)
    : server_(server.canonical()), id_(id), role_(role), txid_(random_txid()) {
  for (const auto& h : host) host_.push_back(h.canonical());
}

std::optional<std::vector<std::uint8_t>> RendezvousClient::poll(Nanos now) {
  if (started_ && now < next_) return std::nullopt;
  started_ = true;
  next_ = now + (answered_ ? kRefreshInterval : kRetryInterval);
  RendezvousMessage m;
  m.type = RendezvousMessage::Type::kRegister;
  m.role = role_;
  m.id = id_;
  m.txid = txid_;
  m.candidates = host_;
  return encode_rendezvous(m);
}

bool RendezvousClient::on_datagram(const std::uint8_t* data, std::size_t size,
                                   const Endpoint& from, Nanos now) {
  if (from.canonical() != server_) return false;
  const auto m = decode_rendezvous(data, size);
  if (!m || m->type == RendezvousMessage::Type::kRegister || m->id != id_ || m->txid != txid_) {
    return false;
  }
  if (m->type == RendezvousMessage::Type::kPeer && m->role == role_) return false;
  if (m->type == RendezvousMessage::Type::kWaiting && m->role != role_) return false;
  if (!answered_) next_ = now + kRefreshInterval;
  answered_ = true;
  observed_ = m->observed;
  if (m->type == RendezvousMessage::Type::kPeer && m->candidates != peer_) {
    peer_ = m->candidates;
    ++peer_changes_;
  }
  return true;
}

std::vector<Endpoint> sendable(const std::vector<Endpoint>& candidates, const UdpSocket& socket) {
  std::vector<Endpoint> out;
  for (const auto& c : candidates) {
    const auto e = c.for_family(socket.is_ipv6());
    if (e && std::find(out.begin(), out.end(), *e) == out.end()) out.push_back(*e);
  }
  return out;
}

void run_rendezvous_server(UdpSocket& socket, const std::atomic<bool>* stop,
                           const std::function<void(const std::string&)>& log, Nanos expiry) {
  RendezvousServer server(expiry);
  std::vector<std::uint8_t> buf(kRendezvousMessageSize + 1);
  std::vector<RendezvousServer::Out> out;
  while (!(stop && stop->load())) {
    socket.wait_readable(100 * kNanosPerMilli);
    for (;;) {
      Endpoint from;
      const auto n = socket.recv_from(buf.data(), buf.size(), &from);
      if (!n) break;
      out.clear();
      const bool changed = server.on_datagram(buf.data(), *n, from, monotonic_now(), out);
      for (const auto& o : out) socket.send_to(o.to, o.bytes.data(), o.bytes.size());
      if (changed && log) {
        const auto m = decode_rendezvous(buf.data(), *n);
        const auto reply = decode_rendezvous(out[0].bytes.data(), out[0].bytes.size());
        const bool met = reply && reply->type == RendezvousMessage::Type::kPeer;
        log(std::string(m->role == RendezvousRole::kMaster ? "master" : "slave") + " at " +
            from.canonical().to_string() + " with " + std::to_string(m->candidates.size()) +
            " host addresses: " + (met ? "peer announced" : "waiting for its peer"));
      }
    }
  }
}

}  // namespace tuushin::net

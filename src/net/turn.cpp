#include "tuushin/net/turn.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

namespace tuushin::net {

namespace {

constexpr int kMaxRestarts = 4;  // of one request, after 401 or 438
constexpr std::uint32_t kUdp = 17;

void put16(std::uint8_t* p, std::size_t v) {
  p[0] = static_cast<std::uint8_t>(v >> 8);
  p[1] = static_cast<std::uint8_t>(v);
}

std::uint16_t get16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

// "437 Allocation Mismatch", from ERROR-CODE, for logs.
// The code and the reason phrase, for logs. The phrase is the server's
// text: it ends at a NUL (coturn counts its terminator in the length), and
// control characters, which could rewrite a terminal, become '?'.
std::string error_text(const stun::Message& m) {
  const auto code = m.error_code();
  if (!code) return "error response without ERROR-CODE";
  std::string s = std::to_string(*code);
  std::size_t n = 0;
  const std::uint8_t* p = m.find(stun::attr::kErrorCode, &n);
  if (!p || n <= 4) return s;
  s += ' ';
  for (std::size_t i = 4; i < n && p[i] != 0; ++i) {
    s += p[i] < 0x20 || p[i] == 0x7f ? '?' : static_cast<char>(p[i]);
  }
  return s;
}

}  // namespace

TurnClient::TurnClient(const Endpoint& server, std::string username, std::string password,
                       Transport transport)
    : server_(server.canonical()),
      transport_(transport),
      username_(std::move(username)),
      password_(std::move(password)),
      state_(transport == Transport::kStream ? State::kDisconnected : State::kStart) {}

bool TurnClient::wants_connection(Nanos now) const {
  return state_ == State::kDisconnected && now >= connect_at_;
}

bool TurnClient::needs_connection() const {
  return transport_ == Transport::kUdp ||
         !(state_ == State::kDisconnected || state_ == State::kFailed ||
           state_ == State::kReleased);
}

void TurnClient::connecting(Nanos now) {
  if (state_ != State::kDisconnected) return;
  state_ = State::kConnecting;
  connect_by_ = now + kConnectTimeout;
}

void TurnClient::connected(Nanos) {
  if (state_ == State::kConnecting) state_ = State::kStart;
}

void TurnClient::disconnected(const std::string& why, Nanos now) {
  if (transport_ != Transport::kStream) return;
  switch (state_) {
    case State::kDisconnected:
    case State::kFailed:
    case State::kReleased:
      return;
    case State::kReleasing:
      transactions_.clear();
      state_ = State::kReleased;
      return;
    default:
      break;
  }
  // A connection that had its allocation is opened again at once, but not
  // twice within kRetryDelay: a server that drops every connection soon
  // after the allocation is not hammered.
  const bool at_once = state_ == State::kAllocated &&
                       (!reconnected_ || now - reconnected_at_ >= kRetryDelay);
  lose(why);
  state_ = State::kDisconnected;
  connect_at_ = now + (at_once ? 0 : kRetryDelay);
  if (at_once) {
    reconnected_ = true;
    reconnected_at_ = now;
  }
}

bool TurnClient::has(Kind kind) const {
  return std::any_of(transactions_.begin(), transactions_.end(),
                     [&](const Transaction& t) { return t.kind == kind; });
}

TurnClient::Permission* TurnClient::find_permission(const Endpoint& peer) {
  const auto ip = peer.address();
  for (auto& p : permissions_) {
    if (p.ip == ip) return &p;
  }
  return nullptr;
}

const TurnClient::Permission* TurnClient::find_permission(const Endpoint& peer) const {
  return const_cast<TurnClient*>(this)->find_permission(peer);
}

TurnClient::Channel* TurnClient::find_channel(const Endpoint& peer) {
  const Endpoint p = peer.canonical();
  for (auto& c : channels_) {
    if (c.peer == p) return &c;
  }
  return nullptr;
}

const TurnClient::Channel* TurnClient::find_channel(const Endpoint& peer) const {
  return const_cast<TurnClient*>(this)->find_channel(peer);
}

bool TurnClient::permitted(const Endpoint& peer) const {
  const Permission* p = find_permission(peer);
  return p && p->confirmed;
}

bool TurnClient::channel_bound(const Endpoint& peer) const {
  const Channel* c = find_channel(peer);
  return c && c->confirmed;
}

void TurnClient::build(Transaction& t) {
  static constexpr std::uint16_t kMethods[] = {stun::method::kAllocate, stun::method::kRefresh,
                                               stun::method::kCreatePermission,
                                               stun::method::kChannelBind};
  t.tid = stun::random_transaction_id();
  stun::Builder b(kMethods[static_cast<int>(t.kind)], stun::Class::kRequest, t.tid);
  switch (t.kind) {
    case Kind::kAllocate:
      b.add_u32(stun::attr::kRequestedTransport, kUdp << 24);
      break;
    case Kind::kRefresh:
      if (t.lifetime != kNoLifetime) b.add_u32(stun::attr::kLifetime, t.lifetime);
      break;
    case Kind::kCreatePermission:
      b.add_address(stun::attr::kXorPeerAddress, t.peer);
      break;
    case Kind::kChannelBind: {
      std::uint8_t v[4] = {};
      put16(v, t.channel);
      b.add(stun::attr::kChannelNumber, v, sizeof v);
      b.add_address(stun::attr::kXorPeerAddress, t.peer);
      break;
    }
  }
  if (t.kind == Kind::kAllocate || t.kind == Kind::kRefresh) {
    b.add_string(stun::attr::kSoftware, "tuushin");
  }
  t.authenticated = have_credentials_;
  if (have_credentials_) {
    b.add_string(stun::attr::kUsername, username_);
    b.add_string(stun::attr::kRealm, realm_);
    b.add_string(stun::attr::kNonce, nonce_);
    b.add_integrity(key_.data(), key_.size());
  }
  b.add_fingerprint();
  t.bytes = b.take();
}

void TurnClient::start(Kind kind, const Endpoint& peer, std::uint16_t channel,
                       std::uint32_t lifetime) {
  Transaction t;
  t.kind = kind;
  t.peer = peer;
  t.channel = channel;
  t.lifetime = lifetime;
  build(t);
  transactions_.push_back(std::move(t));
}

void TurnClient::restart(Transaction t) {
  // A new transaction (RFC 8489, 9.2.5): a new ID and the new nonce.
  ++t.restarts;
  t.sent = 0;
  t.due = 0;
  build(t);
  transactions_.push_back(std::move(t));
}

std::vector<std::uint8_t> TurnClient::transmit(Transaction& t, Nanos now) {
  if (transport_ == Transport::kStream) {
    // Once: the connection delivers it, or fails (RFC 8489, 6.2.2).
    t.sent = kMaxTransmissions;
    t.due = now + kStreamTimeout;
    return t.bytes;
  }
  ++t.sent;
  // 0.5, 1, 2, 4, 8, 16 s between transmissions, then 16 times the first
  // before giving up (RFC 8489, 6.2.1: Rc = 7, Rm = 16).
  t.due = now + (t.sent < kMaxTransmissions ? kInitialRto << (t.sent - 1) : 16 * kInitialRto);
  return t.bytes;
}

std::optional<std::vector<std::uint8_t>> TurnClient::poll(Nanos now) {
  if (state_ == State::kFailed || state_ == State::kReleased || state_ == State::kDisconnected) {
    return std::nullopt;
  }
  if (state_ == State::kConnecting) {
    if (now >= connect_by_) disconnected("the connection did not open in time", now);
    return std::nullopt;
  }

  // Transactions that ran out of transmissions.
  for (std::size_t i = 0; i < transactions_.size();) {
    if (transactions_[i].sent >= kMaxTransmissions && transactions_[i].due <= now) {
      Transaction t = std::move(transactions_[i]);
      transactions_.erase(transactions_.begin() + static_cast<std::ptrdiff_t>(i));
      on_timeout(t, now);
      i = 0;  // on_timeout may have changed the list
    } else {
      ++i;
    }
  }
  if (state_ == State::kFailed || state_ == State::kReleased || state_ == State::kDisconnected) {
    return std::nullopt;
  }

  // New transactions.
  if (state_ == State::kStart || (state_ == State::kWaiting && now >= retry_at_)) {
    state_ = State::kAllocating;
    start(Kind::kAllocate, {}, 0, 0);
  }
  if (state_ == State::kAllocated) {
    if (!has(Kind::kRefresh) && now >= refresh_at_) start(Kind::kRefresh, {}, 0, kNoLifetime);
    for (auto& p : permissions_) {
      if (!p.refused && !p.in_flight && now >= p.due) {
        p.in_flight = true;
        start(Kind::kCreatePermission, p.peer, 0, 0);
      }
    }
    for (auto& c : channels_) {
      if (!c.refused && !c.in_flight && now >= c.due) {
        c.in_flight = true;
        start(Kind::kChannelBind, c.peer, c.number, 0);
      }
    }
  }

  for (auto& t : transactions_) {
    if (t.due <= now) return transmit(t, now);
  }
  return std::nullopt;
}

Nanos TurnClient::next_due() const {
  constexpr Nanos kNever = std::numeric_limits<Nanos>::max();
  if (state_ == State::kFailed || state_ == State::kReleased) return kNever;
  if (state_ == State::kDisconnected) return connect_at_;
  if (state_ == State::kConnecting) return connect_by_;
  if (state_ == State::kStart) return 0;
  Nanos due = kNever;
  for (const auto& t : transactions_) due = std::min(due, t.due);
  if (state_ == State::kWaiting) due = std::min(due, retry_at_);
  if (state_ == State::kAllocated) {
    if (!has(Kind::kRefresh)) due = std::min(due, refresh_at_);
    for (const auto& p : permissions_) {
      if (!p.refused && !p.in_flight) due = std::min(due, p.due);
    }
    for (const auto& c : channels_) {
      if (!c.refused && !c.in_flight) due = std::min(due, c.due);
    }
  }
  return due;
}

TurnClient::Rx TurnClient::on_datagram(const std::uint8_t* data, std::size_t size,
                                       const Endpoint& from, Nanos now, Endpoint* peer,
                                       std::size_t* offset, std::size_t* length) {
  if (from.canonical() != server_) return Rx::kNotTurn;

  // ChannelData: the channel number (0x4000..0x4FFF), the length, the data.
  if (size >= 4 && (data[0] & 0xf0) == 0x40) {
    const std::uint16_t number = get16(data);
    const std::size_t n = get16(data + 2);
    if (n > size - 4) return Rx::kControl;
    for (const auto& c : channels_) {
      // A channel is usable on receipt as soon as it was asked for: the
      // server may send on it before its answer arrives (RFC 8656, 12.6).
      if (c.number == number && !c.refused && (c.in_flight || c.confirmed)) {
        *peer = c.peer;
        *offset = 4;
        *length = n;
        return Rx::kData;
      }
    }
    return Rx::kControl;
  }

  const auto m = stun::Message::parse(data, size);
  if (!m || !m->fingerprint_ok()) return Rx::kControl;
  if (m->cls() == stun::Class::kIndication) {
    if (m->method() != stun::method::kData) return Rx::kControl;
    const auto from_peer = m->address(stun::attr::kXorPeerAddress);
    std::size_t n = 0;
    const std::uint8_t* payload = m->find(stun::attr::kData, &n);
    // Only from an address this end gave a permission (RFC 8656, 11.6).
    if (!from_peer || !payload) return Rx::kControl;
    const Permission* p = find_permission(*from_peer);
    if (!p || p->refused) return Rx::kControl;
    *peer = from_peer->canonical();
    // The message is a copy from data[0]: offsets carry over.
    *offset = static_cast<std::size_t>(payload - m->data());
    *length = n;
    return Rx::kData;
  }
  if (m->cls() == stun::Class::kSuccess || m->cls() == stun::Class::kError) {
    for (std::size_t i = 0; i < transactions_.size(); ++i) {
      if (transactions_[i].tid == m->transaction()) {
        on_response(i, *m, now);
        break;
      }
    }
  }
  return Rx::kControl;
}

void TurnClient::on_response(std::size_t index, const stun::Message& m, Nanos now) {
  const Transaction& t = transactions_[index];
  const auto code = m.error_code();
  if (m.has(stun::attr::kMessageIntegrity)) {
    if (!have_credentials_ || !m.check_integrity(key_.data(), key_.size())) return;
  } else if (t.authenticated &&
             !(m.cls() == stun::Class::kError && code && (*code == 401 || *code == 438))) {
    // Dropped as if lost; the request is repeated (RFC 8489, 9.2.5).
    return;
  }
  Transaction done = std::move(transactions_[index]);
  transactions_.erase(transactions_.begin() + static_cast<std::ptrdiff_t>(index));
  if (m.cls() == stun::Class::kSuccess) {
    on_success(done, m, now);
  } else {
    on_error(std::move(done), m, now);
  }
}

void TurnClient::on_success(const Transaction& t, const stun::Message& m, Nanos now) {
  switch (t.kind) {
    case Kind::kAllocate: {
      const auto relayed = m.address(stun::attr::kXorRelayedAddress);
      if (!relayed) {
        fail("Allocate answered without XOR-RELAYED-ADDRESS");
        return;
      }
      relayed_ = relayed->canonical();
      const auto mapped = m.address(stun::attr::kXorMappedAddress);
      mapped_ = mapped ? mapped->canonical() : Endpoint{};
      refresh_interval_ = kKeepalive;
      if (const auto lifetime = m.u32(stun::attr::kLifetime); lifetime && *lifetime > 0) {
        refresh_interval_ = std::min(kKeepalive, Nanos{*lifetime} * kNanosPerSecond / 2);
      }
      refresh_at_ = now + refresh_interval_;
      state_ = State::kAllocated;
      ++allocations_;
      break;
    }
    case Kind::kRefresh:
      if (state_ == State::kReleasing) {
        state_ = State::kReleased;
        return;
      }
      refresh_at_ = now + refresh_interval_;
      break;
    case Kind::kCreatePermission:
      if (Permission* p = find_permission(t.peer)) {
        p->in_flight = false;
        p->confirmed = true;
        p->due = now + kPermissionRefresh;
      }
      break;
    case Kind::kChannelBind:
      if (Channel* c = find_channel(t.peer)) {
        c->in_flight = false;
        c->confirmed = true;
        c->due = now + kChannelRefresh;
      }
      break;
  }
}

void TurnClient::on_error(Transaction t, const stun::Message& m, Nanos now) {
  const int code = m.error_code().value_or(0);
  const auto realm = m.string(stun::attr::kRealm);
  const auto nonce = m.string(stun::attr::kNonce);

  if (code == 401) {
    // The challenge to a request without credentials; to one with them, a
    // retry is allowed only with credentials that changed (RFC 8489,
    // 9.2.5), which here can only be the realm.
    if (realm && nonce && t.restarts < kMaxRestarts && (!t.authenticated || *realm != realm_)) {
      realm_ = *realm;
      nonce_ = *nonce;
      key_ = stun::long_term_key(username_, realm_, password_);
      have_credentials_ = true;
      restart(std::move(t));
      return;
    }
    fail(error_text(m));
    return;
  }
  if (code == 438) {
    if (nonce && t.restarts < kMaxRestarts) {
      nonce_ = *nonce;
      if (realm && *realm != realm_) {
        realm_ = *realm;
        key_ = stun::long_term_key(username_, realm_, password_);
      }
      restart(std::move(t));
    } else {
      on_timeout(t, now);
    }
    return;
  }

  switch (t.kind) {
    case Kind::kAllocate:
      // 437: this address still holds an allocation, until it expires;
      // 486, 508: the server is short of something for now.
      if (code == 437 || code == 486 || code == 508) {
        last_error_ = error_text(m);
        state_ = State::kWaiting;
        retry_at_ = now + kRetryDelay;
      } else {
        fail(error_text(m));
      }
      return;
    case Kind::kRefresh:
      if (state_ == State::kReleasing) {
        state_ = State::kReleased;
      } else if (code == 437) {
        lose(error_text(m));
      } else {
        refresh_at_ = now + refresh_interval_;
      }
      return;
    case Kind::kCreatePermission:
      if (code == 437) {
        lose(error_text(m));
      } else if (Permission* p = find_permission(t.peer)) {
        p->in_flight = false;
        p->refused = true;
        for (auto& c : channels_) {
          if (c.peer.address() == p->ip) c.refused = true;
        }
      }
      return;
    case Kind::kChannelBind:
      if (code == 437) {
        lose(error_text(m));
      } else if (Channel* c = find_channel(t.peer)) {
        c->in_flight = false;
        c->refused = true;
      }
      return;
  }
}

void TurnClient::on_timeout(const Transaction& t, Nanos now) {
  if (transport_ == Transport::kStream && state_ != State::kReleasing) {
    // The connection delivered nothing back: it is dead or stalled, and
    // takes the allocation with it either way.
    disconnected("no answer from the server over the connection", now);
    return;
  }
  switch (t.kind) {
    case Kind::kAllocate:
      last_error_ = "no answer from the server";
      state_ = State::kWaiting;
      retry_at_ = now + kRetryDelay;
      break;
    case Kind::kRefresh:
      // The path to the server may be down, or this end's NAT may have
      // moved it; either way the next Refresh tells.
      if (state_ == State::kReleasing) {
        state_ = State::kReleased;
      } else {
        refresh_at_ = now;
      }
      break;
    case Kind::kCreatePermission:
      if (Permission* p = find_permission(t.peer)) {
        p->in_flight = false;
        p->due = now;
      }
      break;
    case Kind::kChannelBind:
      if (Channel* c = find_channel(t.peer)) {
        c->in_flight = false;
        c->due = now;
      }
      break;
  }
}

void TurnClient::lose(const std::string& why) {
  // The server no longer knows this end's allocation, nor its permissions
  // and channels; allocate again and ask for them again.
  last_error_ = why;
  transactions_.clear();
  state_ = State::kStart;
  for (auto& p : permissions_) p = Permission{p.ip, p.peer};
  for (auto& c : channels_) c = Channel{c.peer, c.number};
}

void TurnClient::fail(const std::string& why) {
  last_error_ = why;
  transactions_.clear();
  state_ = State::kFailed;
}

void TurnClient::check(Nanos now) {
  if (state_ == State::kAllocated) refresh_at_ = std::min(refresh_at_, now);
  if (state_ == State::kWaiting) retry_at_ = std::min(retry_at_, now);
}

void TurnClient::release() {
  if (state_ == State::kFailed || state_ == State::kReleased) return;
  transactions_.clear();
  if (state_ == State::kAllocated) {
    state_ = State::kReleasing;
    start(Kind::kRefresh, {}, 0, 0);
  } else {
    state_ = State::kReleased;
  }
}

void TurnClient::permit(const Endpoint& peer) {
  const Endpoint p = peer.canonical();
  if (!p.valid() || find_permission(p)) return;
  Permission perm;
  perm.ip = p.address();
  perm.peer = p;
  permissions_.push_back(std::move(perm));
}

void TurnClient::bind_channel(const Endpoint& peer) {
  const Endpoint p = peer.canonical();
  if (!p.valid()) return;
  permit(p);
  if (find_channel(p) || next_channel_ > 0x4fff) return;
  Channel c;
  c.peer = p;
  c.number = next_channel_++;
  c.refused = find_permission(p)->refused;
  channels_.push_back(c);
}

bool TurnClient::wrap(const Endpoint& peer, const std::uint8_t* data, std::size_t size,
                      std::vector<std::uint8_t>& out) {
  out.clear();
  if (state_ != State::kAllocated || size > 0xffff) return false;
  const Endpoint p = peer.canonical();
  bind_channel(p);
  const Permission* perm = find_permission(p);
  if (!perm || perm->refused) return false;
  const Channel* c = find_channel(p);
  if (c && c->confirmed && !c->refused) {
    // Padded to a multiple of 4 bytes over a stream, where the length that
    // follows the padding is how the server finds the next message; over
    // UDP padding is optional and left out (RFC 8656, 12.5).
    const std::size_t padded = transport_ == Transport::kStream ? (size + 3) & ~std::size_t{3} : size;
    out.assign(4 + padded, 0);
    put16(out.data(), c->number);
    put16(out.data() + 2, size);
    if (size > 0) std::memcpy(out.data() + 4, data, size);
    return true;
  }
  stun::Builder b(stun::method::kSend, stun::Class::kIndication, stun::random_transaction_id());
  b.add_address(stun::attr::kXorPeerAddress, p);
  b.add(stun::attr::kData, data, size);
  out = b.take();
  return true;
}

}  // namespace tuushin::net

#include "tuushin/net/stun.hpp"

#include <sodium.h>

#include <algorithm>
#include <cstring>

#include "tuushin/crypto/noise.hpp"

namespace tuushin::net::stun {

namespace {

constexpr std::uint32_t kFingerprintXor = 0x5354554e;  // "STUN"
constexpr std::size_t kIntegritySize = 20;            // HMAC-SHA1

std::uint16_t get16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}
std::uint32_t get32(const std::uint8_t* p) {
  return (std::uint32_t{p[0]} << 24) | (std::uint32_t{p[1]} << 16) | (std::uint32_t{p[2]} << 8) |
         std::uint32_t{p[3]};
}
void put16(std::uint8_t* p, std::uint16_t v) {
  p[0] = static_cast<std::uint8_t>(v >> 8);
  p[1] = static_cast<std::uint8_t>(v);
}
void put32(std::uint8_t* p, std::uint32_t v) {
  p[0] = static_cast<std::uint8_t>(v >> 24);
  p[1] = static_cast<std::uint8_t>(v >> 16);
  p[2] = static_cast<std::uint8_t>(v >> 8);
  p[3] = static_cast<std::uint8_t>(v);
}

bool is_xor_address(std::uint16_t type) {
  return type == attr::kXorMappedAddress || type == attr::kXorPeerAddress ||
         type == attr::kXorRelayedAddress;
}

// The 16 bytes an address is XORed with: the magic cookie, then the
// transaction ID (only IPv6 uses those).
std::array<std::uint8_t, 16> xor_pad(const TransactionId& tid) {
  std::array<std::uint8_t, 16> pad{};
  put32(pad.data(), kMagicCookie);
  std::memcpy(pad.data() + 4, tid.data(), tid.size());
  return pad;
}

}  // namespace

TransactionId random_transaction_id() {
  crypto::init();
  TransactionId t;
  randombytes_buf(t.data(), t.size());
  return t;
}

crypto::Md5Digest long_term_key(const std::string& username, const std::string& realm,
                                const std::string& password) {
  const std::string s = username + ":" + realm + ":" + password;
  return crypto::md5(reinterpret_cast<const std::uint8_t*>(s.data()), s.size());
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
  static const auto table = [] {
    std::array<std::uint32_t, 256> t{};
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
      t[i] = c;
    }
    return t;
  }();
  std::uint32_t c = 0xffffffffu;
  for (std::size_t i = 0; i < size; ++i) c = table[(c ^ data[i]) & 0xff] ^ (c >> 8);
  return c ^ 0xffffffffu;
}

bool looks_like_stun(const std::uint8_t* data, std::size_t size) {
  if (size < kHeaderSize || (data[0] & 0xc0) != 0) return false;
  if (get32(data + 4) != kMagicCookie) return false;
  const std::size_t length = get16(data + 2);
  return length % 4 == 0 && kHeaderSize + length <= size;
}

Builder::Builder(std::uint16_t method, Class cls, const TransactionId& tid) : bytes_(kHeaderSize) {
  const unsigned c = static_cast<unsigned>(cls);
  // The class bits sit at 4 (C0) and 8 (C1) of the method's 12 bits.
  const unsigned type = (method & 0x000f) | ((method & 0x0070) << 1) | ((method & 0x0f80) << 2) |
                        ((c & 1) << 4) | ((c & 2) << 7);
  put16(bytes_.data(), static_cast<std::uint16_t>(type));
  put32(bytes_.data() + 4, kMagicCookie);
  std::memcpy(bytes_.data() + 8, tid.data(), tid.size());
}

void Builder::set_length(std::size_t body) { put16(bytes_.data() + 2, static_cast<std::uint16_t>(body)); }

void Builder::add(std::uint16_t type, const std::uint8_t* value, std::size_t size) {
  const std::size_t at = bytes_.size();
  const std::size_t padded = (size + 3) & ~std::size_t{3};
  bytes_.resize(at + 4 + padded, 0);
  put16(bytes_.data() + at, type);
  put16(bytes_.data() + at + 2, static_cast<std::uint16_t>(size));
  if (size > 0) std::memcpy(bytes_.data() + at + 4, value, size);
  set_length(bytes_.size() - kHeaderSize);
}

void Builder::add_string(std::uint16_t type, const std::string& value) {
  add(type, reinterpret_cast<const std::uint8_t*>(value.data()), value.size());
}

void Builder::add_u32(std::uint16_t type, std::uint32_t value) {
  std::uint8_t v[4];
  put32(v, value);
  add(type, v, sizeof v);
}

void Builder::add_address(std::uint16_t type, const Endpoint& e) {
  const std::vector<std::uint8_t> a = e.address();
  std::uint8_t v[20] = {};
  v[1] = a.size() == 4 ? 0x01 : 0x02;
  std::uint16_t port = e.port();
  std::memcpy(v + 4, a.data(), a.size());
  if (is_xor_address(type)) {
    TransactionId tid;
    std::memcpy(tid.data(), bytes_.data() + 8, tid.size());
    const auto pad = xor_pad(tid);
    port ^= static_cast<std::uint16_t>(kMagicCookie >> 16);
    for (std::size_t i = 0; i < a.size(); ++i) v[4 + i] ^= pad[i];
  }
  put16(v + 2, port);
  add(type, v, 4 + a.size());
}

void Builder::add_error(int code, const std::string& reason) {
  std::vector<std::uint8_t> v(4 + reason.size(), 0);
  v[2] = static_cast<std::uint8_t>(code / 100);
  v[3] = static_cast<std::uint8_t>(code % 100);
  std::memcpy(v.data() + 4, reason.data(), reason.size());
  add(attr::kErrorCode, v.data(), v.size());
}

void Builder::add_integrity(const std::uint8_t* key, std::size_t size) {
  const std::size_t covered = bytes_.size();
  // The length in the header counts the attribute about to be added.
  set_length(covered - kHeaderSize + 4 + kIntegritySize);
  const crypto::Sha1Digest mac = crypto::hmac_sha1(key, size, bytes_.data(), covered);
  add(attr::kMessageIntegrity, mac.data(), mac.size());
}

void Builder::add_fingerprint() {
  const std::size_t covered = bytes_.size();
  set_length(covered - kHeaderSize + 8);
  std::uint8_t v[4];
  put32(v, crc32(bytes_.data(), covered) ^ kFingerprintXor);
  add(attr::kFingerprint, v, sizeof v);
}

std::optional<Message> Message::parse(const std::uint8_t* data, std::size_t size) {
  if (!looks_like_stun(data, size)) return std::nullopt;
  const std::size_t length = get16(data + 2);
  Message m;
  m.bytes_.assign(data, data + kHeaderSize + length);
  const unsigned type = get16(data);
  m.method_ = static_cast<std::uint16_t>((type & 0x000f) | ((type & 0x00e0) >> 1) |
                                         ((type & 0x3e00) >> 2));
  m.cls_ = static_cast<Class>(((type >> 4) & 1) | ((type >> 7) & 2));
  std::memcpy(m.tid_.data(), data + 8, m.tid_.size());

  bool after_integrity = false;
  std::size_t at = kHeaderSize;
  while (at < m.bytes_.size()) {
    if (m.bytes_.size() - at < 4) return std::nullopt;
    const std::uint16_t t = get16(m.bytes_.data() + at);
    const std::uint16_t n = get16(m.bytes_.data() + at + 2);
    const std::size_t padded = (std::size_t{n} + 3) & ~std::size_t{3};
    if (m.bytes_.size() - at - 4 < padded) return std::nullopt;
    if (t == attr::kFingerprint) {
      // The last attribute, over everything before it.
      if (n != 4 || at + 8 != m.bytes_.size()) return std::nullopt;
      const std::uint32_t want = crc32(m.bytes_.data(), at) ^ kFingerprintXor;
      m.fingerprint_ok_ = get32(m.bytes_.data() + at + 4) == want;
      m.attrs_.push_back({t, at + 4, n});
    } else if (t == attr::kMessageIntegrity) {
      if (n != kIntegritySize) return std::nullopt;
      if (!after_integrity) {
        m.integrity_at_ = at;
        m.attrs_.push_back({t, at + 4, n});
      }
      after_integrity = true;
    } else if (!after_integrity) {
      m.attrs_.push_back({t, at + 4, n});
    }
    at += 4 + padded;
  }
  return m;
}

const std::uint8_t* Message::find(std::uint16_t type, std::size_t* size) const {
  for (const auto& a : attrs_) {
    if (a.type == type) {
      if (size) *size = a.size;
      return bytes_.data() + a.offset;
    }
  }
  return nullptr;
}

bool Message::has(std::uint16_t type) const { return find(type, nullptr) != nullptr; }

std::optional<std::string> Message::string(std::uint16_t type) const {
  std::size_t n = 0;
  const std::uint8_t* p = find(type, &n);
  if (!p) return std::nullopt;
  return std::string(reinterpret_cast<const char*>(p), n);
}

std::optional<std::uint32_t> Message::u32(std::uint16_t type) const {
  std::size_t n = 0;
  const std::uint8_t* p = find(type, &n);
  if (!p || n != 4) return std::nullopt;
  return get32(p);
}

std::optional<Endpoint> Message::address(std::uint16_t type) const {
  std::size_t n = 0;
  const std::uint8_t* p = find(type, &n);
  if (!p || n < 4) return std::nullopt;
  const std::size_t len = p[1] == 0x01 ? 4 : p[1] == 0x02 ? 16 : 0;
  if (len == 0 || n != 4 + len) return std::nullopt;
  std::uint8_t a[16];
  std::memcpy(a, p + 4, len);
  std::uint16_t port = get16(p + 2);
  if (is_xor_address(type)) {
    const auto pad = xor_pad(tid_);
    port ^= static_cast<std::uint16_t>(kMagicCookie >> 16);
    for (std::size_t i = 0; i < len; ++i) a[i] ^= pad[i];
  }
  return Endpoint::from_address(a, len, port);
}

std::optional<int> Message::error_code() const {
  std::size_t n = 0;
  const std::uint8_t* p = find(attr::kErrorCode, &n);
  if (!p || n < 4) return std::nullopt;
  const int cls = p[2] & 0x07, number = p[3];
  if (cls < 3 || cls > 6 || number > 99) return std::nullopt;
  return cls * 100 + number;
}

std::vector<std::uint16_t> Message::unknown_required(const std::vector<std::uint16_t>& known) const {
  std::vector<std::uint16_t> out;
  for (const auto& a : attrs_) {
    if (a.type < 0x8000 && std::find(known.begin(), known.end(), a.type) == known.end() &&
        std::find(out.begin(), out.end(), a.type) == out.end()) {
      out.push_back(a.type);
    }
  }
  return out;
}

bool Message::check_integrity(const std::uint8_t* key, std::size_t size) const {
  if (integrity_at_ == 0) return false;
  // The header's length as it was when the MAC was computed: up to the end
  // of MESSAGE-INTEGRITY, without a FINGERPRINT after it.
  std::vector<std::uint8_t> covered(bytes_.begin(),
                                    bytes_.begin() + static_cast<std::ptrdiff_t>(integrity_at_));
  put16(covered.data() + 2, static_cast<std::uint16_t>(integrity_at_ - kHeaderSize + 4 + kIntegritySize));
  const crypto::Sha1Digest mac = crypto::hmac_sha1(key, size, covered.data(), covered.size());
  // Constant time, so that a forger cannot learn the MAC byte by byte.
  return sodium_memcmp(mac.data(), bytes_.data() + integrity_at_ + 4, mac.size()) == 0;
}

}  // namespace tuushin::net::stun

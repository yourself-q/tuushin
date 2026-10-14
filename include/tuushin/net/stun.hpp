#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "tuushin/crypto/legacy_digest.hpp"
#include "tuushin/net/udp.hpp"

namespace tuushin::net::stun {

// STUN messages (RFC 8489), as much of them as a TURN client (RFC 8656)
// needs: the header, the attributes as type-length-value, XOR-encoded
// addresses, MESSAGE-INTEGRITY with long-term credentials and FINGERPRINT.
// Left out: the short-term mechanism's specifics, MESSAGE-INTEGRITY-SHA256
// and PASSWORD-ALGORITHMS (coturn, the server tuushin is tested against,
// accepts only MESSAGE-INTEGRITY), USERHASH, and SASLprep/OpaqueString:
// usernames, realms and passwords are used as the bytes they are given,
// which is what OpaqueString leaves of ASCII.
//
//   0                   1                   2                   3
//   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//  |0 0|     STUN Message Type     |         Message Length        |
//  |                 Magic Cookie (0x2112A442)                     |
//  |                 Transaction ID (96 bits)                      |
//  then attributes: type (16), length (16), value padded to 4 bytes.

constexpr std::uint32_t kMagicCookie = 0x2112A442;
constexpr std::size_t kHeaderSize = 20;

enum class Class : std::uint8_t { kRequest = 0, kIndication = 1, kSuccess = 2, kError = 3 };

namespace method {
constexpr std::uint16_t kBinding = 0x001;
constexpr std::uint16_t kAllocate = 0x003;  // RFC 8656
constexpr std::uint16_t kRefresh = 0x004;
constexpr std::uint16_t kSend = 0x006;
constexpr std::uint16_t kData = 0x007;
constexpr std::uint16_t kCreatePermission = 0x008;
constexpr std::uint16_t kChannelBind = 0x009;
}  // namespace method

namespace attr {
constexpr std::uint16_t kMappedAddress = 0x0001;
constexpr std::uint16_t kUsername = 0x0006;
constexpr std::uint16_t kMessageIntegrity = 0x0008;
constexpr std::uint16_t kErrorCode = 0x0009;
constexpr std::uint16_t kUnknownAttributes = 0x000A;
constexpr std::uint16_t kChannelNumber = 0x000C;  // RFC 8656
constexpr std::uint16_t kLifetime = 0x000D;
constexpr std::uint16_t kXorPeerAddress = 0x0012;
constexpr std::uint16_t kData = 0x0013;
constexpr std::uint16_t kRealm = 0x0014;
constexpr std::uint16_t kNonce = 0x0015;
constexpr std::uint16_t kXorRelayedAddress = 0x0016;
constexpr std::uint16_t kRequestedAddressFamily = 0x0017;
constexpr std::uint16_t kRequestedTransport = 0x0019;
constexpr std::uint16_t kDontFragment = 0x001A;
constexpr std::uint16_t kXorMappedAddress = 0x0020;
constexpr std::uint16_t kSoftware = 0x8022;
constexpr std::uint16_t kFingerprint = 0x8028;
}  // namespace attr

using TransactionId = std::array<std::uint8_t, 12>;

// A transaction ID from a cryptographically secure source, as RFC 8489
// requires: guessing it is all an off-path attacker needs to forge a
// response.
TransactionId random_transaction_id();

// The key of the long-term credential mechanism (RFC 8489, 9.2.2):
// MD5(username ":" realm ":" password).
crypto::Md5Digest long_term_key(const std::string& username, const std::string& realm,
                                const std::string& password);

// True if the bytes look like a STUN message: the two top bits zero, the
// magic cookie, and a length that is a multiple of 4 and fits. ChannelData
// (RFC 8656, 12.4) starts with 0x40..0x4F and so never does.
bool looks_like_stun(const std::uint8_t* data, std::size_t size);

// Builds a message attribute by attribute.
class Builder {
 public:
  Builder(std::uint16_t method, Class cls, const TransactionId& tid);

  void add(std::uint16_t type, const std::uint8_t* value, std::size_t size);
  void add_string(std::uint16_t type, const std::string& value);
  void add_u32(std::uint16_t type, std::uint32_t value);
  // An address attribute; the XOR types (XOR-MAPPED-ADDRESS,
  // XOR-PEER-ADDRESS, XOR-RELAYED-ADDRESS) are encoded XORed with the
  // magic cookie and the transaction ID. An IPv4-mapped IPv6 endpoint is
  // encoded as IPv4.
  void add_address(std::uint16_t type, const Endpoint& e);
  void add_error(int code, const std::string& reason);
  // MESSAGE-INTEGRITY over everything so far, with `key` (for long-term
  // credentials, long_term_key()). Only FINGERPRINT may follow it.
  void add_integrity(const std::uint8_t* key, std::size_t size);
  void add_fingerprint();

  const std::vector<std::uint8_t>& bytes() const { return bytes_; }
  std::vector<std::uint8_t> take() { return std::move(bytes_); }

 private:
  void set_length(std::size_t body);
  std::vector<std::uint8_t> bytes_;
};

// A received message, parsed and checked for structure. Attributes after
// MESSAGE-INTEGRITY other than FINGERPRINT are dropped, as RFC 8489 (6.3)
// requires: they are not covered by the integrity check.
class Message {
 public:
  static std::optional<Message> parse(const std::uint8_t* data, std::size_t size);

  std::uint16_t method() const { return method_; }
  Class cls() const { return cls_; }
  const TransactionId& transaction() const { return tid_; }

  // The first attribute of `type`, if present: its value and length.
  const std::uint8_t* find(std::uint16_t type, std::size_t* size) const;
  bool has(std::uint16_t type) const;
  std::optional<std::string> string(std::uint16_t type) const;
  std::optional<std::uint32_t> u32(std::uint16_t type) const;
  // An address attribute, decoded as Builder::add_address encodes it.
  std::optional<Endpoint> address(std::uint16_t type) const;
  // ERROR-CODE as a number (e.g. 401), if present and well-formed.
  std::optional<int> error_code() const;
  // Comprehension-required attributes (types below 0x8000) that are not in
  // `known`: a request with any of them gets 420 (RFC 8489, 6.3.1).
  std::vector<std::uint16_t> unknown_required(const std::vector<std::uint16_t>& known) const;

  // True if MESSAGE-INTEGRITY is present and matches `key`.
  bool check_integrity(const std::uint8_t* key, std::size_t size) const;
  // True if FINGERPRINT is absent, or present and correct.
  bool fingerprint_ok() const { return fingerprint_ok_; }

 private:
  struct Attr {
    std::uint16_t type;
    std::size_t offset;  // of the value, in bytes_
    std::uint16_t size;
  };
  std::vector<std::uint8_t> bytes_;
  std::uint16_t method_ = 0;
  Class cls_ = Class::kRequest;
  TransactionId tid_{};
  std::vector<Attr> attrs_;
  std::size_t integrity_at_ = 0;  // offset of the MESSAGE-INTEGRITY header; 0 if none
  bool fingerprint_ok_ = true;
};

// CRC-32 (ITU-T V.42, as in RFC 1952), which FINGERPRINT uses.
std::uint32_t crc32(const std::uint8_t* data, std::size_t size);

}  // namespace tuushin::net::stun

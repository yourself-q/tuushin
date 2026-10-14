#include <cstring>
#include <string>
#include <vector>

#include "test.hpp"
#include "tuushin/net/stun.hpp"

using namespace tuushin::net;
namespace stun = tuushin::net::stun;

namespace {

std::vector<std::uint8_t> hex(const char* s) {
  std::vector<std::uint8_t> out;
  int hi = -1;
  for (; *s; ++s) {
    int v;
    if (*s >= '0' && *s <= '9') {
      v = *s - '0';
    } else if (*s >= 'a' && *s <= 'f') {
      v = *s - 'a' + 10;
    } else {
      continue;
    }
    if (hi < 0) {
      hi = v;
    } else {
      out.push_back(static_cast<std::uint8_t>(hi * 16 + v));
      hi = -1;
    }
  }
  return out;
}

const std::uint8_t* bytes(const std::string& s) {
  return reinterpret_cast<const std::uint8_t*>(s.data());
}

// RFC 5769, the test vectors for STUN, exactly as printed there.
const char* kSampleRequest =
    "00 01 00 58 21 12 a4 42 b7 e7 a7 01 bc 34 d6 86 fa 87 df ae"
    "80 22 00 10 53 54 55 4e 20 74 65 73 74 20 63 6c 69 65 6e 74"
    "00 24 00 04 6e 00 01 ff 80 29 00 08 93 2f f9 b1 51 26 3b 36"
    "00 06 00 09 65 76 74 6a 3a 68 36 76 59 20 20 20"
    "00 08 00 14 9a ea a7 0c bf d8 cb 56 78 1e f2 b5 b2 d3 f2 49 c1 b5 71 a2"
    "80 28 00 04 e5 7a 3b cf";
const char* kSampleIpv4Response =
    "01 01 00 3c 21 12 a4 42 b7 e7 a7 01 bc 34 d6 86 fa 87 df ae"
    "80 22 00 0b 74 65 73 74 20 76 65 63 74 6f 72 20"
    "00 20 00 08 00 01 a1 47 e1 12 a6 43"
    "00 08 00 14 2b 91 f5 99 fd 9e 90 c3 8c 74 89 f9 2a f9 ba 53 f0 6b e7 d7"
    "80 28 00 04 c0 7d 4c 96";
const char* kSampleIpv6Response =
    "01 01 00 48 21 12 a4 42 b7 e7 a7 01 bc 34 d6 86 fa 87 df ae"
    "80 22 00 0b 74 65 73 74 20 76 65 63 74 6f 72 20"
    "00 20 00 14 00 02 a1 47 01 13 a9 fa a5 d3 f1 79 bc 25 f4 b5 be d2 b9 d9"
    "00 08 00 14 a3 82 95 4e 4b e6 7b f1 17 84 c9 7c 82 92 c2 75 bf e3 ed 41"
    "80 28 00 04 c8 fb 0b 4c";
const char* kLongTermRequest =
    "00 01 00 60 21 12 a4 42 78 ad 34 33 c6 ad 72 c0 29 da 41 2e"
    "00 06 00 12 e3 83 9e e3 83 88 e3 83 aa e3 83 83 e3 82 af e3 82 b9 00 00"
    "00 15 00 1c 66 2f 2f 34 39 39 6b 39 35 34 64 36 4f 4c 33 34 6f 4c 39 46"
    "53 54 76 79 36 34 73 41"
    "00 14 00 0b 65 78 61 6d 70 6c 65 2e 6f 72 67 00"
    "00 08 00 14 f6 70 24 65 6d d6 4a 3e 02 b8 e0 71 2e 85 c9 a2 8c a8 96 66";

const std::string kShortTermPassword = "VOkJxbRl1RmTxUk/WvJxBt";

}  // namespace

TEST_CASE(stun_parses_the_rfc_5769_request) {
  const auto b = hex(kSampleRequest);
  REQUIRE(b.size() == 108);
  const auto m = stun::Message::parse(b.data(), b.size());
  REQUIRE(m.has_value());
  CHECK(m->method() == stun::method::kBinding);
  CHECK(m->cls() == stun::Class::kRequest);
  CHECK(m->string(stun::attr::kSoftware) == std::string("STUN test client"));
  CHECK(m->string(stun::attr::kUsername) == std::string("evtj:h6vY"));
  CHECK(m->fingerprint_ok());
  // Short-term credentials: the key is the password itself.
  CHECK(m->check_integrity(bytes(kShortTermPassword), kShortTermPassword.size()));
  const std::string wrong = "VOkJxbRl1RmTxUk/WvJxBu";
  CHECK(!m->check_integrity(bytes(wrong), wrong.size()));
  // ICE's PRIORITY (0x0024) is comprehension-required and unknown here.
  const auto unknown = m->unknown_required({stun::attr::kUsername, stun::attr::kMessageIntegrity});
  CHECK(unknown.size() == 1 && unknown[0] == 0x0024);
}

TEST_CASE(stun_decodes_the_rfc_5769_xor_mapped_addresses) {
  const auto b4 = hex(kSampleIpv4Response);
  const auto m4 = stun::Message::parse(b4.data(), b4.size());
  REQUIRE(m4.has_value());
  CHECK(m4->cls() == stun::Class::kSuccess && m4->method() == stun::method::kBinding);
  CHECK(m4->address(stun::attr::kXorMappedAddress) == Endpoint::resolve("192.0.2.1", 32853));
  CHECK(m4->fingerprint_ok());
  CHECK(m4->check_integrity(bytes(kShortTermPassword), kShortTermPassword.size()));

  const auto b6 = hex(kSampleIpv6Response);
  const auto m6 = stun::Message::parse(b6.data(), b6.size());
  REQUIRE(m6.has_value());
  CHECK(m6->address(stun::attr::kXorMappedAddress) ==
        Endpoint::resolve("2001:db8:1234:5678:11:2233:4455:6677", 32853, true));
  CHECK(m6->fingerprint_ok());
  CHECK(m6->check_integrity(bytes(kShortTermPassword), kShortTermPassword.size()));

  // The encoder XORs the same way: re-encoding gives the vector's bytes.
  for (const auto* v : {&b4, &b6}) {
    const auto m = stun::Message::parse(v->data(), v->size());
    stun::Builder b(stun::method::kBinding, stun::Class::kSuccess, m->transaction());
    b.add_address(stun::attr::kXorMappedAddress, *m->address(stun::attr::kXorMappedAddress));
    std::size_t n = 0;
    const std::uint8_t* want = m->find(stun::attr::kXorMappedAddress, &n);
    REQUIRE(b.bytes().size() == stun::kHeaderSize + 4 + n);
    CHECK(std::memcmp(b.bytes().data() + stun::kHeaderSize + 4, want, n) == 0);
  }
}

TEST_CASE(stun_rejects_a_damaged_fingerprint_or_integrity) {
  auto b = hex(kSampleIpv4Response);
  b.back() ^= 1;
  auto m = stun::Message::parse(b.data(), b.size());
  REQUIRE(m.has_value());
  CHECK(!m->fingerprint_ok());
  b = hex(kSampleIpv4Response);
  b[stun::kHeaderSize + 4] ^= 1;  // the SOFTWARE value, covered by both
  m = stun::Message::parse(b.data(), b.size());
  REQUIRE(m.has_value());
  CHECK(!m->fingerprint_ok());
  CHECK(!m->check_integrity(bytes(kShortTermPassword), kShortTermPassword.size()));
}

// RFC 5769, 2.4: long-term credentials, built here byte for byte (its
// padding is zeros, as the builder writes it). The password is given after
// SASLprep, which tuushin does not implement.
TEST_CASE(stun_builds_the_rfc_5769_long_term_request) {
  const auto want = hex(kLongTermRequest);
  const auto parsed = stun::Message::parse(want.data(), want.size());
  REQUIRE(parsed.has_value());
  const std::string username = "\xe3\x83\x9e\xe3\x83\x88\xe3\x83\xaa\xe3\x83\x83\xe3\x82\xaf\xe3\x82\xb9";
  const std::string nonce = "f//499k954d6OL34oL9FSTvy64sA", realm = "example.org";
  CHECK(parsed->string(stun::attr::kUsername) == username);
  const auto key = stun::long_term_key(username, realm, "TheMatrIX");
  CHECK(parsed->check_integrity(key.data(), key.size()));

  stun::Builder b(stun::method::kBinding, stun::Class::kRequest, parsed->transaction());
  b.add_string(stun::attr::kUsername, username);
  b.add_string(stun::attr::kNonce, nonce);
  b.add_string(stun::attr::kRealm, realm);
  b.add_integrity(key.data(), key.size());
  CHECK(b.bytes() == want);
}

// RFC 8489, 9.2.2's example key.
TEST_CASE(stun_long_term_key_is_md5_of_user_realm_pass) {
  const auto k = stun::long_term_key("user", "realm", "pass");
  CHECK(std::vector<std::uint8_t>(k.begin(), k.end()) == hex("8493fbc53ba582fb4c044c456bdc40eb"));
}

// The type field interleaves the class bits with the method's (RFC 8489,
// figure 3), worked out by hand from the figure.
TEST_CASE(stun_encodes_method_and_class_into_the_type) {
  const stun::TransactionId tid{};
  const struct {
    std::uint16_t method;
    stun::Class cls;
    std::uint16_t type;
  } cases[] = {
      {stun::method::kBinding, stun::Class::kRequest, 0x0001},
      {stun::method::kBinding, stun::Class::kSuccess, 0x0101},
      {stun::method::kAllocate, stun::Class::kRequest, 0x0003},
      {stun::method::kAllocate, stun::Class::kError, 0x0113},
      {stun::method::kSend, stun::Class::kIndication, 0x0016},
      {stun::method::kData, stun::Class::kIndication, 0x0017},
      {stun::method::kChannelBind, stun::Class::kSuccess, 0x0109},
      {0x0fff, stun::Class::kError, 0x3fff},
  };
  for (const auto& c : cases) {
    stun::Builder b(c.method, c.cls, tid);
    CHECK(b.bytes()[0] == (c.type >> 8) && b.bytes()[1] == (c.type & 0xff));
    const auto m = stun::Message::parse(b.bytes().data(), b.bytes().size());
    REQUIRE(m.has_value());
    CHECK(m->method() == c.method && m->cls() == c.cls);
  }
}

TEST_CASE(stun_integrity_and_fingerprint_round_trip) {
  const auto tid = stun::random_transaction_id();
  CHECK(tid != stun::random_transaction_id());
  const auto key = stun::long_term_key("alice", "tuushin", "secret");
  stun::Builder b(stun::method::kRefresh, stun::Class::kRequest, tid);
  b.add_u32(stun::attr::kLifetime, 600);
  b.add_string(stun::attr::kUsername, "alice");
  b.add_integrity(key.data(), key.size());
  b.add_fingerprint();
  const auto m = stun::Message::parse(b.bytes().data(), b.bytes().size());
  REQUIRE(m.has_value());
  CHECK(m->transaction() == tid);
  CHECK(m->u32(stun::attr::kLifetime) == 600u);
  CHECK(m->fingerprint_ok());
  CHECK(m->check_integrity(key.data(), key.size()));
  const auto other = stun::long_term_key("alice", "tuushin", "secreT");
  CHECK(!m->check_integrity(other.data(), other.size()));
}

TEST_CASE(stun_ignores_attributes_after_message_integrity) {
  const auto key = stun::long_term_key("u", "r", "p");
  stun::Builder b(stun::method::kAllocate, stun::Class::kSuccess, stun::random_transaction_id());
  b.add_u32(stun::attr::kLifetime, 600);
  b.add_integrity(key.data(), key.size());
  // Appended by someone on the path: not covered, so not believed.
  b.add_address(stun::attr::kXorRelayedAddress, *Endpoint::resolve("198.51.100.7", 4000));
  const auto m = stun::Message::parse(b.bytes().data(), b.bytes().size());
  REQUIRE(m.has_value());
  CHECK(!m->has(stun::attr::kXorRelayedAddress));
  // The length in the header covered the appended attribute; the MAC was
  // computed with it pointing at MESSAGE-INTEGRITY's end, as a check does.
  CHECK(m->check_integrity(key.data(), key.size()));
}

TEST_CASE(stun_error_codes_and_malformed_messages) {
  stun::Builder b(stun::method::kAllocate, stun::Class::kError, stun::random_transaction_id());
  b.add_error(438, "Stale Nonce");
  auto m = stun::Message::parse(b.bytes().data(), b.bytes().size());
  REQUIRE(m.has_value());
  CHECK(m->error_code() == 438);

  auto bytes4 = b.bytes();
  CHECK(stun::looks_like_stun(bytes4.data(), bytes4.size()));
  bytes4[4] ^= 1;  // the magic cookie
  CHECK(!stun::Message::parse(bytes4.data(), bytes4.size()).has_value());
  bytes4 = b.bytes();
  bytes4[3] += 4;  // a length beyond the datagram
  CHECK(!stun::Message::parse(bytes4.data(), bytes4.size()).has_value());
  bytes4 = b.bytes();
  bytes4[stun::kHeaderSize + 3] = 0xff;  // an attribute longer than the message
  CHECK(!stun::Message::parse(bytes4.data(), bytes4.size()).has_value());
  // ChannelData (0x40..) is never taken for STUN.
  const std::uint8_t channel_data[] = {0x40, 0x00, 0x00, 0x10, 0x21, 0x12, 0xa4, 0x42, 0, 0, 0, 0,
                                       0,    0,    0,    0,    0,    0,    0,    0};
  CHECK(!stun::looks_like_stun(channel_data, sizeof channel_data));
}

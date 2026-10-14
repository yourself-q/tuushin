#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tuushin::crypto {

// SHA-1 (FIPS 180-4), HMAC-SHA1 (RFC 2104) and MD5 (RFC 1321), only because
// STUN and TURN require them: a message's MESSAGE-INTEGRITY attribute is an
// HMAC-SHA1, and the key of TURN's long-term credentials is the MD5 of
// "username:realm:password" (RFC 8489, sections 9.2 and 14.5). Both are
// broken as general-purpose hashes; nothing else in tuushin may use them.
//
// They are implemented here rather than taken from libsodium because
// libsodium deliberately leaves them out, and a second crypto library for
// 200 lines of well-specified code would cost more than it saves. The tests
// check them against the vectors of their specifications.

using Sha1Digest = std::array<std::uint8_t, 20>;
using Md5Digest = std::array<std::uint8_t, 16>;

Sha1Digest sha1(const std::uint8_t* data, std::size_t size);
Sha1Digest hmac_sha1(const std::uint8_t* key, std::size_t key_size, const std::uint8_t* data,
                     std::size_t size);
Md5Digest md5(const std::uint8_t* data, std::size_t size);

// Incremental SHA-1, for an HMAC over pieces without copying them together.
class Sha1 {
 public:
  Sha1();
  void update(const std::uint8_t* data, std::size_t size);
  Sha1Digest finish();

 private:
  void block(const std::uint8_t* p);
  std::array<std::uint32_t, 5> h_;
  std::array<std::uint8_t, 64> buf_{};
  std::size_t used_ = 0;
  std::uint64_t length_ = 0;  // bytes
};

}  // namespace tuushin::crypto

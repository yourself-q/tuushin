#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "test.hpp"
#include "tuushin/crypto/legacy_digest.hpp"

using namespace tuushin::crypto;

namespace {

template <std::size_t N>
std::string hex(const std::array<std::uint8_t, N>& d) {
  std::string s;
  char b[3];
  for (const auto x : d) {
    std::snprintf(b, sizeof b, "%02x", x);
    s += b;
  }
  return s;
}

const std::uint8_t* bytes(const std::string& s) {
  return reinterpret_cast<const std::uint8_t*>(s.data());
}

}  // namespace

// RFC 1321, appendix A.5.
TEST_CASE(md5_matches_rfc_1321) {
  const struct {
    const char* in;
    const char* out;
  } vectors[] = {
      {"", "d41d8cd98f00b204e9800998ecf8427e"},
      {"a", "0cc175b9c0f1b6a831c399e269772661"},
      {"abc", "900150983cd24fb0d6963f7d28e17f72"},
      {"message digest", "f96b697d7cb7938d525a2f31aaf161d0"},
      {"abcdefghijklmnopqrstuvwxyz", "c3fcd3d76192e4007dfb496cca67e13b"},
      {"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789",
       "d174ab98d277d9f5a5611c2c9f419d9f"},
      {"1234567890123456789012345678901234567890123456789012345678901234567890123456789"
       "0",
       "57edf4a22be3c955ac49da2e2107b67a"},
  };
  for (const auto& v : vectors) {
    const std::string in = v.in;
    CHECK(hex(md5(bytes(in), in.size())) == v.out);
  }
}

// FIPS 180-2, appendix A, and the empty message.
TEST_CASE(sha1_matches_fips_180) {
  const std::string abc = "abc";
  CHECK(hex(sha1(bytes(abc), abc.size())) == "a9993e364706816aba3e25717850c26c9cd0d89d");
  CHECK(hex(sha1(nullptr, 0)) == "da39a3ee5e6b4b0d3255bfef95601890afd80709");
  const std::string two = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  CHECK(hex(sha1(bytes(two), two.size())) == "84983e441c3bd26ebaae4aa1f95129e5e54670f1");
  // A million 'a's, fed in uneven pieces so that every buffer offset occurs.
  const std::vector<std::uint8_t> a(1000, 'a');
  Sha1 s;
  std::size_t fed = 0;
  for (std::size_t piece = 1; fed < 1000000; piece = piece % 97 + 1) {
    const std::size_t n = std::min(piece, std::size_t{1000000} - fed);
    s.update(a.data(), n);
    fed += n;
  }
  CHECK(hex(s.finish()) == "34aa973cd4c4daa4f61eeb2bdbad27316534016f");
}

// RFC 2202, section 3: test cases 1, 2 and 6 (a key longer than a block).
TEST_CASE(hmac_sha1_matches_rfc_2202) {
  const std::vector<std::uint8_t> k1(20, 0x0b);
  const std::string d1 = "Hi There";
  CHECK(hex(hmac_sha1(k1.data(), k1.size(), bytes(d1), d1.size())) ==
        "b617318655057264e28bc0b6fb378c8ef146be00");
  const std::string k2 = "Jefe", d2 = "what do ya want for nothing?";
  CHECK(hex(hmac_sha1(bytes(k2), k2.size(), bytes(d2), d2.size())) ==
        "effcdf6ae5eb2fa2d27416d5f184df9c259a7c79");
  const std::vector<std::uint8_t> k6(80, 0xaa);
  const std::string d6 = "Test Using Larger Than Block-Size Key - Hash Key First";
  CHECK(hex(hmac_sha1(k6.data(), k6.size(), bytes(d6), d6.size())) ==
        "aa4ae5e15272d00e95705637ce8a3b55ed402112");
}

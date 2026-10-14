#include "tuushin/crypto/legacy_digest.hpp"

#include <algorithm>
#include <cstring>

namespace tuushin::crypto {

namespace {

std::uint32_t rotl(std::uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

std::uint32_t load_be(const std::uint8_t* p) {
  return (std::uint32_t{p[0]} << 24) | (std::uint32_t{p[1]} << 16) | (std::uint32_t{p[2]} << 8) |
         std::uint32_t{p[3]};
}

std::uint32_t load_le(const std::uint8_t* p) {
  return std::uint32_t{p[0]} | (std::uint32_t{p[1]} << 8) | (std::uint32_t{p[2]} << 16) |
         (std::uint32_t{p[3]} << 24);
}

// MD5's additive constants, floor(abs(sin(i + 1)) * 2^32) (RFC 1321, 3.4).
// Every block uses all 64, so the test vectors would catch a wrong one.
constexpr std::uint32_t kMd5K[64] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613,
    0xfd469501, 0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193,
    0xa679438e, 0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d,
    0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
    0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942, 0x8771f681, 0x6d9d6122,
    0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70, 0x289b7ec6, 0xeaa127fa,
    0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244,
    0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
    0xeb86d391};
constexpr int kMd5Shift[16] = {7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21};

void md5_block(std::uint32_t h[4], const std::uint8_t* p) {
  std::uint32_t m[16];
  for (int i = 0; i < 16; ++i) m[i] = load_le(p + 4 * i);
  std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
  for (int i = 0; i < 64; ++i) {
    std::uint32_t f;
    int g;
    if (i < 16) {
      f = (b & c) | (~b & d);
      g = i;
    } else if (i < 32) {
      f = (d & b) | (~d & c);
      g = (5 * i + 1) % 16;
    } else if (i < 48) {
      f = b ^ c ^ d;
      g = (3 * i + 5) % 16;
    } else {
      f = c ^ (b | ~d);
      g = (7 * i) % 16;
    }
    const std::uint32_t t = d;
    d = c;
    c = b;
    b = b + rotl(a + f + kMd5K[i] + m[g], kMd5Shift[(i / 16) * 4 + i % 4]);
    a = t;
  }
  h[0] += a;
  h[1] += b;
  h[2] += c;
  h[3] += d;
}

}  // namespace

Sha1::Sha1() : h_{0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0} {}

void Sha1::block(const std::uint8_t* p) {
  std::uint32_t w[80];
  for (int i = 0; i < 16; ++i) w[i] = load_be(p + 4 * i);
  for (int i = 16; i < 80; ++i) w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
  std::uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3], e = h_[4];
  for (int i = 0; i < 80; ++i) {
    std::uint32_t f, k;
    if (i < 20) {
      f = (b & c) | (~b & d);
      k = 0x5a827999;
    } else if (i < 40) {
      f = b ^ c ^ d;
      k = 0x6ed9eba1;
    } else if (i < 60) {
      f = (b & c) | (b & d) | (c & d);
      k = 0x8f1bbcdc;
    } else {
      f = b ^ c ^ d;
      k = 0xca62c1d6;
    }
    const std::uint32_t t = rotl(a, 5) + f + e + k + w[i];
    e = d;
    d = c;
    c = rotl(b, 30);
    b = a;
    a = t;
  }
  h_[0] += a;
  h_[1] += b;
  h_[2] += c;
  h_[3] += d;
  h_[4] += e;
}

void Sha1::update(const std::uint8_t* data, std::size_t size) {
  length_ += size;
  while (size > 0) {
    const std::size_t n = std::min(size, buf_.size() - used_);
    std::memcpy(buf_.data() + used_, data, n);
    used_ += n;
    data += n;
    size -= n;
    if (used_ == buf_.size()) {
      block(buf_.data());
      used_ = 0;
    }
  }
}

Sha1Digest Sha1::finish() {
  const std::uint64_t bits = length_ * 8;
  const std::uint8_t one = 0x80, zero = 0;
  update(&one, 1);
  while (used_ != 56) update(&zero, 1);
  std::uint8_t len[8];
  for (int i = 0; i < 8; ++i) len[i] = static_cast<std::uint8_t>(bits >> (56 - 8 * i));
  update(len, 8);
  Sha1Digest out;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 4; ++j) out[4 * i + j] = static_cast<std::uint8_t>(h_[i] >> (24 - 8 * j));
  }
  return out;
}

Sha1Digest sha1(const std::uint8_t* data, std::size_t size) {
  Sha1 s;
  s.update(data, size);
  return s.finish();
}

Sha1Digest hmac_sha1(const std::uint8_t* key, std::size_t key_size, const std::uint8_t* data,
                     std::size_t size) {
  std::uint8_t k[64] = {};
  if (key_size > sizeof k) {
    const Sha1Digest d = sha1(key, key_size);
    std::memcpy(k, d.data(), d.size());
  } else if (key_size > 0) {
    std::memcpy(k, key, key_size);
  }
  std::uint8_t pad[64];
  for (int i = 0; i < 64; ++i) pad[i] = k[i] ^ 0x36;
  Sha1 inner;
  inner.update(pad, sizeof pad);
  inner.update(data, size);
  const Sha1Digest ih = inner.finish();
  for (int i = 0; i < 64; ++i) pad[i] = k[i] ^ 0x5c;
  Sha1 outer;
  outer.update(pad, sizeof pad);
  outer.update(ih.data(), ih.size());
  return outer.finish();
}

Md5Digest md5(const std::uint8_t* data, std::size_t size) {
  std::uint32_t h[4] = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476};
  std::size_t i = 0;
  for (; i + 64 <= size; i += 64) md5_block(h, data + i);
  // The tail, 0x80, zeros to 56 mod 64, and the length in bits (LE).
  std::uint8_t tail[128] = {};
  const std::size_t rest = size - i;
  if (rest > 0) std::memcpy(tail, data + i, rest);
  tail[rest] = 0x80;
  const std::size_t total = rest + 1 + 8 <= 64 ? 64 : 128;
  const std::uint64_t bits = static_cast<std::uint64_t>(size) * 8;
  for (int j = 0; j < 8; ++j) tail[total - 8 + j] = static_cast<std::uint8_t>(bits >> (8 * j));
  md5_block(h, tail);
  if (total == 128) md5_block(h, tail + 64);
  Md5Digest out;
  for (int j = 0; j < 4; ++j) {
    for (int b = 0; b < 4; ++b) out[4 * j + b] = static_cast<std::uint8_t>(h[j] >> (8 * b));
  }
  return out;
}

}  // namespace tuushin::crypto

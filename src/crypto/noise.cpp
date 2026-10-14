#include "tuushin/crypto/noise.hpp"

#include <sodium.h>

#include <cctype>
#include <cstring>
#include <stdexcept>

namespace tuushin::crypto {

namespace {

static_assert(crypto_aead_chacha20poly1305_ietf_KEYBYTES == kKeySize, "ChaChaPoly key size");
static_assert(crypto_aead_chacha20poly1305_ietf_ABYTES == kTagSize, "Poly1305 tag size");
static_assert(crypto_aead_chacha20poly1305_ietf_NPUBBYTES == 12, "ChaChaPoly nonce size");
static_assert(crypto_scalarmult_BYTES == kKeySize && crypto_scalarmult_SCALARBYTES == kKeySize,
              "X25519 sizes");
static_assert(crypto_hash_sha256_BYTES == kHashSize, "SHA-256 size");

// Noise, section 5.1 (ChaChaPoly): 32 bits of zeros, then n little-endian.
void make_nonce(std::uint64_t n, unsigned char nonce[12]) {
  std::memset(nonce, 0, 4);
  for (int i = 0; i < 8; ++i) nonce[4 + i] = static_cast<unsigned char>(n >> (8 * i));
}

Hash hmac(const std::uint8_t* key, std::size_t key_len, const std::uint8_t* a, std::size_t a_len,
          const std::uint8_t* b = nullptr, std::size_t b_len = 0) {
  crypto_auth_hmacsha256_state st;
  crypto_auth_hmacsha256_init(&st, key, key_len);
  if (a_len) crypto_auth_hmacsha256_update(&st, a, a_len);
  if (b_len) crypto_auth_hmacsha256_update(&st, b, b_len);
  Hash out;
  crypto_auth_hmacsha256_final(&st, out.data());
  sodium_memzero(&st, sizeof st);
  return out;
}

// Noise, section 4.3: HKDF with HMAC-SHA256, two or three outputs.
void hkdf(const Hash& ck, const std::uint8_t* ikm, std::size_t ikm_len, Hash* o1, Hash* o2,
          Hash* o3) {
  Hash temp = hmac(ck.data(), ck.size(), ikm, ikm_len);
  const std::uint8_t one = 1, two = 2, three = 3;
  *o1 = hmac(temp.data(), temp.size(), &one, 1);
  *o2 = hmac(temp.data(), temp.size(), o1->data(), o1->size(), &two, 1);
  if (o3) *o3 = hmac(temp.data(), temp.size(), o2->data(), o2->size(), &three, 1);
  sodium_memzero(temp.data(), temp.size());
}

}  // namespace

void init() {
  if (sodium_init() < 0) throw std::runtime_error("crypto: libsodium failed to initialize");
}

Key random_key() {
  init();
  Key k;
  randombytes_buf(k.data(), k.size());
  return k;
}

std::string to_hex(const Key& k) {
  static const char* digits = "0123456789abcdef";
  std::string s;
  s.reserve(2 * k.size());
  for (const std::uint8_t b : k) {
    s.push_back(digits[b >> 4]);
    s.push_back(digits[b & 15]);
  }
  return s;
}

std::optional<Key> key_from_hex(const std::string& text) {
  std::size_t a = 0, b = text.size();
  while (a < b && std::isspace(static_cast<unsigned char>(text[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(text[b - 1]))) --b;
  if (b - a != 2 * kKeySize) return std::nullopt;
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  Key k;
  for (std::size_t i = 0; i < kKeySize; ++i) {
    const int hi = nibble(text[a + 2 * i]), lo = nibble(text[a + 2 * i + 1]);
    if (hi < 0 || lo < 0) return std::nullopt;
    k[i] = static_cast<std::uint8_t>(hi << 4 | lo);
  }
  return k;
}

void seal(const Key& k, std::uint64_t n, const std::uint8_t* ad, std::size_t ad_len,
          const std::uint8_t* pt, std::size_t pt_len, std::vector<std::uint8_t>& out) {
  unsigned char nonce[12];
  make_nonce(n, nonce);
  const std::size_t at = out.size();
  out.resize(at + pt_len + kTagSize);
  unsigned long long written = 0;
  crypto_aead_chacha20poly1305_ietf_encrypt(out.data() + at, &written, pt, pt_len, ad, ad_len,
                                            nullptr, nonce, k.data());
  out.resize(at + static_cast<std::size_t>(written));
}

bool open(const Key& k, std::uint64_t n, const std::uint8_t* ad, std::size_t ad_len,
          const std::uint8_t* ct, std::size_t ct_len, std::vector<std::uint8_t>& out) {
  if (ct_len < kTagSize) return false;
  unsigned char nonce[12];
  make_nonce(n, nonce);
  const std::size_t at = out.size();
  out.resize(at + ct_len - kTagSize);
  unsigned long long written = 0;
  if (crypto_aead_chacha20poly1305_ietf_decrypt(out.data() + at, &written, nullptr, ct, ct_len,
                                                ad, ad_len, nonce, k.data()) != 0) {
    out.resize(at);
    return false;
  }
  out.resize(at + static_cast<std::size_t>(written));
  return true;
}

// ---------------------------------------------------------------- NoiseNNpsk0

NoiseNNpsk0::NoiseNNpsk0(Role role, const Key& psk, const std::uint8_t* prologue,
                         std::size_t prologue_len, const Key* ephemeral)
    : role_(role), psk_(psk) {
  init();
  // InitializeSymmetric: the name is longer than HASHLEN, so h = HASH(name).
  const std::size_t name_len = std::strlen(kProtocolName);
  static_assert(sizeof("Noise_NNpsk0_25519_ChaChaPoly_SHA256") - 1 > kHashSize,
                "a protocol name of at most HASHLEN bytes is padded, not hashed");
  crypto_hash_sha256(st_.h.data(), reinterpret_cast<const unsigned char*>(kProtocolName),
                     name_len);
  st_.ck = st_.h;
  mix_hash(st_, prologue, prologue_len);
  // The public key is derived when this end writes its message: a responder
  // that rejects a forged message 1 then has spent no scalar multiplication
  // on it.
  if (ephemeral) {
    st_.e = *ephemeral;
  } else {
    randombytes_buf(st_.e.data(), st_.e.size());
  }
}

NoiseNNpsk0::~NoiseNNpsk0() {
  sodium_memzero(psk_.data(), psk_.size());
  sodium_memzero(&st_, sizeof st_);
}

void NoiseNNpsk0::mix_hash(State& s, const std::uint8_t* data, std::size_t len) {
  crypto_hash_sha256_state hs;
  crypto_hash_sha256_init(&hs);
  crypto_hash_sha256_update(&hs, s.h.data(), s.h.size());
  if (len) crypto_hash_sha256_update(&hs, data, len);
  crypto_hash_sha256_final(&hs, s.h.data());
}

void NoiseNNpsk0::mix_key(State& s, const std::uint8_t* ikm, std::size_t len) {
  Hash ck, k;
  hkdf(s.ck, ikm, len, &ck, &k, nullptr);
  s.ck = ck;
  std::memcpy(s.k.data(), k.data(), kKeySize);
  s.has_k = true;
  s.n = 0;
  sodium_memzero(k.data(), k.size());
}

void NoiseNNpsk0::mix_key_and_hash(State& s, const std::uint8_t* ikm, std::size_t len) {
  Hash ck, th, k;
  hkdf(s.ck, ikm, len, &ck, &th, &k);
  s.ck = ck;
  mix_hash(s, th.data(), th.size());
  std::memcpy(s.k.data(), k.data(), kKeySize);
  s.has_k = true;
  s.n = 0;
  sodium_memzero(k.data(), k.size());
}

void NoiseNNpsk0::encrypt_and_hash(State& s, const std::uint8_t* pt, std::size_t len,
                                   std::vector<std::uint8_t>& out) {
  // Every message of this pattern comes after the psk token, so a key is
  // always set here.
  const std::size_t at = out.size();
  seal(s.k, s.n++, s.h.data(), s.h.size(), pt, len, out);
  mix_hash(s, out.data() + at, out.size() - at);
}

bool NoiseNNpsk0::decrypt_and_hash(State& s, const std::uint8_t* ct, std::size_t len,
                                   std::vector<std::uint8_t>& out) {
  if (!open(s.k, s.n, s.h.data(), s.h.size(), ct, len, out)) return false;
  ++s.n;
  mix_hash(s, ct, len);
  return true;
}

bool NoiseNNpsk0::mix_dh(State& s) {
  unsigned char shared[crypto_scalarmult_BYTES];
  // libsodium refuses a result of all zeros (a low-order public key), which
  // Noise allows implementations to treat as a failure (section 12.1).
  if (crypto_scalarmult(shared, s.e.data(), s.re.data()) != 0) return false;
  mix_key(s, shared, sizeof shared);
  sodium_memzero(shared, sizeof shared);
  return true;
}

std::vector<std::uint8_t> NoiseNNpsk0::write_message(const std::uint8_t* payload,
                                                     std::size_t len) {
  const bool my_turn = role_ == Role::kInitiator ? step_ == 0 : step_ == 1;
  if (!my_turn) throw std::logic_error("NoiseNNpsk0: not this end's turn to write");
  std::vector<std::uint8_t> out;
  out.reserve(kMessageOverhead + len);
  if (role_ == Role::kInitiator) {
    // -> psk, e
    mix_key_and_hash(st_, psk_.data(), psk_.size());
  }
  // e (in a psk handshake the ephemeral key is also mixed into the key,
  // section 9.2, since the payload is encrypted before any DH)
  crypto_scalarmult_base(st_.e_pub.data(), st_.e.data());
  out.insert(out.end(), st_.e_pub.begin(), st_.e_pub.end());
  mix_hash(st_, st_.e_pub.data(), st_.e_pub.size());
  mix_key(st_, st_.e_pub.data(), st_.e_pub.size());
  if (role_ == Role::kResponder) {
    // <- e, ee
    if (!mix_dh(st_)) throw std::logic_error("NoiseNNpsk0: degenerate DH");
  }
  encrypt_and_hash(st_, payload, len, out);
  ++step_;
  return out;
}

std::optional<std::vector<std::uint8_t>> NoiseNNpsk0::read_message(const std::uint8_t* msg,
                                                                   std::size_t len) {
  const bool my_turn = role_ == Role::kResponder ? step_ == 0 : step_ == 1;
  if (!my_turn) throw std::logic_error("NoiseNNpsk0: not this end's turn to read");
  if (msg == nullptr || len < kMessageOverhead) return std::nullopt;
  // Work on a copy, committed only if the message authenticates.
  State s = st_;
  if (role_ == Role::kResponder) {
    // -> psk, e
    mix_key_and_hash(s, psk_.data(), psk_.size());
  }
  std::memcpy(s.re.data(), msg, kKeySize);
  mix_hash(s, s.re.data(), s.re.size());
  mix_key(s, s.re.data(), s.re.size());
  if (role_ == Role::kInitiator) {
    // <- e, ee
    if (!mix_dh(s)) {
      sodium_memzero(&s, sizeof s);
      return std::nullopt;
    }
  }
  std::vector<std::uint8_t> payload;
  if (!decrypt_and_hash(s, msg + kKeySize, len - kKeySize, payload)) {
    sodium_memzero(&s, sizeof s);
    return std::nullopt;
  }
  st_ = s;
  sodium_memzero(&s, sizeof s);
  ++step_;
  return payload;
}

SessionKeys NoiseNNpsk0::split() const {
  if (!complete()) throw std::logic_error("NoiseNNpsk0: handshake not complete");
  Hash k1, k2;
  hkdf(st_.ck, nullptr, 0, &k1, &k2, nullptr);
  SessionKeys keys;
  // The initiator sends with the first key, the responder with the second.
  const Hash& send = role_ == Role::kInitiator ? k1 : k2;
  const Hash& recv = role_ == Role::kInitiator ? k2 : k1;
  std::memcpy(keys.send.data(), send.data(), kKeySize);
  std::memcpy(keys.recv.data(), recv.data(), kKeySize);
  keys.handshake_hash = st_.h;
  sodium_memzero(k1.data(), k1.size());
  sodium_memzero(k2.data(), k2.size());
  return keys;
}

}  // namespace tuushin::crypto

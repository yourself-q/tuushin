#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace tuushin::crypto {

// Authentication for haptic sessions: a pre-shared key, a Noise handshake
// that turns it into fresh keys for each session, and an AEAD for every
// datagram after that. The primitives come from libsodium; nothing here
// implements one.
//
// Why a handshake and not the pre-shared key directly: a key used for many
// sessions needs a nonce that never repeats across all of them, and a
// receiver that accepted a session without contributing anything fresh to
// its keys could be fed a whole recorded session again. The handshake is
// Noise_NNpsk0_25519_ChaChaPoly_SHA256 (Perrin, The Noise Protocol
// Framework, revision 34), the pattern for two parties that share only a
// symmetric key:
//
//   -> psk, e     initiator: ephemeral X25519 key, authenticated by the psk
//   <- e, ee      responder: ephemeral key; both now share DH(e_i, e_r)
//
// Both ends end up with two keys (one per direction) that depend on the psk
// and on both ephemeral keys, so they are new for every session, and an
// eavesdropper who later learns the psk still cannot recover them (forward
// secrecy). WireGuard is built the same way, from a Noise handshake followed
// by datagrams with an explicit counter as the nonce and a replay window.
//
// What the handshake does not do: the first message can be replayed; the
// responder then answers a handshake nobody can finish, and must not treat
// a session as established before a datagram authenticates under the new
// keys (teleop::run_slave_node). Message 2 proves the responder knows the
// psk, and a datagram under the new keys proves the initiator does. To keep
// recorded first messages from costing the responder a key exchange each,
// the teleop layer puts the initiator's time in the payload and answers
// only newer ones (teleop::hello_payload).

constexpr std::size_t kKeySize = 32;
constexpr std::size_t kTagSize = 16;  // Poly1305
constexpr std::size_t kHashSize = 32;  // SHA-256
using Key = std::array<std::uint8_t, kKeySize>;
using Hash = std::array<std::uint8_t, kHashSize>;

// Initializes libsodium. Safe to call more than once and from several
// threads; everything below calls it. Throws std::runtime_error if the
// library cannot initialize (no usable random source).
void init();

// A uniformly random key, e.g. a fresh pre-shared key.
Key random_key();

// Hex form of a key (64 lowercase digits) and its parse, which accepts
// surrounding whitespace and either case.
std::string to_hex(const Key& k);
std::optional<Key> key_from_hex(const std::string& text);

// The identifier the two ends of a pair register under at a rendezvous
// server (net/rendezvous.hpp): BLAKE2b with a 16-byte output, keyed with the
// pre-shared key, over a fixed label. It tells the server which two
// registrations belong together without telling it the key, and nobody
// without the key can compute it. The label keeps it apart from every
// other use of the key.
std::array<std::uint8_t, 16> rendezvous_id(const Key& psk);

// AEAD_CHACHA20_POLY1305 (RFC 8439) with the nonce Noise uses for
// ChaChaPoly: 32 zero bits followed by the 64-bit counter `n`, little-endian.
// A key must never be used twice with the same `n`.
//
// seal appends ciphertext || tag (pt_len + kTagSize bytes) to `out`; open
// appends the plaintext and returns false, appending nothing, if the
// ciphertext, the tag or the associated data were altered. The inputs must
// not point into `out`, which may reallocate.
void seal(const Key& k, std::uint64_t n, const std::uint8_t* ad, std::size_t ad_len,
          const std::uint8_t* pt, std::size_t pt_len, std::vector<std::uint8_t>& out);
bool open(const Key& k, std::uint64_t n, const std::uint8_t* ad, std::size_t ad_len,
          const std::uint8_t* ct, std::size_t ct_len, std::vector<std::uint8_t>& out);

// The result of a completed handshake, from one end's point of view.
struct SessionKeys {
  Key send{};
  Key recv{};
  // Noise's handshake hash: identical at both ends, and unique to this
  // handshake, e.g. for binding a session to it.
  Hash handshake_hash{};
};

class NoiseNNpsk0 {
 public:
  enum class Role { kInitiator, kResponder };
  static constexpr const char* kProtocolName = "Noise_NNpsk0_25519_ChaChaPoly_SHA256";
  // Bytes each handshake message adds to its payload: an ephemeral public
  // key and a tag.
  static constexpr std::size_t kMessageOverhead = kKeySize + kTagSize;

  // `prologue` is data both ends must agree on without sending it (here the
  // session id and the passivity scheme); the handshake fails if they
  // differ. `ephemeral` fixes the ephemeral private key, which only test
  // vectors need; otherwise it is random.
  NoiseNNpsk0(Role role, const Key& psk, const std::uint8_t* prologue, std::size_t prologue_len,
              const Key* ephemeral = nullptr);
  ~NoiseNNpsk0();
  NoiseNNpsk0(const NoiseNNpsk0&) = default;
  NoiseNNpsk0& operator=(const NoiseNNpsk0&) = default;

  // The next message of this end: message 1 for the initiator, message 2
  // for the responder, carrying `payload` encrypted. Throws std::logic_error
  // when it is not this end's turn.
  std::vector<std::uint8_t> write_message(const std::uint8_t* payload, std::size_t len);
  // The other end's next message. Returns its payload, or nullopt if the
  // message is malformed or does not authenticate; the state is then left
  // as it was, so that a forged message cannot break a handshake in
  // progress. Throws std::logic_error when it is not this end's turn.
  std::optional<std::vector<std::uint8_t>> read_message(const std::uint8_t* msg, std::size_t len);

  bool complete() const { return step_ == 2; }
  // The transport keys; throws std::logic_error before complete().
  SessionKeys split() const;

 private:
  struct State {
    Hash ck{};  // chaining key
    Hash h{};   // handshake hash
    Key k{};    // cipher key, valid when has_k
    bool has_k = false;
    std::uint64_t n = 0;
    Key e{};       // our ephemeral private key
    Key e_pub{};   // and its public key
    Key re{};      // the other end's ephemeral public key
  };
  static void mix_hash(State& s, const std::uint8_t* data, std::size_t len);
  static void mix_key(State& s, const std::uint8_t* ikm, std::size_t len);
  static void mix_key_and_hash(State& s, const std::uint8_t* ikm, std::size_t len);
  static void encrypt_and_hash(State& s, const std::uint8_t* pt, std::size_t len,
                               std::vector<std::uint8_t>& out);
  static bool decrypt_and_hash(State& s, const std::uint8_t* ct, std::size_t len,
                               std::vector<std::uint8_t>& out);
  static bool mix_dh(State& s);

  Role role_;
  Key psk_;
  State st_;
  int step_ = 0;  // messages processed so far
};

}  // namespace tuushin::crypto

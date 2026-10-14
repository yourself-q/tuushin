#include <cctype>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "test.hpp"
#include "tuushin/crypto/noise.hpp"

using namespace tuushin::crypto;

namespace {

std::vector<std::uint8_t> hex(const std::string& s) {
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i + 1 < s.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoi(s.substr(i, 2), nullptr, 16)));
  }
  return out;
}

Key key(const std::string& s) {
  const auto b = hex(s);
  Key k{};
  std::memcpy(k.data(), b.data(), k.size());
  return k;
}

// The Noise_NNpsk0_25519_ChaChaPoly_SHA256 vector of the cacophony test
// suite (github.com/haskell-cryptography/cacophony, vectors/cacophony.txt),
// which other Noise implementations check against as well. Messages 3 to 6
// are transport messages, alternating from the initiator, each direction
// counting its nonce from 0, with empty associated data.
const char* kPrologue = "4a6f686e2047616c74";
const char* kPsk = "54686973206973206d7920417573747269616e20706572737065637469766521";
const char* kInitEphemeral = "893e28b9dc6ca8d611ab664754b8ceb7bac5117349a4439a6b0569da977c464a";
const char* kRespEphemeral = "bbdb4cdbd309f1a1f2e1456967fe288cadd6f712d65dc7b7793d5e63da6b375b";
const char* kHandshakeHash = "f4d03dc34495c95729ea6de9e1b59004b59733102488b3e24bc441e0be208eaf";
struct VectorMessage {
  const char* payload;
  const char* ciphertext;
};
const VectorMessage kMessages[] = {
    {"4c756477696720766f6e204d69736573",
     "ca35def5ae56cec33dc2036731ab14896bc4c75dbb07a61f879f8e3afa4c794479b962b8aff8485742ac32f905ba45"
     "369e2465fb59e138a93d67a0d1266b6a54"},
    {"4d757272617920526f746862617264",
     "95ebc60d2b1fa672c1f46a8aa265ef51bfe38e7ccb39ec5be34069f144808843d6062704d5a9c422a8e834423f8c1f"
     "eada7e8d0d910a1a2cd030fb584221e3"},
    {"462e20412e20486179656b", "e632c3763d7669067383433197a3baddf146e9e70ad4b4e9e59e0f"},
    {"4361726c204d656e676572", "64c6bee32ea91c8474bb4c21d7a700109ad45af77b29764ba5eb1e"},
    {"4a65616e2d426170746973746520536179",
     "e2fa0bed0603b62d3ccac2ecabbf3fe33f3e86514909b323361626266cb2471cc8"},
    {"457567656e2042f6686d20766f6e2042617765726b",
     "0c01dc9cec1fe4ddd692e8dd32188aa351088dc91183639a53b57aa4692b5ebdef8b8ca111"},
};

struct Pair {
  NoiseNNpsk0 init, resp;
};

Pair vector_pair() {
  const auto pro = hex(kPrologue);
  const Key psk = key(kPsk), ei = key(kInitEphemeral), er = key(kRespEphemeral);
  return Pair{NoiseNNpsk0(NoiseNNpsk0::Role::kInitiator, psk, pro.data(), pro.size(), &ei),
              NoiseNNpsk0(NoiseNNpsk0::Role::kResponder, psk, pro.data(), pro.size(), &er)};
}

}  // namespace

TEST_CASE(noise_nnpsk0_matches_the_cacophony_vector) {
  auto p = vector_pair();
  const auto pl1 = hex(kMessages[0].payload);
  const auto m1 = p.init.write_message(pl1.data(), pl1.size());
  CHECK(m1 == hex(kMessages[0].ciphertext));
  const auto r1 = p.resp.read_message(m1.data(), m1.size());
  REQUIRE(r1);
  CHECK(*r1 == pl1);

  const auto pl2 = hex(kMessages[1].payload);
  const auto m2 = p.resp.write_message(pl2.data(), pl2.size());
  CHECK(m2 == hex(kMessages[1].ciphertext));
  const auto r2 = p.init.read_message(m2.data(), m2.size());
  REQUIRE(r2);
  CHECK(*r2 == pl2);
  REQUIRE(p.init.complete() && p.resp.complete());

  const SessionKeys ki = p.init.split(), kr = p.resp.split();
  CHECK(ki.handshake_hash == kr.handshake_hash);
  const auto hh = hex(kHandshakeHash);
  CHECK(std::memcmp(ki.handshake_hash.data(), hh.data(), hh.size()) == 0);
  CHECK(ki.send == kr.recv && ki.recv == kr.send && ki.send != ki.recv);

  // Transport messages: the initiator's are messages 3 and 5, the
  // responder's 4 and 6.
  std::uint64_t n_init = 0, n_resp = 0;
  for (int i = 2; i < 6; ++i) {
    const bool from_init = i % 2 == 0;
    const auto pl = hex(kMessages[i].payload);
    std::vector<std::uint8_t> ct;
    const std::uint64_t n = from_init ? n_init++ : n_resp++;
    seal(from_init ? ki.send : kr.send, n, nullptr, 0, pl.data(), pl.size(), ct);
    CHECK(ct == hex(kMessages[i].ciphertext));
    std::vector<std::uint8_t> back;
    CHECK(open(from_init ? kr.recv : ki.recv, n, nullptr, 0, ct.data(), ct.size(), back));
    CHECK(back == pl);
  }
}

TEST_CASE(noise_nnpsk0_rejects_tampering_and_survives_it) {
  auto p = vector_pair();
  const std::uint8_t none = 0;
  const auto m1 = p.init.write_message(&none, 0);
  CHECK(m1.size() == NoiseNNpsk0::kMessageOverhead);
  // Any flipped bit, in the ephemeral key or in the tag, fails.
  for (std::size_t i = 0; i < m1.size(); i += 7) {
    auto bad = m1;
    bad[i] ^= 0x10;
    CHECK(!p.resp.read_message(bad.data(), bad.size()));
  }
  CHECK(!p.resp.read_message(m1.data(), m1.size() - 1));
  // ... and leaves the responder able to read the genuine message.
  REQUIRE(p.resp.read_message(m1.data(), m1.size()));
  const auto m2 = p.resp.write_message(&none, 0);
  auto bad = m2;
  bad[40] ^= 1;
  CHECK(!p.init.read_message(bad.data(), bad.size()));
  REQUIRE(p.init.read_message(m2.data(), m2.size()));
  CHECK(p.init.split().send == p.resp.split().recv);
}

TEST_CASE(noise_nnpsk0_fails_with_another_psk_or_prologue) {
  const std::uint8_t pro_a[] = {1, 2, 3}, pro_b[] = {1, 2, 4};
  const Key psk = random_key();
  Key other = psk;
  other[0] ^= 1;
  const std::uint8_t none = 0;
  {
    NoiseNNpsk0 i(NoiseNNpsk0::Role::kInitiator, psk, pro_a, 3);
    NoiseNNpsk0 r(NoiseNNpsk0::Role::kResponder, other, pro_a, 3);
    const auto m1 = i.write_message(&none, 0);
    CHECK(!r.read_message(m1.data(), m1.size()));
  }
  {
    NoiseNNpsk0 i(NoiseNNpsk0::Role::kInitiator, psk, pro_a, 3);
    NoiseNNpsk0 r(NoiseNNpsk0::Role::kResponder, psk, pro_b, 3);
    const auto m1 = i.write_message(&none, 0);
    CHECK(!r.read_message(m1.data(), m1.size()));
  }
  // A responder without the psk cannot read message 1, so it has nothing to
  // answer with; a reply forged from a genuine one by swapping in another
  // ephemeral key does not authenticate either.
  {
    NoiseNNpsk0 i(NoiseNNpsk0::Role::kInitiator, psk, pro_a, 3);
    NoiseNNpsk0 r(NoiseNNpsk0::Role::kResponder, psk, pro_a, 3);
    NoiseNNpsk0 impostor(NoiseNNpsk0::Role::kResponder, other, pro_a, 3);
    const auto m1 = i.write_message(&none, 0);
    REQUIRE(r.read_message(m1.data(), m1.size()));
    CHECK(!impostor.read_message(m1.data(), m1.size()));
    auto m2 = r.write_message(&none, 0);
    const Key fake = random_key();
    std::memcpy(m2.data(), fake.data(), kKeySize);
    CHECK(!i.read_message(m2.data(), m2.size()));
    CHECK(!i.complete());
  }
}

TEST_CASE(noise_nnpsk0_gives_fresh_keys_every_handshake) {
  const Key psk = random_key();
  const std::uint8_t pro[] = {9};
  const std::uint8_t none = 0;
  SessionKeys first;
  for (int round = 0; round < 2; ++round) {
    NoiseNNpsk0 i(NoiseNNpsk0::Role::kInitiator, psk, pro, 1);
    NoiseNNpsk0 r(NoiseNNpsk0::Role::kResponder, psk, pro, 1);
    const auto m1 = i.write_message(&none, 0);
    REQUIRE(r.read_message(m1.data(), m1.size()));
    const auto m2 = r.write_message(&none, 0);
    REQUIRE(i.read_message(m2.data(), m2.size()));
    const auto k = i.split();
    if (round == 0) {
      first = k;
    } else {
      CHECK(k.send != first.send && k.recv != first.recv);
      CHECK(k.handshake_hash != first.handshake_hash);
    }
  }
}

TEST_CASE(noise_nnpsk0_enforces_turns) {
  const Key psk = random_key();
  const std::uint8_t none = 0;
  NoiseNNpsk0 i(NoiseNNpsk0::Role::kInitiator, psk, nullptr, 0);
  NoiseNNpsk0 r(NoiseNNpsk0::Role::kResponder, psk, nullptr, 0);
  bool threw = false;
  try {
    r.write_message(&none, 0);
  } catch (const std::logic_error&) {
    threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
    i.split();
  } catch (const std::logic_error&) {
    threw = true;
  }
  CHECK(threw);
}

TEST_CASE(aead_detects_altered_data_nonce_and_associated_data) {
  const Key k = random_key();
  const std::uint8_t ad[] = {1, 2, 3, 4};
  const std::uint8_t pt[] = {10, 20, 30, 40, 50};
  std::vector<std::uint8_t> ct;
  seal(k, 7, ad, sizeof ad, pt, sizeof pt, ct);
  CHECK(ct.size() == sizeof pt + kTagSize);
  std::vector<std::uint8_t> out;
  CHECK(open(k, 7, ad, sizeof ad, ct.data(), ct.size(), out));
  CHECK(out == std::vector<std::uint8_t>(pt, pt + sizeof pt));
  out.clear();
  CHECK(!open(k, 8, ad, sizeof ad, ct.data(), ct.size(), out));
  CHECK(out.empty());
  std::uint8_t ad2[] = {1, 2, 3, 5};
  CHECK(!open(k, 7, ad2, sizeof ad2, ct.data(), ct.size(), out));
  for (std::size_t i = 0; i < ct.size(); ++i) {
    auto bad = ct;
    bad[i] ^= 0x80;
    CHECK(!open(k, 7, ad, sizeof ad, bad.data(), bad.size(), out));
  }
  CHECK(!open(k, 7, ad, sizeof ad, ct.data(), kTagSize - 1, out));
  CHECK(out.empty());
}

TEST_CASE(key_hex_round_trip) {
  const Key k = random_key();
  const std::string s = to_hex(k);
  CHECK(s.size() == 64);
  CHECK(key_from_hex(s) == k);
  std::string upper = s;
  for (auto& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  CHECK(key_from_hex("  " + upper + "\n") == k);
  CHECK(!key_from_hex(s.substr(1)));
  CHECK(!key_from_hex(s + "0"));
  std::string bad = s;
  bad[10] = 'g';
  CHECK(!key_from_hex(bad));
}

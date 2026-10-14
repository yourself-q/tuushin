#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "test.hpp"
#include "tuushin/rng.hpp"
#include "tuushin/wire/packet.hpp"

using namespace tuushin;
using namespace tuushin::wire;

namespace {

Header sample_header() {
  Header h;
  h.session = 0xA1B2C3D4u;
  h.seq = 0x01020304u;
  h.send_time = 0x1122334455667788LL;
  h.echo_seq = 77;
  h.echo_hold = 123456;
  return h;
}

HapticPayload sample_payload() {
  HapticPayload p;
  p.dof = 3;
  p.energies = 2;
  p.value = {0.25, -1.5, 3e-9};
  p.aux = {-0.01, 0.0, 12345.678};
  p.energy = {1.0e6 + 0.001, -2.5};
  return p;
}

}  // namespace

TEST_CASE(wire_haptic_round_trip) {
  std::vector<std::uint8_t> buf;
  const Header h = sample_header();
  const HapticPayload p = sample_payload();
  const std::size_t n = encode_haptic(h, p, buf);
  CHECK(n == buf.size());
  CHECK(n == kHeaderSize + 4 + 8 * (2 * 3 + 2));

  const auto hd = decode_header(buf.data(), buf.size());
  REQUIRE(hd.has_value());
  CHECK(hd->type == PacketType::kHaptic);
  CHECK(hd->session == h.session);
  CHECK(hd->seq == h.seq);
  CHECK(hd->flags == 0);
  CHECK(hd->payload_len == n - kHeaderSize);
  CHECK(hd->send_time == h.send_time);
  CHECK(hd->echo_seq == h.echo_seq);
  CHECK(hd->echo_hold == h.echo_hold);

  const auto q = decode_haptic(buf.data(), buf.size());
  REQUIRE(q.has_value());
  CHECK(q->dof == p.dof);
  CHECK(q->energies == p.energies);
  for (std::size_t i = 0; i < p.dof; ++i) {
    // Bit-exact: the format carries binary64 unchanged.
    CHECK(q->value[i] == p.value[i]);
    CHECK(q->aux[i] == p.aux[i]);
  }
  for (std::size_t i = 0; i < p.energies; ++i) CHECK(q->energy[i] == p.energy[i]);
}

TEST_CASE(wire_header_is_little_endian_at_documented_offsets) {
  std::vector<std::uint8_t> buf;
  encode_padded(sample_header(), 0, buf);
  REQUIRE(buf.size() == kHeaderSize);
  CHECK(buf[0] == 'T' && buf[1] == 'U' && buf[2] == kVersion && buf[3] == 1);
  CHECK(buf[4] == 0xD4 && buf[5] == 0xC3 && buf[6] == 0xB2 && buf[7] == 0xA1);
  CHECK(buf[8] == 0x04 && buf[9] == 0x03 && buf[10] == 0x02 && buf[11] == 0x01);
  CHECK(buf[14] == 0 && buf[15] == 0);
  CHECK(buf[16] == 0x88 && buf[23] == 0x11);
  CHECK(buf[24] == 77);
}

TEST_CASE(wire_padded_probe_round_trip) {
  std::vector<std::uint8_t> buf;
  Header h = sample_header();
  h.type = PacketType::kProbe;
  CHECK(encode_padded(h, 100, buf) == kHeaderSize + 100);
  const auto hd = decode_header(buf.data(), buf.size());
  REQUIRE(hd.has_value());
  CHECK(hd->type == PacketType::kProbe);
  CHECK(hd->payload_len == 100);
  // A probe is not a haptic sample.
  CHECK(!decode_haptic(buf.data(), buf.size()).has_value());

  h.type = PacketType::kProbeEcho;
  CHECK(encode_padded(h, kMaxDatagram - kHeaderSize, buf) == kMaxDatagram);
  CHECK(decode_header(buf.data(), buf.size())->type == PacketType::kProbeEcho);
}

TEST_CASE(wire_encoders_reject_oversize) {
  std::vector<std::uint8_t> buf;
  bool threw = false;
  try {
    encode_padded(sample_header(), kMaxDatagram - kHeaderSize + 1, buf);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  HapticPayload p = sample_payload();
  p.dof = kMaxDof + 1;
  threw = false;
  try {
    encode_haptic(sample_header(), p, buf);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  // The largest legal haptic sample fits.
  p.dof = kMaxDof;
  p.energies = kMaxEnergies;
  CHECK(encode_haptic(sample_header(), p, buf) <= kMaxDatagram);
}

TEST_CASE(wire_decoder_rejects_malformed) {
  std::vector<std::uint8_t> good;
  encode_haptic(sample_header(), sample_payload(), good);
  REQUIRE(decode_haptic(good.data(), good.size()).has_value());

  CHECK(!decode_header(nullptr, 0).has_value());
  // Every truncation fails, and so does trailing garbage.
  for (std::size_t n = 0; n < good.size(); ++n) {
    CHECK(!decode_haptic(good.data(), n).has_value());
  }
  std::vector<std::uint8_t> longer = good;
  longer.push_back(0);
  CHECK(!decode_haptic(longer.data(), longer.size()).has_value());

  auto mutated = [&](std::size_t offset, std::uint8_t value) {
    std::vector<std::uint8_t> b = good;
    b[offset] = value;
    return b;
  };
  auto b = mutated(0, 'X');  // magic
  CHECK(!decode_header(b.data(), b.size()).has_value());
  b = mutated(2, kVersion + 1);  // version
  CHECK(!decode_header(b.data(), b.size()).has_value());
  b = mutated(3, 0);  // type
  CHECK(!decode_header(b.data(), b.size()).has_value());
  b = mutated(3, 4);
  CHECK(!decode_header(b.data(), b.size()).has_value());
  b = mutated(14, static_cast<std::uint8_t>(good[14] + 8));  // payload_len
  CHECK(!decode_header(b.data(), b.size()).has_value());
  b = mutated(kHeaderSize, kMaxDof + 1);  // dof
  CHECK(!decode_haptic(b.data(), b.size()).has_value());
  b = mutated(kHeaderSize + 1, kMaxEnergies + 1);  // energies
  CHECK(!decode_haptic(b.data(), b.size()).has_value());
  b = mutated(kHeaderSize, 2);  // dof consistent in range but not with length
  CHECK(!decode_haptic(b.data(), b.size()).has_value());

  // Non-finite values are rejected: a NaN force would propagate straight
  // into the device command.
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    for (std::size_t slot = 0; slot < 2 * 3 + 2; ++slot) {
      std::vector<std::uint8_t> c = good;
      std::memcpy(c.data() + kHeaderSize + 4 + 8 * slot, &bad, 8);
      CHECK(!decode_haptic(c.data(), c.size()).has_value());
    }
  }
}

TEST_CASE(wire_decoder_survives_random_input) {
  // No crash, no exception, and anything accepted is self-consistent.
  Rng rng(11);
  std::vector<std::uint8_t> good;
  encode_haptic(sample_header(), sample_payload(), good);
  std::vector<std::uint8_t> buf;
  int accepted = 0;
  for (int iter = 0; iter < 200000; ++iter) {
    if (iter % 2 == 0) {
      buf.resize(static_cast<std::size_t>(rng.uniform() * 1300));
      for (auto& x : buf) x = static_cast<std::uint8_t>(rng.uniform() * 256);
    } else {
      buf = good;
      const int flips = 1 + static_cast<int>(rng.uniform() * 4);
      for (int k = 0; k < flips; ++k) {
        buf[static_cast<std::size_t>(rng.uniform() * buf.size())] ^=
            static_cast<std::uint8_t>(1u << static_cast<int>(rng.uniform() * 8));
      }
    }
    const auto p = decode_haptic(buf.data(), buf.size());
    if (p) {
      ++accepted;
      CHECK(p->dof <= kMaxDof && p->energies <= kMaxEnergies);
      CHECK(kHeaderSize + p->encoded_size() == buf.size());
    }
  }
  // Bit flips in values or in fields the decoder cannot check are accepted;
  // that is what a checksum one layer down is for.
  CHECK(accepted > 0);
}

TEST_CASE(wire_hold_field_saturates) {
  CHECK(hold_field(-5) == 0);
  CHECK(hold_field(0) == 0);
  CHECK(hold_field(1234) == 1234);
  CHECK(hold_field(Nanos{1} << 40) == std::numeric_limits<std::uint32_t>::max());
}

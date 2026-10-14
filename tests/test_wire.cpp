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
  b = mutated(3, 6);
  CHECK(!decode_header(b.data(), b.size()).has_value());
  b = mutated(3, static_cast<std::uint8_t>(PacketType::kHello));  // a header, but not haptic
  CHECK(decode_header(b.data(), b.size()).has_value());
  CHECK(!decode_haptic(b.data(), b.size()).has_value());
  b = mutated(12, static_cast<std::uint8_t>(good[12] | kFlagSealed));  // sealed: not plain
  CHECK(!decode_haptic(b.data(), b.size()).has_value());
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

TEST_CASE(wire_split_codecs_match_the_whole_datagram) {
  std::vector<std::uint8_t> whole;
  encode_haptic(sample_header(), sample_payload(), whole);
  std::vector<std::uint8_t> parts;
  encode_header(sample_header(), sample_payload().encoded_size(), parts);
  CHECK(parts.size() == kHeaderSize);
  append_haptic_payload(sample_payload(), parts);
  CHECK(parts == whole);
  const auto p = decode_haptic_payload(whole.data() + kHeaderSize, whole.size() - kHeaderSize);
  REQUIRE(p);
  CHECK(p->dof == sample_payload().dof && p->value[0] == sample_payload().value[0]);
  CHECK(!decode_haptic_payload(whole.data() + kHeaderSize, whole.size() - kHeaderSize - 1));

  const std::uint8_t msg[] = {1, 2, 3, 4, 5};
  Header h = sample_header();
  h.type = PacketType::kHelloReply;
  std::vector<std::uint8_t> raw;
  CHECK(encode_raw(h, msg, sizeof msg, raw) == kHeaderSize + sizeof msg);
  const auto hh = decode_header(raw.data(), raw.size());
  REQUIRE(hh);
  CHECK(hh->type == PacketType::kHelloReply && hh->payload_len == sizeof msg);
  CHECK(std::memcmp(raw.data() + kHeaderSize, msg, sizeof msg) == 0);
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

TEST_CASE(wire_probe_echo_carries_receipt_map) {
  std::vector<std::uint8_t> buf;
  const std::uint64_t map = 0x8000000000000101ull;  // bits 0, 8 and 63
  CHECK(encode_probe_echo(sample_header(), map, 100, buf) == kHeaderSize + 100);
  const auto hd = decode_header(buf.data(), buf.size());
  REQUIRE(hd.has_value());
  CHECK(hd->type == PacketType::kProbeEcho);
  CHECK(hd->flags == kFlagReceiptMap);
  CHECK(hd->payload_len == 100);
  // Little-endian at the start of the payload, padding after it.
  CHECK(buf[kHeaderSize] == 0x01 && buf[kHeaderSize + 1] == 0x01 &&
        buf[kHeaderSize + 7] == 0x80);
  CHECK(buf[kHeaderSize + kReceiptMapSize] == 0 && buf.back() == 0);
  const auto r = decode_receipts(buf.data(), buf.size());
  REQUIRE(r.has_value());
  CHECK(*r == map);

  // Exactly the map fits.
  CHECK(encode_probe_echo(sample_header(), map, kReceiptMapSize, buf) ==
        kHeaderSize + kReceiptMapSize);
  CHECK(decode_receipts(buf.data(), buf.size()).value_or(0) == map);
}

TEST_CASE(wire_probe_echo_without_room_has_no_map) {
  // An echo is never larger than its probe, so a probe smaller than the map
  // gets an echo without one; the flag says so and the decoder agrees.
  std::vector<std::uint8_t> buf;
  for (std::size_t payload : {std::size_t{0}, kReceiptMapSize - 1}) {
    CHECK(encode_probe_echo(sample_header(), ~0ull, payload, buf) == kHeaderSize + payload);
    const auto hd = decode_header(buf.data(), buf.size());
    REQUIRE(hd.has_value());
    CHECK(hd->type == PacketType::kProbeEcho);
    CHECK(hd->flags == 0);
    CHECK(!decode_receipts(buf.data(), buf.size()).has_value());
  }
}

TEST_CASE(wire_receipts_only_from_flagged_echoes) {
  std::vector<std::uint8_t> buf;
  // A probe with the same bytes is not an echo.
  Header h = sample_header();
  h.type = PacketType::kProbe;
  h.flags = kFlagReceiptMap;
  encode_padded(h, 16, buf);
  CHECK(!decode_receipts(buf.data(), buf.size()).has_value());
  // An echo without the flag carries padding, not a map.
  h.type = PacketType::kProbeEcho;
  h.flags = 0;
  encode_padded(h, 16, buf);
  CHECK(!decode_receipts(buf.data(), buf.size()).has_value());
  // A flagged echo whose payload is too short for the map is malformed.
  encode_probe_echo(sample_header(), 1, kReceiptMapSize, buf);
  buf[12] = kFlagReceiptMap;
  buf[14] = kReceiptMapSize - 1;
  buf.pop_back();
  CHECK(decode_header(buf.data(), buf.size()).has_value());
  CHECK(!decode_receipts(buf.data(), buf.size()).has_value());
  // Truncations never decode.
  encode_probe_echo(sample_header(), 1, 64, buf);
  for (std::size_t n = 0; n < buf.size(); ++n) {
    CHECK(!decode_receipts(buf.data(), n).has_value());
  }
}

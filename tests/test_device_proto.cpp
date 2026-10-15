#include <cstring>
#include <vector>

#include "test.hpp"
#include "tuushin/device/proto.hpp"
#include "tuushin/rng.hpp"

using namespace tuushin;
using namespace tuushin::device;

namespace {

// Feeds a byte string to a reader and collects the messages' types and bodies.
struct Collected {
  MsgType type;
  std::vector<std::uint8_t> body;
};

std::vector<Collected> read_all(FrameReader& r, const std::vector<std::uint8_t>& bytes) {
  std::vector<Collected> out;
  for (std::uint8_t b : bytes) {
    Message m;
    if (r.push(b, &m)) out.push_back({m.type, std::vector<std::uint8_t>(m.body, m.body + m.size)});
  }
  return out;
}

Message as_message(const Collected& c) {
  return {c.type, c.body.data(), c.body.size()};
}

std::vector<std::uint8_t> frame_of(std::size_t (*enc)(const State&, std::uint8_t*), const State& s) {
  std::uint8_t f[kMaxFrame];
  const std::size_t n = enc(s, f);
  return std::vector<std::uint8_t>(f, f + n);
}

}  // namespace

TEST_CASE(device_crc16_check_value) {
  const char* s = "123456789";
  CHECK(crc16(reinterpret_cast<const std::uint8_t*>(s), 9) == 0x29B1);
}

TEST_CASE(device_cobs_round_trip) {
  Rng rng(7);
  std::uint8_t in[400], enc[420], dec[400];
  for (std::size_t n = 0; n <= 300; ++n) {
    for (int variant = 0; variant < 3; ++variant) {
      for (std::size_t i = 0; i < n; ++i) {
        // Variants: random bytes, runs without zeros (the 254 boundary), many zeros.
        const auto r = static_cast<std::uint8_t>(rng.uniform() * 256.0);
        in[i] = variant == 0 ? r : variant == 1 ? static_cast<std::uint8_t>(1 + r % 255)
                                                : static_cast<std::uint8_t>(r < 128 ? 0 : r);
      }
      const std::size_t k = cobs_encode(in, n, enc);
      CHECK(k <= n + n / 254 + 1);
      bool zero = false;
      for (std::size_t i = 0; i < k; ++i) zero = zero || enc[i] == 0;
      CHECK(!zero);
      const std::size_t m = cobs_decode(enc, k, dec, sizeof dec);
      CHECK(m == n);
      CHECK(std::memcmp(in, dec, n) == 0);
    }
  }
  // Not COBS: a code that runs past the end.
  const std::uint8_t bad[] = {5, 1, 2};
  CHECK(cobs_decode(bad, sizeof bad, dec, sizeof dec) == 0);
}

TEST_CASE(device_messages_round_trip) {
  std::vector<std::uint8_t> stream;
  auto add = [&](std::size_t n, const std::uint8_t* f) { stream.insert(stream.end(), f, f + n); };
  std::uint8_t f[kMaxFrame];

  Command c;
  c.seq = 0xBEEF;
  c.torque = -0.0375f;
  c.flags = kCmdEnable;
  add(encode(c, f), f);
  Set s;
  s.param = Param::kDamping;
  s.value = 1.5e-3f;
  add(encode(s, f), f);
  Info info;
  info.kind = DeviceKind::kSimulated;
  info.pole_pairs = 7;
  info.flags = kInfoCalibrated;
  info.counts_per_rev = 16384;
  info.max_torque = 0.12f;
  info.inertia = 7e-6f;
  info.tick_us = 200;
  info.firmware = 0x010203;
  std::strcpy(info.name, "esp32s3/mt6701");
  add(encode(info, f), f);
  State st;
  st.seq = 65535;
  st.time_us = 0xFFFFFFF0u;
  st.count = -123456;
  st.velocity = 3.25f;
  st.torque = 0.01f;
  st.cmd_seq = 77;
  st.cmd_age_us = 999;
  st.cut_after_us = 2100;
  st.flags = kStateEnabled | kStateCut;
  st.fault = Fault::kOverspeed;
  st.work = -1.0e-7;
  add(encode(st, f), f);
  Ack a;
  a.of = MsgType::kSet;
  a.param = Param::kTorqueLimit;
  a.status = AckStatus::kRejected;
  a.value = 0.05f;
  add(encode(a, f), f);
  add(encode_log("hello, host", f), f);
  add(encode_bare(MsgType::kHello, f), f);

  FrameReader r;
  const auto got = read_all(r, stream);
  REQUIRE(got.size() == 7);
  Command c2;
  REQUIRE(decode(as_message(got[0]), &c2));
  CHECK(c2.seq == c.seq && c2.torque == c.torque && c2.flags == c.flags);
  Set s2;
  REQUIRE(decode(as_message(got[1]), &s2));
  CHECK(s2.param == s.param && s2.value == s.value);
  Info i2;
  REQUIRE(decode(as_message(got[2]), &i2));
  CHECK(i2.kind == info.kind && i2.pole_pairs == 7 && i2.flags == kInfoCalibrated);
  CHECK(i2.counts_per_rev == 16384 && i2.max_torque == info.max_torque && i2.inertia == info.inertia);
  CHECK(i2.tick_us == 200 && i2.firmware == 0x010203 && std::strcmp(i2.name, "esp32s3/mt6701") == 0);
  State st2;
  REQUIRE(decode(as_message(got[3]), &st2));
  CHECK(st2.seq == st.seq && st2.time_us == st.time_us && st2.count == st.count);
  CHECK(st2.velocity == st.velocity && st2.torque == st.torque && st2.cmd_seq == 77);
  CHECK(st2.cmd_age_us == 999 && st2.cut_after_us == 2100 && st2.flags == st.flags);
  CHECK(st2.fault == Fault::kOverspeed && st2.work == st.work);
  Ack a2;
  REQUIRE(decode(as_message(got[4]), &a2));
  CHECK(a2.of == MsgType::kSet && a2.param == Param::kTorqueLimit && a2.status == AckStatus::kRejected);
  CHECK(a2.value == 0.05f);
  char text[80];
  REQUIRE(decode_log(as_message(got[5]), text, sizeof text));
  CHECK(std::strcmp(text, "hello, host") == 0);
  CHECK(got[6].type == MsgType::kHello && got[6].body.empty());
  // A message decoded as another type fails.
  CHECK(!decode(as_message(got[3]), &c2));
  CHECK(r.crc_errors() == 0 && r.bad_frames() == 0);
}

TEST_CASE(device_reader_resynchronizes) {
  State st;
  st.seq = 5;
  st.count = 42;
  const auto good = frame_of(encode, st);
  std::vector<std::uint8_t> stream;
  // A reader started mid-frame, a bootloader's banner, a frame with a
  // flipped bit, and an overlong run: each costs only itself.
  stream.insert(stream.end(), good.begin() + 5, good.end());
  const char* banner = "ets Jul 29 2019 12:21:46\r\nrst:0x1 (POWERON_RESET)\r\n";
  stream.insert(stream.end(), banner, banner + std::strlen(banner));
  stream.push_back(0);
  stream.insert(stream.end(), good.begin(), good.end());
  auto flipped = good;
  flipped[7] ^= 0x10;
  stream.insert(stream.end(), flipped.begin(), flipped.end());
  stream.insert(stream.end(), 500, 0x55);
  stream.push_back(0);
  stream.insert(stream.end(), good.begin(), good.end());

  FrameReader r;
  const auto got = read_all(r, stream);
  REQUIRE(got.size() == 2);
  State s2;
  REQUIRE(decode(as_message(got[1]), &s2));
  CHECK(s2.seq == 5 && s2.count == 42);
  CHECK(r.crc_errors() + r.bad_frames() == 4);
}

TEST_CASE(device_longer_body_is_accepted) {
  // A later version may append fields; this one reads what it knows.
  std::uint8_t body[12] = {0x34, 0x12, 0, 0, 0x80, 0x3F, kCmdEnable, 9, 9, 9, 9, 9};
  std::uint8_t f[kMaxFrame];
  const std::size_t n = encode_frame(MsgType::kCommand, body, sizeof body, f);
  FrameReader r;
  const auto got = read_all(r, std::vector<std::uint8_t>(f, f + n));
  REQUIRE(got.size() == 1);
  Command c;
  REQUIRE(decode(as_message(got[0]), &c));
  CHECK(c.seq == 0x1234 && c.torque == 1.0f && c.flags == kCmdEnable);
  // A shorter one is not.
  const std::size_t k = encode_frame(MsgType::kCommand, body, 6, f);
  FrameReader r2;
  const auto short_one = read_all(r2, std::vector<std::uint8_t>(f, f + k));
  REQUIRE(short_one.size() == 1);
  CHECK(!decode(as_message(short_one[0]), &c));
}

TEST_CASE(device_command_carries_a_work_cap) {
  std::uint8_t f[kMaxFrame];
  Command c;
  c.seq = 7;
  c.torque = 0.05f;
  c.flags = kCmdEnable | kCmdWorkCap;
  c.work_cap = 1.0 / 3.0;  // a double: the cap sits on a total that keeps growing
  const std::size_t n = encode(c, f);
  FrameReader r;
  const auto got = read_all(r, std::vector<std::uint8_t>(f, f + n));
  REQUIRE(got.size() == 1);
  CHECK(got[0].body.size() == 15);
  Command c2;
  REQUIRE(decode(as_message(got[0]), &c2));
  CHECK(c2.flags == c.flags && c2.work_cap == c.work_cap && c2.torque == c.torque);
  // Without the flag the cap is not sent, and a command that claims one
  // without carrying it is refused rather than applied without it.
  c.flags = kCmdEnable;
  FrameReader r2;
  const std::size_t k = encode(c, f);
  const auto bare = read_all(r2, std::vector<std::uint8_t>(f, f + k));
  REQUIRE(bare.size() == 1);
  CHECK(bare[0].body.size() == 7);
  REQUIRE(decode(as_message(bare[0]), &c2));
  CHECK(c2.work_cap == 0.0);
  std::uint8_t body[7] = {1, 0, 0, 0, 0x80, 0x3F, kCmdEnable | kCmdWorkCap};
  FrameReader r3;
  const std::size_t m = encode_frame(MsgType::kCommand, body, sizeof body, f);
  const auto lying = read_all(r3, std::vector<std::uint8_t>(f, f + m));
  REQUIRE(lying.size() == 1);
  CHECK(!decode(as_message(lying[0]), &c2));
}

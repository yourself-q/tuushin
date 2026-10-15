#include "tuushin/device/proto.hpp"

#include <algorithm>
#include <cstring>

namespace tuushin::device {

namespace {

// Little-endian writers and readers, byte by byte.
class Out {
 public:
  explicit Out(std::uint8_t* p) : p_(p) {}
  void u8(std::uint8_t v) { p_[n_++] = v; }
  void u16(std::uint16_t v) {
    u8(static_cast<std::uint8_t>(v));
    u8(static_cast<std::uint8_t>(v >> 8));
  }
  void u32(std::uint32_t v) {
    u16(static_cast<std::uint16_t>(v));
    u16(static_cast<std::uint16_t>(v >> 16));
  }
  void u64(std::uint64_t v) {
    u32(static_cast<std::uint32_t>(v));
    u32(static_cast<std::uint32_t>(v >> 32));
  }
  void f32(float v) {
    std::uint32_t b;
    std::memcpy(&b, &v, sizeof b);
    u32(b);
  }
  void f64(double v) {
    std::uint64_t b;
    std::memcpy(&b, &v, sizeof b);
    u64(b);
  }
  void bytes(const void* p, std::size_t n) {
    std::memcpy(p_ + n_, p, n);
    n_ += n;
  }
  std::size_t size() const { return n_; }

 private:
  std::uint8_t* p_;
  std::size_t n_ = 0;
};

class In {
 public:
  In(const std::uint8_t* p, std::size_t n) : p_(p), n_(n) {}
  bool has(std::size_t k) const { return n_ - i_ >= k; }
  std::uint8_t u8() { return p_[i_++]; }
  std::uint16_t u16() {
    const std::uint16_t lo = u8();
    return static_cast<std::uint16_t>(lo | (static_cast<std::uint16_t>(u8()) << 8));
  }
  std::uint32_t u32() {
    const std::uint32_t lo = u16();
    return lo | (static_cast<std::uint32_t>(u16()) << 16);
  }
  std::uint64_t u64() {
    const std::uint64_t lo = u32();
    return lo | (static_cast<std::uint64_t>(u32()) << 32);
  }
  float f32() {
    const std::uint32_t b = u32();
    float v;
    std::memcpy(&v, &b, sizeof v);
    return v;
  }
  double f64() {
    const std::uint64_t b = u64();
    double v;
    std::memcpy(&v, &b, sizeof v);
    return v;
  }
  void bytes(void* p, std::size_t k) {
    std::memcpy(p, p_ + i_, k);
    i_ += k;
  }

 private:
  const std::uint8_t* p_;
  std::size_t n_;
  std::size_t i_ = 0;
};

constexpr std::size_t kCommandSize = 7;
constexpr std::size_t kSetSize = 5;
constexpr std::size_t kInfoSize = 40;
constexpr std::size_t kStateSize = 34;
constexpr std::size_t kAckSize = 7;

bool is(const Message& msg, MsgType t, std::size_t size) {
  return msg.type == t && msg.size >= size;
}

}  // namespace

std::uint16_t crc16(const std::uint8_t* data, std::size_t n, std::uint16_t crc) {
  for (std::size_t i = 0; i < n; ++i) {
    crc = static_cast<std::uint16_t>(crc ^ (static_cast<std::uint16_t>(data[i]) << 8));
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::size_t cobs_encode(const std::uint8_t* in, std::size_t n, std::uint8_t* out) {
  std::size_t code_at = 0, o = 1;
  std::uint8_t code = 1;
  for (std::size_t i = 0; i < n; ++i) {
    if (in[i] == 0) {
      out[code_at] = code;
      code_at = o++;
      code = 1;
      continue;
    }
    out[o++] = in[i];
    if (++code == 0xFF) {
      out[code_at] = code;
      code_at = o++;
      code = 1;
    }
  }
  out[code_at] = code;
  return o;
}

std::size_t cobs_decode(const std::uint8_t* in, std::size_t n, std::uint8_t* out,
                        std::size_t cap) {
  std::size_t i = 0, o = 0;
  while (i < n) {
    const std::uint8_t code = in[i++];
    if (code == 0) return 0;
    for (std::uint8_t k = 1; k < code; ++k) {
      if (i >= n || in[i] == 0 || o >= cap) return 0;
      out[o++] = in[i++];
    }
    // A group shorter than 255 stands for a zero, unless it ends the frame.
    if (code != 0xFF && i < n) {
      if (o >= cap) return 0;
      out[o++] = 0;
    }
  }
  return o;
}

std::size_t encode_frame(MsgType type, const std::uint8_t* body, std::size_t n,
                         std::uint8_t* out) {
  if (n + 1 > kMaxMessage) return 0;
  std::uint8_t msg[kMaxMessage + 2];
  msg[0] = static_cast<std::uint8_t>(type);
  if (n > 0) std::memcpy(msg + 1, body, n);
  const std::uint16_t crc = crc16(msg, n + 1);
  msg[n + 1] = static_cast<std::uint8_t>(crc);
  msg[n + 2] = static_cast<std::uint8_t>(crc >> 8);
  const std::size_t k = cobs_encode(msg, n + 3, out);
  out[k] = 0;
  return k + 1;
}

std::size_t encode(const Command& m, std::uint8_t* out) {
  std::uint8_t b[kCommandSize];
  Out w(b);
  w.u16(m.seq);
  w.f32(m.torque);
  w.u8(m.flags);
  return encode_frame(MsgType::kCommand, b, w.size(), out);
}

std::size_t encode(const Set& m, std::uint8_t* out) {
  std::uint8_t b[kSetSize];
  Out w(b);
  w.u8(static_cast<std::uint8_t>(m.param));
  w.f32(m.value);
  return encode_frame(MsgType::kSet, b, w.size(), out);
}

std::size_t encode(const Info& m, std::uint8_t* out) {
  std::uint8_t b[kInfoSize];
  Out w(b);
  w.u8(m.protocol);
  w.u8(static_cast<std::uint8_t>(m.kind));
  w.u8(m.pole_pairs);
  w.u8(m.flags);
  w.u32(m.counts_per_rev);
  w.f32(m.max_torque);
  w.f32(m.inertia);
  w.u32(m.tick_us);
  w.u32(m.firmware);
  w.bytes(m.name, sizeof m.name);
  return encode_frame(MsgType::kInfo, b, w.size(), out);
}

std::size_t encode(const State& m, std::uint8_t* out) {
  std::uint8_t b[kStateSize];
  Out w(b);
  w.u16(m.seq);
  w.u32(m.time_us);
  w.u32(static_cast<std::uint32_t>(m.count));
  w.f32(m.velocity);
  w.f32(m.torque);
  w.u16(m.cmd_seq);
  w.u16(m.cmd_age_us);
  w.u16(m.cut_after_us);
  w.u8(m.flags);
  w.u8(static_cast<std::uint8_t>(m.fault));
  w.f64(m.work);
  return encode_frame(MsgType::kState, b, w.size(), out);
}

std::size_t encode(const Ack& m, std::uint8_t* out) {
  std::uint8_t b[kAckSize];
  Out w(b);
  w.u8(static_cast<std::uint8_t>(m.of));
  w.u8(static_cast<std::uint8_t>(m.param));
  w.u8(static_cast<std::uint8_t>(m.status));
  w.f32(m.value);
  return encode_frame(MsgType::kAck, b, w.size(), out);
}

std::size_t encode_log(const char* text, std::uint8_t* out) {
  const std::size_t n = std::min(std::strlen(text), kMaxLogText);
  return encode_frame(MsgType::kLog, reinterpret_cast<const std::uint8_t*>(text), n, out);
}

std::size_t encode_bare(MsgType type, std::uint8_t* out) {
  return encode_frame(type, nullptr, 0, out);
}

bool decode(const Message& msg, Command* m) {
  if (!is(msg, MsgType::kCommand, kCommandSize)) return false;
  In r(msg.body, msg.size);
  m->seq = r.u16();
  m->torque = r.f32();
  m->flags = r.u8();
  return true;
}

bool decode(const Message& msg, Set* m) {
  if (!is(msg, MsgType::kSet, kSetSize)) return false;
  In r(msg.body, msg.size);
  m->param = static_cast<Param>(r.u8());
  m->value = r.f32();
  return true;
}

bool decode(const Message& msg, Info* m) {
  if (!is(msg, MsgType::kInfo, kInfoSize)) return false;
  In r(msg.body, msg.size);
  m->protocol = r.u8();
  m->kind = static_cast<DeviceKind>(r.u8());
  m->pole_pairs = r.u8();
  m->flags = r.u8();
  m->counts_per_rev = r.u32();
  m->max_torque = r.f32();
  m->inertia = r.f32();
  m->tick_us = r.u32();
  m->firmware = r.u32();
  r.bytes(m->name, sizeof m->name);
  m->name[sizeof m->name - 1] = '\0';
  return true;
}

bool decode(const Message& msg, State* m) {
  if (!is(msg, MsgType::kState, kStateSize)) return false;
  In r(msg.body, msg.size);
  m->seq = r.u16();
  m->time_us = r.u32();
  m->count = static_cast<std::int32_t>(r.u32());
  m->velocity = r.f32();
  m->torque = r.f32();
  m->cmd_seq = r.u16();
  m->cmd_age_us = r.u16();
  m->cut_after_us = r.u16();
  m->flags = r.u8();
  m->fault = static_cast<Fault>(r.u8());
  m->work = r.f64();
  return true;
}

bool decode(const Message& msg, Ack* m) {
  if (!is(msg, MsgType::kAck, kAckSize)) return false;
  In r(msg.body, msg.size);
  m->of = static_cast<MsgType>(r.u8());
  m->param = static_cast<Param>(r.u8());
  m->status = static_cast<AckStatus>(r.u8());
  m->value = r.f32();
  return true;
}

bool decode_log(const Message& msg, char* text, std::size_t cap) {
  if (msg.type != MsgType::kLog || cap == 0) return false;
  const std::size_t n = msg.size < cap - 1 ? msg.size : cap - 1;
  std::memcpy(text, msg.body, n);
  text[n] = '\0';
  return true;
}

const char* fault_name(Fault f) {
  switch (f) {
    case Fault::kNone: return "none";
    case Fault::kEncoder: return "encoder";
    case Fault::kEncoderJump: return "encoder jump";
    case Fault::kOverspeed: return "overspeed";
    case Fault::kCalibration: return "calibration";
    case Fault::kSupply: return "supply";
  }
  return "unknown";
}

bool FrameReader::push(std::uint8_t byte, Message* out) {
  if (byte != 0) {
    if (n_ < sizeof raw_) {
      raw_[n_++] = byte;
    } else {
      overflow_ = true;
    }
    return false;
  }
  // A delimiter: whatever came before it is a frame, or nothing.
  const std::size_t n = n_;
  const bool overflow = overflow_;
  n_ = 0;
  overflow_ = false;
  if (n == 0) return false;
  ++frames_;
  const std::size_t k = overflow ? 0 : cobs_decode(raw_, n, msg_, sizeof msg_);
  if (k < 3) {
    ++bad_frames_;
    return false;
  }
  const std::uint16_t want = static_cast<std::uint16_t>(msg_[k - 2] | (msg_[k - 1] << 8));
  if (crc16(msg_, k - 2) != want) {
    ++crc_errors_;
    return false;
  }
  out->type = static_cast<MsgType>(msg_[0]);
  out->body = msg_ + 1;
  out->size = k - 3;
  return true;
}

}  // namespace tuushin::device

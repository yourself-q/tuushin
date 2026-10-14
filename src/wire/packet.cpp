#include "tuushin/wire/packet.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace tuushin::wire {

namespace {

constexpr std::uint8_t kMagic0 = 'T';
constexpr std::uint8_t kMagic1 = 'U';

// Explicit little-endian byte handling, independent of the host's order and
// of struct layout.
void put_u16(std::vector<std::uint8_t>& b, std::uint16_t v) {
  b.push_back(static_cast<std::uint8_t>(v));
  b.push_back(static_cast<std::uint8_t>(v >> 8));
}

void put_u32(std::vector<std::uint8_t>& b, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}

void put_u64(std::vector<std::uint8_t>& b, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}

void put_f64(std::vector<std::uint8_t>& b, double v) {
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
                "binary64 doubles required");
  std::uint64_t bits;
  std::memcpy(&bits, &v, 8);
  put_u64(b, bits);
}

std::uint16_t get_u16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t get_u32(const std::uint8_t* p) {
  std::uint32_t v = 0;
  for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
  return v;
}

std::uint64_t get_u64(const std::uint8_t* p) {
  std::uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  return v;
}

double get_f64(const std::uint8_t* p) {
  const std::uint64_t bits = get_u64(p);
  double v;
  std::memcpy(&v, &bits, 8);
  return v;
}

void put_header(const Header& h, std::size_t payload_len, std::vector<std::uint8_t>& out) {
  if (kHeaderSize + payload_len > kMaxDatagram) {
    throw std::invalid_argument("wire: datagram exceeds kMaxDatagram");
  }
  out.clear();
  out.reserve(kHeaderSize + payload_len);
  out.push_back(kMagic0);
  out.push_back(kMagic1);
  out.push_back(kVersion);
  out.push_back(static_cast<std::uint8_t>(h.type));
  put_u32(out, h.session);
  put_u32(out, h.seq);
  put_u16(out, h.flags);
  put_u16(out, static_cast<std::uint16_t>(payload_len));
  put_u64(out, static_cast<std::uint64_t>(h.send_time));
  put_u32(out, h.echo_seq);
  put_u32(out, h.echo_hold);
}

}  // namespace

std::uint32_t hold_field(Nanos hold) {
  if (hold <= 0) return 0;
  constexpr Nanos kMax = std::numeric_limits<std::uint32_t>::max();
  return static_cast<std::uint32_t>(hold < kMax ? hold : kMax);
}

void append_haptic_payload(const HapticPayload& p, std::vector<std::uint8_t>& out) {
  if (p.dof > kMaxDof || p.energies > kMaxEnergies) {
    throw std::invalid_argument("wire: too many axes or energies");
  }
  out.push_back(p.dof);
  out.push_back(p.energies);
  put_u16(out, 0);
  for (std::size_t i = 0; i < p.dof; ++i) put_f64(out, p.value[i]);
  for (std::size_t i = 0; i < p.dof; ++i) put_f64(out, p.aux[i]);
  for (std::size_t i = 0; i < p.energies; ++i) put_f64(out, p.energy[i]);
}

std::size_t encode_haptic(const Header& h, const HapticPayload& p, std::vector<std::uint8_t>& out) {
  if (p.dof > kMaxDof || p.energies > kMaxEnergies) {
    throw std::invalid_argument("wire: too many axes or energies");
  }
  Header hh = h;
  hh.type = PacketType::kHaptic;
  put_header(hh, p.encoded_size(), out);
  append_haptic_payload(p, out);
  return out.size();
}

std::size_t encode_header(const Header& h, std::size_t payload_len, std::vector<std::uint8_t>& out) {
  put_header(h, payload_len, out);
  return out.size();
}

std::size_t encode_raw(const Header& h, const std::uint8_t* payload, std::size_t len,
                       std::vector<std::uint8_t>& out) {
  put_header(h, len, out);
  if (len) out.insert(out.end(), payload, payload + len);
  return out.size();
}

std::size_t encode_padded(const Header& h, std::size_t padding, std::vector<std::uint8_t>& out) {
  put_header(h, padding, out);
  out.resize(kHeaderSize + padding, 0);
  return out.size();
}

std::size_t encode_probe_echo(const Header& h, std::uint64_t receipts, std::size_t payload,
                              std::vector<std::uint8_t>& out) {
  Header hh = h;
  hh.type = PacketType::kProbeEcho;
  const bool with_map = payload >= kReceiptMapSize;
  hh.flags = with_map ? kFlagReceiptMap : 0;
  put_header(hh, payload, out);
  if (with_map) put_u64(out, receipts);
  out.resize(kHeaderSize + payload, 0);
  return out.size();
}

std::optional<Header> decode_header(const std::uint8_t* data, std::size_t size) {
  if (data == nullptr || size < kHeaderSize || size > kMaxDatagram) return std::nullopt;
  if (data[0] != kMagic0 || data[1] != kMagic1 || data[2] != kVersion) return std::nullopt;
  const std::uint8_t type = data[3];
  if (type < 1 || type > 6) return std::nullopt;
  Header h;
  h.type = static_cast<PacketType>(type);
  h.session = get_u32(data + 4);
  h.seq = get_u32(data + 8);
  h.flags = get_u16(data + 12);
  h.payload_len = get_u16(data + 14);
  h.send_time = static_cast<Nanos>(get_u64(data + 16));
  h.echo_seq = get_u32(data + 24);
  h.echo_hold = get_u32(data + 28);
  if (kHeaderSize + h.payload_len != size) return std::nullopt;
  return h;
}

std::optional<HapticPayload> decode_haptic(const std::uint8_t* data, std::size_t size) {
  const auto h = decode_header(data, size);
  if (!h || h->type != PacketType::kHaptic || (h->flags & kFlagSealed)) return std::nullopt;
  return decode_haptic_payload(data + kHeaderSize, h->payload_len);
}

std::optional<HapticPayload> decode_haptic_payload(const std::uint8_t* p, std::size_t len) {
  if (p == nullptr || len < 4) return std::nullopt;
  HapticPayload out;
  out.dof = p[0];
  out.energies = p[1];
  if (out.dof > kMaxDof || out.energies > kMaxEnergies) return std::nullopt;
  if (out.encoded_size() != len) return std::nullopt;
  const std::uint8_t* q = p + 4;
  for (std::size_t i = 0; i < out.dof; ++i, q += 8) out.value[i] = get_f64(q);
  for (std::size_t i = 0; i < out.dof; ++i, q += 8) out.aux[i] = get_f64(q);
  for (std::size_t i = 0; i < out.energies; ++i, q += 8) out.energy[i] = get_f64(q);
  for (std::size_t i = 0; i < out.dof; ++i) {
    if (!std::isfinite(out.value[i]) || !std::isfinite(out.aux[i])) return std::nullopt;
  }
  for (std::size_t i = 0; i < out.energies; ++i) {
    if (!std::isfinite(out.energy[i])) return std::nullopt;
  }
  return out;
}

std::optional<std::uint64_t> decode_receipts(const std::uint8_t* data, std::size_t size) {
  const auto h = decode_header(data, size);
  if (!h || h->type != PacketType::kProbeEcho || (h->flags & kFlagReceiptMap) == 0 ||
      h->payload_len < kReceiptMapSize) {
    return std::nullopt;
  }
  return get_u64(data + kHeaderSize);
}

}  // namespace tuushin::wire

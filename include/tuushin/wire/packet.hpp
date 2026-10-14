#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "tuushin/time.hpp"

namespace tuushin::wire {

// On-the-wire format of tuushin datagrams, version 1.
//
// Every datagram starts with the same 32-byte header. All integers and
// floating-point values are little-endian; doubles are IEEE 754 binary64.
//
//   offset size field
//        0    2 magic        'T' 'U'
//        2    1 version      1
//        3    1 type         PacketType
//        4    4 session      random per run, chosen by the initiator
//        8    4 seq          per-sender sequence number, starts at 1
//       12    2 flags        per type, see below; 0 unless defined
//       14    2 payload_len  bytes after the header
//       16    8 send_time    sender's monotonic clock, ns
//       24    4 echo_seq     newest seq received from the peer, 0 if none
//       28    4 echo_hold    ns between receiving echo_seq and sending this
//                            datagram, saturated at 2^32 - 1
//
// The echo fields let each side measure the round-trip time from ordinary
// traffic, without probe packets and without synchronized clocks: the peer
// remembers when it sent echo_seq (t1) and computes
//
//   rtt = t_receive - t1 - echo_hold,
//
// as RTCP does with its LSR/DLSR fields (RFC 3550, section 6.4.1). With
// send_time it also yields the one-way delay plus the clock offset, which is
// all a receiver needs for jitter and for clock_sync.hpp.
//
// Sequence numbers are 32 bits: at 1 kHz they wrap after 49 days, so a
// session is expected to restart before that; comparisons do not handle wrap.
constexpr std::uint8_t kVersion = 1;
constexpr std::size_t kHeaderSize = 32;
// Largest datagram this library sends or accepts. It fits a 1280-byte IPv6
// minimum MTU with room for IP and UDP headers, so it is never fragmented.
constexpr std::size_t kMaxDatagram = 1200;

enum class PacketType : std::uint8_t {
  kHaptic = 1,      // HapticPayload, or its sealed form (kFlagSealed)
  kProbe = 2,       // measurement probe; payload is padding
  kProbeEcho = 3,   // reflector's answer to a probe; payload is padding
  kHello = 4,       // handshake message 1 (crypto::NoiseNNpsk0), initiator
  kHelloReply = 5,  // handshake message 2, responder; echo_seq = the hello's seq
  kPunch = 6,       // opens the sender's NAT towards the receiver (net/rendezvous.hpp);
                    // no payload, session 0; receivers ignore it
};

struct Header {
  PacketType type = PacketType::kHaptic;
  std::uint32_t session = 0;
  std::uint32_t seq = 0;
  std::uint16_t flags = 0;
  std::uint16_t payload_len = 0;
  Nanos send_time = 0;
  std::uint32_t echo_seq = 0;
  std::uint32_t echo_hold = 0;  // ns
};

// Saturating conversion for Header::echo_hold.
std::uint32_t hold_field(Nanos hold);

// Flag of kProbeEcho: the payload starts with an 8-byte receipt map, a
// little-endian bit field in which bit i is set if the reflector had received
// probe echo_seq - 1 - i of the same session when it sent this echo. The map
// tells a lost probe from a lost echo: when an echo goes missing, the maps
// of the next ones still say whether its probe arrived. TWAMP (RFC 5357)
// separates the directions by the reflector's sequence numbers alone, which
// gives the count of losses per direction but not which packets they were;
// a replayable trace needs the latter.
constexpr std::uint16_t kFlagReceiptMap = 1;
constexpr std::size_t kReceiptMapSize = 8;

// Flags of kHaptic. kFlagFinal: the sender is ending the session; it is set
// on every datagram from then on, so one loss does not hide it. The scheme
// field names the passivity scheme the sender runs (teleop::Passivity:
// 0 none, 1 channel, 2 reflective): the energies mean different things under
// different schemes, so a receiver must refuse a session whose scheme it
// does not run.
constexpr std::uint16_t kFlagFinal = 1;
constexpr int kSchemeShift = 1;
constexpr std::uint16_t kSchemeMask = 0x3 << kSchemeShift;
// Flag of kHaptic: the payload is sealed with the session's key
// (crypto::seal, nonce = seq, associated data = the 32-byte header as sent),
// so it is ciphertext || tag, kTagSize bytes longer than the HapticPayload
// inside. The header stays readable, and any change to it fails the tag. The
// flag only says what the sender did: a receiver with a key refuses
// datagrams without it. kHello and kHelloReply carry the scheme field too,
// and their payload is the handshake message.
constexpr std::uint16_t kFlagSealed = 1 << 3;
constexpr std::size_t kTagSize = 16;

// Haptic sample, payload of kHaptic:
//
//   offset size field
//        0    1 dof           axes, 0..kMaxDof
//        1    1 energies      cumulative energy values, 0..kMaxEnergies
//        2    2 reserved      0
//        4  8*d value[d]      power variable per axis (velocity or force)
//      ...  8*d aux[d]        auxiliary per axis (e.g. position), SI units
//      ...  8*e energy[e]     cumulative energies in J
//
// Energies are cumulative so that a late, lost or reordered datagram can only
// understate them (see control/tdpa.hpp); binary64 keeps millijoule
// resolution after days of operation, which binary32 would not.
constexpr std::size_t kMaxDof = 7;
constexpr std::size_t kMaxEnergies = 4;

struct HapticPayload {
  std::uint8_t dof = 0;
  std::uint8_t energies = 0;
  std::array<double, kMaxDof> value{};
  std::array<double, kMaxDof> aux{};
  std::array<double, kMaxEnergies> energy{};

  std::size_t encoded_size() const { return 4 + 8 * (2 * dof + energies); }
};

// Encoders append to `out` (which is cleared first) and return the datagram
// size. They throw std::invalid_argument for payloads that do not fit.
std::size_t encode_haptic(const Header& h, const HapticPayload& p, std::vector<std::uint8_t>& out);
// The header alone, with payload_len set to `payload_len` (h.payload_len is
// ignored), for a payload the caller appends, e.g. a sealed one.
std::size_t encode_header(const Header& h, std::size_t payload_len, std::vector<std::uint8_t>& out);
// A HapticPayload's bytes, appended to `out` as they follow the header.
void append_haptic_payload(const HapticPayload& p, std::vector<std::uint8_t>& out);
// A datagram whose payload is `len` given bytes (kHello, kHelloReply).
std::size_t encode_raw(const Header& h, const std::uint8_t* payload, std::size_t len,
                       std::vector<std::uint8_t>& out);
// Probe and echo datagrams carry `padding` zero bytes so that probes can be
// sized like the traffic they stand in for.
std::size_t encode_padded(const Header& h, std::size_t padding, std::vector<std::uint8_t>& out);
// A kProbeEcho of `payload` bytes; the receipt map is included (and
// kFlagReceiptMap set) if the payload has room for it.
std::size_t encode_probe_echo(const Header& h, std::uint64_t receipts, std::size_t payload,
                              std::vector<std::uint8_t>& out);

// Decoders return nullopt for anything malformed: wrong magic or version,
// truncated, inconsistent lengths, out-of-range counts, non-finite values.
// They never throw, since datagrams come from the network.
std::optional<Header> decode_header(const std::uint8_t* data, std::size_t size);
std::optional<HapticPayload> decode_haptic(const std::uint8_t* data, std::size_t size);
// A HapticPayload from its `len` bytes alone (e.g. once unsealed).
std::optional<HapticPayload> decode_haptic_payload(const std::uint8_t* payload, std::size_t len);
// The receipt map of a kProbeEcho, if it carries one.
std::optional<std::uint64_t> decode_receipts(const std::uint8_t* data, std::size_t size);

}  // namespace tuushin::wire

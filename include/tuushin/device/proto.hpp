#pragma once

#include <cstddef>
#include <cstdint>

// The link between a host and a device's servo loop on a microcontroller,
// over a byte stream (USB CDC, a UART, a pseudo-terminal): what a host
// driver (device/remote.hpp) and the firmware (firmware/) both compile, so
// that the two sides cannot disagree on it. Nothing here allocates, throws
// or touches the operating system.
//
// Framing: each message is [type][body][CRC-16] with the CRC-16/CCITT-FALSE
// of type and body, little-endian, encoded with COBS (Cheshire and Baker,
// IEEE/ACM ToN 7(2), 1999) and terminated by a zero byte. A reader that
// starts mid-stream, or meets bytes that are not frames (a bootloader's
// banner, a log line), loses at most the frame it is in: the next zero
// starts a new one, and the CRC rejects what is not a message. Every field
// is little-endian whatever the machine, written byte by byte.
//
// Roles: the device streams its state at a fixed period on its own clock,
// whatever the host does; the host sends a command whenever it has a new
// torque, which the device applies on its next control tick. A state says
// which command was in effect and since when, so that the host can measure
// the actuation delay, and the device's watchdog cuts the torque when no
// command has come for a while, as the servo loops of haptic devices do,
// and says so in its states.
//
// Units are SI at the device's output shaft: radians, N m, rad/s, joules.
// The position is an encoder count, accumulated over turns: the host needs
// to know the quantization, and a count carries it exactly.

namespace tuushin::device {

constexpr std::uint8_t kProtocolVersion = 1;

// Type and body, before the CRC and the COBS encoding.
constexpr std::size_t kMaxMessage = 64;
// A whole frame on the wire: COBS adds a byte per 254 and the delimiter.
constexpr std::size_t kMaxFrame = kMaxMessage + 2 + 2 + 1;

enum class MsgType : std::uint8_t {
  // host -> device
  kHello = 0x01,      // asks for the device's Info
  kCommand = 0x02,    // Command
  kSet = 0x03,        // Set: a parameter
  kZero = 0x04,       // makes the present position count 0
  kCalibrate = 0x05,  // finds the motor's electrical zero (the shaft must be free)
  // device -> host
  kInfo = 0x81,
  kState = 0x82,
  kAck = 0x83,  // the answer to Set, Zero and Calibrate
  kLog = 0x84,  // a line of text, for a person
};

// Command::flags
constexpr std::uint8_t kCmdEnable = 0x01;      // apply the torque; without it the device idles
constexpr std::uint8_t kCmdClearFault = 0x02;  // clear a latched fault (only while not enabled)

struct Command {
  std::uint16_t seq = 0;
  float torque = 0.0f;  // N m, along +count
  std::uint8_t flags = 0;
};

enum class Param : std::uint8_t {
  kWatchdogUs = 1,     // cut the torque this long after the last command; 0: never
  kTorqueLimit = 2,    // N m, at most the device's max_torque
  kStatePeriodUs = 3,  // how often a State is sent
  kDamping = 4,        // N m s/rad of damping the servo loop adds on its own ticks
  kVelocityLimit = 5,  // rad/s; faster than this is a fault (a runaway)
  kLinkTimeoutUs = 6,  // with no command at all for this long, disable the driver
};

struct Set {
  Param param = Param::kWatchdogUs;
  float value = 0.0f;
};

// Info::kind
enum class DeviceKind : std::uint8_t {
  kTorqueMotor = 0,  // a motor under field-oriented control: commands are torques
  kSimulated = 1,    // the firmware's core on a simulated motor (tuushin-devsim, QEMU)
};

// Info::flags
constexpr std::uint8_t kInfoCalibrated = 0x01;  // the electrical zero is known

struct Info {
  std::uint8_t protocol = kProtocolVersion;
  DeviceKind kind = DeviceKind::kTorqueMotor;
  std::uint8_t pole_pairs = 0;
  std::uint8_t flags = 0;
  std::uint32_t counts_per_rev = 0;
  float max_torque = 0.0f;  // N m the hardware is built for
  float inertia = 0.0f;     // kg m^2 of the moving parts, nominal, 0 if unknown
  std::uint32_t tick_us = 0;  // the servo loop's period
  std::uint32_t firmware = 0;  // version, major << 16 | minor << 8 | patch
  char name[16] = {};          // e.g. "esp32s3/mt6701", NUL-padded
};

// State::flags
constexpr std::uint8_t kStateEnabled = 0x01;     // the driver is on
constexpr std::uint8_t kStateCut = 0x02;         // the watchdog cut the torque
constexpr std::uint8_t kStateFault = 0x04;       // a fault is latched (State::fault)
constexpr std::uint8_t kStateCalibrated = 0x08;  // the electrical zero is known
constexpr std::uint8_t kStateSaturated = 0x10;   // the torque was clamped (limit or voltage)
constexpr std::uint8_t kStateCalibrating = 0x20;

// State::fault, latched until cleared
enum class Fault : std::uint8_t {
  kNone = 0,
  kEncoder = 1,     // the encoder did not answer
  kEncoderJump = 2, // the reading moved further in a tick than the shaft can
  kOverspeed = 3,   // faster than the velocity limit
  kCalibration = 4, // the calibration did not see the motor turn as expected
  kSupply = 5,      // the supply voltage is out of range
};

struct State {
  std::uint16_t seq = 0;
  std::uint32_t time_us = 0;    // the device's clock when it sampled the encoder
  std::int32_t count = 0;       // encoder counts since the zero, over turns
  float velocity = 0.0f;        // rad/s, the servo loop's estimate
  float torque = 0.0f;          // N m the device applies now (after limits, cuts, damping)
  std::uint16_t cmd_seq = 0;    // the command in effect (or cut)
  std::uint16_t cmd_age_us = 0; // how long it has been in effect, saturating
  std::uint16_t cut_after_us = 0;  // if cut: how long after it took effect, saturating
  std::uint8_t flags = 0;
  Fault fault = Fault::kNone;
  // Work done on the shaft by the commanded torque (N m times the
  // displacement while it acted, over every tick) since the device started
  // (J): what the host's energy books try to know, from the device's side.
  double work = 0.0;
};

// Ack::status
enum class AckStatus : std::uint8_t { kOk = 0, kRejected = 1, kFailed = 2 };

struct Ack {
  MsgType of = MsgType::kSet;
  Param param = Param::kWatchdogUs;  // for a Set
  AckStatus status = AckStatus::kOk;
  float value = 0.0f;  // for a Set: the value now in effect
};

constexpr std::size_t kMaxLogText = kMaxMessage - 1;

// CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, no reflection;
// "123456789" -> 0x29B1.
std::uint16_t crc16(const std::uint8_t* data, std::size_t n, std::uint16_t crc = 0xFFFF);

// COBS. encode writes at most n + n / 254 + 1 bytes and no delimiter;
// decode returns the decoded length, or 0 if the input is not valid COBS
// or would not fit in `cap`.
std::size_t cobs_encode(const std::uint8_t* in, std::size_t n, std::uint8_t* out);
std::size_t cobs_decode(const std::uint8_t* in, std::size_t n, std::uint8_t* out, std::size_t cap);

// Writes a complete frame (COBS, delimiter included) of the message with
// `type` and `body` to `out` (at least kMaxFrame bytes). Returns its length,
// 0 if the body is too long.
std::size_t encode_frame(MsgType type, const std::uint8_t* body, std::size_t n, std::uint8_t* out);

// Each writes the whole frame of one message to `out` (kMaxFrame bytes) and
// returns its length.
std::size_t encode(const Command& m, std::uint8_t* out);
std::size_t encode(const Set& m, std::uint8_t* out);
std::size_t encode(const Info& m, std::uint8_t* out);
std::size_t encode(const State& m, std::uint8_t* out);
std::size_t encode(const Ack& m, std::uint8_t* out);
std::size_t encode_log(const char* text, std::uint8_t* out);
// A message without a body: Hello, Zero, Calibrate.
std::size_t encode_bare(MsgType type, std::uint8_t* out);

// A message as the reader passes it on: its type and body, valid until the
// next call of FrameReader::push.
struct Message {
  MsgType type;
  const std::uint8_t* body;
  std::size_t size;
};

// A fault's name for a person: "encoder", "overspeed", ...; "none" for kNone.
const char* fault_name(Fault f);

// Each fills `m` from a message of its type; false if the type or the size
// is wrong. A longer body than this version knows is accepted, and the rest
// ignored, so that a later version can append fields.
bool decode(const Message& msg, Command* m);
bool decode(const Message& msg, Set* m);
bool decode(const Message& msg, Info* m);
bool decode(const Message& msg, State* m);
bool decode(const Message& msg, Ack* m);
// Copies the text of a Log into `text` (cap bytes, NUL-terminated).
bool decode_log(const Message& msg, char* text, std::size_t cap);

// Splits a byte stream into messages.
class FrameReader {
 public:
  // Feeds one byte; true when it completed a frame that holds a valid
  // message, which `out` then describes.
  bool push(std::uint8_t byte, Message* out);

  std::uint32_t frames() const { return frames_; }
  std::uint32_t crc_errors() const { return crc_errors_; }
  // Frames that were not valid COBS, too long, or too short for a CRC.
  std::uint32_t bad_frames() const { return bad_frames_; }

 private:
  std::uint8_t raw_[kMaxFrame];
  std::uint8_t msg_[kMaxFrame];
  std::size_t n_ = 0;
  bool overflow_ = false;
  std::uint32_t frames_ = 0;
  std::uint32_t crc_errors_ = 0;
  std::uint32_t bad_frames_ = 0;
};

}  // namespace tuushin::device

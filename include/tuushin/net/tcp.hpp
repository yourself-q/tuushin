#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tuushin/net/udp.hpp"

namespace tuushin::net {

// A connection that carries a stream of bytes, never blocking: what TURN
// runs over where UDP does not get through (paths.hpp).
class Stream {
 public:
  virtual ~Stream() = default;

  enum class State { kConnecting, kOpen, kClosed };
  // Advances a connection in progress, without waiting.
  virtual State state() = 0;
  // Why it closed, for logs; empty while it has not.
  virtual const std::string& error() const = 0;

  // Takes `size` bytes whole, and writes what the kernel takes now; the
  // rest waits for flush(). False, taking nothing, once closed or if the
  // bytes waiting would pass kMaxBacklog: on a connection that backed up
  // that far, what the bytes carry would be stale before they left.
  virtual bool write(const std::uint8_t* data, std::size_t size) = 0;
  // Writes what waits, as far as the kernel takes it.
  virtual void flush() = 0;
  // Reads what has arrived, up to `capacity` bytes: the count, 0 if nothing
  // has, -1 once the connection is closed (by the other end or an error).
  virtual long read(std::uint8_t* buffer, std::size_t capacity) = 0;
  // Whether bytes have arrived that read() returns without the socket
  // becoming readable again (TLS decrypts a record at a time).
  virtual bool has_buffered() const { return false; }

  // The socket, to wait on; and whether to wait for it to take more bytes
  // too (it is connecting, or bytes are waiting).
  virtual int native_handle() const = 0;
  virtual bool wants_write() const = 0;

  static constexpr std::size_t kMaxBacklog = 64 * 1024;
};

// TCP (POSIX), non-blocking from the connect on, with Nagle's algorithm off
// (TCP_NODELAY): a message leaves when it is written, not when the previous
// one is acknowledged.
class TcpStream : public Stream {
 public:
  // Starts connecting to `to` from `local` (port 0 for an ephemeral one;
  // invalid for any address), marked with `dscp` if it is not negative.
  // Throws std::system_error if no socket can be made or bound; a connect
  // that fails at once leaves the stream closed, with the reason.
  TcpStream(const Endpoint& to, const Endpoint& local, int dscp = -1);
  ~TcpStream() override;
  TcpStream(const TcpStream&) = delete;
  TcpStream& operator=(const TcpStream&) = delete;

  State state() override;
  const std::string& error() const override { return error_; }
  bool write(const std::uint8_t* data, std::size_t size) override;
  void flush() override;
  long read(std::uint8_t* buffer, std::size_t capacity) override;
  int native_handle() const override { return fd_; }
  bool wants_write() const override {
    return state_ == State::kConnecting || (state_ == State::kOpen && head_ < out_.size());
  }

  // This end's address and port, once bound.
  Endpoint local_endpoint() const;

 private:
  void close_with(const std::string& why);

  int fd_ = -1;
  State state_ = State::kConnecting;
  std::string error_;
  std::vector<std::uint8_t> out_;
  std::size_t head_ = 0;  // out_[head_..] waits
};

}  // namespace tuushin::net

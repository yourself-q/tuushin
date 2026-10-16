#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "tuushin/net/paths.hpp"
#include "tuushin/net/udp.hpp"
#include "tuushin/time.hpp"

namespace test {

// The next datagram at `socket` within a second, as text.
inline std::string receive(tuushin::net::UdpSocket& socket, tuushin::net::Endpoint* from) {
  std::uint8_t buf[2048];
  const tuushin::Nanos deadline = tuushin::monotonic_now() + tuushin::kNanosPerSecond;
  while (tuushin::monotonic_now() < deadline) {
    socket.wait_readable(tuushin::kNanosPerMilli);
    if (const auto n = socket.recv_from(buf, sizeof buf, from)) {
      return std::string(reinterpret_cast<char*>(buf), *n);
    }
  }
  return {};
}

// Paths with a relay and a buffer for what arrives, pumped until a
// condition holds.
struct PathsUnderTest {
  explicit PathsUnderTest(tuushin::net::UdpSocket& direct) : paths(direct) {}

  bool pump(const std::function<bool()>& done,
            tuushin::Nanos timeout = 2 * tuushin::kNanosPerSecond) {
    using tuushin::monotonic_now;
    const tuushin::Nanos deadline = monotonic_now() + timeout;
    while (!done() && monotonic_now() < deadline) {
      paths.service(monotonic_now());
      paths.wait_readable(tuushin::kNanosPerMilli);
      paths.service(monotonic_now());
      std::uint8_t buf[2048];
      tuushin::net::Route from;
      while (const auto n = paths.recv(buf, sizeof buf, &from)) {
        received.emplace_back(from, std::string(reinterpret_cast<char*>(buf), *n));
      }
    }
    return done();
  }

  tuushin::net::Paths paths;
  std::vector<std::pair<tuushin::net::Route, std::string>> received;
};

}  // namespace test

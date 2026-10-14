#include "tuushin/time.hpp"

#include <chrono>

namespace tuushin {

Nanos monotonic_now() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace tuushin

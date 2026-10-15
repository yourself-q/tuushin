#include "tuushin/control/reflection.hpp"

#include <algorithm>

namespace tuushin::control {

ReflectiveStore::Grant ReflectiveStore::allocate(double want_slave, double want_master,
                                                 double reserved) {
  want_slave = std::max(0.0, want_slave);
  want_master = std::max(0.0, want_master);
  const double available = std::max(0.0, level());
  const double want = want_slave + want_master;
  Grant g;
  if (want <= available) {
    g.slave = want_slave;
    g.master = want_master;
  } else {
    // want > available >= 0, so want > 0. Scaling both requests by the same
    // factor distributes the excess in proportion to each output, which is
    // Panzirsch et al.'s eqs. (26)-(28).
    const double scale = available / want;
    g.slave = want_slave * scale;
    g.master = want_master * scale;
    g.limited = true;
  }
  const double free = std::max(0.0, level() - std::max(0.0, reserved));
  if (g.master > free) {
    g.master = free;
    g.slave = std::min(want_slave, available - free);
    g.limited = true;
  }
  granted_ += g.master;
  return g;
}

}  // namespace tuushin::control

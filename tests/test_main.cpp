#include <cstring>

#include "test.hpp"

// Usage: tuushin_tests [substring]  -- runs the cases whose name contains it.
int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int ran = 0;
  for (const auto& c : test::registry()) {
    if (filter && !std::strstr(c.name, filter)) continue;
    const int before = test::failures();
    c.fn();
    std::printf("%s %s\n", test::failures() == before ? "ok  " : "FAIL", c.name);
    ++ran;
  }
  std::printf("%d cases, %d failed checks\n", ran, test::failures());
  return test::failures() == 0 && ran > 0 ? 0 : 1;
}

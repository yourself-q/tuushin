#pragma once

// Minimal test harness: no dependency, one binary, every TEST_CASE runs.

#include <cmath>
#include <cstdio>
#include <vector>

namespace test {

struct Case {
  const char* name;
  void (*fn)();
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

inline int& failures() {
  static int count = 0;
  return count;
}

struct Registrar {
  Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

}  // namespace test

#define TEST_CASE(name)                                       \
  static void name();                                         \
  static const ::test::Registrar name##_registrar(#name, &name); \
  static void name()

#define CHECK(cond)                                                              \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      ++::test::failures();                                                      \
    }                                                                            \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                        \
  do {                                                                               \
    const double check_a_ = (a), check_b_ = (b);                                     \
    if (!(std::fabs(check_a_ - check_b_) <= (tol))) {                                \
      std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %.9g, %s = %.9g (tol %g)\n", \
                   __FILE__, __LINE__, #a, check_a_, #b, check_b_, (double)(tol));   \
      ++::test::failures();                                                          \
    }                                                                                \
  } while (0)

#define REQUIRE(cond)                                                              \
  do {                                                                             \
    if (!(cond)) {                                                                 \
      std::fprintf(stderr, "%s:%d: REQUIRE failed: %s\n", __FILE__, __LINE__, #cond); \
      ++::test::failures();                                                        \
      return;                                                                      \
    }                                                                              \
  } while (0)

#include "test.hpp"
#include "tuushin/codec/deadband.hpp"

using tuushin::Vec1;
using tuushin::Vec3;
using tuushin::codec::DeadbandEncoder;
using tuushin::codec::DeadbandParams;

TEST_CASE(deadband_first_sample_is_always_sent) {
  DeadbandEncoder<1> enc(DeadbandParams{0.1, 0.0, 0});
  CHECK(enc.update(Vec1{{0.0}}, 0));
  CHECK(enc.sent_count() == 1);
}

TEST_CASE(deadband_weber_threshold) {
  DeadbandEncoder<1> enc(DeadbandParams{0.1, 0.0, 0});
  CHECK(enc.update(Vec1{{1.0}}, 0));
  CHECK(!enc.update(Vec1{{1.05}}, 1));  // 5 % change: inside the deadband
  // Just under 10 %: still inside. (Exactly 10 % is not tested: 1.1 - 1.0 is
  // not exactly 0.1 in binary floating point.)
  CHECK(!enc.update(Vec1{{1.0999}}, 2));
  CHECK(enc.update(Vec1{{1.11}}, 3));   // beyond 10 % of the last sent value
  CHECK_NEAR(enc.last_sent()[0], 1.11, 1e-12);
  // The reference moved, so the band is now 10 % of 1.11.
  CHECK(!enc.update(Vec1{{1.0}}, 4));
  CHECK(enc.update(Vec1{{0.99}}, 5));
}

TEST_CASE(deadband_absolute_floor_suppresses_noise_near_zero) {
  DeadbandEncoder<1> enc(DeadbandParams{0.1, 0.01, 0});
  CHECK(enc.update(Vec1{{0.0}}, 0));
  CHECK(!enc.update(Vec1{{0.005}}, 1));
  CHECK(!enc.update(Vec1{{-0.009}}, 2));
  CHECK(enc.update(Vec1{{0.02}}, 3));
}

TEST_CASE(deadband_max_interval_forces_refresh) {
  DeadbandEncoder<1> enc(DeadbandParams{0.1, 0.0, 100});
  CHECK(enc.update(Vec1{{1.0}}, 0));
  CHECK(!enc.update(Vec1{{1.0}}, 99));
  CHECK(enc.update(Vec1{{1.0}}, 100));
  CHECK(!enc.update(Vec1{{1.0}}, 150));
  CHECK(enc.update(Vec1{{1.0}}, 200));
}

TEST_CASE(deadband_vector_uses_euclidean_norm) {
  DeadbandEncoder<3> enc(DeadbandParams{0.1, 0.0, 0});
  CHECK(enc.update(Vec3{{3.0, 4.0, 0.0}}, 0));      // |x| = 5, band = 0.5
  CHECK(!enc.update(Vec3{{3.3, 4.3, 0.0}}, 1));     // |dx| = 0.42
  CHECK(enc.update(Vec3{{3.0, 4.0, 0.6}}, 2));      // |dx| = 0.6
}

TEST_CASE(deadband_reset_sends_next_sample) {
  DeadbandEncoder<1> enc(DeadbandParams{0.5, 0.0, 0});
  CHECK(enc.update(Vec1{{1.0}}, 0));
  CHECK(!enc.update(Vec1{{1.1}}, 1));
  enc.reset();
  CHECK(enc.update(Vec1{{1.1}}, 2));
}

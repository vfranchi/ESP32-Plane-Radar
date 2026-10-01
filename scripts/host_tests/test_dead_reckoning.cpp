/**
 * Host test for the dead-reckoning maths.
 *
 * The interesting property on hardware is what happens between polls and when
 * the polls stop, which cannot be provoked on a bench; the maths is plain C++
 * with no Arduino dependency, so it is checked here instead -- see
 * scripts/run_host_tests.sh.
 */
#include <cmath>
#include <cstdio>

#include "services/dead_reckoning.h"
#include "test_util.h"

namespace {

using services::dead_reckoning::extrapolate;
using services::dead_reckoning::kKmPerDeg;
using services::dead_reckoning::kMaxElapsedSec;

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

}  // namespace

int main() {
  float lat = 0.0f;
  float lon = 0.0f;

  // --- zero speed or zero elapsed time leaves the position alone ---
  extrapolate(10.0f, 20.0f, 90.0f, 0.0f, 3.0f, 1.0f, &lat, &lon);
  CHECK(near(lat, 10.0f, 1e-6f) && near(lon, 20.0f, 1e-6f));

  extrapolate(10.0f, 20.0f, 90.0f, 450.0f, 0.0f, 1.0f, &lat, &lon);
  CHECK(near(lat, 10.0f, 1e-6f) && near(lon, 20.0f, 1e-6f));

  // --- track 0 (north) moves +lat only; 360 kt for 1 s = 0.1852 km ---
  extrapolate(0.0f, 0.0f, 0.0f, 360.0f, 1.0f, 1.0f, &lat, &lon);
  CHECK(lat > 0.0f);
  CHECK(near(lon, 0.0f, 1e-6f));
  CHECK(near(lat, 0.1852f / kKmPerDeg, 1e-5f));

  // --- track 90 (east) moves +lon, divided by cos(center_lat) ---
  extrapolate(0.0f, 0.0f, 90.0f, 360.0f, 1.0f, 0.5f, &lat, &lon);
  CHECK(near(lat, 0.0f, 1e-6f));
  CHECK(near(lon, 0.1852f / (kKmPerDeg * 0.5f), 1e-4f));

  // --- track 180 (south) moves -lat only ---
  extrapolate(10.0f, 20.0f, 180.0f, 360.0f, 1.0f, 1.0f, &lat, &lon);
  CHECK(lat < 10.0f);
  CHECK(near(lon, 20.0f, 1e-6f));

  // --- track 270 (west) moves -lon only ---
  extrapolate(10.0f, 20.0f, 270.0f, 360.0f, 1.0f, 1.0f, &lat, &lon);
  CHECK(near(lat, 10.0f, 1e-6f));
  CHECK(lon < 20.0f);

  // --- elapsed time is clamped, so a failed fetch cannot fly a plane away ---
  float far_lat = 0.0f;
  float far_lon = 0.0f;
  extrapolate(0.0f, 0.0f, 0.0f, 360.0f, 60.0f, 1.0f, &far_lat, &far_lon);
  extrapolate(0.0f, 0.0f, 0.0f, 360.0f, kMaxElapsedSec, 1.0f, &lat, &lon);
  CHECK(near(far_lat, lat, 1e-6f));

  // --- a degenerate cos(lat) must not divide by zero ---
  extrapolate(89.9f, 0.0f, 90.0f, 360.0f, 1.0f, 0.0f, &lat, &lon);
  CHECK(std::isfinite(lon));

  // --- null out-pointers are tolerated ---
  extrapolate(1.0f, 2.0f, 3.0f, 400.0f, 1.0f, 1.0f, nullptr, nullptr);

  return testSummary("dead_reckoning");
}

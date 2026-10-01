#pragma once

#include <cmath>

// Dead-reckoning maths, deliberately free of Arduino.h so it can be unit-tested
// on the build machine. Header-only on purpose: scripts/run_host_tests.sh
// compiles each test on its own and links no project object, so a separate .cpp
// would build under PlatformIO and fail the host test with undefined reference.
namespace services::dead_reckoning {

/** Degrees of latitude per km, matching the radar's flat projection. */
constexpr float kKmPerDeg = 111.0f;
/** Knots to km/h. */
constexpr float kKmPerKnot = 1.852f;
/**
 * Longest extrapolation allowed, in seconds. The feed can go quiet (a failed
 * fetch, a dropped link): without this the aircraft keeps flying along its last
 * track for as long as the board is up.
 */
constexpr float kMaxElapsedSec = 4.5f;

namespace detail {

constexpr float kDegToRad = 3.14159265f / 180.0f;
/** Below this, the longitude divisor is treated as a pole and clamped. */
constexpr float kMinCosLat = 0.01f;

}  // namespace detail

/**
 * Dead-reckon a position forward along a ground track.
 *
 * `cos_center_lat` is the same cosine the renderer's flat projection uses for
 * longitude, so the extrapolated point lands on the drawn track line instead of
 * drifting with latitude.
 */
inline void extrapolate(float lat, float lon, float track_deg, float gs_knots,
                        float elapsed_sec, float cos_center_lat, float* out_lat,
                        float* out_lon) {
  if (out_lat == nullptr || out_lon == nullptr) {
    return;
  }

  *out_lat = lat;
  *out_lon = lon;
  if (gs_knots <= 0.0f || elapsed_sec <= 0.0f) {
    return;
  }

  const float dt = elapsed_sec > kMaxElapsedSec ? kMaxElapsedSec : elapsed_sec;
  const float dist_km = gs_knots * kKmPerKnot * dt / 3600.0f;
  const float track_rad = track_deg * detail::kDegToRad;
  const float cos_lat =
      cos_center_lat < detail::kMinCosLat ? detail::kMinCosLat : cos_center_lat;

  *out_lat = lat + dist_km * cosf(track_rad) / kKmPerDeg;
  *out_lon = lon + dist_km * sinf(track_rad) / (kKmPerDeg * cos_lat);
}

}  // namespace services::dead_reckoning

#include "services/dead_reckoning.h"

#include <cmath>

namespace services::dead_reckoning {
namespace {

constexpr float kDegToRad = 3.14159265f / 180.0f;
/** Below this, the longitude divisor is treated as a pole and clamped. */
constexpr float kMinCosLat = 0.01f;

}  // namespace

void extrapolate(float lat, float lon, float track_deg, float gs_knots,
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
  const float track_rad = track_deg * kDegToRad;
  const float cos_lat = cos_center_lat < kMinCosLat ? kMinCosLat : cos_center_lat;

  *out_lat = lat + dist_km * cosf(track_rad) / kKmPerDeg;
  *out_lon = lon + dist_km * sinf(track_rad) / (kKmPerDeg * cos_lat);
}

}  // namespace services::dead_reckoning

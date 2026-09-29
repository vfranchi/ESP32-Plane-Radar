#pragma once

namespace services::dead_reckoning {

/** Degrees of latitude per km, matching the radar's flat projection. */
constexpr float kKmPerDeg = 111.0f;
/** Knots to km/h. */
constexpr float kKmPerKnot = 1.852f;
/** Longest extrapolation allowed, guards against a failed fetch (seconds). */
constexpr float kMaxElapsedSec = 4.5f;

/**
 * Dead-reckon a position forward along a ground track.
 *
 * `cos_center_lat` is the same cosine the renderer's flat projection uses for
 * longitude, so the extrapolated point lands on the drawn track line instead of
 * drifting with latitude.
 */
void extrapolate(float lat, float lon, float track_deg, float gs_knots,
                 float elapsed_sec, float cos_center_lat, float* out_lat,
                 float* out_lon);

}  // namespace services::dead_reckoning

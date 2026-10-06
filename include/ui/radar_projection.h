#pragma once

#include <cmath>

#include "services/radar_location.h"

// Shared flat-earth projection used by the radar grid and the runway overlay.
//
// The ESP32-C3 is RV32IMC: no FPU, so every float op -- and especially cosf() and
// sqrtf() -- is a libgcc software routine. This header exists so the hot paths can
// hoist the ones that do not change per frame and skip the ones they do not need.
namespace ui::projection {

constexpr float kKmPerDeg = 111.0f;
constexpr float kDegToRad = 3.14159265f / 180.0f;

// cos(centre latitude), the east-west scale factor. Recomputed only when the centre
// actually moves: the runway scan used to call cosf() once per airport, which cost
// ~29 ms of a 60 ms scan. Reads centerLat() so follow mode moves it with the aircraft.
inline float cosCenterLat() {
  static double cached_lat = 1e9;
  static float cached_cos = 1.0f;
  const double lat = services::location::centerLat();
  if (lat != cached_lat) {  // also picks up a location saved from the portal
    cached_lat = lat;
    cached_cos = cosf(static_cast<float>(lat) * kDegToRad);
  }
  return cached_cos;
}

// East/north offsets from the radar centre, in km.
inline void eastNorthKm(float lat, float lon, float* dx_km, float* dy_km) {
  *dx_km = static_cast<float>(lon - services::location::centerLon()) * kKmPerDeg *
           cosCenterLat();
  *dy_km = static_cast<float>(lat - services::location::centerLat()) * kKmPerDeg;
}

// Squared distance from the centre (km^2), so radius tests skip the soft-float sqrtf().
inline float distanceSqKm(float lat, float lon) {
  float dx_km = 0.0f;
  float dy_km = 0.0f;
  eastNorthKm(lat, lon, &dx_km, &dy_km);
  return dx_km * dx_km + dy_km * dy_km;
}

}  // namespace ui::projection

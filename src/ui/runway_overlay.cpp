#include "ui/runway_overlay.h"

#include <lgfx/v1/lgfx_fonts.hpp>

#include <cmath>
#include <cstdlib>

#include "data/large_airports.h"
#include "hardware/display_font.h"
#include "services/radar_location.h"
#include "ui/radar_projection.h"
#include "ui/radar_range.h"
#include "ui/radar_theme.h"

namespace ui::runway {
namespace {

constexpr size_t kMaxAirportLabels = 32;

bool s_in_range[data::large_airports::kAirportCount];
bool s_label_pending[data::large_airports::kAirportCount];

constexpr size_t kMaxCachedRunways = 128;  // ~1 KB; a dense 25 km view has far fewer
constexpr size_t kMaxCachedLabels = kMaxAirportLabels;

struct CachedRunway {
  int16_t x0;
  int16_t y0;
  int16_t x1;
  int16_t y1;
};

struct CachedLabel {
  uint16_t ap_idx;
  int16_t x;
  int16_t y;
};

// The visible set and its screen geometry depend only on the centre and the range, but
// finding it costs ~30 ms of soft-float projection per frame. Cache it instead.
struct RunwayCache {
  double center_lat = 1e9;
  double center_lon = 1e9;
  float outer_km = -1.0f;
  size_t airport_count = 0;
  size_t runway_count = 0;
  size_t label_count = 0;
  CachedRunway runways[kMaxCachedRunways];
  CachedLabel labels[kMaxCachedLabels];
};

RunwayCache s_cache;

size_t s_last_airport_count = 0;
size_t s_last_runway_draw_count = 0;

bool cacheIsStale(float outer_km) {
  return s_cache.center_lat != services::location::lat() ||
         s_cache.center_lon != services::location::lon() ||
         s_cache.outer_km != outer_km;
}

bool s_runway_label_ready = false;
bool s_runway_label_use_vlw = false;
float s_runway_label_vlw_size = 0.38f;
const lgfx::GFXfont* s_runway_label_gfx = &fonts::FreeSansBold12pt7b;

int measureVlwHeight(lgfx::LGFXBase& gfx, float size) {
  gfx.setTextSize(size);
  return gfx.fontHeight();
}

float findVlwSizeForHeight(lgfx::LGFXBase& gfx, int target_px) {
  float lo = 0.2f;
  float hi = 1.2f;
  for (int i = 0; i < 14; ++i) {
    const float mid = (lo + hi) * 0.5f;
    if (measureVlwHeight(gfx, mid) < target_px) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return hi;
}

void initRunwayLabelStyle(lgfx::LGFXBase& gfx) {
  if (s_runway_label_ready) {
    return;
  }

  const int target = radar::kRunwayLabelHeightPx;
  if (displayFontIsSmooth()) {
    s_runway_label_use_vlw = true;
    s_runway_label_vlw_size = findVlwSizeForHeight(gfx, target);
  } else {
    s_runway_label_gfx = &fonts::FreeSansBold12pt7b;
    s_runway_label_use_vlw = false;
  }
  s_runway_label_ready = true;
}

void applyRunwayLabelStyle(lgfx::LGFXBase& gfx) {
  if (s_runway_label_use_vlw) {
    displayFontSetSmoothSize(gfx, s_runway_label_vlw_size);
  } else {
    displayFontSetBitmap(gfx, s_runway_label_gfx);
  }
}

float e7ToDeg(int32_t e7) { return static_cast<float>(e7) * 1e-7f; }

void offsetKmFromCenter(float lat, float lon, float* dx_km, float* dy_km,
                        float* dist_km) {
  projection::eastNorthKm(lat, lon, dx_km, dy_km);
  *dist_km = sqrtf((*dx_km) * (*dx_km) + (*dy_km) * (*dy_km));
}

void latLonToScreen(float lat, float lon, int* out_x, int* out_y) {
  const float outer_km = radar::rangeCurrent().outer_km;
  const float px_per_km =
      static_cast<float>(radar::kGridOuterRadius) / outer_km;

  float dx_km = 0.0f;
  float dy_km = 0.0f;
  float dist_km = 0.0f;
  offsetKmFromCenter(lat, lon, &dx_km, &dy_km, &dist_km);

  *out_x = radar::kCenterX + static_cast<int>(lroundf(dx_km * px_per_km));
  *out_y = radar::kCenterY - static_cast<int>(lroundf(dy_km * px_per_km));
}

int distSqFromCenter(int x, int y) {
  const int dx = x - radar::kCenterX;
  const int dy = y - radar::kCenterY;
  return dx * dx + dy * dy;
}

void clipPointToOuterRing(int x0, int y0, int* x1, int* y1) {
  const int max_r = radar::kGridOuterRadius;
  const int max_r_sq = max_r * max_r;
  if (distSqFromCenter(*x1, *y1) <= max_r_sq) {
    return;
  }

  const int dx = *x1 - x0;
  const int dy = *y1 - y0;
  float t = 1.0f;
  for (int step = 0; step < 20; ++step) {
    const int px = x0 + static_cast<int>(lroundf(dx * t));
    const int py = y0 + static_cast<int>(lroundf(dy * t));
    if (distSqFromCenter(px, py) <= max_r_sq) {
      *x1 = px;
      *y1 = py;
      return;
    }
    t -= 0.05f;
    if (t <= 0.0f) {
      *x1 = x0;
      *y1 = y0;
      return;
    }
  }
}

bool segmentIntersectsDisc(int x0, int y0, int x1, int y1) {
  const int cx = radar::kCenterX;
  const int cy = radar::kCenterY;
  const int r = radar::kGridOuterRadius;
  const int r_sq = r * r;

  if (distSqFromCenter(x0, y0) <= r_sq || distSqFromCenter(x1, y1) <= r_sq) {
    return true;
  }

  const int dx = x1 - x0;
  const int dy = y1 - y0;
  const int fx = x0 - cx;
  const int fy = y0 - cy;
  const int a = dx * dx + dy * dy;
  if (a == 0) {
    return false;
  }
  const int b = 2 * (fx * dx + fy * dy);
  const int c = fx * fx + fy * fy - r_sq;
  int disc = b * b - 4 * a * c;
  if (disc < 0) {
    return false;
  }
  disc = static_cast<int>(sqrtf(static_cast<float>(disc)));
  const float inv2a = 1.0f / (2.0f * static_cast<float>(a));
  const float t0 = (-static_cast<float>(b) - disc) * inv2a;
  const float t1 = (-static_cast<float>(b) + disc) * inv2a;
  return (t0 >= 0.0f && t0 <= 1.0f) || (t1 >= 0.0f && t1 <= 1.0f);
}

void drawBoldRunwayLabel(lgfx::LGFXBase& gfx, const char* ident, int mx, int my) {
  const int tw = gfx.textWidth(ident);
  const int th = gfx.fontHeight();
  constexpr int kPadX = 2;
  constexpr int kPadY = 1;

  gfx.setTextDatum(textdatum_t::bottom_center);
  const int left = mx - tw / 2 - kPadX;
  const int top = my - th - kPadY;
  gfx.fillRect(left, top, tw + kPadX * 2, th + kPadY, radar::kColorBackground);
  gfx.setTextColor(radar::kColorRunwayLabel, radar::kColorBackground);
  gfx.drawString(ident, mx - 1, my);
  gfx.drawString(ident, mx + 1, my);
  gfx.drawString(ident, mx, my);
}

// Screen segment for one runway, clipped to the visible ring. Geometry only, so the
// result can be cached; false means the runway is not visible at this centre/range.
bool computeRunwaySegment(const data::large_airports::Runway& rw,
                          CachedRunway* out) {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  latLonToScreen(e7ToDeg(rw.le_lat_e7), e7ToDeg(rw.le_lon_e7), &x0, &y0);
  latLonToScreen(e7ToDeg(rw.he_lat_e7), e7ToDeg(rw.he_lon_e7), &x1, &y1);

  if (!segmentIntersectsDisc(x0, y0, x1, y1)) {
    return false;
  }

  clipPointToOuterRing(x0, y0, &x1, &y1);
  clipPointToOuterRing(x1, y1, &x0, &y0);

  out->x0 = static_cast<int16_t>(x0);
  out->y0 = static_cast<int16_t>(y0);
  out->x1 = static_cast<int16_t>(x1);
  out->y1 = static_cast<int16_t>(y1);
  return true;
}

void offsetLabelFromCenter(int ax, int ay, int* lx, int* ly) {
  const int dx = ax - radar::kCenterX;
  const int dy = ay - radar::kCenterY;
  const float len = sqrtf(static_cast<float>(dx * dx + dy * dy));
  const int gap = radar::kRunwayLabelGapPx;
  if (len < 1.0f) {
    *lx = ax;
    *ly = ay - gap;
    return;
  }
  *lx = ax + static_cast<int>(lroundf(dx / len * static_cast<float>(gap)));
  *ly = ay + static_cast<int>(lroundf(dy / len * static_cast<float>(gap)));
}

void clipPointOntoOuterRing(int* x, int* y) {
  const int cx = radar::kCenterX;
  const int cy = radar::kCenterY;
  const int r = radar::kGridOuterRadius;
  const int dx = *x - cx;
  const int dy = *y - cy;
  const int d_sq = dx * dx + dy * dy;
  const int r_sq = r * r;
  if (d_sq <= r_sq || d_sq == 0) {
    return;
  }
  const float scale = static_cast<float>(r) / sqrtf(static_cast<float>(d_sq));
  *x = cx + static_cast<int>(lroundf(static_cast<float>(dx) * scale));
  *y = cy + static_cast<int>(lroundf(static_cast<float>(dy) * scale));
}

// Two integer Bresenham passes keep the 2 px weight. drawWideLine costs several ms
// per call here: it does per-pixel float coverage over the line's bounding box, and
// this chip has no FPU, so two diagonal runways measured 7 ms of the frame.
void drawRunwayThickLine(lgfx::LGFXBase& gfx, const CachedRunway& r) {
  gfx.drawLine(r.x0, r.y0, r.x1, r.y1, radar::kColorRunway);
  if (abs(r.x1 - r.x0) >= abs(r.y1 - r.y0)) {
    gfx.drawLine(r.x0, r.y0 + 1, r.x1, r.y1 + 1, radar::kColorRunway);
  } else {
    gfx.drawLine(r.x0 + 1, r.y0, r.x1 + 1, r.y1, radar::kColorRunway);
  }
}

void airportLabelPos(const data::large_airports::Airport& ap, CachedLabel* out) {
  int ax = 0;
  int ay = 0;
  latLonToScreen(e7ToDeg(ap.lat_e7), e7ToDeg(ap.lon_e7), &ax, &ay);
  clipPointOntoOuterRing(&ax, &ay);

  int lx = 0;
  int ly = 0;
  offsetLabelFromCenter(ax, ay, &lx, &ly);
  out->x = static_cast<int16_t>(lx);
  out->y = static_cast<int16_t>(ly);
}

// Scans every airport and runway, resolves the visible geometry, and stores it.
// Expensive (soft-float projection over 1166 airports) so it only runs when the
// centre or the range changes.
void rebuildCache(float radius_km, float outer_km) {
  s_cache.center_lat = services::location::lat();
  s_cache.center_lon = services::location::lon();
  s_cache.outer_km = outer_km;
  s_cache.airport_count = 0;
  s_cache.runway_count = 0;
  s_cache.label_count = 0;

  for (size_t i = 0; i < data::large_airports::kAirportCount; ++i) {
    s_in_range[i] = false;
    s_label_pending[i] = false;
  }

  const float radius_sq = radius_km * radius_km;
  for (size_t i = 0; i < data::large_airports::kRunwayCount; ++i) {
    const auto& rw = data::large_airports::kRunways[i];
    const uint16_t ap_idx = rw.airport_idx;
    if (!s_in_range[ap_idx]) {
      const auto& ap = data::large_airports::kAirports[ap_idx];
      // Squared compare: same test, and skips a soft-float sqrtf() per airport.
      s_in_range[ap_idx] =
          projection::distanceSqKm(e7ToDeg(ap.lat_e7), e7ToDeg(ap.lon_e7)) <=
          radius_sq;
      if (s_in_range[ap_idx]) {
        ++s_cache.airport_count;
      }
    }
    if (!s_in_range[ap_idx]) {
      continue;
    }
    if (s_cache.runway_count >= kMaxCachedRunways) {
      break;
    }
    CachedRunway seg{};
    if (!computeRunwaySegment(rw, &seg)) {
      continue;
    }
    s_cache.runways[s_cache.runway_count++] = seg;
    if (!s_label_pending[ap_idx] && s_cache.label_count < kMaxCachedLabels) {
      s_label_pending[ap_idx] = true;
      CachedLabel label{};
      label.ap_idx = ap_idx;
      airportLabelPos(data::large_airports::kAirports[ap_idx], &label);
      s_cache.labels[s_cache.label_count++] = label;
    }
  }
}

}  // namespace

void drawLargeAirportRunways(lgfx::LGFXBase& gfx) {
  if (!radar::showRunways()) {
    return;
  }
  displayFontEnsureLoaded(gfx);

  const float outer_km = radar::rangeCurrent().outer_km;
  if (cacheIsStale(outer_km)) {
    rebuildCache(radar::fetchRadiusKm(), outer_km);
  }

  s_last_airport_count = s_cache.airport_count;
  s_last_runway_draw_count = s_cache.runway_count;

  for (size_t i = 0; i < s_cache.runway_count; ++i) {
    drawRunwayThickLine(gfx, s_cache.runways[i]);
  }

  if (s_cache.label_count == 0) {
    return;
  }

  initRunwayLabelStyle(gfx);
  applyRunwayLabelStyle(gfx);
  for (size_t i = 0; i < s_cache.label_count; ++i) {
    const CachedLabel& l = s_cache.labels[i];
    drawBoldRunwayLabel(gfx, data::large_airports::kAirports[l.ap_idx].ident,
                        l.x, l.y);
  }
}

size_t lastAirportCount() { return s_last_airport_count; }

size_t lastRunwayDrawCount() { return s_last_runway_draw_count; }

}  // namespace ui::runway

#include "services/flight_follow.h"

#include <Arduino.h>
#include <Preferences.h>

#include <cstring>

namespace services::follow {
namespace {

// The follow target is a radar setting: same namespace as range/units, so it is reset by
// the same credential wipe. Handles are opened per operation (never held) because
// ui::radar owns the same namespace and two open handles would conflict.
constexpr char kPrefsNamespace[] = "planeradar";
constexpr char kPrefsFollowKey[] = "follow";

Target s_target;
Session s_session;
Info s_info;
Trail s_trail;
RouteCache s_route_cache;

float s_origin_lat = 0.0f;
float s_origin_lon = 0.0f;
bool s_has_origin = false;
float s_dest_lat = 0.0f;
float s_dest_lon = 0.0f;
bool s_has_dest = false;

// Distance trend: the destination's distance must shrink. Two samples per poll, kept only
// while the aircraft is airborne, because that is when the geometry means something.
float s_trend_dest_km = 0.0f;
float s_trend_origin_km = 0.0f;
bool s_trend_valid = false;
int s_trend_away = 0;

/** Point the module at `id` (empty string = off) and drop the per-flight state. */
void adoptTarget(const char* id) {
  s_session.reset();
  s_trail.clear();
  s_route_cache.reset();
  s_origin_lat = 0.0f;
  s_origin_lon = 0.0f;
  s_has_origin = false;
  s_dest_lat = 0.0f;
  s_dest_lon = 0.0f;
  s_has_dest = false;
  s_trend_valid = false;
  s_trend_away = 0;

  s_target = Target();
  s_info = Info();
  if (id == nullptr || id[0] == '\0') {
    return;
  }
  s_target.active = true;
  s_target.is_hex = looksLikeHex(id);
  strncpy(s_target.id, id, sizeof(s_target.id) - 1);
  strncpy(s_info.id, id, sizeof(s_info.id) - 1);
  s_info.state = State::kNotSeen;
}

/**
 * Swaps which end is called the destination, so the panel names the airport the aircraft is
 * actually flying to and the ETA stops counting down to the one behind it.
 */
void swapRouteEnds(const char* why) {
  char code[kCodeLen] = {};
  memcpy(code, s_info.origin, sizeof(code));
  memcpy(s_info.origin, s_info.destination, sizeof(code));
  memcpy(s_info.destination, code, sizeof(code));
  const float lat = s_origin_lat;
  const float lon = s_origin_lon;
  s_origin_lat = s_dest_lat;
  s_origin_lon = s_dest_lon;
  s_dest_lat = lat;
  s_dest_lon = lon;
  s_trend_away = 0;
  Serial.printf("follow: %s, route now %s > %s\n", why, s_info.origin, s_info.destination);
}

/**
 * Watches the two airport distances against the previous poll. A destination whose distance
 * grows while the other end's shrinks is the wrong end, whichever way the lookup named it.
 * The streak requirement keeps a hold or a single noisy position from flipping the display.
 */
void trackRouteTrend(float lat, float lon) {
  const float d_dest = s_info.to_dest_km;
  const float d_origin = greatCircleKm(lat, lon, s_origin_lat, s_origin_lon);
  if (s_trend_valid) {
    if (routeTrendSaysReversed(s_trend_dest_km, s_trend_origin_km, d_dest, d_origin)) {
      if (++s_trend_away >= kRouteTrendPolls) {
        swapRouteEnds("flying the other way round");
      }
    } else if (routeTrendSaysClosing(s_trend_dest_km, s_trend_origin_km, d_dest, d_origin)) {
      s_trend_away = 0;
    }
  }
  s_trend_dest_km = d_dest;
  s_trend_origin_km = d_origin;
  s_trend_valid = true;
}

}  // namespace

void init() {
  char buf[kIdLen] = {};
  Preferences prefs;
  if (prefs.begin(kPrefsNamespace, true)) {
    prefs.getString(kPrefsFollowKey, buf, sizeof(buf));
    prefs.end();
  }
  char id[kIdLen] = {};
  normalizeId(buf, id, sizeof(id));
  adoptTarget(id);
  if (s_target.active) {
    Serial.printf("follow: restored target %s (%s)\n", s_target.id,
                  s_target.is_hex ? "hex" : "callsign");
  }
}

void setTargetFromPortal(const char* text) {
  char id[kIdLen] = {};
  normalizeId(text, id, sizeof(id));

  Preferences prefs;
  if (prefs.begin(kPrefsNamespace, false)) {
    if (id[0] == '\0') {
      prefs.remove(kPrefsFollowKey);
    } else {
      prefs.putString(kPrefsFollowKey, id);
    }
    prefs.end();
  }

  adoptTarget(id);

  if (s_target.active) {
    Serial.printf("follow: target %s (%s)\n", s_target.id,
                  s_target.is_hex ? "hex" : "callsign");
  } else {
    Serial.println("follow: off");
  }
}

void reset() { setTargetFromPortal(""); }

const Target& target() { return s_target; }
const Session& session() { return s_session; }
const Info& info() { return s_info; }
const Trail& trail() { return s_trail; }

void onReport(bool found, bool airborne, float lat, float lon, float gs_knots,
              unsigned long now_ms) {
  s_info.state = s_session.update(found, airborne, now_ms);
  s_info.gs_knots = found ? gs_knots : 0.0f;
  s_info.found = found;
  s_info.since_seen_min = found ? 0.0f : s_session.minutesSinceSeen(now_ms);
  s_info.to_dest_km =
      (found && s_has_dest) ? greatCircleKm(lat, lon, s_dest_lat, s_dest_lon) : 0.0f;
  if (found && airborne) {
    s_trail.push(lat, lon, kTrailMinMoveKm);
    trackRouteTrend(lat, lon);
  }
}

void setRoute(const char* origin_code, const char* dest_code, float origin_lat, float origin_lon,
              float dest_lat, float dest_lon, float route_km) {
  snprintf(s_info.origin, sizeof(s_info.origin), "%s", origin_code != nullptr ? origin_code : "");
  snprintf(s_info.destination, sizeof(s_info.destination), "%s",
           dest_code != nullptr ? dest_code : "");
  s_info.route_known = dest_code != nullptr && dest_code[0] != '\0';
  s_info.route_km = route_km;
  s_origin_lat = origin_lat;
  s_origin_lon = origin_lon;
  s_dest_lat = dest_lat;
  s_dest_lon = dest_lon;
  s_has_origin = s_info.route_known && (origin_lat != 0.0f || origin_lon != 0.0f);
  s_has_dest = s_info.route_known && (dest_lat != 0.0f || dest_lon != 0.0f);
  // A new route starts its own trend: the previous samples belonged to other coordinates.
  s_trend_valid = false;
  s_trend_away = 0;
  s_route_cache.succeed(millis(), currentDay());
  Serial.printf("follow: route %s > %s (%.0f km)\n", s_info.origin, s_info.destination,
                static_cast<double>(route_km));
}

bool routeWanted() {
  return s_target.active &&
         s_route_cache.wantedAt(millis(), currentDay(), s_target.is_hex);
}

void noteRouteAttempt() { s_route_cache.noteAttempt(millis()); }

void noteRouteFailure() {
  s_route_cache.fail();
  s_info.route_known = false;
  s_info.route_km = 0.0f;
  s_has_dest = false;
}

void noteRouteDeferred() {
  // No fail() and no clearing: a skipped lookup is not an answer, so whatever route is
  // already on screen stays there while the retry window is shortened.
  s_route_cache.defer(millis());
}

unsigned long currentDay() { return millis() / 86400000UL; }

}  // namespace services::follow

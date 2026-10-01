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

/** Point the module at `id` (empty string = off) and drop the per-flight state. */
void adoptTarget(const char* id) {
  s_session.reset();
  s_trail.clear();
  s_route_cache.reset();
  s_dest_lat = 0.0f;
  s_dest_lon = 0.0f;
  s_has_dest = false;

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
  s_info.to_dest_km =
      (found && s_has_dest) ? greatCircleKm(lat, lon, s_dest_lat, s_dest_lon) : 0.0f;
  if (found && airborne) {
    s_trail.push(lat, lon, kTrailMinMoveKm);
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
  s_route_cache.succeed(millis(), currentDay());
  Serial.printf("follow: route %s > %s (%.0f km)\n", s_info.origin, s_info.destination,
                static_cast<double>(route_km));
}

void orientRouteToPosition(float ac_lat, float ac_lon, float heading_deg, bool heading_valid) {
  // A route with no positions cannot be checked, and a valid heading is what makes the check
  // meaningful at all.
  if (!heading_valid || !s_has_origin || !s_has_dest) {
    return;
  }
  if (!routeLooksReversed(ac_lat, ac_lon, heading_deg, s_origin_lat, s_origin_lon, s_dest_lat,
                          s_dest_lon)) {
    return;
  }

  // Swap the ends: same airports, right way round for this leg. to_dest_km follows from the
  // new destination on the next poll, so the ETA stops pointing behind the aircraft.
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
  Serial.printf("follow: heading says the other leg, route now %s > %s\n", s_info.origin,
                s_info.destination);
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

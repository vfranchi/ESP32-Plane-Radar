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
RouteCache s_route;

/** Point the module at `id` (empty string = off) and drop the per-flight state. */
void adoptTarget(const char* id) {
  s_session.reset();
  s_trail.clear();
  s_route.reset();

  // The wall clock belongs to the board, not to the flight: a new target must not have
  // to wait for another response to learn what time it is.
  const services::fa::Clock clock = s_info.clock;
  s_target = Target();
  s_info = Info();
  s_info.clock = clock;
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

/** Uppercase copy of an airport code; an empty string stays "unknown". */
void copyCode(const char* src, char* out) {
  out[0] = '\0';
  if (src == nullptr) {
    return;
  }
  size_t w = 0;
  for (const char* p = src; *p != '\0' && w + 1 < kCodeLen; ++p) {
    out[w++] = (*p >= 'a' && *p <= 'z') ? static_cast<char>(*p - 'a' + 'A') : *p;
  }
  out[w] = '\0';
}

void setClock(long clock_epoch) {
  if (clock_epoch > 0) {
    s_info.clock.set(clock_epoch, millis());
  }
}

void setRouteFromLookup(const char* origin, const char* dest, long arrival_epoch,
                        long clock_epoch) {
  setClock(clock_epoch);
  copyCode(origin, s_info.origin);
  copyCode(dest, s_info.destination);
  s_info.route_known = s_info.origin[0] != '\0' && s_info.destination[0] != '\0';
  s_info.arrival_epoch = arrival_epoch > 0 ? arrival_epoch : 0;

  const unsigned long now = millis();
  s_route.succeed(now, services::fa::dayOf(s_info.clock.nowAt(now)));
  Serial.printf("follow: route %s > %s, arrival %s\n",
                s_info.origin[0] != '\0' ? s_info.origin : "?",
                s_info.destination[0] != '\0' ? s_info.destination : "?",
                s_info.arrival_epoch != 0 ? "known" : "unknown");
}

void clearRoute() {
  s_info.origin[0] = '\0';
  s_info.destination[0] = '\0';
  s_info.route_known = false;
  s_info.arrival_epoch = 0;
  s_route.reset();
}

bool routeWanted() {
  if (!s_target.active) {
    return false;
  }
  const unsigned long now = millis();
  return s_route.wantedAt(now, services::fa::dayOf(s_info.clock.nowAt(now)));
}

void noteRouteAttempt() {
  const unsigned long now = millis();
  s_route.noteAttempt(now, services::fa::dayOf(s_info.clock.nowAt(now)));
}

void noteRouteDeferred() { s_route.defer(millis()); }

void noteRouteFailure() { s_route.fail(); }

const RouteCache& routeCache() { return s_route; }

float etaMinutesAt(unsigned long now_ms) {
  if (!s_info.route_known || s_info.arrival_epoch <= 0) {
    return -1.0f;
  }
  return services::fa::minutesUntil(s_info.arrival_epoch, s_info.clock.nowAt(now_ms));
}

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
  if (found && airborne) {
    s_trail.push(lat, lon, kTrailMinMoveKm);
  }
}

}  // namespace services::follow

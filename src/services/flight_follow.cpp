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

/** Point the module at `id` (empty string = off) and drop the per-flight state. */
void adoptTarget(const char* id) {
  s_session.reset();
  s_trail.clear();

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
  s_info.found = found;
  s_info.since_seen_min = found ? 0.0f : s_session.minutesSinceSeen(now_ms);
  if (found && airborne) {
    s_trail.push(lat, lon, kTrailMinMoveKm);
  }
}

}  // namespace services::follow

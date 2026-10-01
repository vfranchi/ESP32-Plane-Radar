#pragma once

#include <cmath>
#include <cstddef>

// Follow-a-flight mode: target identity, session state, trail and ETA maths.
//
// Free of Arduino, NVS and LovyanGFX so scripts/run_host_tests.sh can compile it
// straight with the host compiler: the part that decides *what* to show is testable
// off-board, only the network and the drawing are not.
namespace services::follow {

constexpr size_t kIdLen = 9;      // callsign (8 + NUL) or 6 hex + NUL
constexpr size_t kIataLen = 4;    // IATA code + NUL
constexpr size_t kTrailMax = 64;  // ~5.3 min of trail at the 5 s poll

constexpr float kKmPerDeg = 111.0f;
constexpr float kKmPerKnot = 1.852f;
/** Nominal cruise for a flight that has not departed: an estimate, labelled as one. */
constexpr float kCruiseKmh = 750.0f;
/** Below this ground speed the aircraft is taxiing: treat it as on the ground. */
constexpr float kAirborneGsKnots = 40.0f;
/** Trail samples closer than this to the previous one are jitter, not motion. */
constexpr float kTrailMinMoveKm = 0.1f;
/** Cached route lifetime. A callsign is reused day to day, hence also the day check. */
constexpr unsigned long kRouteTtlMs = 12UL * 60UL * 60UL * 1000UL;
/** Minimum spacing between failed route lookups: never hammer a foreign API. */
constexpr unsigned long kRouteRetryMs = 300000UL;

enum class State {
  kIdle,      // no target configured
  kNotSeen,   // target configured, never seen in the feed
  kGrounded,  // seen on the ground, not airborne yet this session
  kLive,      // airborne right now
  kLanded,    // was airborne this session, now on the ground or gone
};

struct Target {
  bool active = false;
  bool is_hex = false;
  char id[kIdLen] = {};
};

/** Trim and upper-case a feed or portal identifier. Always NUL-terminates. */
inline void normalizeId(const char* raw, char* out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return;
  }
  out[0] = '\0';
  if (raw == nullptr) {
    return;
  }
  const char* p = raw;
  while (*p == ' ' || *p == '\t') {
    ++p;  // skip leading blanks
  }
  size_t w = 0;
  while (*p != '\0' && w + 1 < out_len) {
    const char c = *p;
    if (c == ' ' || c == '\t') {
      break;  // the ADS-B feed pads callsigns with trailing spaces
    }
    out[w++] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
    ++p;
  }
  out[w] = '\0';
}

/** A 6-character hex string is an ICAO address; anything else is a callsign. */
inline bool looksLikeHex(const char* id) {
  if (id == nullptr) {
    return false;
  }
  size_t n = 0;
  while (id[n] != '\0') {
    const char c = id[n];
    const bool is_hex_char = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
                             (c >= 'a' && c <= 'f');
    if (!is_hex_char) {
      return false;
    }
    ++n;
  }
  return n == 6;
}

inline State deriveState(bool found, bool airborne, bool ever_airborne) {
  if (!found) {
    return ever_airborne ? State::kLanded : State::kNotSeen;
  }
  if (airborne) {
    return State::kLive;
  }
  return ever_airborne ? State::kLanded : State::kGrounded;
}

/** Per-follow session memory: the only way to tell "landed" from "not started". */
struct Session {
  bool ever_airborne = false;
  unsigned long first_air_ms = 0;
  unsigned long last_air_ms = 0;
  unsigned long landed_ms = 0;
  unsigned long ground_since_ms = 0;

  void reset() {
    ever_airborne = false;
    first_air_ms = 0;
    last_air_ms = 0;
    landed_ms = 0;
    ground_since_ms = 0;
  }

  /** Feed one poll result; returns the state the screen should show. */
  State update(bool found, bool airborne, unsigned long now_ms) {
    if (airborne) {
      if (!ever_airborne) {
        ever_airborne = true;
        first_air_ms = now_ms;
      }
      last_air_ms = now_ms;
      ground_since_ms = 0;
      return State::kLive;
    }
    if (found && ground_since_ms == 0) {
      ground_since_ms = now_ms;
    }
    const State state = deriveState(found, false, ever_airborne);
    if (state == State::kLanded && landed_ms == 0) {
      // First ground/no-contact sighting after departure: the observed touchdown.
      landed_ms = ground_since_ms != 0 ? ground_since_ms : now_ms;
    }
    return state;
  }

  /** Observed block time in minutes; 0 before departure or right after take-off. */
  float airMinutes() const {
    if (!ever_airborne || last_air_ms <= first_air_ms) {
      return 0.0f;
    }
    return static_cast<float>(last_air_ms - first_air_ms) / 60000.0f;
  }
};

/** Estimated time to go, in minutes; negative when the inputs are unusable. */
inline float etaMinutes(float to_dest_km, float gs_knots) {
  if (gs_knots <= 0.0f || to_dest_km <= 0.0f) {
    return -1.0f;
  }
  return to_dest_km / (gs_knots * kKmPerKnot) * 60.0f;
}

/** Estimated block time for a flight that has not departed. */
inline float estimatedTripMinutes(float route_km) {
  if (route_km <= 0.0f) {
    return -1.0f;
  }
  return route_km / kCruiseKmh * 60.0f;
}

/** Haversine: called once per poll, never per redraw (soft-float sqrtf/atan2f). */
inline float greatCircleKm(float lat1, float lon1, float lat2, float lon2) {
  const float kDegToRad = 3.14159265f / 180.0f;
  const float dlat = (lat2 - lat1) * kDegToRad;
  const float dlon = (lon2 - lon1) * kDegToRad;
  const float a = sinf(dlat * 0.5f) * sinf(dlat * 0.5f) +
                  cosf(lat1 * kDegToRad) * cosf(lat2 * kDegToRad) *
                      sinf(dlon * 0.5f) * sinf(dlon * 0.5f);
  const float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return 6371.0f * c;
}

/** Ring buffer of world positions; index 0 is always the oldest sample. */
struct Trail {
  float lat[kTrailMax] = {};
  float lon[kTrailMax] = {};
  size_t count = 0;
  size_t head = 0;  // next write slot

  size_t size() const { return count; }

  void clear() {
    count = 0;
    head = 0;
  }

  /** Append, unless the sample sits within min_move_km of the newest one. */
  void push(float new_lat, float new_lon, float min_move_km) {
    if (count > 0) {
      const size_t last = (head + kTrailMax - 1) % kTrailMax;
      if (greatCircleKm(lat[last], lon[last], new_lat, new_lon) < min_move_km) {
        return;
      }
    }
    lat[head] = new_lat;
    lon[head] = new_lon;
    head = (head + 1) % kTrailMax;
    if (count < kTrailMax) {
      ++count;
    }
  }

  /** Oldest-first access; i must be < size(). */
  void at(size_t i, float* out_lat, float* out_lon) const {
    if (i >= count) {
      *out_lat = 0.0f;
      *out_lon = 0.0f;
      return;
    }
    const size_t start = (head + kTrailMax - count) % kTrailMax;
    const size_t slot = (start + i) % kTrailMax;
    *out_lat = lat[slot];
    *out_lon = lon[slot];
  }
};

/**
 * Purely computational half of the route lookup: when is a cached route still good, and
 * when should another one be fetched.
 *
 * The day is part of the route's identity because the same callsign flies a different
 * city pair on a different day, so a TTL alone would serve yesterday's airports.
 */
struct RouteCache {
  bool valid = false;
  unsigned long fetched_ms = 0;
  unsigned long fetched_day = 0;
  bool attempted = false;
  unsigned long last_attempt_ms = 0;

  void reset() { *this = RouteCache(); }

  bool usableAt(unsigned long now_ms, unsigned long day) const {
    if (!valid) {
      return false;
    }
    if (day != fetched_day) {
      return false;  // a new day: the callsign may be flying somewhere else now
    }
    return (now_ms - fetched_ms) < kRouteTtlMs;
  }

  bool wantedAt(unsigned long now_ms, unsigned long day, bool target_is_hex) const {
    if (target_is_hex) {
      return false;  // adsbdb resolves callsigns, not ICAO hex addresses
    }
    if (usableAt(now_ms, day)) {
      return false;
    }
    if (attempted && (now_ms - last_attempt_ms) < kRouteRetryMs) {
      return false;  // failed recently: back off
    }
    return true;
  }

  void noteAttempt(unsigned long now_ms) {
    attempted = true;
    last_attempt_ms = now_ms;
  }

  void succeed(unsigned long now_ms, unsigned long day) {
    valid = true;
    fetched_ms = now_ms;
    fetched_day = day;
  }

  void fail() { valid = false; }
};

}  // namespace services::follow

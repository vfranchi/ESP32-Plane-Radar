#pragma once

#include <cmath>
#include <cstddef>

#include "services/fa_route.h"

// Follow-a-flight mode: target identity, session state and trail.
//
// Free of Arduino, NVS and LovyanGFX so scripts/run_host_tests.sh can compile it
// straight with the host compiler: the part that decides *what* to show is testable
// off-board, only the network and the drawing are not.
namespace services::follow {

constexpr size_t kIdLen = 9;      // callsign (8 + NUL) or 6 hex + NUL
constexpr size_t kTrailMax = 64;  // ~5.3 min of trail at the 5 s poll

/** Below this ground speed the aircraft is taxiing: treat it as on the ground. */
constexpr float kAirborneGsKnots = 40.0f;
/** Trail samples closer than this to the previous one are jitter, not motion. */
constexpr float kTrailMinMoveKm = 0.1f;

/** Display length of an airport code: IATA (3) or ICAO (4) + NUL. */
constexpr size_t kCodeLen = services::fa::kCodeLen;
/** Cached route lifetime. A callsign flies one leg a day, hence also the day check. */
constexpr unsigned long kRouteTtlMs = 12UL * 60UL * 60UL * 1000UL;
/** Minimum spacing between failed route lookups: never hammer a foreign API. */
constexpr unsigned long kRouteRetryMs = 300000UL;
/**
 * Retry window after a lookup that never ran because the heap had no room for the TLS
 * handshake. Buffers are recycled within a minute, unlike a real API failure -- but "a
 * minute" was a flat 60 s and that is too coarse to catch the window: measured on the
 * board the first boot attempt missed the floor by 12 bytes and the very next one, 60 s
 * later, had 45 KB. The wait escalates 10 s -> 20 s -> 40 s and caps here, so a transient
 * low block heals in one or two tries while a persistently starved heap still backs off.
 */
constexpr unsigned long kRouteDeferMs = 60000UL;
/** First wait after a heap deferral; doubles per consecutive deferral up to kRouteDeferMs. */
constexpr unsigned long kRouteDeferFirstMs = 10000UL;
/**
 * Hard daily budget. AeroAPI's Personal tier charges $0.005 per result set and gives $5
 * of free usage a month, so 12 lookups a day is $0.06/day: inside the credit even when
 * every single lookup fails and retries.
 */
constexpr unsigned kRouteLookupsPerDay = 12;

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
  bool ever_seen = false;
  unsigned long first_air_ms = 0;
  unsigned long last_air_ms = 0;
  unsigned long last_seen_ms = 0;
  unsigned long landed_ms = 0;
  unsigned long ground_since_ms = 0;

  void reset() {
    ever_airborne = false;
    ever_seen = false;
    first_air_ms = 0;
    last_air_ms = 0;
    last_seen_ms = 0;
    landed_ms = 0;
    ground_since_ms = 0;
  }

  /** Feed one poll result; returns the state the screen should show. */
  State update(bool found, bool airborne, unsigned long now_ms) {
    if (found) {
      ever_seen = true;
      last_seen_ms = now_ms;
    }
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

  /**
   * Minutes since the feed last reported this aircraft, or 0 when it was never seen in this
   * session. This firmware has no clock, so the age is what tells "the feed lost it a minute
   * ago" from "it never departed" -- which look identical on the panel otherwise.
   */
  float minutesSinceSeen(unsigned long now_ms) const {
    if (!ever_seen || now_ms <= last_seen_ms) {
      return 0.0f;
    }
    return static_cast<float>(now_ms - last_seen_ms) / 60000.0f;
  }

  /** Observed block time in minutes; 0 before departure or right after take-off. */
  float airMinutes() const {
    if (!ever_airborne || last_air_ms <= first_air_ms) {
      return 0.0f;
    }
    return static_cast<float>(last_air_ms - first_air_ms) / 60000.0f;
  }
};

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
 * When a cached route is still good, and when another lookup is allowed.
 *
 * The day is part of the route's identity because the same callsign flies a different
 * city pair on a different day, so a TTL alone would serve yesterday's airports. The
 * daily budget bounds the spend when every lookup fails. A day of 0 means the clock has
 * never been synced (no Date header seen yet): the budget then counts without resetting,
 * which still lets the first successful response establish it.
 */
struct RouteCache {
  bool valid = false;
  unsigned long fetched_ms = 0;
  unsigned long fetched_day = 0;
  bool attempted = false;
  unsigned long last_attempt_ms = 0;
  unsigned long retry_ms = kRouteRetryMs;
  /** Current step of the heap-deferral ladder; 0 while not deferring. */
  unsigned long defer_ms = 0;
  unsigned long budget_day = 0;
  unsigned lookups_today = 0;

  void reset() { *this = RouteCache(); }

  bool usableAt(unsigned long now_ms, unsigned long day) const {
    if (!valid) {
      return false;
    }
    if (day != 0 && day != fetched_day) {
      return false;  // a new day: the callsign may be flying somewhere else now
    }
    return (now_ms - fetched_ms) < kRouteTtlMs;
  }

  bool budgetLeft(unsigned long day) const {
    return day != budget_day || lookups_today < kRouteLookupsPerDay;
  }

  bool wantedAt(unsigned long now_ms, unsigned long day) const {
    if (usableAt(now_ms, day) || !budgetLeft(day)) {
      return false;
    }
    if (attempted && (now_ms - last_attempt_ms) < retry_ms) {
      return false;  // failed recently: back off
    }
    return true;
  }

  void noteAttempt(unsigned long now_ms, unsigned long day) {
    if (day != budget_day) {
      budget_day = day;
      lookups_today = 0;
    }
    ++lookups_today;
    attempted = true;
    last_attempt_ms = now_ms;
    retry_ms = kRouteRetryMs;
  }

  /**
   * The lookup never ran (no room for the handshake). Unlike fail(), this keeps whatever
   * route is already known -- an aged route still beats a blank panel -- and pushes the
   * next try out by the short window, doubling it per consecutive deferral (10 s, 20 s,
   * 40 s, then the kRouteDeferMs cap) so a transient low block is caught while it is open.
   */
  void defer(unsigned long now_ms) {
    defer_ms = defer_ms == 0
                   ? kRouteDeferFirstMs
                   : (defer_ms >= kRouteDeferMs / 2 ? kRouteDeferMs : defer_ms * 2);
    attempted = true;
    last_attempt_ms = now_ms;
    retry_ms = defer_ms;
  }

  void succeed(unsigned long now_ms, unsigned long day) {
    valid = true;
    fetched_ms = now_ms;
    fetched_day = day;
    defer_ms = 0;  // a resolved route starts the ladder over
  }

  void fail() { valid = false; }
};

// --- stateful API, implemented in src/services/flight_follow.cpp ---

/** Everything the info panel needs, refreshed by onReport(). */
struct Info {
  State state = State::kIdle;
  char id[kIdLen] = {};
  float gs_knots = 0.0f;
  /** Minutes since the feed last had this aircraft; 0 while it is being reported. */
  float since_seen_min = 0.0f;
  /** Whether the feed is reporting it right now, so the panel can say when it is not. */
  bool found = false;
  /**
   * True while the fix being drawn came from the OpenSky fallback: 10 s resolution
   * anonymous, and the panel says so instead of pretending it is the feed.
   */
  bool fix_from_fallback = false;
  /** Resolved airport codes (IATA when the API has it, else ICAO); empty = unknown. */
  char origin[kCodeLen] = {};
  char destination[kCodeLen] = {};
  bool route_known = false;
  /** Scheduled/estimated arrival, epoch seconds; 0 = no countdown to show. */
  long arrival_epoch = 0;
  /**
   * Wall clock from the last AeroAPI Date header. It lives here, not in the client, so
   * the panel can count down between lookups and survives losing the feed.
   */
  services::fa::Clock clock;
};

/** Load the saved target from NVS. Call once, after services::location::init(). */
void init();

/** Portal field. Blank clears the target; either way it persists and resets the session. */
void setTargetFromPortal(const char* text);

/** Stop following (BOOT double tap, factory reset): same as an empty portal field. */
void reset();

const Target& target();
const Session& session();
const Info& info();
const Trail& trail();

/**
 * Feed the panel a resolved route. `origin`/`dest` are display codes (IATA or ICAO,
 * either may be empty when the API does not know that end), `arrival_epoch` the
 * expected arrival in epoch seconds, `clock_epoch` the Date header of the same response
 * (0 when it could not be parsed -- the route is still worth showing).
 */
void setRouteFromLookup(const char* origin, const char* dest, long arrival_epoch,
                        long clock_epoch);

/** True when the cache says another lookup is due and the daily budget allows it. */
bool routeWanted();

/** Record an attempt; call right before the request, never after. */
void noteRouteAttempt();
/** The lookup could not even run (heap): keep the route, retry in a minute. */
void noteRouteDeferred();
/** The lookup ran and answered without a usable route: back off for five minutes. */
void noteRouteFailure();

const RouteCache& routeCache();

/** Panel ETA in minutes at `now_ms`, or -1 when there is nothing to count down to. */
float etaMinutesAt(unsigned long now_ms);

/**
 * Remember the followed aircraft's ICAO address (lowercased), learned from the primary feed
 * while the flight is still visible. OpenSky has no callsign search, so this hex is the only
 * handle the fallback has -- and it is persisted, because a reboot in the middle of a
 * coverage gap must not lose the identity of the flight being followed.
 */
void noteTargetHex(const char* hex);

/** Stored hex, "" when unknown. Lowercased. */
const char* targetHex();

/**
 * What the panel calls the flight. A hex target is the only handle the user can type, but
 * "ITY673" is what names the flight on screen -- set it whenever the aircraft's callsign
 * becomes known (from the feed or from the fallback).
 */
void setDisplayId(const char* id);

/** True while the fix on screen came from the OpenSky fallback rather than the feed. */
void setFixFromFallback(bool from_fallback);
bool fixFromFallback();

/** Feed one target poll result: state, session, trail and info all follow from it. */
void onReport(bool found, bool airborne, float lat, float lon, float gs_knots,
              unsigned long now_ms);

}  // namespace services::follow

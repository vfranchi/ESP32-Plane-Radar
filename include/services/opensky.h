#pragma once

#include <cstddef>

// OpenSky fallback for follow mode: the pure half.
//
// adsb.fi stays the radar's only area feed. OpenSky is consulted only when the *followed*
// flight is missing from it -- a gap that is normal, because adsb.fi is a network of
// ground receivers and a flight leaves it as soon as it goes out to sea or crosses a
// coverage hole. OpenSky's network reached the same aircraft over water in the test that
// motivated this, through a serial-only query (one credit, no bounding box).
//
// Free of Arduino so scripts/run_host_tests.sh compiles it with the host compiler: the
// JSON glue (positional state rows) and the TLS session live in adsb_client.cpp.
namespace services::open_sky {

constexpr double kMpsToKnots = 1.943844;
constexpr double kMetersToFeet = 3.280840;

/** OpenSky reports m/s; the panel has always said knots. */
inline float msToKnots(float meters_per_second) {
  return meters_per_second * static_cast<float>(kMpsToKnots);
}

/** OpenSky reports metres; the panel has always said feet. */
inline float metersToFeet(float meters) { return meters * static_cast<float>(kMetersToFeet); }

/**
 * Age of a position fix, in seconds, from two server-side timestamps:
 * the response's `time` and the row's `time_position`. Both come from OpenSky, so this
 * needs no local clock. Clamped to [0, 30]: a stale fix must not be flung far by the
 * dead-reckoning, and OpenSky drops positions older than 15 s anyway.
 */
inline float positionAgeSeconds(long response_time, long time_position) {
  if (response_time <= 0 || time_position <= 0 || time_position > response_time) {
    return 0.0f;
  }
  const long age = response_time - time_position;
  return age > 30 ? 30.0f : static_cast<float>(age);
}

/** Next UTC midnight at or after `epoch` (0 when the epoch is unusable). */
inline long nextUtcMidnight(long epoch) {
  if (epoch <= 0) {
    return 0;
  }
  constexpr long kDay = 86400L;
  return (epoch / kDay + 1) * kDay;
}

/**
 * How often the fallback may ask, so a long gap does not burn the day's credits.
 *
 * `remaining` is the response's X-Rate-Limit-Remaining (server-side truth; -1 while unknown,
 * which falls back to the tier baseline). Every fallback call costs exactly one credit: it is
 * a serial-only query (`?icao24=`), never a bounding box. A 5 % reserve is kept so the last
 * calls of the day are not spent on a margin.
 */
struct CreditBudget {
  long remaining = -1;  // credits left today, -1 = unknown
  int baseline_s = 5;   // never faster than the primary feed's poll
  int unknown_s = 60;   // until the first X-Rate-Limit-Remaining arrives, assume the worst
  int max_s = 300;      // a slow fallback beats no fallback, but not by much

  /** Seconds between fallback requests at `now_epoch`. */
  int intervalSeconds(long now_epoch) const {
    if (baseline_s < 1) {
      return 1;
    }
    if (remaining < 0 || now_epoch <= 0) {
      return unknown_s;  // nothing measured yet: do not guess a tier
    }
    const long seconds_left = nextUtcMidnight(now_epoch) - now_epoch;
    const long budget = (remaining * 95) / 100;  // keep a 5 % reserve
    if (budget < 1) {
      return max_s;  // spent: only the refill brings it back
    }
    const long needed = seconds_left / budget;
    const long interval = needed > baseline_s ? needed : baseline_s;
    return interval > max_s ? max_s : static_cast<int>(interval);
  }

  void noteRemaining(long credits) { remaining = credits; }
};

}  // namespace services::open_sky

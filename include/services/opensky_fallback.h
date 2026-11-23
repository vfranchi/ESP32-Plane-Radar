#pragma once

#include <cstdint>

// OpenSky as a fallback for the *followed* flight only.
//
// adsb.fi is the radar's feed and stays that way. It is a network of ground receivers,
// so a flight leaves it as soon as it goes out to sea or crosses a coverage hole -- the
// case this exists for. OpenSky still has the aircraft, and a serial-only query
// (`?icao24=`, no bounding box) returns it for exactly one credit.
//
// The implementation lives in src/services/adsb_client.cpp, next to the chunked-body
// framing and the abort/poll hooks it shares with the feed; this header is Arduino-free so
// the host tests can include it.
namespace services::open_sky {

struct Fix {
  bool found = false;
  bool attempted = false;
  bool rate_limited = false;  // HTTP 429: honour retry_after_s
  int retry_after_s = 0;
  float lat = 0.0f;
  float lon = 0.0f;
  float gs_knots = 0.0f;
  float track_deg = 0.0f;
  bool track_valid = false;
  bool on_ground = false;
  char alt[12] = {};  // "GND" or "25525 ft"
  /** Callsign as OpenSky reports it (padded, trimmed here): a hex target has no other way
   *  to learn the flight's name while the feed cannot see it. */
  char callsign[9] = {};
  uint32_t pos_age_ms = 0;
};

/**
 * GET /states/all?icao24=<hex>&extended=1 on a transient TLS session. One credit.
 *
 * `hex` must be lowercase (OpenSky's form). Closes its session before returning, so the
 * 2x16 KB mbedTLS record buffers are back before the next feed poll.
 */
bool lookupFix(const char* hex, Fix* out);

/** True when the budget and the backoff allow another fallback request now. */
bool dueNow(unsigned long now_ms);

/** Record that a request was made (starts the backoff); call right before the GET. */
void noteAttempt(unsigned long now_ms);

/** Seconds to wait before the next attempt, from the last X-Rate-Limit-Remaining. */
unsigned long nextAttemptMs(long now_epoch);

/** Drop the stored budget/backoff (a new boot, a new target). */
void resetBudget();

}  // namespace services::open_sky

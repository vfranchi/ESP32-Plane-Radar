#pragma once

#include <cstddef>
#include <cstdint>

namespace services::adsb {

struct Aircraft {
  float lat;
  float lon;
  float nose_deg;
  float track_deg;
  /**
   * False when the feed reported no track/heading at all. pickTrackHeading()
   * then returns a 0-degree default, and dead-reckoning on that default would
   * fly the aircraft due north, so the caller must not extrapolate.
   */
  bool track_valid;
  /**
   * True when the feed says the airframe is on the ground -- either literally ("ground")
   * or a numeric altitude at apron level. Some receivers report a taxiing aircraft as
   * "0 ft" rather than "ground", and a fast taxi then looks airborne by ground speed alone.
   */
  bool on_ground;
  float gs_knots;
  /** Age of the position fix at fetch time (ms), from the feed's seen_pos.
   *  Added to the elapsed time when dead-reckoning so the drawn position
   *  reflects the estimated current location, not the already-stale fix. */
  uint32_t pos_age_ms;
  char callsign[9];
  char type[5];
  char alt[12];
  /**
   * Light/small aircraft: ADS-B emitter category A1/A2, or an N-number callsign
   * when the transponder reports no category. Set at parse time, so no extra
   * request is needed to tell a Cessna from a 737.
   */
  bool is_private;
};

constexpr size_t kMaxAircraft = 64;

struct NearestAircraft;

/** Create the internal lock. Call once before any fetch/snapshot. */
void init();

/**
 * Copy the current aircraft into `out` (up to `max_out`) and report the fetch
 * timestamp, all under a lock — safe to call from a different thread than the
 * one running fetchUpdate(). Returns the number copied.
 */
size_t snapshotAircraft(Aircraft* out, size_t max_out,
                        unsigned long* out_last_update_ms);

size_t aircraftCount();
const Aircraft* aircraftList();

/**
 * Closest aircraft as of the last successful fetch, computed by the fetch task
 * from the list it had just parsed. Callers (MQTT telemetry) read this instead
 * of copying the list: the copy cost 3.3 KB of heap per publish and starved the
 * TLS handshake. valid=false before the first fetch.
 */
const NearestAircraft& nearest();

/**
 * millis() timestamp of the last successful fetch (0 before the first). The
 * stored lat/lon are the values as of this time; callers can dead-reckon
 * newer positions from track_deg/gs_knots and the elapsed time.
 */
unsigned long lastUpdateMs();

/**
 * millis() timestamp of the last successful *target* lookup. Kept apart from
 * lastUpdateMs() on purpose: the area list and the followed aircraft are refreshed at
 * different rates (every 3rd poll vs every poll), and one shared base time would make
 * each set of aircraft dead-reckon from the other's fetch moment.
 */
unsigned long targetUpdateMs();

/**
 * Look one aircraft up by callsign or ICAO hex (adsb.fi v2 -- v3 has no such path and
 * answers 400, which also counts against the feed's rate limit). Fills a slot separate
 * from the area list, so following a flight never disturbs the surrounding traffic.
 *
 * Same host as the area fetch, so it rides the existing keep-alive session instead of
 * paying a second TLS handshake and a second pair of 16 KB mbedTLS record buffers.
 * Covered by the same in-flight guard the watchdog reads.
 *
 * Returns false only when the request could not be made at all. An empty answer
 * (`"total": 0`) is a state -- the flight is not being tracked right now -- not an error.
 */
bool fetchTarget(const char* id, bool is_hex);

/** True while the last target lookup produced an aircraft. */
bool targetValid();

/** Copy of the followed aircraft, taken under the same lock as the area list. */
bool targetSnapshot(Aircraft* out);

/**
 * True while a fetch is in flight. Set on entry to fetchUpdate() and cleared on
 * every exit path, so a render loop on another task can show the activity dot.
 * A plain volatile bool is enough: the flag is advisory, so a stale read only
 * costs one frame.
 */
bool fetchInProgress();

/** Milliseconds the in-flight fetch has been running; 0 when idle. */
unsigned long fetchElapsedMs();

/**
 * Ask the in-flight fetch to give up; it returns false at its next wait point
 * and the next poll starts clean. Driven by the master timeout watchdog.
 */
void requestFetchAbort();

/** Hook invoked during long HTTP I/O (e.g. wifiLoop). Optional. */
using PollFn = void (*)();
void setPollFn(PollFn fn);

/** Fetch aircraft within fetch_radius_km of center_lat/lon from adsb.fi. */
bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km);

}  // namespace services::adsb

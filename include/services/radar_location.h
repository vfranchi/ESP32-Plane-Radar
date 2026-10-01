#pragma once

namespace services::location {

/** Load saved lat/lon from NVS, or use config defaults. Call once before WiFi setup. */
void init();

/** Factory defaults when nothing is stored (also used for portal field prefill). */
double lat();
double lon();

/**
 * Radar centre actually in use. Equal to lat()/lon() unless follow mode overrides it.
 *
 * Kept separate from the stored location on purpose: lat()/lon() stay the *configured*
 * position (portal prefill, NVS), while these report what the projector, the runway
 * cache and the ADS-B fetch must use right now.
 */
double centerLat();
double centerLon();

/**
 * Follow mode: move the radar centre onto the tracked aircraft. Stored as float so the
 * write is a single 32-bit store -- the render task writes it while the fetch task reads.
 */
void setFollowCenter(float lat, float lon);
void clearFollowCenter();
bool followCenterActive();

/** Parse portal strings, validate, persist to NVS, update runtime values. */
bool saveFromStrings(const char* lat_str, const char* lon_str);

/** Clear stored coordinates (e.g. with WiFi credential reset). */
void clear();

}  // namespace services::location

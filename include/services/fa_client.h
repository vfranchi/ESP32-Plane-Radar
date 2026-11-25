#pragma once

#include <cstddef>

// FlightAware AeroAPI client: the API key's storage and one route lookup per followed
// flight.
//
// The lookup itself lives in src/services/adsb_client.cpp, next to the feed's HTTP
// machinery (chunked framing, the abort/poll hooks and the keep-alive session) -- one
// TLS stack, no second copy of it. This header is Arduino-free so the host tests and
// the pure follow-mode logic can include it without dragging in the network.
//
// Cost: AeroAPI's Personal tier charges $0.005 per result set with $5 of free usage a
// month ($10 for ADS-B feeders), and caps at 10 result sets a minute. The caller (follow
// mode's RouteCache) is what keeps the spend down.
namespace services::fa {

/** Longest key accepted by the portal field, including the NUL. */
constexpr size_t kKeyLen = 33;

struct RouteResult {
  bool found = false;      // true only when both airport codes came back
  bool attempted = false;  // false only when the request was never made
  bool low_block = false;  // the lookup never ran: the heap had no room for the handshake
  bool key_rejected = false;  // 401/403: the key is wrong or revoked, stop trying
  char origin[5] = {};
  char dest[5] = {};
  /** Expected arrival in epoch seconds (0 = the API reported none). */
  long arrival_epoch = 0;
  /** UTC seconds from the response's Date header; 0 when it could not be parsed. */
  long clock_epoch = 0;
};

/** Load the saved key from NVS. Call once, after the NVS-backed settings are up. */
void loadKey();

/** Store (or, with an empty string, clear) the key. Never logs the key itself. */
void saveKeyFromPortal(const char* text);

bool keyPresent();

/**
 * The stored key, "" when unset. Exists so the request can carry it; never log it.
 */
const char* apiKey();

/**
 * One GET /flights/{ident}. `is_registration` selects ident_type=registration (a tail
 * number, which is the only handle a Mode-S hex target has); otherwise the ident is
 * treated as a flight designator. Closes its TLS session before returning.
 */
bool lookupRoute(const char* ident, bool is_registration, RouteResult* out);

}  // namespace services::fa

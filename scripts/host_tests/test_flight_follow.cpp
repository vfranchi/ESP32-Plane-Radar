/**
 * Host test for the follow-flight logic.
 *
 * The part that decides *what* the panel shows -- state machine, session times, trail
 * ring, ETA and route-cache lifetime -- is plain C++ with no Arduino and no NVS, so it
 * is checked here rather than on the bench (see scripts/run_host_tests.sh). Only the
 * network and the drawing of follow mode need the board.
 */
#include <cmath>
#include <cstdio>
#include <cstring>

#include "services/flight_follow.h"
#include "test_util.h"

namespace {

using services::follow::deriveState;
using services::follow::estimatedTripMinutes;
using services::follow::etaMinutes;
using services::follow::greatCircleKm;
using services::follow::kIdLen;
using services::follow::kTrailMax;
using services::follow::looksLikeHex;
using services::follow::normalizeId;
using services::follow::routeLooksReversed;
using services::follow::RouteCache;
using services::follow::Session;
using services::follow::State;
using services::follow::Trail;

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

constexpr unsigned long kMinute = 60UL * 1000UL;
constexpr unsigned long kHour = 60UL * kMinute;

}  // namespace

int main() {
  // --- identifier normalization: the feed pads callsigns with trailing spaces ---
  char id[kIdLen];
  normalizeId("glo1724 ", id, sizeof(id));
  CHECK(std::strcmp(id, "GLO1724") == 0);
  normalizeId("  e4988d", id, sizeof(id));
  CHECK(std::strcmp(id, "E4988D") == 0);
  normalizeId("", id, sizeof(id));
  CHECK(id[0] == '\0');
  normalizeId(nullptr, id, sizeof(id));
  CHECK(id[0] == '\0');

  // --- a 6-character hex string is an ICAO address, anything else is a callsign ---
  CHECK(looksLikeHex("E4988D"));
  CHECK(looksLikeHex("e4988d"));
  CHECK(!looksLikeHex("GLO1724"));
  CHECK(!looksLikeHex("E4988"));    // 5 characters
  CHECK(!looksLikeHex("E4988DZ"));  // 7 characters
  CHECK(!looksLikeHex(""));         // empty

  // --- state machine ---
  CHECK(deriveState(false, false, false) == State::kNotSeen);
  CHECK(deriveState(false, false, true) == State::kLanded);
  CHECK(deriveState(true, true, false) == State::kLive);
  CHECK(deriveState(true, false, false) == State::kGrounded);
  CHECK(deriveState(true, false, true) == State::kLanded);

  // --- session bookkeeping: one departure, one touchdown, a frozen block time ---
  Session s;
  CHECK(s.update(false, false, 1000) == State::kNotSeen);
  CHECK(!s.ever_airborne);
  CHECK(s.update(true, false, 4000) == State::kGrounded);
  CHECK(s.ground_since_ms == 4000);
  CHECK(s.update(true, true, 10000) == State::kLive);
  CHECK(s.first_air_ms == 10000 && s.last_air_ms == 10000);
  CHECK(s.update(true, true, 40000) == State::kLive);
  CHECK(s.last_air_ms == 40000);
  CHECK(near(s.airMinutes(), 0.5f, 1e-4f));  // 30 s airborne
  CHECK(s.update(true, false, 70000) == State::kLanded);
  CHECK(s.landed_ms == 70000);
  CHECK(s.update(false, false, 100000) == State::kLanded);
  CHECK(near(s.airMinutes(), 0.5f, 1e-4f));  // stays 30 s, not 90 s
  s.reset();
  CHECK(!s.ever_airborne && s.airMinutes() == 0.0f);

  // --- ETA: unknown inputs are negative, never a huge number ---
  CHECK(near(etaMinutes(111.0f, 600.0f), 6.0f, 0.05f));  // 111 km at 1111 km/h
  CHECK(etaMinutes(10.0f, 0.0f) < 0.0f);
  CHECK(etaMinutes(0.0f, 400.0f) < 0.0f);
  CHECK(near(estimatedTripMinutes(750.0f), 60.0f, 0.05f));
  CHECK(estimatedTripMinutes(0.0f) < 0.0f);

  // --- great circle: one degree of latitude is ~111 km ---
  CHECK(near(greatCircleKm(0.0f, 0.0f, 1.0f, 0.0f), 111.19f, 1.0f));
  CHECK(near(greatCircleKm(-23.43f, -46.47f, -23.43f, -46.47f), 0.0f, 1e-3f));

  // --- trail: oldest-first ordering, jitter filter, capacity cap ---
  Trail t;
  CHECK(t.size() == 0);
  t.push(10.0f, 20.0f, 0.1f);
  CHECK(t.size() == 1);
  t.push(10.0001f, 20.0f, 0.1f);  // under 100 m: jitter, ignored
  CHECK(t.size() == 1);
  t.push(10.01f, 20.0f, 0.1f);  // over 100 m: kept
  CHECK(t.size() == 2);
  float la = 0.0f;
  float lo = 0.0f;
  t.at(0, &la, &lo);
  CHECK(near(la, 10.0f, 1e-6f));
  t.at(1, &la, &lo);
  CHECK(near(la, 10.01f, 1e-6f));
  for (size_t i = 0; i < kTrailMax + 10; ++i) {
    t.push(11.0f + 0.01f * static_cast<float>(i), 20.0f, 0.1f);
  }
  CHECK(t.size() == kTrailMax);
  t.at(0, &la, &lo);
  CHECK(la > 11.0f);  // the oldest samples were dropped
  t.at(t.size() - 1, &la, &lo);
  CHECK(la > 11.0f + 0.01f * static_cast<float>(kTrailMax));  // newest is last
  t.clear();
  CHECK(t.size() == 0);

  // --- route cache: a callsign is reused across days, so the day is part of its identity ---
  RouteCache rc;
  CHECK(!rc.usableAt(1000, 0));                        // nothing resolved yet
  CHECK(rc.wantedAt(1000, 0, /*target_is_hex=*/false));  // wanted, never attempted
  rc.noteAttempt(1000);
  CHECK(!rc.wantedAt(1000 + kMinute, 0, false));       // inside the 5 min backoff
  CHECK(rc.wantedAt(1000 + 6 * kMinute, 0, false));    // retry allowed
  rc.succeed(1000 + 6 * kMinute, 0);
  CHECK(rc.usableAt(1000 + 7 * kMinute, 0));           // resolved, same day
  CHECK(rc.usableAt(1000 + 11 * kHour, 0));            // under the 12 h TTL
  CHECK(!rc.usableAt(1000 + 13 * kHour, 0));           // past the TTL
  CHECK(!rc.usableAt(1000 + 7 * kMinute, 1));          // day rolled over
  CHECK(rc.wantedAt(1000 + 7 * kMinute, 1, false));    // -> asks for a fresh lookup
  rc.fail();
  CHECK(!rc.usableAt(1000 + 7 * kMinute, 0));  // a failure drops the cache
  // --- a lookup that never ran (no room for the handshake) keeps the route and retries soon ---
  RouteCache deferred;
  deferred.noteAttempt(1000);
  CHECK(!deferred.wantedAt(1000 + kMinute, 0, false));  // nothing resolved yet, still backing off
  deferred.defer(1000);                                 // the handshake never fit in the heap
  CHECK(!deferred.wantedAt(1000 + 30 * 1000, 0, false));   // a defer is not an instant retry
  CHECK(deferred.wantedAt(1000 + kMinute + 1, 0, false));  // but ~1 min, not the 5 min of a failure
  deferred.succeed(1000 + kMinute + 1, 0);
  CHECK(deferred.usableAt(1000 + kMinute + 2, 0));
  // The contrast that matters: a defer keeps the route already on screen, only fail() drops it.
  deferred.defer(1000 + 2 * kMinute);
  CHECK(deferred.usableAt(1000 + 2 * kMinute + 1, 0));
  deferred.fail();
  CHECK(!deferred.usableAt(1000 + 2 * kMinute + 2, 0));

  // --- the filed route can be the other leg: the heading decides which end is ahead ---
  // Live case that exposed this: AZU4269 at -23.7007/-46.3556, track 78 deg, adsbdb said
  // REC > VCP, but the aircraft was 111 km out of VCP flying away from it.
  const float vcp_lat = -23.007f, vcp_lon = -47.135f, rec_lat = -8.126f, rec_lon = -34.923f;
  CHECK(routeLooksReversed(-23.7007f, -46.3556f, 78.13f, rec_lat, rec_lon, vcp_lat, vcp_lon));
  // Same spot, heading back at VCP: what adsbdb said is then right, so nothing to swap.
  CHECK(!routeLooksReversed(-23.7007f, -46.3556f, 313.0f, rec_lat, rec_lon, vcp_lat, vcp_lon));
  // Just off VCP climbing out toward REC, filed VCP > REC: correct as filed.
  CHECK(!routeLooksReversed(-23.10f, -47.00f, 40.0f, vcp_lat, vcp_lon, rec_lat, rec_lon));

  RouteCache hex_cache;
  CHECK(!hex_cache.wantedAt(10000, 0, /*target_is_hex=*/true));  // adsbdb resolves callsigns only
  CHECK(hex_cache.wantedAt(10000, 0, false));
  RouteCache blank_day;
  CHECK(!blank_day.usableAt(10000, 0));

  return testSummary("flight_follow");
}

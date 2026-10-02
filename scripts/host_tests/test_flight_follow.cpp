/**
 * Host test for the follow-flight logic.
 *
 * The part that decides *what* the panel shows -- state machine, session times, trail
 * ring, age of the last fix -- is plain C++ with no Arduino and no NVS, so it is checked
 * here rather than on the bench (see scripts/run_host_tests.sh). Only the network and the
 * drawing of follow mode need the board.
 */
#include <cmath>
#include <cstdio>
#include <cstring>

#include "services/flight_follow.h"
#include "test_util.h"

namespace {

using services::follow::deriveState;
using services::follow::greatCircleKm;
using services::follow::kIdLen;
using services::follow::kTrailMax;
using services::follow::looksLikeHex;
using services::follow::normalizeId;
using services::follow::Session;
using services::follow::State;
using services::follow::Trail;

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

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

  // --- the age of the last fix is what tells "feed lost it" from "never departed" ---
  Session seen;
  CHECK(seen.minutesSinceSeen(60000) == 0.0f);  // never seen in this session
  seen.update(true, true, 10000);
  CHECK(seen.ever_seen);
  CHECK(seen.minutesSinceSeen(10000) == 0.0f);        // just reported
  CHECK(near(seen.minutesSinceSeen(10000 + 120000), 2.0f, 0.01f));
  seen.update(false, false, 200000);                   // out of the feed's sight
  // Still counted from the last *fix* (10 s), not from the poll that failed to find it.
  CHECK(near(seen.minutesSinceSeen(290000), 280000.0f / 60000.0f, 0.01f));
  seen.reset();
  CHECK(!seen.ever_seen);
  CHECK(seen.minutesSinceSeen(999999) == 0.0f);

  return testSummary("flight_follow");
}

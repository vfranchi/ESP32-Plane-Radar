/**
 * Host test for the FlightAware route lookup's pure half.
 *
 * The wall clock (parsed out of the HTTP Date header, because the board has no RTC),
 * the choice of which of the 14 days of flights around an ident is the one in the air,
 * and the arrival countdown are plain arithmetic -- no Arduino, no TLS, no display.
 */
#include <cstdio>
#include <cstring>

#include "services/fa_route.h"
#include "test_util.h"

namespace {

using services::fa::Candidate;
using services::fa::Clock;
using services::fa::dayOf;
using services::fa::daysFromCivil;
using services::fa::epochFromHttpDate;
using services::fa::epochFromIso;
using services::fa::minutesUntil;
using services::fa::pickFlight;

constexpr long kNow = 1795440076L;  // 2026-11-23T13:21:16Z

}  // namespace

int main() {
  // --- civil date -> epoch days (leap years, century rule, pre-1970) ---
  CHECK(daysFromCivil(1970, 1, 1) == 0);
  CHECK(daysFromCivil(2000, 3, 1) == 11017);
  CHECK(daysFromCivil(2024, 2, 29) == 19782);
  CHECK(daysFromCivil(2026, 11, 23) == 20780);
  CHECK(daysFromCivil(1969, 12, 31) == -1);

  // --- the two timestamp forms must agree on the same instant ---
  CHECK(epochFromIso("2026-11-23T13:21:16Z") == kNow);
  CHECK(epochFromHttpDate("Mon, 23 Nov 2026 13:21:16 GMT") == kNow);
  CHECK(epochFromHttpDate("Thu, 01 Jan 1970 00:00:00 GMT") == 0);
  CHECK(dayOf(kNow) == 20780);

  // --- malformed input is rejected, never guessed ---
  CHECK(epochFromIso("") == -1);
  CHECK(epochFromIso("2026-11-23") == -1);
  CHECK(epochFromIso("2026-13-23T13:21:16Z") == -1);
  CHECK(epochFromHttpDate("Mon, 23 Nov 2026 13:21:16") == -1);  // no weekday comma
  CHECK(epochFromHttpDate("garbage") == -1);
  CHECK(epochFromHttpDate("Mon, 23 Foo 2026 13:21:16 GMT") == -1);

  // --- clock extrapolates between lookups ---
  Clock clock;
  CHECK(!clock.valid());
  CHECK(clock.nowAt(5000) == 0);
  clock.set(kNow, 1000);
  CHECK(clock.valid());
  CHECK(clock.nowAt(1000) == kNow);
  CHECK(clock.nowAt(11000) == kNow + 10);
  CHECK(clock.nowAt(1000 - 500) == kNow);  // millis() wrap: never extrapolate backwards

  // --- flight selection: the leg in the air wins over yesterday's and tomorrow's ---
  Candidate c[4];
  // 0: yesterday's leg, finished
  c[0].scheduled_off = kNow - 30 * 3600L;
  c[0].actual_off = kNow - 30 * 3600L;
  c[0].scheduled_in = kNow - 27 * 3600L;
  c[0].actual_in = kNow - 27 * 3600L;
  // 1: today's leg, in the air, lands in 16 min
  c[1].scheduled_off = kNow - 90 * 60L;
  c[1].actual_off = kNow - 89 * 60L;
  c[1].scheduled_in = kNow + 20 * 60L;
  c[1].estimated_in = kNow + 16 * 60L;
  // 2: tomorrow's leg, filed only
  c[2].scheduled_off = kNow + 20 * 3600L;
  c[2].scheduled_in = kNow + 23 * 3600L;
  // 3: today's other leg, cancelled
  c[3].scheduled_off = kNow + 3600L;
  c[3].cancelled = true;
  CHECK(pickFlight(c, 4, kNow) == 1);
  CHECK(minutesUntil(1795440076L + 16 * 60, kNow) == 16.0f);

  // --- after landing, the most recent departed leg is still the right answer ---
  Candidate landed[1];
  landed[0].scheduled_off = kNow - 3 * 3600L;
  landed[0].actual_off = kNow - 3 * 3600L;
  landed[0].scheduled_in = kNow - 30 * 60L;
  landed[0].actual_in = kNow - 27 * 60L;
  CHECK(pickFlight(landed, 1, kNow) == 0);
  CHECK(minutesUntil(landed[0].actual_in, kNow) == -27.0f);

  // --- nothing in the air: the next departure is the answer ---
  Candidate future[2];
  future[0].scheduled_off = kNow + 5 * 3600L;
  future[1].scheduled_off = kNow + 2 * 3600L;
  CHECK(pickFlight(future, 2, kNow) == 1);

  // --- unusable lists and unusable times ---
  Candidate no_times[1];
  CHECK(pickFlight(no_times, 1, kNow) == -1);
  CHECK(pickFlight(nullptr, 0, kNow) == -1);
  CHECK(pickFlight(c, 4, 0) == -1);
  CHECK(minutesUntil(0, kNow) == -1.0f);
  CHECK(minutesUntil(kNow, 0) == -1.0f);

  // --- diverted legs are skipped like cancelled ones ---
  Candidate diverted[2];
  diverted[0].scheduled_off = kNow - 60 * 60L;
  diverted[0].actual_off = kNow - 59 * 60L;
  diverted[0].scheduled_in = kNow + 60 * 60L;
  diverted[0].diverted = true;
  diverted[1].scheduled_off = kNow + 4 * 3600L;
  CHECK(pickFlight(diverted, 2, kNow) == 1);

  return testSummary("fa_route");
}

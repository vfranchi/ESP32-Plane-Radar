/**
 * Host test for the OpenSky fallback's pure half: unit conversion, fix age, and the credit
 * budget that decides how often the fallback may ask.
 *
 * The budget is the only thing standing between a long coverage gap and a 429: at the
 * primary feed's 5 s cadence an hour of fallback is 720 calls, and the anonymous tier has
 * 400 credits for the whole day.
 */
#include <cmath>
#include <cstdio>

#include "services/opensky.h"
#include "test_util.h"

namespace {

using services::open_sky::CreditBudget;
using services::open_sky::metersToFeet;
using services::open_sky::msToKnots;
using services::open_sky::nextUtcMidnight;
using services::open_sky::positionAgeSeconds;

constexpr long kNow = 1795458996L;  // 2026-11-23T18:36:36Z, 5.4 h to UTC midnight

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

}  // namespace

int main() {
  // --- units: OpenSky says m/s and metres, the panel says knots and feet ---
  CHECK(near(msToKnots(100.0f), 194.3844, 0.01));
  CHECK(near(msToKnots(128.6f), 250.0, 0.2));  // 250 kt cruise
  CHECK(near(msToKnots(0.0f), 0.0, 0.001));
  CHECK(near(metersToFeet(1000.0f), 3280.84, 0.01));
  CHECK(near(metersToFeet(7780.02f), 25525.0, 1.0));  // the TAM3332 fix measured in this session

  // --- fix age from two server-side timestamps, no local clock involved ---
  CHECK(near(positionAgeSeconds(1000, 998), 2.0, 0.001));
  CHECK(near(positionAgeSeconds(1000, 1000), 0.0, 0.001));
  CHECK(near(positionAgeSeconds(1000, 900), 30.0, 0.001));   // clamped: stale is not flung far
  CHECK(near(positionAgeSeconds(1000, 0), 0.0, 0.001));      // missing time_position
  CHECK(near(positionAgeSeconds(1000, 1001), 0.0, 0.001));   // clock skew must not go negative
  CHECK(near(positionAgeSeconds(0, 500), 0.0, 0.001));       // no response time

  // --- quota refills at UTC midnight ---
  CHECK(nextUtcMidnight(kNow) == 1795478400L);
  CHECK(nextUtcMidnight(1795478400L) == 1795564800L);
  CHECK(nextUtcMidnight(0) == 0);

  // --- budget: never spend the day's credits in one gap ---
  CreditBudget standard;  // 4,000/day: the primary cadence is affordable with 5.4 h left
  standard.remaining = 4000;
  CHECK(standard.intervalSeconds(kNow) == 5);

  CreditBudget anonymous_tier;  // 400/day: ~51 s, and it widens as the day is spent
  anonymous_tier.remaining = 400;
  CHECK(anonymous_tier.intervalSeconds(kNow) == 51);
  anonymous_tier.remaining = 8000;
  CHECK(anonymous_tier.intervalSeconds(kNow) == 5);  // a feeder tier buys the fast cadence

  CreditBudget unknown;  // no response yet: assume the worst tier
  CHECK(unknown.intervalSeconds(kNow) == 60);
  CHECK(unknown.intervalSeconds(0) == 60);

  CreditBudget nearly_spent;
  nearly_spent.remaining = 10;
  CHECK(nearly_spent.intervalSeconds(kNow) == nearly_spent.max_s);

  CreditBudget spent;
  spent.remaining = 0;
  CHECK(spent.intervalSeconds(kNow) == spent.max_s);

  // Close to midnight the baseline still wins: the day ending is no reason to crawl.
  CreditBudget late;
  late.remaining = 3000;
  CHECK(late.intervalSeconds(1795478400L - 600) == 5);

  // noteRemaining is how a response feeds the next decision.
  CreditBudget fed;
  fed.noteRemaining(400);
  CHECK(fed.remaining == 400);

  return testSummary("opensky");
}

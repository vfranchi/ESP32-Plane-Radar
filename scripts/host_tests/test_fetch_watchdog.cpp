/**
 * Host test for the fetch watchdog ladder.
 *
 * Provoking a real hang on the board needs a link that dies mid-request, which
 * cannot be arranged on a bench. What has to be right is the decision, and that
 * is plain C++ -- see scripts/run_host_tests.sh.
 */
#include <cstdio>
#include <string>

#include "services/fetch_watchdog.h"

namespace {

using services::adsb::FetchWatchdogAction;
using services::adsb::FetchWatchdogLimits;

constexpr FetchWatchdogLimits kLimits{15000, 5000};

int g_checks = 0;
int g_failures = 0;

const char* name(FetchWatchdogAction action) {
  switch (action) {
    case FetchWatchdogAction::kIdle:
      return "kIdle";
    case FetchWatchdogAction::kWait:
      return "kWait";
    case FetchWatchdogAction::kAbort:
      return "kAbort";
    case FetchWatchdogAction::kRestart:
      return "kRestart";
  }
  return "?";
}

void expect(FetchWatchdogAction got, FetchWatchdogAction want, const std::string& what) {
  ++g_checks;
  if (got == want) {
    std::printf("ok   %s\n", what.c_str());
    return;
  }
  std::printf("FAIL %s: got %s, want %s\n", what.c_str(), name(got), name(want));
  ++g_failures;
}

FetchWatchdogAction step(bool in_flight, unsigned long elapsed_ms, bool abort_pending,
                         unsigned long abort_age_ms) {
  return services::adsb::fetchWatchdogStep(in_flight, elapsed_ms, abort_pending,
                                           abort_age_ms, kLimits);
}

}  // namespace

int main() {
  expect(step(false, 0, false, 0), FetchWatchdogAction::kIdle, "idle fetch is left alone");
  expect(step(false, 99999, true, 99999), FetchWatchdogAction::kIdle,
         "idle wins over a stale abort request");
  expect(step(true, 14999, false, 0), FetchWatchdogAction::kWait,
         "just under the master timeout: still waiting");
  expect(step(true, 15000, false, 0), FetchWatchdogAction::kAbort,
         "at the master timeout: abort");
  expect(step(true, 60000, false, 0), FetchWatchdogAction::kAbort,
         "a long hang is still just an abort");
  expect(step(true, 15100, true, 0), FetchWatchdogAction::kWait,
         "abort pending: the grace window starts at 0");
  expect(step(true, 15100, true, 4999), FetchWatchdogAction::kWait,
         "abort pending: inside the grace window");
  expect(step(true, 15100, true, 5000), FetchWatchdogAction::kRestart,
         "grace over: restart");
  expect(step(true, 20000, true, 120000), FetchWatchdogAction::kRestart,
         "a wedged fetch stays a restart");
  expect(step(true, 999999, true, 1), FetchWatchdogAction::kWait,
         "a pending abort is never re-requested");

  if (g_failures == 0) {
    std::printf("\n%d/%d checks passed\n", g_checks, g_checks);
    return 0;
  }
  std::printf("\n%d of %d checks FAILED\n", g_failures, g_checks);
  return 1;
}

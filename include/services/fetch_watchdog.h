#pragma once

/**
 * Master timeout for the ADS-B fetch.
 *
 * Every network step of a fetch has its own timeout, but a weak or dropped link
 * can park a socket read well past all of them and leave the fetch task in
 * flight forever -- the display then sits on "fetching" until the board is
 * rebooted by hand. This is the ladder that gets it out (see adsbFetchTask in
 * main.cpp): past the master timeout the fetch is asked to give up, and if it
 * never honours that, the board is restarted.
 *
 * Kept free of Arduino/FreeRTOS so the decision can be tested on the host.
 */
namespace services::adsb {

enum class FetchWatchdogAction {
  kIdle,     ///< nothing in flight
  kWait,     ///< in flight and still inside its timeouts
  kAbort,    ///< over the master timeout: ask the fetch to give up
  kRestart,  ///< the abort was ignored, the fetch is wedged: restart the board
};

struct FetchWatchdogLimits {
  /** Hard ceiling for one fetch (ms). */
  unsigned long master_timeout_ms;
  /** Time the fetch gets to honour the abort before the board restarts (ms). */
  unsigned long abort_grace_ms;
};

/**
 * One step of the ladder.
 *
 * @param in_flight     a fetch is currently running
 * @param elapsed_ms    how long that fetch has been running
 * @param abort_pending an abort has already been requested and not yet honoured
 * @param abort_age_ms  ms since that abort request (ignored when !abort_pending)
 */
inline FetchWatchdogAction fetchWatchdogStep(bool in_flight, unsigned long elapsed_ms,
                                             bool abort_pending, unsigned long abort_age_ms,
                                             const FetchWatchdogLimits& limits) {
  if (!in_flight) {
    return FetchWatchdogAction::kIdle;
  }
  if (abort_pending) {
    return abort_age_ms >= limits.abort_grace_ms ? FetchWatchdogAction::kRestart
                                                 : FetchWatchdogAction::kWait;
  }
  return elapsed_ms >= limits.master_timeout_ms ? FetchWatchdogAction::kAbort
                                                : FetchWatchdogAction::kWait;
}

}  // namespace services::adsb

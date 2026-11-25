#pragma once

#include <cstddef>

// FlightAware AeroAPI route lookup, the pure half.
//
// The firmware has no RTC and no SNTP, so the wall clock comes from the API's own
// HTTP `Date` header -- a free, response-local UTC time. Everything here (clock,
// flight selection, arrival countdown) is arithmetic on that number, so it is
// host-testable: scripts/run_host_tests.sh compiles it with the plain host compiler.
//
// Header-only and free of Arduino: no NVS, no network, no LovyanGFX.
namespace services::fa {

constexpr size_t kCodeLen = 5;            // IATA (3) or ICAO (4) + NUL
constexpr long kSecondsPerDay = 86400L;

inline bool isDigit(char c) { return c >= '0' && c <= '9'; }

/** Value of two ASCII digits, or -1. */
inline int twoDigits(const char* p) {
  if (p == nullptr || !isDigit(p[0]) || !isDigit(p[1])) {
    return -1;
  }
  return (p[0] - '0') * 10 + (p[1] - '0');
}

/** Value of four ASCII digits, or -1. */
inline int fourDigits(const char* p) {
  if (p == nullptr || !isDigit(p[0]) || !isDigit(p[1]) || !isDigit(p[2]) ||
      !isDigit(p[3])) {
    return -1;
  }
  return (p[0] - '0') * 1000 + (p[1] - '0') * 100 + (p[2] - '0') * 10 + (p[3] - '0');
}

/** Month number (1-12) from a three-letter English abbreviation, or 0. */
inline int monthFromName(const char* p) {
  static const char kNames[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                     "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  if (p == nullptr) {
    return 0;
  }
  for (int i = 0; i < 12; ++i) {
    if (p[0] == kNames[i][0] && p[1] == kNames[i][1] && p[2] == kNames[i][2]) {
      return i + 1;
    }
  }
  return 0;
}

/**
 * Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's civil_from_days
 * inverse). Valid for any year, so no epoch-1970 assumption leaks into the caller.
 */
inline long daysFromCivil(int y, int m, int d) {
  y -= m <= 2 ? 1 : 0;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy =
      static_cast<unsigned>((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1);
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return static_cast<long>(era) * 146097L + static_cast<long>(doe) - 719468L;
}

/** Epoch seconds from "YYYY-MM-DDTHH:MM:SSZ" (the AeroAPI timestamp form), or -1. */
inline long epochFromIso(const char* iso) {
  if (iso == nullptr) {
    return -1;
  }
  const int year = fourDigits(iso);
  const int month = twoDigits(iso + 5);
  const int day = twoDigits(iso + 8);
  const int hour = twoDigits(iso + 11);
  const int minute = twoDigits(iso + 14);
  const int second = twoDigits(iso + 17);
  if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 ||
      hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) {
    return -1;
  }
  if (iso[4] != '-' || iso[7] != '-' || (iso[10] != 'T' && iso[10] != ' ') ||
      iso[13] != ':' || iso[16] != ':') {
    return -1;
  }
  return daysFromCivil(year, month, day) * kSecondsPerDay + hour * 3600L +
         minute * 60L + second;
}

/**
 * Epoch seconds from an HTTP `Date` header ("Mon, 23 Nov 2026 13:21:16 GMT"), or -1.
 * This is where the firmware's wall clock comes from: the Date header is part of the
 * response that carried the route, so no SNTP round trip and no RTC are involved.
 */
inline long epochFromHttpDate(const char* date) {
  if (date == nullptr) {
    return -1;
  }
  const char* p = date;
  while (*p != '\0' && *p != ',') {
    ++p;
  }
  if (*p != ',') {
    return -1;
  }
  ++p;
  while (*p == ' ') {
    ++p;
  }
  const int day = twoDigits(p);
  p += 3;  // "23 "
  const int month = monthFromName(p);
  p += 4;  // "Nov "
  const int year = fourDigits(p);
  p += 5;  // "2026 "
  const int hour = twoDigits(p);
  const int minute = twoDigits(p + 3);
  const int second = twoDigits(p + 6);
  if (day < 1 || day > 31 || month == 0 || year < 1970 || hour < 0 || hour > 23 ||
      minute < 0 || minute > 59 || second < 0 || second > 60 || p[2] != ':' ||
      p[5] != ':' || p[8] != ' ' || p[9] != 'G' || p[10] != 'M' || p[11] != 'T') {
    return -1;
  }
  return daysFromCivil(year, month, day) * kSecondsPerDay + hour * 3600L +
         minute * 60L + second;
}

/**
 * Wall clock derived from one response: the epoch from its Date header plus the
 * millis() value that was read next to it. Later reads extrapolate, so the panel can
 * count down between lookups without another network call.
 */
struct Clock {
  long epoch = 0;           // 0 = never synced
  unsigned long at_ms = 0;  // millis() when `epoch` was read

  bool valid() const { return epoch > 0; }

  void set(long epoch_seconds, unsigned long now_ms) {
    epoch = epoch_seconds > 0 ? epoch_seconds : 0;
    at_ms = now_ms;
  }

  /** Current epoch estimate at a later millis() reading; 0 when never synced. */
  long nowAt(unsigned long now_ms) const {
    if (epoch <= 0) {
      return 0;
    }
    const unsigned long elapsed = now_ms >= at_ms ? (now_ms - at_ms) : 0UL;
    return epoch + static_cast<long>(elapsed / 1000UL);
  }
};

/** One entry of the `flights` array of GET /flights/{ident}. Epoch seconds, 0 = absent. */
struct Candidate {
  long scheduled_out = 0;
  long estimated_out = 0;
  long actual_out = 0;
  long scheduled_off = 0;
  long estimated_off = 0;
  long actual_off = 0;
  long scheduled_on = 0;
  long estimated_on = 0;
  long actual_on = 0;
  long scheduled_in = 0;
  long estimated_in = 0;
  long actual_in = 0;
  bool cancelled = false;
  bool diverted = false;
};

/** Earliest real departure evidence, best estimate first. 0 when unknown. */
inline long departureEpoch(const Candidate& c) {
  if (c.actual_off != 0) {
    return c.actual_off;
  }
  if (c.actual_out != 0) {
    return c.actual_out;
  }
  if (c.estimated_off != 0) {
    return c.estimated_off;
  }
  if (c.estimated_out != 0) {
    return c.estimated_out;
  }
  if (c.scheduled_off != 0) {
    return c.scheduled_off;
  }
  return c.scheduled_out;
}

/** Arrival estimate, actual first. 0 when unknown. */
inline long arrivalEpoch(const Candidate& c) {
  if (c.actual_in != 0) {
    return c.actual_in;
  }
  if (c.actual_on != 0) {
    return c.actual_on;
  }
  if (c.estimated_in != 0) {
    return c.estimated_in;
  }
  if (c.estimated_on != 0) {
    return c.estimated_on;
  }
  if (c.scheduled_in != 0) {
    return c.scheduled_in;
  }
  return c.scheduled_on;
}

/**
 * Index of the flight that covers `now`, or -1 when the list cannot answer.
 *
 * `/flights/{ident}` returns ~14 days of recent and scheduled flights for the same
 * ident, so the caller cannot just take the first: a callsign flies one leg a day. In
 * order: the flight already departed and not yet arrived (latest departure wins), else
 * the next one to depart, else the most recent departed one (a landed flight still has
 * to show where it came from). Cancelled and diverted legs never qualify.
 */
inline int pickFlight(const Candidate* c, size_t n, long now) {
  if (c == nullptr || now <= 0) {
    return -1;
  }
  int in_progress = -1;
  long in_progress_dep = 0;
  int upcoming = -1;
  long upcoming_dep = 0;
  int departed = -1;
  long departed_dep = 0;
  for (size_t i = 0; i < n; ++i) {
    if (c[i].cancelled || c[i].diverted) {
      continue;
    }
    const long dep = departureEpoch(c[i]);
    if (dep <= 0) {
      continue;
    }
    const long arr = arrivalEpoch(c[i]);
    if (dep <= now) {
      if (departed < 0 || dep > departed_dep) {
        departed = static_cast<int>(i);
        departed_dep = dep;
      }
      if (arr == 0 || arr >= now) {
        if (in_progress < 0 || dep > in_progress_dep) {
          in_progress = static_cast<int>(i);
          in_progress_dep = dep;
        }
      }
    } else if (upcoming < 0 || dep < upcoming_dep) {
      upcoming = static_cast<int>(i);
      upcoming_dep = dep;
    }
  }
  if (in_progress >= 0) {
    return in_progress;
  }
  if (upcoming >= 0) {
    return upcoming;
  }
  return departed;
}

/** Minutes from `now` to `arrival_epoch`, negative when the inputs are unusable. */
inline float minutesUntil(long arrival_epoch, long now_epoch) {
  if (arrival_epoch <= 0 || now_epoch <= 0) {
    return -1.0f;
  }
  return static_cast<float>(arrival_epoch - now_epoch) / 60.0f;
}

/** Day counter for cache invalidation: a callsign flies a different leg tomorrow. */
inline unsigned long dayOf(long epoch) {
  return epoch > 0 ? static_cast<unsigned long>(epoch / kSecondsPerDay) : 0UL;
}

}  // namespace services::fa

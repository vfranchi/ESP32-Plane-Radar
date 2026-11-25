#pragma once

#include "services/fa_route.h"  // services::fa::Clock, services::fa::epochFromHttpDate()

// Wall clock from HTTP response Date headers.
//
// The board has no RTC and no SNTP. Every response carries Date, so pairing it with
// millis() once is enough to know the time -- and the credit budget, the FlightAware ETA
// and the OpenSky fallback interval all need it.
namespace services::clock {

using Clock = services::fa::Clock;  // epoch + millis() anchor, host-tested

/** Store the epoch parsed from a response's Date header. Unparsable values are ignored. */
void setFromHttpDate(const char* date, unsigned long now_ms);

/** Current epoch estimate at `now_ms`; 0 when never synced. */
long nowEpoch(unsigned long now_ms);

bool valid(unsigned long now_ms);

void reset();

}  // namespace services::clock

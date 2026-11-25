#include "services/wall_clock.h"

#include <Arduino.h>

namespace services::clock {
namespace {

Clock s_clock;
bool s_announced = false;

}  // namespace

void setFromHttpDate(const char* date, unsigned long now_ms) {
  const long epoch = services::fa::epochFromHttpDate(date);
  if (epoch <= 0) {
    return;  // no Date header, or one we cannot parse: keep the previous estimate
  }
  const bool first = !s_clock.valid();
  s_clock.set(epoch, now_ms);
  if (first && !s_announced) {
    s_announced = true;
    Serial.printf("clock: epoch %ld\n", epoch);
  }
}

long nowEpoch(unsigned long now_ms) { return s_clock.nowAt(now_ms); }

bool valid(unsigned long now_ms) { return s_clock.nowAt(now_ms) > 0; }

void reset() {
  s_clock = Clock();
  s_announced = false;
}

}  // namespace services::clock

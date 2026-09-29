#pragma once

#include <LovyanGFX.hpp>

#include <cstddef>

namespace ui::runway {

void drawLargeAirportRunways(lgfx::LGFXBase& gfx);

// Diagnostics from the last drawLargeAirportRunways() call, for the perf log.
size_t lastAirportCount();
size_t lastRunwayDrawCount();

}  // namespace ui::runway

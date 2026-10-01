#pragma once

#include <cstdlib>

#include <LovyanGFX.hpp>

// Thick-line drawing for a chip with no FPU.
//
// LovyanGFX's drawWideLine prices per pixel of the line's *bounding box* and does float
// coverage math for each one. On the ESP32-C3 that measured 12.9 ms for a single 214 px
// line, and a diagonal's bounding box is fat, so short diagonals were not cheap either.
// Two integer Bresenham passes give the same visual weight for microseconds.
namespace ui::line {

/** Draw a line of 2 * half_width px, rounded down to 1 or 2 px. */
inline void drawThick(lgfx::LGFXBase& gfx, int x0, int y0, int x1, int y1,
                      float half_width, uint16_t color) {
  gfx.drawLine(x0, y0, x1, y1, color);
  if (half_width <= 0.5f) {
    return;  // 1 px is already what was asked for
  }
  if (abs(x1 - x0) >= abs(y1 - y0)) {
    gfx.drawLine(x0, y0 + 1, x1, y1 + 1, color);
  } else {
    gfx.drawLine(x0 + 1, y0, x1 + 1, y1, color);
  }
}

}  // namespace ui::line

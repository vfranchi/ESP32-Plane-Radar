#pragma once

namespace ui {

/** Draw the static sonar/radar grid (black disc, green overlay, labels). */
void radarDisplayDraw();

/** Debug: dump the frame sprite over Serial as hex, framed by a header line
 *  ("FRAME w h bytes") and "FRAME END", so the rendered screen can be inspected
 *  offline instead of pointing a camera at the round panel. The sprite is the
 *  same buffer that gets pushed to the panel, so the dump is the screen.
 *  Triggered by 'F' arriving on the serial port (see main.cpp). */
void radarDisplayDumpFrame();

/** Redraw aircraft only (blits cached grid; no full-screen clear). */
void radarDisplayRefreshAircraft();

}  // namespace ui

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

/**
 * Show the device's local IP as an overlay plate over the radar for `duration_ms`,
 * sliding in and out. Called once after the first Wi-Fi connect, so whoever walks
 * up to the device can read the config portal's address. An empty or 0.0.0.0
 * address arms nothing.
 */
void radarDisplayShowIpSplash(const char* ip, unsigned long duration_ms);

/** True while the splash is still being drawn (diagnostics / bench checks). */
bool radarDisplaySplashActive();

}  // namespace ui

/**
 * Plane Radar — WiFi setup, then radar UI on the round GC9A01 display.
 */

#include <Arduino.h>
#include <WiFi.h>

#include <cstring>

#include "config.h"
#include "hardware/display.h"
#include "services/adsb_client.h"
#include "services/fetch_watchdog.h"
#include "services/flight_follow.h"
#include "services/mqtt_client.h"
#include "services/radar_location.h"
#include "services/wifi_setup.h"
#include "ui/radar_display.h"
#include "ui/radar_range.h"
#include "ui/status_screens.h"

namespace {

bool g_radar_visible = false;
unsigned long g_wifi_down_since = 0;
unsigned long g_last_reconnect_ms = 0;
unsigned long g_last_redraw_ms = 0;
// millis() when the fetch was asked to give up; 0 = no abort pending.
unsigned long g_fetch_abort_ms = 0;

void showRadarIfConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    g_radar_visible = false;
    return;
  }
  ui::radarDisplayDraw();
  g_radar_visible = true;
}

void onRangeTap() {
  ui::radar::rangeNext();
  char range_label[12];
  ui::radar::formatCurrentRing3Label(range_label, sizeof(range_label));
  Serial.printf("Range: %s (outer ~%.0f km)\n", range_label,
                ui::radar::rangeCurrent().outer_km);

  if (g_radar_visible && WiFi.status() == WL_CONNECTED) {
    ui::radarDisplayDraw();
  }
}

// A single BOOT tap cycles the range, but it is held back for one double-tap window so a
// second tap can cancel it: without the delay the first tap has already changed the range
// by the time the second one arrives. With no flight followed the double tap is swallowed
// rather than cycling twice -- invisible to the user, and the gesture stays unambiguous.
constexpr unsigned long kDoubleTapWindowMs = 350;
bool g_tap_pending = false;
unsigned long g_tap_pending_ms = 0;

void handleBootButton() {
  bootButtonPollLongPress();

  if (bootButtonConsumeTap()) {
    const unsigned long since = millis() - g_tap_pending_ms;
    if (g_tap_pending && since <= kDoubleTapWindowMs) {
      g_tap_pending = false;
      if (services::follow::target().active) {
        services::follow::reset();
        Serial.println("follow: stopped by BOOT double tap");
      }
      return;
    }
    if (g_tap_pending) {
      g_tap_pending = false;  // the window had already run out: deliver it now
      onRangeTap();
    }
    g_tap_pending = true;
    g_tap_pending_ms = millis();
    return;
  }

  if (g_tap_pending && (millis() - g_tap_pending_ms) > kDoubleTapWindowMs) {
    g_tap_pending = false;
    onRangeTap();
  }
}

/**
 * 'T' on the serial port followed by a callsign or hex code sets the followed flight.
 * A bench convenience -- the config portal is the real path -- and it is the only way to
 * aim follow mode at a flight without opening the portal.
 */
void readFollowTargetFromSerial() {
  char buf[services::follow::kIdLen] = {};
  size_t n = 0;
  const unsigned long deadline = millis() + 1000;
  while (millis() < deadline && n + 1 < sizeof(buf)) {
    if (Serial.available() > 0) {
      const int c = Serial.read();
      if (c == '\n' || c == '\r') {
        break;
      }
      buf[n++] = static_cast<char>(c);
    } else {
      delay(5);
    }
  }
  buf[n] = '\0';
  services::follow::setTargetFromPortal(buf);
}

// ADS-B fetch runs on its own task: the HTTPS request blocks for ~1-2 s, and
// keeping it off the main loop lets the radar keep redrawing (dead-reckoned) at
// 4 Hz throughout. The task publishes into the shared aircraft buffer under a
// lock; the render loop reads a snapshot.
void adsbFetchTask(void*) {
  unsigned poll = 0;
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (services::follow::target().active) {
        // The followed flight is the whole point, so it is looked up every poll. The
        // surrounding traffic only needs refreshing now and then, which is what keeps
        // this inside the feed's one-request-per-second limit.
        const services::follow::Target& tgt = services::follow::target();
        services::adsb::fetchTarget(tgt.id, tgt.is_hex);

        services::adsb::Aircraft target{};
        const bool found = services::adsb::targetSnapshot(&target);
        // Fast and off the apron: that is "live". Ground speed alone is not enough -- a
        // taxiing jet passes 40 kt, and a receiver that reports it at "0 ft" instead of
        // "ground" would otherwise make a landed airframe look airborne.
        const bool airborne = found &&
                              target.gs_knots >= services::follow::kAirborneGsKnots &&
                              !target.on_ground;
        services::follow::onReport(found, airborne, target.lat, target.lon,
                                   target.gs_knots, millis());

        if (poll % 3 == 0) {
          services::adsb::fetchUpdate(services::location::centerLat(),
                                      services::location::centerLon(),
                                      ui::radar::fetchRadiusKm());
        }
      } else {
        // The MQTT client stays connected across the fetch: halving the frame
        // sprite (RGB332, 57.6 KB) leaves a ~40 KB largest block, which is what
        // the 2x16 KB mbedtls buffers need, so no yield is necessary any more.
        services::adsb::fetchUpdate(services::location::centerLat(),
                                    services::location::centerLon(),
                                    ui::radar::fetchRadiusKm());
      }
      ++poll;
    }
    vTaskDelay(pdMS_TO_TICKS(config::kAdsbFetchIntervalMs));
  }
}

/**
 * Master timeout for the fetch. Its own timeouts do not cover a socket read
 * parked by a weak or dropped link, which is what left the display stuck on
 * "fetching" until a manual reboot. Past kAdsbFetchMasterTimeoutMs the fetch is
 * asked to give up; if it never returns, the board restarts -- nvs keeps WiFi,
 * location and range, so it comes back configured.
 */
void adsbWatchdog() {
  using services::adsb::FetchWatchdogAction;
  const bool aborted = g_fetch_abort_ms != 0;
  const services::adsb::FetchWatchdogLimits limits{config::kAdsbFetchMasterTimeoutMs,
                                                   config::kAdsbFetchAbortGraceMs};

  switch (services::adsb::fetchWatchdogStep(services::adsb::fetchInProgress(),
                                            services::adsb::fetchElapsedMs(), aborted,
                                            aborted ? millis() - g_fetch_abort_ms : 0,
                                            limits)) {
    case FetchWatchdogAction::kIdle:
      g_fetch_abort_ms = 0;
      break;
    case FetchWatchdogAction::kWait:
      break;
    case FetchWatchdogAction::kAbort: {
      const unsigned long elapsed = services::adsb::fetchElapsedMs();
      Serial.printf("adsb: fetch stuck for %lu ms -- aborting\n", elapsed);
      services::adsb::requestFetchAbort();
      g_fetch_abort_ms = millis();
      break;
    }
    case FetchWatchdogAction::kRestart:
      Serial.println("adsb: fetch never returned -- restarting the board");
      Serial.flush();
      ESP.restart();
      break;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("Plane Radar");

  bootButtonInit();
  displayInit();
  if (wifiShowsSetupScreenOnBoot()) {
    statusScreenPortal();
  }
  services::location::init();
  ui::radar::rangeInit();
  services::adsb::init();
  services::follow::init();
  services::mqtt::init();

  if (wifiSetupConnect()) {
    showRadarIfConnected();
  }

  // Start the background ADS-B fetch (pinned to core 0, away from the render
  // loop on core 1). It checks Wi-Fi state each cycle.
  xTaskCreatePinnedToCore(adsbFetchTask, "adsb", 16384, nullptr, 1, nullptr, 0);
}

void loop() {
  handleBootButton();

  if (Serial.available() > 0) {
    const int cmd = Serial.read();
    if (cmd == 'F') {
      // Dumps the frame sprite (see ui::radarDisplayDumpFrame).
      ui::radarDisplayDumpFrame();
    } else if (cmd == 'T') {
      readFollowTargetFromSerial();
    }
  }
  wifiLoop();
  adsbWatchdog();
  services::mqtt::loop();

  if (WiFi.status() != WL_CONNECTED) {
    if (g_radar_visible) {
      Serial.println("WiFi lost — will reconnect");
      g_radar_visible = false;
    }

    if (g_wifi_down_since == 0) {
      g_wifi_down_since = millis();
    }

    const unsigned long down_ms = millis() - g_wifi_down_since;
    if (down_ms >= config::kWifiDownGraceMs &&
        millis() - g_last_reconnect_ms >= config::kWifiReconnectIntervalMs) {
      g_last_reconnect_ms = millis();
      if (wifiReconnect()) {
        g_wifi_down_since = 0;
        showRadarIfConnected();
      }
    }
  } else {
    g_wifi_down_since = 0;
    if (!g_radar_visible) {
      showRadarIfConnected();
    } else if (millis() - g_last_redraw_ms >= config::kRadarRedrawIntervalMs) {
      // Redraw at 4 Hz with dead-reckoned positions; the ADS-B fetch runs on
      // its own task (adsbFetchTask).
      g_last_redraw_ms = millis();
      ui::radarDisplayRefreshAircraft();
    }
  }

  delay(10);
}

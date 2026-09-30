/**
 * Plane Radar — WiFi setup, then radar UI on the round GC9A01 display.
 */

#include <Arduino.h>
#include <WiFi.h>

#include "config.h"
#include "hardware/display.h"
#include "services/adsb_client.h"
#include "services/fetch_watchdog.h"
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

void handleBootButton() {
  bootButtonPollLongPress();
  if (bootButtonConsumeTap()) {
    onRangeTap();
  }
}

// ADS-B fetch runs on its own task: the HTTPS request blocks for ~1-2 s, and
// keeping it off the main loop lets the radar keep redrawing (dead-reckoned) at
// 4 Hz throughout. The task publishes into the shared aircraft buffer under a
// lock; the render loop reads a snapshot.
void adsbFetchTask(void*) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      services::adsb::fetchUpdate(services::location::lat(),
                                  services::location::lon(),
                                  ui::radar::fetchRadiusKm());
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

  if (wifiSetupConnect()) {
    showRadarIfConnected();
  }

  // Start the background ADS-B fetch (pinned to core 0, away from the render
  // loop on core 1). It checks Wi-Fi state each cycle.
  xTaskCreatePinnedToCore(adsbFetchTask, "adsb", 16384, nullptr, 1, nullptr, 0);
}

void loop() {
  handleBootButton();
  wifiLoop();
  adsbWatchdog();

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

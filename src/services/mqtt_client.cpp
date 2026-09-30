#include "services/mqtt_client.h"

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_heap_caps.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.h"
#include "services/adsb_client.h"
#include "services/mqtt_config.h"
#include "services/mqtt_discovery.h"
#include "services/nearest_aircraft.h"
#include "services/radar_location.h"
#include "ui/radar_display.h"
#include "ui/radar_range.h"

namespace services::mqtt {

namespace {

enum class State : uint8_t { Disabled, WaitingLink, Connecting, Discovery, Ready };

constexpr size_t kNodeIdLen = 48;  // configurable base topic, slashes flattened
constexpr size_t kBaseTopicLen = 56;
constexpr size_t kClientIdLen = 64;
// Payload buffer: the host test caps every discovery payload at 600 B, and
// MQTT_MAX_PACKET_SIZE (768) must still fit topic (~52) + header.
constexpr size_t kPayloadMax = 640;
constexpr size_t kCmdMax = 64;

MqttConfig s_cfg{};
WiFiClient s_net;
PubSubClient s_mqtt(s_net);
State s_state = State::Disabled;
char s_node[kNodeIdLen] = {};
char s_base[kBaseTopicLen] = {};
char s_client_id[kClientIdLen] = {};
char s_topic[128] = {};
Ctx s_ctx{};
size_t s_discovery_index = 0;
unsigned long s_last_discovery_ms = 0;
unsigned long s_last_state_ms = 0;
unsigned long s_last_connect_ms = 0;

// Nothing big lives in .bss: holding the 3.3 KB aircraft snapshot plus a payload
// buffer permanently is enough to push the ADS-B TLS handshake over the edge
// ("SSL - Memory allocation failed" in a loop, measured on the bench board).
// They are malloc'd for the publish and released on the way out, so the fetch
// task keeps the heap it needs.
adsb::Aircraft* s_snapshot = nullptr;
char* s_payload = nullptr;

bool ensurePublishBuffers() {
  if (s_snapshot == nullptr) {
    s_snapshot = static_cast<adsb::Aircraft*>(
        std::malloc(sizeof(adsb::Aircraft) * adsb::kMaxAircraft));
  }
  if (s_payload == nullptr) {
    s_payload = static_cast<char*>(std::malloc(kPayloadMax));
  }
  return s_snapshot != nullptr && s_payload != nullptr;
}

void releasePublishBuffers() {
  std::free(s_snapshot);
  s_snapshot = nullptr;
  std::free(s_payload);
  s_payload = nullptr;
}

/** Allocates around the publish and returns the memory on every exit path. */
struct PublishBuffers {
  const bool ok;
  PublishBuffers() : ok(ensurePublishBuffers()) {}
  ~PublishBuffers() { releasePublishBuffers(); }
  PublishBuffers(const PublishBuffers&) = delete;
  PublishBuffers& operator=(const PublishBuffers&) = delete;
};

void buildTopics() {
  const uint32_t mac6 = static_cast<uint32_t>(ESP.getEfuseMac() & 0xFFFFFF);

  // Base topic: user-configured, else the firmware default.
  if (s_cfg.topic[0] != '\0') {
    std::strncpy(s_base, s_cfg.topic, sizeof(s_base) - 1);
    s_base[sizeof(s_base) - 1] = '\0';
  } else {
    std::snprintf(s_base, sizeof(s_base), "%s/%06x",
                  config::kMqttDefaultTopicPrefix, static_cast<unsigned>(mac6));
  }

  // Node id flattens the separators: it goes into discovery topics and unique
  // ids, where a slash would create phantom topic segments.
  size_t i = 0;
  for (; s_base[i] != '\0' && i + 1 < sizeof(s_node); ++i) {
    s_node[i] = (s_base[i] == '/') ? '_' : s_base[i];
  }
  s_node[i] = '\0';

  // Client id stays unique per chip even when two boards are cloned with the
  // same configured topic: a shared client id makes them kick each other off.
  std::snprintf(s_client_id, sizeof(s_client_id), "%s_%06x", s_node,
                static_cast<unsigned>(mac6));

  s_ctx.node = s_node;
  s_ctx.base = s_base;
  s_ctx.sw = config::kMqttSwVersion;
  s_ctx.prefix = s_cfg.prefix[0] != '\0' ? s_cfg.prefix
                                         : config::kMqttDefaultDiscoveryPrefix;
  s_ctx.dev_name = s_cfg.name[0] != '\0' ? s_cfg.name : config::kMqttDefaultDeviceName;
}

void publishStateTopic(const char* key, const char* value, bool retained) {
  std::snprintf(s_topic, sizeof(s_topic), "%s/state/%s", s_base, key);
  // QoS 0: fire and forget, no ack wait inside the render loop.
  if (!s_mqtt.publish(s_topic, value, retained)) {
    Serial.printf("MQTT: publish %s failed (heap %u)\n", s_topic,
                  static_cast<unsigned>(ESP.getFreeHeap()));
  }
}

void publishRangeState() {
  char label[16];
  std::snprintf(label, sizeof(label), "%d km",
                static_cast<int>(lroundf(ui::radar::rangeCurrent().ring3_km)));
  publishStateTopic("range", label, false);
}

void publishSwitchState(const char* key, bool on) {
  publishStateTopic(key, on ? "ON" : "OFF", false);
}

void publishLocationState() {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.6f", services::location::lat());
  publishStateTopic("lat", buf, false);
  std::snprintf(buf, sizeof(buf), "%.6f", services::location::lon());
  publishStateTopic("lon", buf, false);
}

void publishTelemetry() {
  PublishBuffers buffers;
  if (!buffers.ok) {
    Serial.println("MQTT: no heap for the publish buffers, telemetry skipped");
    return;
  }

  char buf[32];
  std::snprintf(buf, sizeof(buf), "%u",
                static_cast<unsigned>(adsb::aircraftCount()));
  publishStateTopic("ac_count", buf, false);

  std::snprintf(buf, sizeof(buf), "%d", WiFi.RSSI());
  publishStateTopic("rssi", buf, false);
  std::snprintf(buf, sizeof(buf), "%u",
                static_cast<unsigned>(ESP.getFreeHeap()));
  publishStateTopic("heap", buf, false);

  const size_t n = adsb::snapshotAircraft(s_snapshot, adsb::kMaxAircraft, nullptr);
  const adsb::NearestAircraft nearest = adsb::findNearest(
      s_snapshot, n, services::location::lat(), services::location::lon());
  if (nearestPayload(nearest, s_payload, kPayloadMax) > 0) {
    std::snprintf(s_topic, sizeof(s_topic), "%s/state/nearest", s_base);
    if (!s_mqtt.publish(s_topic, s_payload, false)) {
      Serial.println("MQTT: nearest publish failed");
    }
  }

  // Diagnostic snapshot. Note the temporary Strings: both outlive the full
  // expression, which is all infoPayload() needs.
  const String ip = WiFi.localIP().toString();
  const String ssid = WiFi.SSID();
  if (infoPayload(ip.c_str(), ssid.c_str(), millis() / 1000UL,
                  config::kMqttSwVersion, s_payload, kPayloadMax) > 0) {
    std::snprintf(s_topic, sizeof(s_topic), "%s/state/info", s_base);
    if (!s_mqtt.publish(s_topic, s_payload, false)) {
      Serial.println("MQTT: info publish failed");
    }
  }
}

void handleCommand(char* topic, const char* value) {
  const char* key = std::strrchr(topic, '/');
  if (key == nullptr) return;
  ++key;

  if (std::strcmp(key, "range") == 0) {
    const int km = leadingKm(value);
    for (size_t i = 0; i < ui::radar::kRangePresetCount; ++i) {
      if (static_cast<int>(lroundf(ui::radar::kRangePresets[i].ring3_km)) == km) {
        ui::radar::rangeSetIndex(static_cast<uint8_t>(i));
        break;
      }
    }
    publishRangeState();
  } else if (std::strcmp(key, "miles") == 0) {
    ui::radar::setUseMiles(payloadIsOn(value));
    publishSwitchState("miles", ui::radar::useMiles());
  } else if (std::strcmp(key, "runways") == 0) {
    ui::radar::setShowRunways(payloadIsOn(value));
    publishSwitchState("runways", ui::radar::showRunways());
  } else if (std::strcmp(key, "debug") == 0) {
    ui::radar::setDebugOverlay(payloadIsOn(value));
    publishSwitchState("debug", ui::radar::debugOverlay());
  } else if (std::strcmp(key, "lat") == 0 || std::strcmp(key, "lon") == 0) {
    const double requested = std::atof(value);
    char lat_buf[24];
    char lon_buf[24];
    if (std::strcmp(key, "lat") == 0) {
      std::snprintf(lat_buf, sizeof(lat_buf), "%.6f", requested);
      std::snprintf(lon_buf, sizeof(lon_buf), "%.6f", services::location::lon());
    } else {
      std::snprintf(lat_buf, sizeof(lat_buf), "%.6f", services::location::lat());
      std::snprintf(lon_buf, sizeof(lon_buf), "%.6f", requested);
    }
    if (!services::location::saveFromStrings(lat_buf, lon_buf)) {
      Serial.println("MQTT: rejected out-of-range coordinates");
    }
    // Re-publish the truth, not the request: an invalid value must not show
    // as applied in Home Assistant.
    publishLocationState();
  } else {
    Serial.printf("MQTT: unknown command topic %s\n", topic);
    return;
  }

  // Grid geometry depends on range/units/location: full redraw, same call the
  // BOOT-button path uses. Safe here: loop() is the render task.
  if (WiFi.status() == WL_CONNECTED) {
    ui::radarDisplayDraw();
  }
}

void onMessage(char* topic, uint8_t* payload, unsigned int length) {
  if (length >= kCmdMax) {
    length = kCmdMax - 1;
  }
  // PubSubClient hands over a NON NUL-terminated buffer.
  char value[kCmdMax];
  std::memcpy(value, payload, length);
  value[length] = '\0';
  handleCommand(topic, value);
}

void publishDiscoveryStep() {
  PublishBuffers buffers;
  if (!buffers.ok) {
    Serial.println("MQTT: no heap for the publish buffers, discovery held back");
    return;
  }
  const auto entity = static_cast<Entity>(s_discovery_index);
  if (discoveryTopic(entity, s_ctx, s_topic, sizeof(s_topic)) == 0) {
    return;
  }
  const size_t n = discoveryPayload(entity, s_ctx, s_payload, kPayloadMax);
  if (n == 0) {
    Serial.printf("MQTT: discovery payload %u too large\n",
                  static_cast<unsigned>(s_discovery_index));
    return;
  }
  const bool ok = s_mqtt.publish(s_topic, s_payload, true);  // retained
  Serial.printf("MQTT: discovery %u/%u %s\n",
                static_cast<unsigned>(s_discovery_index + 1),
                static_cast<unsigned>(Entity::Count), ok ? "ok" : "FAILED");
}

void connectBroker() {
  std::snprintf(s_topic, sizeof(s_topic), "%s/status", s_base);
  const bool ok = s_mqtt.connect(s_client_id, s_cfg.user, s_cfg.pass, s_topic, 0,
                                 true, "offline");
  if (!ok) {
    Serial.printf("MQTT: connect failed, rc=%d\n", s_mqtt.state());
    return;
  }
  s_mqtt.publish(s_topic, "online", true);
  std::snprintf(s_topic, sizeof(s_topic), "%s/cmd/#", s_base);
  s_mqtt.subscribe(s_topic);
  Serial.printf("MQTT: connected to %s:%u as %s (base '%s')\n", s_cfg.host, s_cfg.port,
                s_client_id, s_base);
  s_discovery_index = 0;
  s_last_discovery_ms = 0;
  s_state = State::Discovery;
}

}  // namespace

// The MQTT packet ceiling lives in platformio.ini; if the two drift, discovery
// payloads get truncated silently at run time.
static_assert(MQTT_MAX_PACKET_SIZE == config::kMqttPacketSize,
              "MQTT_MAX_PACKET_SIZE must match config::kMqttPacketSize");

void init() {
  loadConfig(s_cfg);
  buildTopics();
  // The failure this feature walks closest to is a TLS handshake that cannot
  // allocate: "free heap" alone hides it, the largest usable block does not.
  Serial.printf("MQTT: heap %u, largest block %u\n",
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(
                    heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
  if (!s_cfg.enabled) {
    s_state = State::Disabled;
    Serial.println("MQTT: disabled (portal checkbox off)");
    return;
  }
  if (!configured(s_cfg)) {
    s_state = State::Disabled;
    Serial.println("MQTT: enabled but no broker host configured");
    return;
  }
  s_net.setTimeout(3000);  // bounds the TCP connect; the default can hang for ~15 s
  s_mqtt.setServer(s_cfg.host, s_cfg.port);
  s_mqtt.setCallback(onMessage);
  s_mqtt.setBufferSize(config::kMqttPacketSize);
  s_state = State::WaitingLink;
  Serial.printf("MQTT: node %s client %s broker %s:%u topic '%s' prefix '%s'\n", s_node,
                s_client_id, s_cfg.host, s_cfg.port, s_base, s_ctx.prefix);
}

void loop() {
  if (s_state == State::Disabled) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (s_mqtt.connected()) {
      s_mqtt.disconnect();
    }
    s_state = State::WaitingLink;
    return;
  }

  if (!s_mqtt.connected()) {
    if (s_state == State::Discovery || s_state == State::Ready) {
      s_state = State::Connecting;  // broker dropped us
    }
    if (millis() - s_last_connect_ms < config::kMqttReconnectIntervalMs) {
      return;
    }
    s_last_connect_ms = millis();
    connectBroker();
    return;
  }

  s_mqtt.loop();

  if (s_state == State::Connecting || s_state == State::WaitingLink) {
    s_state = State::Discovery;  // connectBroker() already moved us; safety net
    s_discovery_index = 0;
  }

  if (s_state == State::Discovery) {
    if (millis() - s_last_discovery_ms < config::kMqttDiscoverySpacingMs) {
      return;
    }
    s_last_discovery_ms = millis();
    if (s_discovery_index < static_cast<size_t>(Entity::Count)) {
      publishDiscoveryStep();
      ++s_discovery_index;
      return;
    }
    // Discovery done: publish every controllable/reported value once, so HA
    // has real states before the first command.
    publishRangeState();
    publishSwitchState("miles", ui::radar::useMiles());
    publishSwitchState("runways", ui::radar::showRunways());
    publishSwitchState("debug", ui::radar::debugOverlay());
    publishLocationState();
    s_last_state_ms = millis();
    s_state = State::Ready;
    Serial.println("MQTT: discovery complete");
    return;
  }

  if (millis() - s_last_state_ms >= config::kMqttStateIntervalMs) {
    s_last_state_ms = millis();
    if (services::adsb::fetchInProgress() || ESP.getFreeHeap() < config::kMqttMinFreeHeap) {
      // A publish burst here would race the TLS handshake for the heap.
      Serial.printf("MQTT: telemetry skipped, heap %u\n",
                    static_cast<unsigned>(ESP.getFreeHeap()));
      return;
    }
    publishTelemetry();
  }
}

}  // namespace services::mqtt

/**
 * Host test for the Home Assistant MQTT discovery payloads.
 *
 * Pure C++ (no Arduino.h) so it runs on the build machine -- see
 * scripts/run_host_tests.sh.
 */
#include "services/mqtt_discovery.h"
#include "test_util.h"

#include <cstring>

using services::mqtt::Ctx;
using services::mqtt::Entity;

static const Ctx kCtx{"planeradar_a1b2c3", "planeradar/a1b2c3", "1.1.0-mqtt",
                      "homeassistant", "Plane Radar"};

static bool balanced(const char* s) {
  int depth = 0;
  for (; *s; ++s) {
    if (*s == '{') ++depth;
    if (*s == '}') --depth;
    if (depth < 0) return false;
  }
  return depth == 0;
}

int main() {
  char topic[128];
  char payload[600];

  const size_t tn =
      services::mqtt::discoveryTopic(Entity::Range, kCtx, topic, sizeof(topic));
  CHECK(std::strcmp(topic, "homeassistant/select/planeradar_a1b2c3/range/config") == 0);
  CHECK(tn == std::strlen(topic));

  const size_t pn =
      services::mqtt::discoveryPayload(Entity::Range, kCtx, payload, sizeof(payload));
  CHECK(pn > 0 && pn == std::strlen(payload));
  CHECK(balanced(payload));
  CHECK(std::strstr(payload, "\"~\":\"planeradar/a1b2c3\"") != nullptr);
  CHECK(std::strstr(payload, "\"cmd_t\":\"~/cmd/range\"") != nullptr);
  CHECK(std::strstr(payload, "\"stat_t\":\"~/state/range\"") != nullptr);
  CHECK(std::strstr(payload, "\"avty_t\":\"~/status\"") != nullptr);
  CHECK(std::strstr(payload, "\"uniq_id\":\"planeradar_a1b2c3_range\"") != nullptr);
  CHECK(std::strstr(payload,
                    "\"options\":[\"5 km\",\"10 km\",\"15 km\",\"25 km\"]") != nullptr);
  CHECK(std::strstr(payload, "\"ids\":[\"planeradar_a1b2c3\"]") != nullptr);
  CHECK(std::strstr(payload, "\"name\":\"Plane Radar\"") != nullptr);

  // Every entity must serialize, fit the 512-byte MQTT packet and be balanced.
  for (size_t i = 0; i < static_cast<size_t>(Entity::Count); ++i) {
    const auto e = static_cast<Entity>(i);
    const size_t n = services::mqtt::discoveryPayload(e, kCtx, payload, sizeof(payload));
    CHECK(n > 0);
    // PubSubClient's ceiling covers topic + payload + header, not the payload
    // alone: leave ~150 B of headroom for the topic and the packet header.
    CHECK(n < 600);
    CHECK(balanced(payload));

    const size_t t = services::mqtt::discoveryTopic(e, kCtx, topic, sizeof(topic));
    CHECK(t > 0 && t < 128);
    CHECK(std::strstr(topic, "/planeradar_a1b2c3/") != nullptr);
  }

  // Buffer too small must report 0 rather than truncate silently.
  CHECK(services::mqtt::discoveryPayload(Entity::Range, kCtx, payload, 16) == 0);

  // Command payload parsing.
  CHECK(services::mqtt::leadingKm("25 km") == 25);
  CHECK(services::mqtt::leadingKm("10") == 10);
  CHECK(services::mqtt::leadingKm("5km") == 5);
  CHECK(services::mqtt::leadingKm("banana") == -1);
  CHECK(services::mqtt::payloadIsOn("ON"));
  CHECK(services::mqtt::payloadIsOn("on"));
  CHECK(!services::mqtt::payloadIsOn("OFF"));
  CHECK(!services::mqtt::payloadIsOn(""));

  // Nearest telemetry: one JSON document driving state + attributes.
  services::adsb::NearestAircraft n{};
  n.valid = true;
  std::strcpy(n.callsign, "GLO1234");
  std::strcpy(n.type, "B738");
  std::strcpy(n.alt, "32000");
  n.distance_km = 12.34f;
  n.gs_knots = 430.0f;
  n.track_deg = 271.5f;
  const size_t nn = services::mqtt::nearestPayload(n, payload, sizeof(payload));
  CHECK(nn > 0 && balanced(payload));
  CHECK(std::strstr(payload, "\"callsign\":\"GLO1234\"") != nullptr);
  CHECK(std::strstr(payload, "\"distance_km\":12.3") != nullptr);

  // Diagnostic info: one document, state = ip, attributes = the rest.
  const size_t ipn = services::mqtt::infoPayload("10.0.0.26", "MinhaRede", 123456UL,
                                                 "1.1.0-mqtt", payload,
                                                 sizeof(payload));
  CHECK(ipn > 0 && balanced(payload));
  CHECK(std::strstr(payload, "\"ip\":\"10.0.0.26\"") != nullptr);
  CHECK(std::strstr(payload, "\"ssid\":\"MinhaRede\"") != nullptr);
  CHECK(std::strstr(payload, "\"uptime_s\":123456") != nullptr);
  // A quote inside the SSID must be escaped, not break the document.
  CHECK(services::mqtt::infoPayload("10.0.0.26", "a\"b", 1UL, "x", payload,
                                    sizeof(payload)) > 0);
  CHECK(std::strstr(payload, "\"ssid\":\"a\\\"b\"") != nullptr);

  // No aircraft -> "none", never an empty string (HA shows "unknown" otherwise).
  services::adsb::NearestAircraft none{};
  const size_t zn = services::mqtt::nearestPayload(none, payload, sizeof(payload));
  CHECK(zn > 0);
  CHECK(std::strstr(payload, "\"callsign\":\"none\"") != nullptr);

  return testSummary("mqtt_discovery");
}

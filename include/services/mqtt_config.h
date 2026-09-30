#pragma once

#include <cstddef>
#include <cstdint>

namespace services::mqtt {

constexpr size_t kBrokerHostLen = 64;
constexpr size_t kBrokerUserLen = 32;
constexpr size_t kBrokerPassLen = 96;
constexpr size_t kTopicLen = 40;
constexpr size_t kPrefixLen = 32;
constexpr size_t kDeviceNameLen = 40;

/** Everything the radar needs to reach a broker and announce itself in HA.
 *  Every text field stays EMPTY until the user types it in the config portal;
 *  the defaults live in config.h and are applied at use time, never pre-filled. */
struct MqttConfig {
  bool enabled;                        // portal checkbox
  char host[kBrokerHostLen + 1];       // broker IP or hostname
  uint16_t port;                       // empty field -> kMqttDefaultPort
  char user[kBrokerUserLen + 1];
  char pass[kBrokerPassLen + 1];
  char topic[kTopicLen + 1];           // base topic; empty -> planeradar/<mac6>
  char prefix[kPrefixLen + 1];         // discovery prefix; empty -> homeassistant
  char name[kDeviceNameLen + 1];       // HA device name; empty -> Plane Radar
};

/** True when a broker host is filled in; without one there is nothing to reach. */
bool configured(const MqttConfig& cfg);

/** Load from nvs (ns "mqtt"); absent keys stay empty/default. */
void loadConfig(MqttConfig& out);

/** Persist; returns false when the nvs handle cannot be opened. */
bool saveConfig(const MqttConfig& cfg);

/** Drop every stored key (used by the BOOT long-press wipe). */
void clearConfig();

/** Apply the portal form values ("T"/"on" checkbox semantics) and persist. */
void saveFromPortal(const char* host, const char* port, const char* user,
                    const char* pass, const char* topic, const char* prefix,
                    const char* name, const char* enabled_checkbox);

}  // namespace services::mqtt

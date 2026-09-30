/**
 * Host test for the node suffix derived from the eFuse MAC.
 *
 * Pure C++ (no Arduino.h) so it runs on the build machine -- see
 * scripts/run_host_tests.sh.
 *
 * The bug this guards: two radars from the same production batch shared the MQTT
 * base topic AND the MQTT client id, because the suffix took the low 24 bits of
 * ESP.getEfuseMac() -- which are the OUI. Sharing a client id makes the two
 * boards kick each other off the broker.
 */
#include "services/mqtt_config.h"
#include "test_util.h"

using services::mqtt::macSuffix;

int main() {
  // Real MACs from the two bench boards. getEfuseMac() packs mac[0] into the LOW
  // byte, so these are the byte-reversed addresses:
  //   e8:3d:c1:84:b8:78  ->  0x78b884c13de8
  //   e8:3d:c1:81:d0:00  ->  0x00d081c13de8
  const uint64_t board_a = 0x78b884c13de8ULL;
  const uint64_t board_b = 0x00d081c13de8ULL;

  // The suffix is the per-device half (mac[3..5]), read in MAC order, so it
  // looks like the tail of the address printed by esptool.
  CHECK(macSuffix(board_a) == 0x84b878);
  CHECK(macSuffix(board_b) == 0x81d000);

  // The point of the whole change: same batch, different suffix.
  CHECK(macSuffix(board_a) != macSuffix(board_b));

  // The old expression is the regression: identical OUI on both boards.
  CHECK((board_a & 0xFFFFFF) == (board_b & 0xFFFFFF));
  CHECK(services::mqtt::macSuffix(board_a) != (board_a & 0xFFFFFF));

  return testSummary("mqtt_config");
}

/**
 * Host test for the nearest-aircraft math used by the MQTT telemetry.
 *
 * Pure C++ (no Arduino.h) so it runs on the build machine -- see
 * scripts/run_host_tests.sh.
 */
#include "services/nearest_aircraft.h"
#include "test_util.h"

#include <cstring>

using services::adsb::Aircraft;

namespace {

Aircraft make(float lat, float lon, const char* cs) {
  Aircraft a{};
  a.lat = lat;
  a.lon = lon;
  std::strncpy(a.callsign, cs, sizeof(a.callsign) - 1);
  return a;
}

}  // namespace

int main() {
  const double lat0 = -23.2;
  const double lon0 = -45.9;

  // 0.009 deg of latitude == 0.999 km (111 km/deg).
  CHECK(services::adsb::flatDistanceKm(lat0, lon0, lat0 + 0.009f, (float)lon0) > 0.99f);
  CHECK(services::adsb::flatDistanceKm(lat0, lon0, lat0 + 0.009f, (float)lon0) < 1.01f);

  Aircraft list[3] = {make((float)lat0 + 0.09f, (float)lon0, "FAR1"),
                      make((float)lat0 + 0.02f, (float)lon0, "NEAR"),
                      make((float)lat0 + 0.05f, (float)lon0, "MID")};
  const auto n = services::adsb::findNearest(list, 3, lat0, lon0);
  CHECK(n.valid);
  CHECK(std::strcmp(n.callsign, "NEAR") == 0);
  CHECK(n.distance_km > 2.0f && n.distance_km < 2.4f);  // 0.02 * 111 = 2.22 km

  const auto empty = services::adsb::findNearest(list, 0, lat0, lon0);
  CHECK(!empty.valid);
  CHECK(empty.callsign[0] == '\0');
  CHECK(empty.distance_km == 0.0f);

  const auto null_list = services::adsb::findNearest(nullptr, 4, lat0, lon0);
  CHECK(!null_list.valid);

  return testSummary("nearest_aircraft");
}

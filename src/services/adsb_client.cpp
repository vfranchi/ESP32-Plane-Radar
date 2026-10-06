#include "services/adsb_client.h"

#include <esp_heap_caps.h>
#include "services/nearest_aircraft.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include <ArduinoJson.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cctype>
#include <cstring>

#include "config.h"
#include "services/http_body_framing.h"

namespace services::adsb {

namespace {

constexpr char kApiBase[] = "https://opendata.adsb.fi/api/v3/lat/";
// v2 only: the callsign/hex lookup paths do not exist under v3 (v3 answers 400, and an
// invalid request still counts against the feed's rate limit).
constexpr char kApiTargetBase[] = "https://opendata.adsb.fi/api/v2/";
constexpr float kKmPerNm = 1.852f;
constexpr int kConnectTimeoutMs = 5000;  // TLS handshake needs room
constexpr unsigned long kRequestTimeoutMs = 6000;

Aircraft s_aircraft[kMaxAircraft];
size_t s_aircraft_count = 0;
unsigned long s_last_update_ms = 0;
PollFn s_poll_fn = nullptr;
SemaphoreHandle_t s_mutex = nullptr;
// Set for the whole of fetchUpdate(), cleared on every return path.
volatile bool s_fetch_active = false;
// Wall clock the in-flight fetch started at; 0 when idle.
volatile unsigned long s_fetch_start_ms = 0;
// Set by requestFetchAbort() to break the in-flight fetch out of its waits.
volatile bool s_fetch_abort = false;

// The one TLS session, shared by the area fetch and the target lookup: both are on
// opendata.adsb.fi, so a second session would only buy a second pair of 16 KB mbedtls
// record buffers this board cannot afford. Namespace scope, not function scope, because
// HTTPClient's destructor calls _client->stop() without consulting _reuse -- a per-call
// instance silently tore the session down and made every poll re-handshake.
WiFiClientSecure s_client;
HTTPClient s_http;

// Follow mode: the tracked aircraft gets its own slot so following one flight never
// disturbs the surrounding traffic. Same lock as the area list.
Aircraft s_target{};
bool s_target_valid = false;
unsigned long s_target_update_ms = 0;

/** Marks the fetch in flight and guarantees the clear on any early return. */
struct FetchInProgressGuard {
  FetchInProgressGuard() {
    s_fetch_abort = false;
    s_fetch_start_ms = millis();
    s_fetch_active = true;
  }
  ~FetchInProgressGuard() {
    s_fetch_active = false;
    s_fetch_start_ms = 0;
  }
};

/**
 * Re-checked at every wait point of the fetch. Either the watchdog asked us to
 * give up, or the station dropped its link mid-request -- and a fetch with no
 * link can only burn time getting nowhere.
 */
/** Whether the poll that just ran found the TLS socket already open. The signal
 *  that separates "connection reused" from "handshook again" in the serial log. */
bool s_socket_reused = false;

bool fetchShouldStop() {
  return s_fetch_abort || WiFi.status() != WL_CONNECTED;
}

/** Nearest aircraft as of the last fetch, for the MQTT telemetry. */
NearestAircraft s_nearest{};

/** Publish parsed aircraft to the shared buffer atomically. */
void publish(const Aircraft* src, size_t count, double lat0, double lon0) {
  if (s_mutex != nullptr) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
  }
  for (size_t i = 0; i < count; ++i) {
    s_aircraft[i] = src[i];
  }
  s_aircraft_count = count;
  s_last_update_ms = millis();  // base time for dead-reckoning
  // Computed here, where the list is already under the lock: the MQTT task used
  // to do it from a copy of the list, and that 3.3 KB malloc per publish was
  // what starved the TLS handshake.
  s_nearest = findNearest(s_aircraft, count, lat0, lon0);
  if (s_mutex != nullptr) {
    xSemaphoreGive(s_mutex);
  }
}


void pollNetwork() {
  if (s_poll_fn != nullptr) {
    s_poll_fn();
  }
}

int performGetWithPoll(HTTPClient& http) {
  http.setConnectTimeout(kConnectTimeoutMs);
  const unsigned long deadline = millis() + kRequestTimeoutMs;
  while (millis() < deadline && !fetchShouldStop()) {
    pollNetwork();
    const int code = http.GET();
    if (code > 0) {
      return code;
    }
    if (code != HTTPC_ERROR_CONNECTION_REFUSED &&
        code != HTTPC_ERROR_NOT_CONNECTED) {
      return code;
    }
    delay(5);
  }
  return HTTPC_ERROR_READ_TIMEOUT;
}

/**
 * Pumps the socket one byte at a time, in blocks, without buffering the whole
 * response.
 *
 * Each refill runs the network poll callback, which is why HTTPClient's own
 * body readers (getString/writeToStream) can't be used here -- they block
 * without giving the fetch task a chance to poll. That also means they can't
 * de-chunk for us, so the wire image comes out raw and BodyFramer unwraps it.
 *
 * Where the body ends is BodyFramer's business, not this class's: it stops
 * pulling at the right byte. Reading a little past that point into buffer_ is
 * harmless while the connection is torn down after every fetch.
 */
class PollingSocketSource {
 public:
  PollingSocketSource(HTTPClient& http, WiFiClient& stream,
                      unsigned long deadline)
      : http_(&http), stream_(&stream), deadline_(deadline) {}

  /** Next raw byte, or -1 once the socket closes or the deadline passes. */
  int read() {
    if (pos_ >= len_ && !refill()) {
      return -1;
    }
    return static_cast<unsigned char>(buffer_[pos_++]);
  }

 private:
  bool refill() {
    pos_ = 0;
    len_ = 0;
    while (millis() < deadline_ && !fetchShouldStop()) {
      pollNetwork();
      const int available = stream_->available();
      if (available > 0) {
        const int to_read = available > static_cast<int>(sizeof(buffer_))
                                ? static_cast<int>(sizeof(buffer_))
                                : available;
        const int read_bytes = stream_->readBytes(buffer_, to_read);
        if (read_bytes > 0) {
          len_ = static_cast<size_t>(read_bytes);
          return true;
        }
      }
      if (!http_->connected() && stream_->available() <= 0) {
        break;  // server closed and the socket is drained
      }
      delay(1);
    }
    return false;
  }

  HTTPClient* http_;
  WiFiClient* stream_;
  unsigned long deadline_;
  char buffer_[512];
  size_t pos_ = 0;
  size_t len_ = 0;
};

using BodyReader = services::http::BodyFramer<PollingSocketSource>;

float kmToNauticalMiles(float km) { return km / kKmPerNm; }

bool readJsonFloat(const JsonObject& obj, const char* key, float* out) {
  if (obj[key].is<float>() || obj[key].is<double>() || obj[key].is<int>()) {
    *out = obj[key].as<float>();
    return true;
  }
  return false;
}

float pickNoseHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickTrackHeading(const JsonObject& plane, bool* valid) {
  float v = 0.0f;
  if (readJsonFloat(plane, "track", &v)) {
    *valid = true;
    return v;
  }
  if (readJsonFloat(plane, "true_heading", &v)) {
    *valid = true;
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    *valid = true;
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    *valid = true;
    return v;
  }
  *valid = false;
  return 0.0f;
}

float pickGroundSpeed(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "gs", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "tas", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "ias", &v)) {
    return v;
  }
  return 0.0f;
}

/**
 * Below this altitude (ft) an airframe counts as being on the ground even when the feed did
 * not say so, because some receivers report a taxiing aircraft as "0 ft" instead of "ground".
 */
constexpr float kOnGroundAltFt = 100.0f;

/**
 * Area-list ground filter: literal "ground" only, so an apron aircraft reported as "0 ft" is
 * still drawn as traffic. Deliberately narrower than the followed-flight rule above, which
 * must not call a taxiing airframe "live".
 */
bool isOnGround(const JsonObject& plane) {
  if (!plane["alt_baro"].is<const char*>()) {
    return false;
  }
  return strcmp(plane["alt_baro"].as<const char*>(), "ground") == 0;
}

void copyJsonStringTrimmed(const JsonObject& obj, const char* key, char* out,
                           size_t out_len) {
  out[0] = '\0';
  if (out_len == 0 || !obj[key].is<const char*>()) {
    return;
  }
  const char* s = obj[key].as<const char*>();
  size_t n = strnlen(s, out_len - 1);
  while (n > 0 && s[n - 1] == ' ') {
    --n;
  }
  memcpy(out, s, n);
  out[n] = '\0';
}

void formatAltitudeTag(const JsonObject& plane, char* out, size_t out_len,
                       bool* on_ground) {
  out[0] = '\0';
  if (on_ground != nullptr) {
    *on_ground = false;
  }
  if (out_len == 0) {
    return;
  }

  if (plane["alt_baro"].is<const char*>()) {
    const char* s = plane["alt_baro"].as<const char*>();
    if (strcmp(s, "ground") == 0) {
      strncpy(out, "GND", out_len - 1);
      out[out_len - 1] = '\0';
      if (on_ground != nullptr) {
        *on_ground = true;
      }
      return;
    }
  }

  float alt = 0.0f;
  if (readJsonFloat(plane, "alt_baro", &alt) ||
      readJsonFloat(plane, "alt_geom", &alt)) {
    snprintf(out, out_len, "%d ft", static_cast<int>(lroundf(alt)));
    // "0 ft" from a receiver that does not send the "ground" token is still the apron.
    if (on_ground != nullptr && alt < kOnGroundAltFt) {
      *on_ground = true;
    }
  }
}

/**
 * Private / general aviation: emitter category A1 (light) or A2 (small). Many
 * transponders report no category at all, so fall back to the callsign -- an
 * N-number (N + digit) is a US registration, which a light airframe broadcasts
 * as its callsign.
 */
bool isPrivateAircraft(const JsonObject& plane, const char* callsign) {
  if (plane["category"].is<const char*>()) {
    const char* c = plane["category"].as<const char*>();
    return strcmp(c, "A1") == 0 || strcmp(c, "A2") == 0;
  }
  return callsign[0] == 'N' && isdigit(static_cast<unsigned char>(callsign[1]));
}

void fillTagFields(Aircraft* ac, const JsonObject& plane) {
  copyJsonStringTrimmed(plane, "flight", ac->callsign, sizeof(ac->callsign));
  if (ac->callsign[0] == '\0') {
    copyJsonStringTrimmed(plane, "hex", ac->callsign, sizeof(ac->callsign));
  }

  copyJsonStringTrimmed(plane, "t", ac->type, sizeof(ac->type));
  formatAltitudeTag(plane, ac->alt, sizeof(ac->alt), &ac->on_ground);
  ac->is_private = isPrivateAircraft(plane, ac->callsign);
}

/**
 * Fill one Aircraft from a feed entry.
 *
 * Applies no ground filter: the caller decides, because the area list skips parked
 * aircraft while follow mode must still see a flight sitting on the apron.
 */
bool fillAircraft(const JsonObject& plane, Aircraft* ac) {
  if (!plane["lat"].is<float>() || !plane["lon"].is<float>()) {
    return false;
  }
  ac->lat = plane["lat"].as<float>();
  ac->lon = plane["lon"].as<float>();
  ac->nose_deg = pickNoseHeading(plane);
  ac->track_deg = pickTrackHeading(plane, &ac->track_valid);
  ac->gs_knots = pickGroundSpeed(plane);

  // seen_pos: seconds since this position was measured. Used as the dead-reckoning age
  // offset, capped so a very stale fix is not flung far.
  float seen_pos = 0.0f;
  readJsonFloat(plane, "seen_pos", &seen_pos);
  if (seen_pos < 0.0f) seen_pos = 0.0f;
  if (seen_pos > 30.0f) seen_pos = 30.0f;
  ac->pos_age_ms = static_cast<uint32_t>(seen_pos * 1000.0f);

  fillTagFields(ac, plane);
  return true;
}

}  // namespace

const NearestAircraft& nearest() { return s_nearest; }

void init() {
  if (s_mutex == nullptr) {
    s_mutex = xSemaphoreCreateMutex();
  }
}

void setPollFn(PollFn fn) { s_poll_fn = fn; }

size_t aircraftCount() { return s_aircraft_count; }

const Aircraft* aircraftList() { return s_aircraft; }

unsigned long lastUpdateMs() { return s_last_update_ms; }

bool fetchInProgress() { return s_fetch_active; }

unsigned long fetchElapsedMs() {
  return s_fetch_active ? millis() - s_fetch_start_ms : 0;
}

void requestFetchAbort() { s_fetch_abort = true; }

size_t snapshotAircraft(Aircraft* out, size_t max_out,
                        unsigned long* out_last_update_ms) {
  if (s_mutex != nullptr) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
  }
  const size_t count =
      s_aircraft_count < max_out ? s_aircraft_count : max_out;
  for (size_t i = 0; i < count; ++i) {
    out[i] = s_aircraft[i];
  }
  if (out_last_update_ms != nullptr) {
    *out_last_update_ms = s_last_update_ms;
  }
  if (s_mutex != nullptr) {
    xSemaphoreGive(s_mutex);
  }
  return count;
}

/**
 * GET `url` and stream-parse it into `doc` through `filter`, over an existing TLS session.
 *
 * The session is passed in rather than owned here: the feed keeps one connection alive
 * between polls (the handshake costs ~1 s of CPU on this chip -- measured 899-1140 ms on
 * the device against 10 ms from a LAN host), while a one-shot lookup on another host
 * passes its own transient pair, whose destructor frees the 2x16 KB mbedTLS record
 * buffers on return.
 */
bool getJsonDocument(WiFiClientSecure& client, HTTPClient& http, const String& url,
                     const char* tag, JsonDocument& doc, const JsonDocument& filter,
                     bool keep_alive, bool* out_socket_reused) {
  client.setInsecure();
  if (out_socket_reused != nullptr) {
    *out_socket_reused = client.connected();
  }

  if (!http.begin(client, url)) {
    Serial.printf("%s: http.begin failed\n", tag);
    return false;
  }
  // Only with this does HTTPClient::disconnect() keep the socket open instead of
  // calling _client->stop() -- end() consult its _reuse flag, the destructor does not.
  http.setReuse(keep_alive);

  // HTTPClient only records Transfer-Encoding in the collected headers when it is asked
  // for up front; _transferEncoding itself is private.
  static const char* kWantedHeaders[] = {"Transfer-Encoding"};
  http.collectHeaders(kWantedHeaders, 1);
  http.setTimeout(kRequestTimeoutMs);

  const int code = performGetWithPoll(http);
  if (code != HTTP_CODE_OK) {
    if (fetchShouldStop()) {
      Serial.printf("%s: fetch abandoned (abort or link lost)\n", tag);
    } else {
      Serial.printf("%s: HTTP %d\n", tag, code);
    }
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  if (stream == nullptr) {
    Serial.printf("%s: no response stream\n", tag);
    http.end();
    return false;
  }

  // On HTTP/1.1 the CDN answers with Transfer-Encoding: chunked, and getStreamPtr()
  // hands back the raw socket -- chunk sizes and all. BodyFramer strips that back off.
  const services::http::BodyFraming framing =
      http.header("Transfer-Encoding").equalsIgnoreCase("chunked")
          ? services::http::BodyFraming::kChunked
          : services::http::BodyFraming::kIdentity;

  PollingSocketSource source(http, *stream, millis() + kRequestTimeoutMs);
  BodyReader body(source, framing, http.getSize());
  const DeserializationError err =
      deserializeJson(doc, body, DeserializationOption::Filter(filter));
  // Read off the terminating chunk the parser stopped short of, so the socket sits at
  // the end of the message. That is what makes reuse possible at all.
  body.drain();
  http.end();

  if (err) {
    if (fetchShouldStop()) {
      Serial.printf("%s: fetch abandoned (abort or link lost)\n", tag);
    } else if (body.framingError()) {
      Serial.printf("%s: malformed chunked body\n", tag);
    } else if (body.bytesRead() == 0) {
      Serial.printf("%s: empty response\n", tag);
    } else {
      Serial.printf("%s: JSON parse error: %s\n", tag, err.c_str());
    }
    return false;
  }
  return true;
}

bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km) {
  const FetchInProgressGuard fetch_guard;
  const float dist_nm = kmToNauticalMiles(fetch_radius_km);

  String url = kApiBase;
  url += String(center_lat, 6);
  url += "/lon/";
  url += String(center_lon, 6);
  url += "/dist/";
  url += String(dist_nm, 1);

  // Keep only the fields we render; the rest never reaches RAM.
  JsonDocument filter;
  JsonObject f = filter["ac"].add<JsonObject>();
  for (const char* key :
       {"lat", "lon", "true_heading", "mag_heading", "track", "dir", "gs",
        "tas", "ias", "alt_baro", "alt_geom", "seen_pos", "flight", "hex", "t",
        "category"}) {
    f[key] = true;
  }

  JsonDocument doc;
  if (!getJsonDocument(s_client, s_http, url, "adsb", doc, filter, /*keep_alive=*/true,
                       &s_socket_reused)) {
    return false;
  }

  // Parse into a local buffer, then publish atomically so a reader on another
  // thread never sees a half-updated list.
  Aircraft parsed[kMaxAircraft];
  size_t n = 0;
  JsonArray ac = doc["ac"].as<JsonArray>();
  if (!ac.isNull()) {
    for (JsonObject plane : ac) {
      if (n >= kMaxAircraft) {
        break;
      }
      if (isOnGround(plane) && !config::kAdsbShowGroundAircraft) {
        continue;
      }
      if (fillAircraft(plane, &parsed[n])) {
        ++n;
      }
    }
  }

  publish(parsed, n, center_lat, center_lon);
  // heap/largest block after every fetch: this board runs within a few KB of
  // failing the next TLS handshake, and the number that predicts it is the
  // largest free block, not the free heap.
  Serial.printf("adsb: %u aircraft heap %u free of %u, largest block %u, sock %u\n",
                static_cast<unsigned>(n),
                static_cast<unsigned>(ESP.getFreeHeap()),
                static_cast<unsigned>(ESP.getHeapSize()),
                static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
                static_cast<unsigned>(s_socket_reused));
  return true;
}

unsigned long targetUpdateMs() { return s_target_update_ms; }

bool targetValid() {
  bool valid = false;
  if (s_mutex != nullptr) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
  }
  valid = s_target_valid;
  if (s_mutex != nullptr) {
    xSemaphoreGive(s_mutex);
  }
  return valid;
}

bool targetSnapshot(Aircraft* out) {
  if (out == nullptr) {
    return false;
  }
  if (s_mutex != nullptr) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
  }
  const bool valid = s_target_valid;
  if (valid) {
    *out = s_target;
  }
  if (s_mutex != nullptr) {
    xSemaphoreGive(s_mutex);
  }
  return valid;
}

bool fetchTarget(const char* id, bool is_hex) {
  // Never build a request that can only be invalid: the feed counts 400/404 against the
  // rate limit, and a blank target means follow mode is off.
  if (id == nullptr || id[0] == '\0') {
    return false;
  }
  const FetchInProgressGuard fetch_guard;  // the watchdog ladder covers this lookup too

  String url = kApiTargetBase;
  url += is_hex ? "hex/" : "callsign/";
  url += id;

  // Same fields as the area fetch, so track_valid/pos_age_ms keep their meaning.
  JsonDocument filter;
  JsonObject f = filter["ac"].add<JsonObject>();
  for (const char* key :
       {"lat", "lon", "true_heading", "mag_heading", "track", "dir", "gs", "tas",
        "ias", "alt_baro", "alt_geom", "seen_pos", "flight", "hex", "t",
        "category"}) {
    f[key] = true;
  }

  JsonDocument doc;
  if (!getJsonDocument(s_client, s_http, url, "adsb-target", doc, filter,
                       /*keep_alive=*/true, nullptr)) {
    return false;  // the request itself failed; the caller keeps the previous state
  }

  Aircraft found{};
  bool have = false;
  JsonArray ac = doc["ac"].as<JsonArray>();
  if (!ac.isNull()) {
    for (JsonObject plane : ac) {
      Aircraft candidate{};
      if (!fillAircraft(plane, &candidate)) {
        continue;
      }
      const bool airborne = !isOnGround(plane);
      if (have && !airborne) {
        continue;  // a reused callsign: keep the airborne airframe
      }
      found = candidate;
      have = true;
      if (airborne) {
        break;
      }
    }
  }

  if (s_mutex != nullptr) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
  }
  s_target_valid = have;
  if (have) {
    s_target = found;
    // The followed aircraft's own base time: the area list has its own, because the two
    // are refreshed at different rates.
    s_target_update_ms = millis();
  }
  if (s_mutex != nullptr) {
    xSemaphoreGive(s_mutex);
  }

  // "not in feed" is a state (the flight is not being tracked right now), not a failure.
  Serial.printf("adsb: target %s %s\n", id, have ? "found" : "not in feed");
  return true;
}

}  // namespace services::adsb

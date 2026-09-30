#pragma once

namespace services::mqtt {

/** Load config, build the node id. Call once in setup() after location init. */
void init();

/** Non-blocking: connect, publish discovery, poll, publish telemetry.
 *  Call every loop() iteration; never from the ADS-B fetch task. */
void loop();

/** Hand the socket's heap back to the ADS-B fetch: closes the connection (a
 *  clean DISCONNECT, so the will is not published and HA keeps the entities
 *  available) and blocks reconnects until resumeAfterFetch(). The fetch's TLS
 *  handshake needs one large block and this board has ~50 KB free. */
void releaseForFetch();

/** Let the client reconnect again. Call right after the fetch returns. */
void resumeAfterFetch();

}  // namespace services::mqtt

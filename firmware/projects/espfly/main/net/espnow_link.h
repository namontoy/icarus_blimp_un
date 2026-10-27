/* espnow_link.h - ESP-NOW ground link: NVS-free WiFi/ESP-NOW bring-up,
 * inbound command queue, outbound telemetry send.
 *
 * Owns its own WiFi/ESP-NOW init (esp_netif, the default event loop,
 * esp_wifi, esp_now) so main.c's app_main stays a short, readable sequence
 * of "init NVS, init board, init network, create tasks" per the task
 * contract. Requires nvs_flash_init() to already have been called (WiFi
 * init reads calibration data from NVS) - main.c does this before calling
 * blimp_espnow_link_init().
 */
#ifndef BLIMP_NET_ESPNOW_LINK_H
#define BLIMP_NET_ESPNOW_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "protocol.h"

/* Brings up esp_netif, the default event loop, WiFi (STA mode, no AP
 * association - see board_config.h's channel comment for why that matters),
 * and ESP-NOW; registers the receive callback and adds
 * BLIMP_ESPNOW_PEER_MAC as a peer. Call once, after nvs_flash_init(). */
esp_err_t blimp_espnow_link_init(void);

/* Tears down ESP-NOW and WiFi. Not currently called anywhere in this
 * skeleton (the vehicle never intentionally goes comms-dark mid-run - a
 * failsafe means "hold position", not "shut off the radio") - provided for
 * completeness/testability. */
void blimp_espnow_link_deinit(void);

/* Non-blocking. Copies the most recently received, magic/version/CRC-
 * validated command packet into *out and returns true; returns false (and
 * leaves *out untouched) if nothing new has arrived since the last call.
 * Safe to poll from comms_task's own loop - never blocks the caller. */
bool blimp_espnow_link_get_command(blimp_cmd_packet_t *out);

/* Best-effort send - ESP-NOW has no delivery acknowledgement/retry at this
 * layer (esp_now_send()'s own send-callback, not used here, only reports
 * whether the radio accepted the frame, not whether a peer received it),
 * and telemetry does not need one: a dropped telemetry frame just means one
 * missing sample in the ground log, never a control-affecting event. */
esp_err_t blimp_espnow_link_send_telemetry(const blimp_telemetry_packet_t *pkt);

#endif /* BLIMP_NET_ESPNOW_LINK_H */

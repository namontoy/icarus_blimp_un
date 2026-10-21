/*
 * The blimp's own Wi-Fi access point, and the 'wifi' console command.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise NVS, the network stack and the Wi-Fi driver, and configure the
 * access point from the CONFIG_NET_WIFI_* settings. Starts the radio too
 * when CONFIG_NET_WIFI_AT_BOOT is set. Safe to call more than once.
 *
 * Failure is reported, not fatal: a bench instrument without a radio is
 * still a bench instrument, and the USB console is how we find out why.
 */
esp_err_t net_wifi_init(void);

/** Bring the access point up / take it down. Idempotent. */
esp_err_t net_wifi_start(void);
esp_err_t net_wifi_stop(void);

bool net_wifi_is_up(void);

/** Stations currently associated, counted from the driver's events. */
int net_wifi_station_count(void);

/** The SSID actually in use (prefix plus MAC suffix), "" before init. */
const char *net_wifi_ssid(void);

/** Dotted-quad address of the access point into buf. */
esp_err_t net_wifi_ip_str(char *buf, size_t len);

/** Register the 'wifi' command with the console. */
void register_wifi(void);

#ifdef __cplusplus
}
#endif

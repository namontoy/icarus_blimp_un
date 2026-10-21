/*
 * UDP command and telemetry link over the access point, and the 'link'
 * console command. Packet formats are in link_protocol.h.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open the UDP socket on CONFIG_NET_LINK_PORT and start the link task, which
 * receives commands, guards the link timeout and streams telemetry to the
 * last host that sent a valid packet. Needs the motor driver initialised;
 * does not need the access point to be up yet.
 */
esp_err_t net_link_start(void);

typedef struct {
    bool     running;
    bool     have_peer;
    bool     alive;         /* a valid packet arrived within the timeout    */
    bool     link_motors;   /* standing motor commands came over the link   */
    char     peer[24];      /* "a.b.c.d:port" of the last valid sender      */
    uint32_t age_ms;        /* since the last valid packet                  */
    uint32_t rx_ok;
    uint32_t rx_bad;
    uint32_t tx;
} net_link_status_t;

void net_link_get_status(net_link_status_t *out);

/** Register the 'link' command with the console. */
void register_link(void);

#ifdef __cplusplus
}
#endif

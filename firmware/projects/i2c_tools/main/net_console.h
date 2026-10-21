/*
 * The bench console over UDP: every registered command, reachable over the
 * access point without a cable.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open the UDP socket on CONFIG_NET_CONSOLE_PORT and start the task that
 * runs received command lines through the console registry. Call after the
 * REPL has been created (which initialises the registry) and after every
 * register_*() call, so the first datagram can already be 'help'.
 */
esp_err_t net_console_start(void);

#ifdef __cplusplus
}
#endif

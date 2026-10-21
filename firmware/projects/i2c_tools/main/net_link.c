/*
 * net_link.c - UDP command and telemetry link over the access point.
 *
 * One packet per datagram, format in link_protocol.h. The blimp never
 * initiates: it answers whoever sent the last valid packet, the simplest
 * session model there is and the one esp-drone used - one ground station at
 * a time, and reconnecting is just sending again.
 *
 * The contract that matters for safety: a MOTOR_SET is a standing command
 * only while the host keeps repeating it. Two timers guard the standing
 * state, deliberately independent of each other:
 *
 *   - this file's link timeout (CONFIG_NET_LINK_TIMEOUT_MS): if the motors
 *     were last commanded over the air and no valid packet of any kind
 *     arrives within the window, every motor coasts and the event is logged
 *     once. A dead laptop, a dropped association and a host script stuck in
 *     a breakpoint all look the same from here, and all get the same answer.
 *   - the motor driver's own failsafe (CONFIG_MOTOR_FAILSAFE_MS): coasts if
 *     no thrust command of any origin arrives while a motor is driven. It
 *     catches a host that keeps sending heartbeats but has stopped sending
 *     motor commands, which the link timeout alone would miss.
 *
 * So a host that wants a motor to keep turning sends MOTOR_SET at 20 Hz or
 * so, not once. HEARTBEAT keeps the link alive and the telemetry flowing but
 * drives nothing. Motors set from the USB console are not touched by the
 * link timeout: a 'motor --hold' typed at the desk is deliberate.
 *
 * Telemetry keeps flowing for a while after the host goes quiet, so a host
 * that paused still sees that the motors coasted and the link went stale.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "cmd_motor.h"
#include "link_protocol.h"
#include "net_link.h"
#include "net_wifi.h"

static const char *TAG = "net_link";

/* recvfrom() timeout. Bounds both the telemetry jitter and how late the
   link timeout can fire; 10 ms of polling is free at this scale. */
#define LINK_RX_POLL_MS         10

/* Telemetry keeps going this many timeouts past the last packet. */
#define LINK_PEER_HOLD_MULT     10

#define LINK_TASK_STACK         4096
#define LINK_TASK_PRIO          4   /* above the consoles, below Wi-Fi/lwIP */

static struct {
    bool               running;
    int                sock;
    struct sockaddr_in peer;          /* whoever sent the last valid packet */
    bool               have_peer;
    volatile int64_t   last_rx_us;
    volatile uint32_t  rx_ok;
    volatile uint32_t  rx_bad;
    volatile uint32_t  tx;
    uint16_t           tx_seq;
    volatile bool      link_motors;   /* standing commands came over the air */
} s;

/* --------------------------------------------------------------- receive */

/*
 * Right size for its type, and a CRC that matches. Checks the CRC in place:
 * the last four bytes are zeroed for the computation, which is how the
 * sender computed it.
 */
static bool packet_ok(uint8_t *buf, int n, size_t expect)
{
    if (n != (int)expect) {
        return false;
    }
    uint32_t sent;
    memcpy(&sent, buf + n - 4, 4);
    memset(buf + n - 4, 0, 4);
    return link_crc32(buf, (size_t)n) == sent;
}

static void handle_motor_set(const link_motor_set_t *p)
{
    /* Copy out of the packed struct: the driver takes a float array and the
       compiler is right that a pointer into a packed struct may not be
       aligned for one. */
    float cmd[MOTOR_COUNT];
    memcpy(cmd, p->cmd, sizeof(cmd));

    esp_err_t err = motors_set_all(cmd);
    if (err != ESP_OK) {
        /* A NaN out of a host script, or a motor driver that never came
           up. Worth a line each time: it means a command was dropped. */
        ESP_LOGW(TAG, "MOTOR_SET rejected: %s", esp_err_to_name(err));
        return;
    }
    bool driven = false;
    for (int m = 0; m < MOTOR_COUNT; m++) {
        if (cmd[m] != 0.0f) {
            driven = true;
        }
    }
    s.link_motors = driven;
}

static void handle_motor_stop(const link_motor_stop_t *p)
{
    if (p->brake) {
        motors_brake();
    } else {
        motors_coast();
    }
    s.link_motors = false;
}

static void handle_datagram(uint8_t *buf, int n, const struct sockaddr_in *from, int64_t now)
{
    if (n < (int)sizeof(link_header_t)) {
        s.rx_bad++;
        return;
    }
    link_header_t h;
    memcpy(&h, buf, sizeof(h));
    if (h.magic != LINK_PROTO_MAGIC || h.version != LINK_PROTO_VERSION) {
        s.rx_bad++;
        return;
    }

    switch (h.type) {
    case LINK_PKT_HEARTBEAT:
        if (!packet_ok(buf, n, sizeof(link_heartbeat_t))) {
            s.rx_bad++;
            return;
        }
        break;
    case LINK_PKT_MOTOR_SET: {
        if (!packet_ok(buf, n, sizeof(link_motor_set_t))) {
            s.rx_bad++;
            return;
        }
        link_motor_set_t p;
        memcpy(&p, buf, sizeof(p));
        handle_motor_set(&p);
        break;
    }
    case LINK_PKT_MOTOR_STOP: {
        if (!packet_ok(buf, n, sizeof(link_motor_stop_t))) {
            s.rx_bad++;
            return;
        }
        link_motor_stop_t p;
        memcpy(&p, buf, sizeof(p));
        handle_motor_stop(&p);
        break;
    }
    default:
        s.rx_bad++;
        return;
    }

    /* Any valid packet, of any type, is proof of life and names the peer. */
    s.peer = *from;
    s.have_peer = true;
    s.last_rx_us = now;
    s.rx_ok++;
}

/* --------------------------------------------------------------- timeout */

static void check_link_timeout(int64_t now)
{
    if (!s.link_motors) {
        return;
    }
    int64_t age_ms = (now - s.last_rx_us) / 1000;
    if (age_ms < CONFIG_NET_LINK_TIMEOUT_MS) {
        return;
    }
    /* Clear the flag first so this logs once, then coast. motors_coast()
       also disarms the driver's failsafe, so nothing stays latched. */
    s.link_motors = false;
    motors_coast();
    ESP_LOGW(TAG, "link lost: no packet for %lld ms, all motors coasting", (long long)age_ms);
}

/* ------------------------------------------------------------- telemetry */

static void send_telemetry(int64_t now)
{
    if (!s.have_peer) {
        return;
    }
    int64_t age_ms = (now - s.last_rx_us) / 1000;
    if (age_ms > (int64_t)CONFIG_NET_LINK_TIMEOUT_MS * LINK_PEER_HOLD_MULT) {
        return;
    }

    link_telemetry_t t;
    memset(&t, 0, sizeof(t));
    t.h.magic   = LINK_PROTO_MAGIC;
    t.h.version = LINK_PROTO_VERSION;
    t.h.type    = LINK_PKT_TELEMETRY;
    t.h.seq     = s.tx_seq++;
    t.t_us      = (uint32_t)now;
    for (int m = 0; m < MOTOR_COUNT; m++) {
        t.motor_cmd[m] = motor_get((motor_id_t)m);
    }
    t.failsafe    = (uint8_t)motors_failsafe_state();
    t.link_alive  = age_ms < CONFIG_NET_LINK_TIMEOUT_MS;
    t.link_motors = s.link_motors;
    t.stations    = (uint8_t)net_wifi_station_count();
    t.rx_ok       = s.rx_ok;
    t.rx_bad      = s.rx_bad;
    t.free_heap   = esp_get_free_heap_size();
    t.link_age_ms = age_ms > 65535 ? 65535 : (uint16_t)age_ms;
    t.crc32       = link_crc32(&t, sizeof(t));

    int sent = sendto(s.sock, &t, sizeof(t), 0, (struct sockaddr *)&s.peer, sizeof(s.peer));
    if (sent == (int)sizeof(t)) {
        s.tx++;
    }
}

/* ------------------------------------------------------------------ task */

static void link_task(void *arg)
{
    (void)arg;
    const int64_t period_us = 1000000 / CONFIG_NET_LINK_TELEMETRY_HZ;
    int64_t next_tx_us = esp_timer_get_time() + period_us;
    uint8_t buf[LINK_MAX_PACKET_BYTES];

    for (;;) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(s.sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        int64_t now = esp_timer_get_time();
        if (n > 0) {
            handle_datagram(buf, n, &from, now);
        }
        check_link_timeout(now);
        if (now >= next_tx_us) {
            send_telemetry(now);
            next_tx_us += period_us;
            if (next_tx_us < now) {     /* fell behind: resync, don't burst */
                next_tx_us = now + period_us;
            }
        }
    }
}

esp_err_t net_link_start(void)
{
    if (s.running) {
        return ESP_OK;
    }

    s.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s.sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_NET_LINK_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s.sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind to UDP %d: errno %d", CONFIG_NET_LINK_PORT, errno);
        close(s.sock);
        return ESP_FAIL;
    }
    struct timeval tv = { .tv_sec = 0, .tv_usec = LINK_RX_POLL_MS * 1000 };
    setsockopt(s.sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (xTaskCreate(link_task, "net_link", LINK_TASK_STACK, NULL, LINK_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "could not start the link task");
        close(s.sock);
        return ESP_ERR_NO_MEM;
    }
    s.running = true;
    ESP_LOGI(TAG, "UDP link on port %d: telemetry %d Hz, timeout %d ms",
             CONFIG_NET_LINK_PORT, CONFIG_NET_LINK_TELEMETRY_HZ, CONFIG_NET_LINK_TIMEOUT_MS);
    return ESP_OK;
}

void net_link_get_status(net_link_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->running     = s.running;
    out->have_peer   = s.have_peer;
    out->link_motors = s.link_motors;
    out->rx_ok       = s.rx_ok;
    out->rx_bad      = s.rx_bad;
    out->tx          = s.tx;
    if (s.have_peer) {
        int64_t age = (esp_timer_get_time() - s.last_rx_us) / 1000;
        out->age_ms = age < 0 ? 0 : (uint32_t)age;
        out->alive  = age < CONFIG_NET_LINK_TIMEOUT_MS;
        char ip[16];
        inet_ntoa_r(s.peer.sin_addr, ip, sizeof(ip));
        snprintf(out->peer, sizeof(out->peer), "%s:%u", ip, (unsigned)ntohs(s.peer.sin_port));
    }
}

/* --------------------------------------------------------------- console */

static int do_link_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    net_link_status_t st;
    net_link_get_status(&st);
    if (!st.running) {
        printf("link not running - see the boot log for why\n");
        return 1;
    }
    printf("UDP port %d   telemetry %d Hz   timeout %d ms\n",
           CONFIG_NET_LINK_PORT, CONFIG_NET_LINK_TELEMETRY_HZ, CONFIG_NET_LINK_TIMEOUT_MS);
    if (!st.have_peer) {
        printf("no host has sent a valid packet yet\n");
    } else {
        printf("peer %s   %s, last packet %" PRIu32 " ms ago\n", st.peer,
               st.alive ? "alive" : "STALE", st.age_ms);
    }
    printf("rx ok %" PRIu32 "   rx rejected %" PRIu32 "   tx %" PRIu32 "\n",
           st.rx_ok, st.rx_bad, st.tx);
    printf("motors %s\n", st.link_motors ? "driven over the link" : "not driven over the link");
    return 0;
}

void register_link(void)
{
    const esp_console_cmd_t cmd = {
        .command = "link",
        .help = "Status of the UDP command/telemetry link",
        .hint = NULL,
        .func = &do_link_cmd,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

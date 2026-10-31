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
 *     were last commanded over the air and no MOTOR_SET or MOTOR_STOP
 *     arrives within the window, every motor coasts and the event is logged
 *     once. A dead laptop, a dropped association and a host script stuck in
 *     a breakpoint all look the same from here, and all get the same answer.
 *     Only motor packets count, since 2026-10-29: a heartbeat comes from its
 *     own thread on the host and proves the program runs, not that anyone is
 *     still steering, and another station's packets must not keep this
 *     one's command alive (docs/link-robustness-review.md, F2).
 *   - the motor driver's own failsafe (CONFIG_MOTOR_FAILSAFE_MS): coasts if
 *     no thrust command of any origin arrives while a motor is driven. It is
 *     the backstop for this task itself stopping, and the only timer on a
 *     motor run from the console.
 *
 * So a host that wants a motor to keep turning sends MOTOR_SET at 20 Hz or
 * so, not once; the repeat is the keep-alive and carries the value, so a lost
 * packet is repaired by the next one. HEARTBEAT keeps the link alive and the
 * telemetry flowing but drives nothing. A MOTOR_SET older than the last one
 * from the same sender (by its sequence number) is dropped and counted, so a
 * packet the radio delayed past a STOP cannot start the motors again;
 * MOTOR_STOP is obeyed from anyone, always. Motors set from the USB console
 * are not touched by the link timeout: a 'motor --hold' typed at the desk is
 * deliberate.
 *
 * Telemetry keeps flowing for a while after the host goes quiet, so a host
 * that paused still sees that the motors coasted and the link went stale.
 *
 * The capture (capture.c) is driven from here too: LOG_CMD starts, stops,
 * clears or queries it, and GET reads it out in chunks of 16 records. The
 * chunks are built in a static buffer, because 1.2 kB would be a third of
 * this task's stack, and sent back to back from this task: a burst of eight
 * holds telemetry up by a few milliseconds, which the host asked for.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "capture.h"
#include "cmd_motor.h"
#include "ctl_task.h"
#include "link_protocol.h"
#include "net_link.h"
#include "net_wifi.h"
#include "sensor_stream.h"

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
    uint32_t           sensors_seq;   /* the stream's sample last sent       */
    volatile uint32_t  tx_sensors;
    /* The last accepted MOTOR_SET or MOTOR_STOP: the link timeout runs from
       it, and the next MOTOR_SET from the same sender must be newer. */
    volatile int64_t   last_motor_rx_us;
    struct sockaddr_in motor_from;
    uint16_t           motor_seq;
    bool               have_motor;
    volatile uint32_t  rx_stale;      /* MOTOR_SETs dropped as out of date   */
    volatile uint32_t  rx_refused;    /* MOTOR_SETs refused while the loop holds the motors */
    volatile uint32_t  last_rx_u32;   /* last_rx_us cut to 32 bits: one atomic word
                                         for the control task (net_link_last_rx) */
} s;

/* The one LOG_CHUNK under construction; the link task alone touches it. */
static link_log_chunk_t s_chunk;

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

static bool same_sender(const struct sockaddr_in *a, const struct sockaddr_in *b)
{
    return a->sin_addr.s_addr == b->sin_addr.s_addr && a->sin_port == b->sin_port;
}

/*
 * A motor packet from the sender of the last one, within the link timeout,
 * whose sequence number is not newer: delayed or duplicated by the radio.
 * Judged per sender and per session, because a host that restarted, or a
 * second station, starts a count of its own, and a session silent for a
 * whole timeout is over anyway. The difference is taken as int16_t so the
 * 16-bit count wraps without a jump; each host numbers all its packets with
 * one counter, so its motor packets are increasing but not consecutive.
 */
static bool motor_packet_stale(uint16_t seq, const struct sockaddr_in *from, int64_t now)
{
    if (!s.have_motor || !same_sender(from, &s.motor_from) ||
        (now - s.last_motor_rx_us) / 1000 >= CONFIG_NET_LINK_TIMEOUT_MS) {
        return false;
    }
    return (int16_t)(uint16_t)(seq - s.motor_seq) <= 0;
}

static void note_motor_packet(uint16_t seq, const struct sockaddr_in *from, int64_t now)
{
    s.motor_from = *from;
    s.motor_seq = seq;
    s.have_motor = true;
    s.last_motor_rx_us = now;
}

static bool handle_motor_set(const link_motor_set_t *p)
{
    /* The bench hold (ctl_task.c) owns the motors: a MOTOR_SET is counted and
       dropped, without the error line motors_set_all() would earn 20 times a
       second, and it is not a motor packet for the link timeout either. A
       MOTOR_STOP still lands: it kills the hold (cmd_motor.h, the stop hook). */
    if (motors_loop_owns()) {
        s.rx_refused++;
        return false;
    }
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
        return false;
    }
    bool driven = false;
    for (int m = 0; m < MOTOR_COUNT; m++) {
        if (cmd[m] != 0.0f) {
            driven = true;
        }
    }
    s.link_motors = driven;
    return true;
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

/* ------------------------------------------------------------- capture */

static void send_to(const void *pkt, size_t len, const struct sockaddr_in *to)
{
    int sent = sendto(s.sock, pkt, len, 0, (const struct sockaddr *)to, sizeof(*to));
    if (sent == (int)len) {
        s.tx++;
    }
}

static void send_log_info(uint8_t result, const struct sockaddr_in *to)
{
    capture_info_t ci;
    capture_info(&ci);

    link_log_info_t p;
    memset(&p, 0, sizeof(p));
    p.h.magic   = LINK_PROTO_MAGIC;
    p.h.version = LINK_PROTO_VERSION;
    p.h.type    = LINK_PKT_LOG_INFO;
    p.h.seq     = s.tx_seq++;
    p.state       = ci.have_buffer ? ci.state : LINK_LOG_IDLE;
    p.result      = (!ci.have_buffer && result == LINK_LOG_OK) ? LINK_LOG_ERR_NO_BUFFER : result;
    p.record_size = LINK_LOG_RECORD_BYTES;
    p.format      = LINK_LOG_FORMAT;
    p.capture_id  = ci.capture_id;
    p.period_us   = (uint16_t)CTL_PERIOD_US;
    p.count       = ci.count;
    p.capacity    = ci.capacity;
    p.wanted      = ci.wanted;
    p.t_start_us  = ci.t_start_us;
    p.imu_accel_g       = ci.imu_accel_g;
    p.imu_gyro_dps      = ci.imu_gyro_dps;
    p.mag_lsb_per_gauss = ci.mag_lsb_per_gauss;
    p.mag_chip          = ci.mag_chip;
    p.baro_chip         = ci.baro_chip;
    p.max_seconds       = (uint16_t)(ci.capacity / (1000000u / CTL_PERIOD_US));
    strncpy(p.fw, esp_app_get_description()->version, sizeof(p.fw) - 1);
    p.crc32 = link_crc32(&p, sizeof(p));
    send_to(&p, sizeof(p), to);
}

static void handle_log_cmd(const link_log_cmd_t *p, const struct sockaddr_in *from)
{
    switch (p->cmd) {
    case LINK_LOG_CMD_INFO:
        send_log_info(LINK_LOG_OK, from);
        return;
    case LINK_LOG_CMD_START: {
        esp_err_t err = capture_start(p->seconds);
        send_log_info(err == ESP_OK ? LINK_LOG_OK :
                      err == ESP_ERR_INVALID_STATE ? LINK_LOG_ERR_BUSY :
                      err == ESP_ERR_INVALID_SIZE ? LINK_LOG_ERR_LENGTH : LINK_LOG_ERR_NO_BUFFER, from);
        return;
    }
    case LINK_LOG_CMD_STOP:
        capture_stop();
        send_log_info(LINK_LOG_OK, from);
        return;
    case LINK_LOG_CMD_CLEAR:
        capture_clear();
        send_log_info(LINK_LOG_OK, from);
        return;
    case LINK_LOG_CMD_GET: {
        unsigned chunks = p->count ? p->count : 1;
        if (chunks > LINK_LOG_GET_MAX_CHUNKS) {
            chunks = LINK_LOG_GET_MAX_CHUNKS;
        }
        for (unsigned k = 0; k < chunks; k++) {
            const uint32_t first = p->first + k * LINK_LOG_CHUNK_RECORDS;
            uint16_t id = 0;
            const uint32_t n = capture_read(first, LINK_LOG_CHUNK_RECORDS,
                                            (log_record_t *)s_chunk.data, &id);
            if (n == 0) {
                /* Past the end, or the capture changed while we read: the
                   INFO tells the host which, by count and capture_id. */
                send_log_info(LINK_LOG_ERR_RANGE, from);
                return;
            }
            memset(&s_chunk.h, 0, sizeof(s_chunk.h));
            s_chunk.h.magic   = LINK_PROTO_MAGIC;
            s_chunk.h.version = LINK_PROTO_VERSION;
            s_chunk.h.type    = LINK_PKT_LOG_CHUNK;
            s_chunk.h.seq     = s.tx_seq++;
            s_chunk.capture_id = id;
            s_chunk.count      = (uint8_t)n;
            s_chunk._reserved  = 0;
            s_chunk.first      = first;
            if (n < LINK_LOG_CHUNK_RECORDS) {
                memset(s_chunk.data + n * LINK_LOG_RECORD_BYTES, 0,
                       (LINK_LOG_CHUNK_RECORDS - n) * LINK_LOG_RECORD_BYTES);
            }
            s_chunk.crc32 = 0;
            s_chunk.crc32 = link_crc32(&s_chunk, sizeof(s_chunk));
            send_to(&s_chunk, sizeof(s_chunk), from);
            if (n < LINK_LOG_CHUNK_RECORDS) {
                return;                 /* the end of what is stored */
            }
        }
        return;
    }
    default:
        send_log_info(LINK_LOG_ERR_CMD, from);
        return;
    }
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
        if (motor_packet_stale(h.seq, from, now)) {
            s.rx_stale++;           /* valid, so still proof of life below */
            break;
        }
        link_motor_set_t p;
        memcpy(&p, buf, sizeof(p));
        if (handle_motor_set(&p)) {
            note_motor_packet(h.seq, from, now);
        }
        break;
    }
    case LINK_PKT_MOTOR_STOP: {
        if (!packet_ok(buf, n, sizeof(link_motor_stop_t))) {
            s.rx_bad++;
            return;
        }
        link_motor_stop_t p;
        memcpy(&p, buf, sizeof(p));
        handle_motor_stop(&p);      /* from anyone, whatever its number */
        if (!motor_packet_stale(h.seq, from, now)) {
            /* A newer STOP moves the sender's mark past every MOTOR_SET it
               sent before it, so one the radio delays is dropped. */
            note_motor_packet(h.seq, from, now);
        }
        break;
    }
    case LINK_PKT_LOG_CMD: {
        if (!packet_ok(buf, n, sizeof(link_log_cmd_t))) {
            s.rx_bad++;
            return;
        }
        link_log_cmd_t p;
        memcpy(&p, buf, sizeof(p));
        handle_log_cmd(&p, from);
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
    s.last_rx_u32 = (uint32_t)now;
    s.rx_ok++;
}

/* --------------------------------------------------------------- timeout */

static void check_link_timeout(int64_t now)
{
    if (!s.link_motors) {
        return;
    }
    /* Motor packets only: see the top of this file. link_motors is set only
       by an accepted MOTOR_SET, which also set last_motor_rx_us. */
    int64_t age_ms = (now - s.last_motor_rx_us) / 1000;
    if (age_ms < CONFIG_NET_LINK_TIMEOUT_MS) {
        return;
    }
    /* Clear the flag first so this logs once, then coast. motors_coast()
       also disarms the driver's failsafe, so nothing stays latched. */
    s.link_motors = false;
    motors_coast();
    ESP_LOGW(TAG, "link lost: no motor command for %lld ms, all motors coasting", (long long)age_ms);
}

/* ------------------------------------------------------------- telemetry */

/* The blimp answers the last valid sender, and keeps answering for a while
   after it goes quiet, so a host that paused still sees the link go stale. */
static bool peer_listening(int64_t now, int64_t *age_ms)
{
    if (!s.have_peer) {
        return false;
    }
    *age_ms = (now - s.last_rx_us) / 1000;
    return *age_ms <= (int64_t)CONFIG_NET_LINK_TIMEOUT_MS * LINK_PEER_HOLD_MULT;
}

static void send_telemetry(int64_t now)
{
    int64_t age_ms = 0;
    if (!peer_listening(now, &age_ms)) {
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

/*
 * The sensor stream's newest sample, once per sample. sensor_stream.c keeps
 * the latest reading and a sequence number; this sends it when the number
 * has moved. The stream and the telemetry tick run at the same rate by
 * default, so one SENSORS packet follows each TELEMETRY. While the stream is
 * paused by a console command, or off, nothing is sent, and the ground
 * station shows the gap rather than a frozen number.
 */
static void send_sensors(int64_t now)
{
    int64_t age_ms = 0;
    if (!peer_listening(now, &age_ms)) {
        return;
    }
    link_sensors_t p;
    uint32_t seq = 0;
    if (!sensor_stream_latest(&p, &seq) || seq == s.sensors_seq) {
        return;
    }
    s.sensors_seq = seq;

    p.h.magic     = LINK_PROTO_MAGIC;
    p.h.version   = LINK_PROTO_VERSION;
    p.h.type      = LINK_PKT_SENSORS;
    p.h.seq       = s.tx_seq++;
    p.h._reserved = 0;
    p.crc32       = 0;
    p.crc32       = link_crc32(&p, sizeof(p));

    int sent = sendto(s.sock, &p, sizeof(p), 0, (struct sockaddr *)&s.peer, sizeof(s.peer));
    if (sent == (int)sizeof(p)) {
        s.tx++;
        s.tx_sensors++;
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
            send_sensors(now);
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
    out->rx_stale    = s.rx_stale;
    out->rx_refused  = s.rx_refused;
    out->tx          = s.tx;
    out->tx_sensors  = s.tx_sensors;
    out->have_motor  = s.have_motor;
    if (s.have_motor) {
        int64_t age = (esp_timer_get_time() - s.last_motor_rx_us) / 1000;
        out->motor_age_ms = age < 0 ? 0 : (uint32_t)age;
    }
    if (s.have_peer) {
        int64_t age = (esp_timer_get_time() - s.last_rx_us) / 1000;
        out->age_ms = age < 0 ? 0 : (uint32_t)age;
        out->alive  = age < CONFIG_NET_LINK_TIMEOUT_MS;
        char ip[16];
        inet_ntoa_r(s.peer.sin_addr, ip, sizeof(ip));
        snprintf(out->peer, sizeof(out->peer), "%s:%u", ip, (unsigned)ntohs(s.peer.sin_port));
    }
}

bool net_link_last_rx(uint32_t *t_us)
{
    *t_us = s.last_rx_u32;
    return s.have_peer;
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
    printf("rx ok %" PRIu32 "   rx rejected %" PRIu32 "   motor commands out of date %" PRIu32
           "   tx %" PRIu32 " (of which sensors %" PRIu32 ")\n",
           st.rx_ok, st.rx_bad, st.rx_stale, st.tx, st.tx_sensors);
    if (st.rx_refused) {
        printf("motor commands refused while the control loop held the motors: %" PRIu32 "\n",
               st.rx_refused);
    }
    printf("motors %s", st.link_motors ? "driven over the link" : "not driven over the link");
    if (st.have_motor) {
        printf(", last motor command %" PRIu32 " ms ago", st.motor_age_ms);
    }
    printf("\n");
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

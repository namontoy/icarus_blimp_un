/*
 * cam_link.c - the two ESP32-CAMs' rows, and 'cam'. The forward camera
 * (target_cam) on UART1, the downward one (optflow_cam) on UART2 since
 * 2026-11-02; the wiring is in cam_link.h; what counts as a row, and why
 * strictly, in cam_row.c and flow_row.c.
 *
 * Each port has its own reader task and line buffer, and both offer every
 * line to both parsers: a balloon row goes to the balloon mailbox and a flow
 * row to the flow mailbox whichever wire it came on. A swapped pair of cables
 * then still works (and 'cam' says which port each kind arrives on), and the
 * row's column count, not the port, keeps a flow row from ever reading as a
 * balloon.
 *
 * The S3 writes to a camera only on 'cam --tx' / '--tx-down', a line a
 * person typed: target_cam takes single-letter commands, and a stray 's'
 * would stop its rows.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "cam_link.h"
#include "cam_row.h"
#include "flow_row.h"

static const char *TAG = "cam";

#define CAM_RX_BUF          2048        /* 1.5 s of rows; the reader drains it every 20 ms */
#define CAM_LINE_MAX        384         /* a v2 row is about 200 characters */
#define CAM_TASK_CORE       0
#define CAM_TASK_PRIO       3           /* with the sensor stream, below the link (4) */
#define CAM_TASK_STACK      4096
#define CAM_RECENT          8           /* lines kept per port for 'cam --raw' */
#define CAM_INFO            16          /* '#' lines kept per port for '--tx'  */
#define CAM_KEEP_CHARS      120
#define CAM_TX_WAIT_MS      600         /* how long --tx collects replies  */

typedef struct {
    uint32_t t_us;
    char     text[CAM_KEEP_CHARS];
} kept_line_t;

/* One UART and the camera on it. The configuration is fixed at build time;
   the counters live in st, under s.mux. */
typedef struct {
    uart_port_t       uart;
    const char       *uart_name;        /* "UART1": what 'cam' prints      */
    const char       *who;              /* "forward", "downward"           */
    const char       *task_name;
    int               rx_gpio, tx_gpio, baud;
    cam_port_status_t st;
    kept_line_t       recent[CAM_RECENT];
    uint32_t          recent_n;         /* lines ever kept; the ring index is n % size */
    kept_line_t       info[CAM_INFO];
    uint32_t          info_n;
    char              line[CAM_LINE_MAX];   /* the reader task's own */
    size_t            len;
    bool              overlong;
} cam_port_t;

static cam_port_t ports[CAM_PORTS] = {
    [CAM_PORT_FWD] = {
        .uart = UART_NUM_1, .uart_name = "UART1", .who = "forward", .task_name = "cam_rx",
        .rx_gpio = CONFIG_CAM_FWD_UART_RX_GPIO, .tx_gpio = CONFIG_CAM_FWD_UART_TX_GPIO,
        .baud = CONFIG_CAM_FWD_UART_BAUD,
    },
    [CAM_PORT_DOWN] = {
        .uart = UART_NUM_2, .uart_name = "UART2", .who = "downward", .task_name = "cam_dn",
        .rx_gpio = CONFIG_CAM_DOWN_UART_RX_GPIO, .tx_gpio = CONFIG_CAM_DOWN_UART_TX_GPIO,
        .baud = CONFIG_CAM_DOWN_UART_BAUD,
    },
};

static struct {
    portMUX_TYPE      mux;              /* guards this and every port's st and rings */
    cam_link_status_t st;               /* the rows' counts; the port fields are copied in cam_link_status() */
    cam_result_t      latest;
    cam_link_row_t    latest_row;       /* the same row, every column kept */
    bool              have;
    int               sign;
    uint32_t          window_start_us, window_rows;
    /* optflow_cam's rows: their own mailbox, so a flow row never reads as a
       balloon and 'hold follow' never takes one for the forward camera */
    cam_flow_t        flow;
    flow_row_t        flow_row;         /* the same row, every column kept */
    bool              have_flow;
    uint32_t          flow_window_start_us, flow_window_rows, flow_window_ok;
    float             flow_sum_x, flow_sum_y;   /* px since boot or 'cam --zero' */
} s = {
    .mux  = portMUX_INITIALIZER_UNLOCKED,
    .sign = (CONFIG_CAM_FWD_BEARING_SIGN < 0) ? -1 : 1,
};

static uint32_t now_us(void)
{
    return (uint32_t)esp_timer_get_time();
}

static void keep(kept_line_t *ring, size_t size, uint32_t *n, const char *text, uint32_t t)
{
    kept_line_t *k = &ring[*n % size];
    k->t_us = t;
    strncpy(k->text, text, sizeof(k->text) - 1);
    k->text[sizeof(k->text) - 1] = '\0';
    (*n)++;
}

/* A row into the control task's mailbox format; false for anything else. */
static bool parse_row(const char *line, cam_result_t *r, cam_row_t *row_out)
{
    cam_row_t row;
    if (!cam_row_parse(line, &row)) {
        return false;
    }
    *row_out = row;
    memset(r, 0, sizeof(*r));
    r->detected = row.detected;
    r->cx_px    = (uint16_t)lroundf(row.cx_px);
    r->cy_px    = (uint16_t)lroundf(row.cy_px);
    r->area_px  = (uint16_t)fminf(row.area_px, 65535.0f);
    r->az_deg   = row.az_deg;
    r->el_deg   = row.el_deg;
    /* width_px / height_px: the frame size is not in the row; 0 = unknown. */
    return true;
}

static void handle_line(cam_port_t *p, char *line, size_t len)
{
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) {
        line[--len] = '\0';
    }
    if (len == 0) {
        return;
    }
    const uint32_t t = now_us();
    cam_result_t r;
    cam_row_t parsed;
    flow_row_t fr;
    const bool info = line[0] == '#';
    const bool row = !info && parse_row(line, &r, &parsed);
    const bool flow = !info && !row && flow_row_parse(line, &fr);

    portENTER_CRITICAL(&s.mux);
    keep(p->recent, CAM_RECENT, &p->recent_n, line, t);
    if (info) {
        keep(p->info, CAM_INFO, &p->info_n, line, t);
        p->st.info_lines++;
        if (strstr(line, "streaming off")) {
            p->st.streaming_off = true;
        } else if (strstr(line, "streaming on")) {
            p->st.streaming_off = false;
        }
    } else if (row) {
        r.frame_seq = s.latest.frame_seq + 1u;
        r.t_us = t;
        s.latest = r;
        s.latest_row.frame_seq = r.frame_seq;
        s.latest_row.t_us = t;
        s.latest_row.row = parsed;
        s.have = true;
        s.st.rows++;
        s.st.rows_detected += r.detected ? 1u : 0u;
        s.st.last_row_us = t;
        s.st.csv_version = parsed.version;
        p->st.rows++;
        p->st.streaming_off = false;
        if (s.window_start_us == 0u || (uint32_t)(t - s.window_start_us) >= 1000000u) {
            s.st.rows_last_s = s.window_rows;
            s.window_start_us = t;
            s.window_rows = 0;
        }
        s.window_rows++;
    } else if (flow) {
        s.flow.frame_seq = s.flow.frame_seq + 1u;
        s.flow.t_us      = t;
        s.flow.dx_px     = fr.dx_px;
        s.flow.dy_px     = fr.dy_px;
        s.flow.spread_px = fr.spread_px;
        s.flow.ok        = fr.ok ? 1 : 0;
        s.flow.valid     = fr.valid;
        s.flow.total     = fr.total;
        s.flow_row = fr;
        s.have_flow = true;
        s.flow_sum_x += fr.dx_px;
        s.flow_sum_y += fr.dy_px;
        s.st.flow_rows++;
        s.st.flow_rows_ok += fr.ok ? 1u : 0u;
        s.st.flow_last_row_us = t;
        p->st.flow_rows++;
        p->st.streaming_off = false;
        if (s.flow_window_start_us == 0u || (uint32_t)(t - s.flow_window_start_us) >= 1000000u) {
            s.st.flow_rows_last_s = s.flow_window_rows;
            s.st.flow_ok_last_s = s.flow_window_ok;
            s.flow_window_start_us = t;
            s.flow_window_rows = 0;
            s.flow_window_ok = 0;
        }
        s.flow_window_rows++;
        s.flow_window_ok += fr.ok ? 1u : 0u;
    } else {
        p->st.rejected++;
    }
    portEXIT_CRITICAL(&s.mux);
}

static void reader_task(void *arg)
{
    cam_port_t *p = arg;
    uint8_t buf[128];
    for (;;) {
        const int n = uart_read_bytes(p->uart, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }
        portENTER_CRITICAL(&s.mux);
        p->st.bytes += (uint32_t)n;
        portEXIT_CRITICAL(&s.mux);
        for (int i = 0; i < n; i++) {
            const char c = (char)buf[i];
            if (c == '\n') {
                if (p->overlong) {
                    portENTER_CRITICAL(&s.mux);
                    p->st.overflows++;
                    portEXIT_CRITICAL(&s.mux);
                } else {
                    p->line[p->len] = '\0';
                    handle_line(p, p->line, p->len);
                }
                p->len = 0;
                p->overlong = false;
            } else if (p->len < CAM_LINE_MAX - 1) {
                p->line[p->len++] = c;
            } else {
                p->overlong = true;
            }
        }
    }
}

/* One port up: driver, pins (always set explicitly: UART2 has no IO-MUX pins
   of its own, so its defaults route nowhere useful), the pull-up and the
   reader. A failure stays on that port: the other camera still runs. */
static esp_err_t port_start(cam_port_t *p)
{
    const uart_config_t cfg = {
        .baud_rate  = p->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(p->uart, CAM_RX_BUF, 0, 0, NULL, 0);
    if (err == ESP_OK) {
        err = uart_param_config(p->uart, &cfg);
    }
    if (err == ESP_OK) {
        const int tx = p->tx_gpio >= 0 ? p->tx_gpio : UART_PIN_NO_CHANGE;
        err = uart_set_pin(p->uart, tx, p->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        /* An unplugged camera must read as an idle line, not as noise. */
        gpio_pullup_en((gpio_num_t)p->rx_gpio);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s camera, %s on RX GPIO%d: %s", p->who, p->uart_name, p->rx_gpio, esp_err_to_name(err));
        return err;
    }
    if (xTaskCreatePinnedToCore(reader_task, p->task_name, CAM_TASK_STACK, p, CAM_TASK_PRIO, NULL,
                                CAM_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "%s camera: reader task not created", p->who);
        return ESP_ERR_NO_MEM;
    }
    portENTER_CRITICAL(&s.mux);
    p->st.running = true;
    portEXIT_CRITICAL(&s.mux);
    ESP_LOGI(TAG, "%s camera on %s: RX GPIO%d, TX GPIO%d, %d baud", p->who, p->uart_name, p->rx_gpio,
             p->tx_gpio, p->baud);
    return ESP_OK;
}

esp_err_t cam_link_start(void)
{
    /* The configuration is static; copy it into the status once, so a port
       that failed to start still reports its pins. */
    for (int i = 0; i < CAM_PORTS; i++) {
        ports[i].st.rx_gpio = ports[i].rx_gpio;
        ports[i].st.tx_gpio = ports[i].tx_gpio;
        ports[i].st.baud    = ports[i].baud;
    }
    const esp_err_t fwd  = port_start(&ports[CAM_PORT_FWD]);
    const esp_err_t down = port_start(&ports[CAM_PORT_DOWN]);
    return fwd != ESP_OK ? fwd : down;
}

bool cam_link_latest(cam_result_t *out)
{
    portENTER_CRITICAL(&s.mux);
    const bool have = s.have;
    *out = s.latest;
    portEXIT_CRITICAL(&s.mux);
    return have;
}

bool cam_link_latest_row(cam_link_row_t *out)
{
    portENTER_CRITICAL(&s.mux);
    const bool have = s.have;
    *out = s.latest_row;
    portEXIT_CRITICAL(&s.mux);
    return have;
}

bool cam_link_latest_flow(cam_flow_t *out)
{
    portENTER_CRITICAL(&s.mux);
    const bool have = s.have_flow;
    *out = s.flow;
    portEXIT_CRITICAL(&s.mux);
    return have;
}

int cam_link_bearing_sign(void)
{
    return s.sign;
}

void cam_link_status(cam_link_status_t *out)
{
    portENTER_CRITICAL(&s.mux);
    *out = s.st;
    for (int i = 0; i < CAM_PORTS; i++) {
        out->port[i] = ports[i].st;
    }
    portEXIT_CRITICAL(&s.mux);
    /* The top-level port fields are the forward port's, as before the second
       camera: net_link.c relays them as the forward camera's health. */
    const cam_port_status_t *f = &out->port[CAM_PORT_FWD];
    out->running       = f->running;
    out->rx_gpio       = f->rx_gpio;
    out->tx_gpio       = f->tx_gpio;
    out->baud          = f->baud;
    out->bytes         = f->bytes;
    out->info_lines    = f->info_lines;
    out->rejected      = f->rejected;
    out->overflows     = f->overflows;
    out->streaming_off = f->streaming_off;
    if (out->last_row_us == 0u || (uint32_t)(now_us() - out->last_row_us) > 1500000u) {
        out->rows_last_s = 0;
    }
    if (out->flow_last_row_us == 0u || (uint32_t)(now_us() - out->flow_last_row_us) > 1500000u) {
        out->flow_rows_last_s = 0;
        out->flow_ok_last_s = 0;
    }
}

/* ------------------------------------------------------------- console */

/* optflow_cam's rows: the rate, how many measured, the latest, and the sum
   since 'cam --zero', so a person can slide a surface 30 cm and read it. */
static void print_flow(const cam_link_status_t *st)
{
    cam_flow_t f;
    flow_row_t fr;
    float sx, sy;
    portENTER_CRITICAL(&s.mux);
    f = s.flow;
    fr = s.flow_row;
    sx = s.flow_sum_x;
    sy = s.flow_sum_y;
    portEXIT_CRITICAL(&s.mux);
    printf("optical flow (optflow_cam): %" PRIu32 " rows/s, %" PRIu32 " measured; rows %" PRIu32
           " (measured %" PRIu32 "), last %" PRIu32 " ms ago\n",
           st->flow_rows_last_s, st->flow_ok_last_s, st->flow_rows, st->flow_rows_ok,
           (now_us() - f.t_us) / 1000u);
    printf("latest: %s  dx %+.3f  dy %+.3f px  spread %.3f  blocks %u of %u (flat %u, edge %u, ambig %u), "
           "match %" PRIu32 " us\n",
           f.ok ? "ok" : "REFUSED", (double)f.dx_px, (double)f.dy_px, (double)f.spread_px, (unsigned)f.valid,
           (unsigned)f.total, (unsigned)fr.flat, (unsigned)fr.edge, (unsigned)fr.ambig, fr.proc_us);
    printf("moved since boot or 'cam --zero': x %+.1f px, y %+.1f px (image axes; mm = px x distance / 144)\n",
           (double)sx, (double)sy);
}

/* One port's counts, and what to check when nothing useful arrives on it.
   The wording of the forward port's hints is what software/cam_watch.py
   looks for ('nothing has arrived', 'no row parses'); the downward port's
   hints say it differently, so a missing downward camera never reads there
   as a missing forward one. */
static void print_port(const cam_port_t *p, const cam_port_status_t *ps, bool forward)
{
    printf("  %s camera, %s RX GPIO%d", p->who, p->uart_name, ps->rx_gpio);
    if (ps->tx_gpio >= 0) {
        printf(" TX GPIO%d", ps->tx_gpio);
    } else {
        printf(" no TX");
    }
    if (!ps->running) {
        printf(": not running (see the boot log)\n");
        return;
    }
    printf(": balloon rows %" PRIu32 ", flow rows %" PRIu32 ", '#' lines %" PRIu32 ", rejected %" PRIu32
           ", overlong %" PRIu32 ", bytes %" PRIu32 "\n",
           ps->rows, ps->flow_rows, ps->info_lines, ps->rejected, ps->overflows, ps->bytes);
    const char *expect = forward ? "target_cam" : "optflow_cam";
    if (ps->bytes == 0) {
        if (forward) {
            printf("  nothing has arrived: check the camera is powered, CAM IO14 -> 470 ohm -> "
                   "S3 GPIO%d, and the grounds joined\n", ps->rx_gpio);
        } else {
            printf("  downward camera silent so far: its power, CAM IO14 -> 470 ohm -> S3 GPIO%d, "
                   "the grounds\n", ps->rx_gpio);
        }
    } else if (ps->rows == 0 && ps->flow_rows == 0) {
        if (forward) {
            printf("  bytes arrive but no row parses: a wrong baud, or neither target_cam's nor "
                   "optflow_cam's rows; 'cam --raw' shows what arrives\n");
        } else {
            printf("  downward bytes but no rows yet: a wrong baud or another firmware; 'cam --raw'\n");
        }
    } else if (forward ? ps->rows == 0 : ps->flow_rows == 0) {
        printf("  (this port carries the other camera's rows, not %s's: swapped cables? both "
               "mailboxes take either port)\n", expect);
    }
}

static void print_status(void)
{
    cam_link_status_t st;
    cam_result_t r;
    cam_link_status(&st);
    const bool have = cam_link_latest(&r);
    const cam_port_status_t *fwd = &st.port[CAM_PORT_FWD];
    const cam_port_status_t *down = &st.port[CAM_PORT_DOWN];

    /* The first line keeps its pre-2026-11-02 shape: the scripts take the
       first "N rows/s" in the reply as the forward camera's rate. */
    if (!fwd->running) {
        printf("forward camera: UART1 not running (see the boot log)\n");
    } else {
        printf("UART1, RX GPIO%d", fwd->rx_gpio);
        if (fwd->tx_gpio >= 0) {
            printf(", TX GPIO%d", fwd->tx_gpio);
        } else {
            printf(", no TX");
        }
        printf(", %d baud: %" PRIu32 " rows/s", fwd->baud, st.rows_last_s);
        if (have) {
            printf(", last row %" PRIu32 " ms ago (CSV v%d)", (now_us() - r.t_us) / 1000u, st.csv_version);
        }
        printf("\n");
    }
    printf("balloon rows %" PRIu32 " (detected in %" PRIu32 "); per port:\n", st.rows, st.rows_detected);
    print_port(&ports[CAM_PORT_FWD], fwd, true);
    print_port(&ports[CAM_PORT_DOWN], down, false);
    if (st.flow_rows > 0) {
        print_flow(&st);
    }
    if (have && r.detected) {
        printf("latest: DETECTED  az %+.1f deg  el %+.1f deg  cx %u  cy %u  area %u px\n",
               (double)r.az_deg, (double)r.el_deg, r.cx_px, r.cy_px, r.area_px);
        const float b = (float)cam_link_bearing_sign() * r.az_deg;
        printf("core/'s bearing: %+.1f deg, to the blimp's %s (sign %+d: 'cam --sign' to change)\n",
               (double)b, b > 0.0f ? "LEFT" : b < 0.0f ? "RIGHT" : "nose", cam_link_bearing_sign());
    } else if (have) {
        printf("latest: nothing detected\n");
    }
    if (fwd->streaming_off) {
        printf("the camera said '# streaming off': 'cam --tx s' turns its rows back on\n");
    }
    if (down->streaming_off) {
        printf("the downward camera's last '#' line said its rows are off\n");
    }
}

static void print_recent(void)
{
    const uint32_t t = now_us();
    for (int pi = 0; pi < CAM_PORTS; pi++) {
        const cam_port_t *p = &ports[pi];
        kept_line_t copy[CAM_RECENT];
        uint32_t n;
        portENTER_CRITICAL(&s.mux);
        memcpy(copy, p->recent, sizeof(copy));
        n = p->recent_n;
        portEXIT_CRITICAL(&s.mux);
        printf("%s (%s camera, RX GPIO%d):\n", p->uart_name, p->who, p->rx_gpio);
        if (n == 0) {
            printf("  no line received yet\n");
            continue;
        }
        for (uint32_t i = n > CAM_RECENT ? n - CAM_RECENT : 0; i < n; i++) {
            const kept_line_t *k = &copy[i % CAM_RECENT];
            printf("%6" PRIu32 " ms ago  %s\n", (t - k->t_us) / 1000u, k->text);
        }
    }
}

static int send_line(cam_port_t *p, int argc, char **argv)
{
    if (p->tx_gpio < 0) {
        printf("no TX pin configured for the %s camera (%s)\n", p->who,
               p == &ports[CAM_PORT_FWD] ? "CONFIG_CAM_FWD_UART_TX_GPIO = -1" : "CONFIG_CAM_DOWN_UART_TX_GPIO = -1");
        return 1;
    }
    if (!p->st.running) {
        printf("%s is not running (see the boot log)\n", p->uart_name);
        return 1;
    }
    char out[CAM_KEEP_CHARS];
    size_t len = 0;
    for (int i = 2; i < argc; i++) {
        const int w = snprintf(out + len, sizeof(out) - len, "%s%s", i > 2 ? " " : "", argv[i]);
        if (w < 0 || (size_t)w >= sizeof(out) - len - 2) {
            printf("too long\n");
            return 1;
        }
        len += (size_t)w;
    }
    out[len++] = '\n';
    const uint32_t t0 = now_us();
    uart_write_bytes(p->uart, out, len);
    vTaskDelay(pdMS_TO_TICKS(CAM_TX_WAIT_MS));

    kept_line_t copy[CAM_INFO];
    uint32_t n;
    portENTER_CRITICAL(&s.mux);
    memcpy(copy, p->info, sizeof(copy));
    n = p->info_n;
    portEXIT_CRITICAL(&s.mux);
    int shown = 0;
    for (uint32_t i = n > CAM_INFO ? n - CAM_INFO : 0; i < n; i++) {
        if ((int32_t)(copy[i % CAM_INFO].t_us - t0) >= 0) {
            printf("%s\n", copy[i % CAM_INFO].text);
            shown++;
        }
    }
    printf("(sent %u bytes on %s; %d '#' line%s back in %d ms)\n", (unsigned)len, p->uart_name, shown,
           shown == 1 ? "" : "s", CAM_TX_WAIT_MS);
    return 0;
}

static int cam_usage(void)
{
    printf("cam                  both cameras' links: rows/s, the latest detection and flow, core/'s bearing\n"
           "cam --raw            the last %d lines of each port exactly as they arrived\n"
           "cam --tx <text>      send one line to the forward camera ('?' lists its settings; 's' toggles its rows)\n"
           "cam --tx-down <text> the same to the downward camera (UART2)\n"
           "cam --sign <-1|1>    core/'s bearing = sign x the camera's az, until the next boot\n"
           "cam --zero           restart the optical flow's 'moved since' sum\n",
           CAM_RECENT);
    return 1;
}

static int do_cam_cmd(int argc, char **argv)
{
    if (argc == 1) {
        print_status();
        return 0;
    }
    if (strcmp(argv[1], "--raw") == 0 && argc == 2) {
        print_recent();
        return 0;
    }
    if (strcmp(argv[1], "--tx") == 0 && argc >= 3) {
        return send_line(&ports[CAM_PORT_FWD], argc, argv);
    }
    if (strcmp(argv[1], "--tx-down") == 0 && argc >= 3) {
        return send_line(&ports[CAM_PORT_DOWN], argc, argv);
    }
    if (strcmp(argv[1], "--zero") == 0 && argc == 2) {
        portENTER_CRITICAL(&s.mux);
        s.flow_sum_x = s.flow_sum_y = 0.0f;
        portEXIT_CRITICAL(&s.mux);
        printf("optical flow sum zeroed\n");
        return 0;
    }
    if (strcmp(argv[1], "--sign") == 0 && argc == 3) {
        const int v = atoi(argv[2]);
        if (v != 1 && v != -1) {
            printf("the sign is 1 or -1\n");
            return 1;
        }
        s.sign = v;
        printf("core/'s bearing = %+d x az until the next boot (Kconfig: %+d)\n", v,
               CONFIG_CAM_FWD_BEARING_SIGN < 0 ? -1 : 1);
        return 0;
    }
    return cam_usage();
}

void register_cam(void)
{
    const esp_console_cmd_t cmd = {
        .command = "cam",
        .help = "The cameras' links: the forward one (target_cam, UART1) and the downward one (optflow_cam, "
                "UART2): rows, the latest detection and flow, and the bearing core/ gets. 'cam --raw' shows "
                "the last lines, 'cam --tx ?' asks the forward camera.",
        .hint = "[--raw | --tx <text> | --tx-down <text> | --sign <-1|1> | --zero]",
        .func = &do_cam_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

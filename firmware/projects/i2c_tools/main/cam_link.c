/*
 * cam_link.c - the forward ESP32-CAM's rows on UART1, and 'cam'. The wiring
 * is in cam_link.h; what counts as a row, and why strictly, in cam_row.c.
 *
 * The S3 writes to the camera only on 'cam --tx', a line a person typed:
 * target_cam takes single-letter commands, and a stray 's' would stop its
 * rows.
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

static const char *TAG = "cam";

#define CAM_UART            UART_NUM_1
#define CAM_RX_BUF          2048        /* 1.5 s of rows; the reader drains it every 20 ms */
#define CAM_LINE_MAX        384         /* a v2 row is about 200 characters */
#define CAM_TASK_CORE       0
#define CAM_TASK_PRIO       3           /* with the sensor stream, below the link (4) */
#define CAM_TASK_STACK      4096
#define CAM_RECENT          8           /* lines kept for 'cam --raw'      */
#define CAM_INFO            16          /* '#' lines kept for 'cam --tx'   */
#define CAM_KEEP_CHARS      120
#define CAM_TX_WAIT_MS      600         /* how long --tx collects replies  */

typedef struct {
    uint32_t t_us;
    char     text[CAM_KEEP_CHARS];
} kept_line_t;

static struct {
    portMUX_TYPE      mux;              /* guards everything below but the line buffer */
    cam_link_status_t st;
    cam_result_t      latest;
    bool              have;
    int               sign;
    uint32_t          window_start_us, window_rows;
    kept_line_t       recent[CAM_RECENT];
    uint32_t          recent_n;         /* lines ever kept; the ring index is n % size */
    kept_line_t       info[CAM_INFO];
    uint32_t          info_n;
    char              line[CAM_LINE_MAX];   /* the reader task's own */
    size_t            len;
    bool              overlong;
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
static bool parse_row(const char *line, cam_result_t *r, int *version)
{
    cam_row_t row;
    if (!cam_row_parse(line, &row)) {
        return false;
    }
    memset(r, 0, sizeof(*r));
    r->detected = row.detected;
    r->cx_px    = (uint16_t)lroundf(row.cx_px);
    r->cy_px    = (uint16_t)lroundf(row.cy_px);
    r->area_px  = (uint16_t)fminf(row.area_px, 65535.0f);
    r->az_deg   = row.az_deg;
    r->el_deg   = row.el_deg;
    /* width_px / height_px: the frame size is not in the row; 0 = unknown. */
    *version = row.version;
    return true;
}

static void handle_line(char *line, size_t len)
{
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) {
        line[--len] = '\0';
    }
    if (len == 0) {
        return;
    }
    const uint32_t t = now_us();
    cam_result_t r;
    int version = 0;
    const bool info = line[0] == '#';
    const bool row = !info && parse_row(line, &r, &version);

    portENTER_CRITICAL(&s.mux);
    keep(s.recent, CAM_RECENT, &s.recent_n, line, t);
    if (info) {
        keep(s.info, CAM_INFO, &s.info_n, line, t);
        s.st.info_lines++;
        if (strstr(line, "streaming off")) {
            s.st.streaming_off = true;
        } else if (strstr(line, "streaming on")) {
            s.st.streaming_off = false;
        }
    } else if (row) {
        r.frame_seq = s.latest.frame_seq + 1u;
        r.t_us = t;
        s.latest = r;
        s.have = true;
        s.st.rows++;
        s.st.rows_detected += r.detected ? 1u : 0u;
        s.st.last_row_us = t;
        s.st.csv_version = version;
        s.st.streaming_off = false;
        if (s.window_start_us == 0u || (uint32_t)(t - s.window_start_us) >= 1000000u) {
            s.st.rows_last_s = s.window_rows;
            s.window_start_us = t;
            s.window_rows = 0;
        }
        s.window_rows++;
    } else {
        s.st.rejected++;
    }
    portEXIT_CRITICAL(&s.mux);
}

static void reader_task(void *arg)
{
    (void)arg;
    uint8_t buf[128];
    for (;;) {
        const int n = uart_read_bytes(CAM_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }
        portENTER_CRITICAL(&s.mux);
        s.st.bytes += (uint32_t)n;
        portEXIT_CRITICAL(&s.mux);
        for (int i = 0; i < n; i++) {
            const char c = (char)buf[i];
            if (c == '\n') {
                if (s.overlong) {
                    portENTER_CRITICAL(&s.mux);
                    s.st.overflows++;
                    portEXIT_CRITICAL(&s.mux);
                } else {
                    s.line[s.len] = '\0';
                    handle_line(s.line, s.len);
                }
                s.len = 0;
                s.overlong = false;
            } else if (s.len < CAM_LINE_MAX - 1) {
                s.line[s.len++] = c;
            } else {
                s.overlong = true;
            }
        }
    }
}

esp_err_t cam_link_start(void)
{
    const uart_config_t cfg = {
        .baud_rate  = CONFIG_CAM_FWD_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(CAM_UART, CAM_RX_BUF, 0, 0, NULL, 0);
    if (err == ESP_OK) {
        err = uart_param_config(CAM_UART, &cfg);
    }
    if (err == ESP_OK) {
        const int tx = CONFIG_CAM_FWD_UART_TX_GPIO >= 0 ? CONFIG_CAM_FWD_UART_TX_GPIO : UART_PIN_NO_CHANGE;
        err = uart_set_pin(CAM_UART, tx, CONFIG_CAM_FWD_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        /* An unplugged camera must read as an idle line, not as noise. */
        gpio_pullup_en((gpio_num_t)CONFIG_CAM_FWD_UART_RX_GPIO);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART1 on RX GPIO%d: %s", CONFIG_CAM_FWD_UART_RX_GPIO, esp_err_to_name(err));
        return err;
    }
    if (xTaskCreatePinnedToCore(reader_task, "cam_rx", CAM_TASK_STACK, NULL, CAM_TASK_PRIO, NULL,
                                CAM_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "reader task not created");
        return ESP_ERR_NO_MEM;
    }
    portENTER_CRITICAL(&s.mux);
    s.st.running = true;
    s.st.rx_gpio = CONFIG_CAM_FWD_UART_RX_GPIO;
    s.st.tx_gpio = CONFIG_CAM_FWD_UART_TX_GPIO;
    s.st.baud    = CONFIG_CAM_FWD_UART_BAUD;
    portEXIT_CRITICAL(&s.mux);
    ESP_LOGI(TAG, "forward camera on UART1: RX GPIO%d, TX GPIO%d, %d baud", CONFIG_CAM_FWD_UART_RX_GPIO,
             CONFIG_CAM_FWD_UART_TX_GPIO, CONFIG_CAM_FWD_UART_BAUD);
    return ESP_OK;
}

bool cam_link_latest(cam_result_t *out)
{
    portENTER_CRITICAL(&s.mux);
    const bool have = s.have;
    *out = s.latest;
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
    portEXIT_CRITICAL(&s.mux);
    if (out->last_row_us == 0u || (uint32_t)(now_us() - out->last_row_us) > 1500000u) {
        out->rows_last_s = 0;
    }
}

/* ------------------------------------------------------------- console */

static void print_status(void)
{
    cam_link_status_t st;
    cam_result_t r;
    cam_link_status(&st);
    const bool have = cam_link_latest(&r);
    if (!st.running) {
        printf("forward camera: UART1 not running (see the boot log)\n");
        return;
    }
    printf("UART1, RX GPIO%d", st.rx_gpio);
    if (st.tx_gpio >= 0) {
        printf(", TX GPIO%d", st.tx_gpio);
    } else {
        printf(", no TX");
    }
    printf(", %d baud: %" PRIu32 " rows/s", st.baud, st.rows_last_s);
    if (have) {
        printf(", last row %" PRIu32 " ms ago (CSV v%d)", (now_us() - r.t_us) / 1000u, st.csv_version);
    }
    printf("\nrows %" PRIu32 " (detected in %" PRIu32 "), '#' lines %" PRIu32 ", rejected %" PRIu32
           ", overlong %" PRIu32 ", bytes %" PRIu32 "\n",
           st.rows, st.rows_detected, st.info_lines, st.rejected, st.overflows, st.bytes);
    if (!have) {
        if (st.bytes == 0) {
            printf("nothing has arrived: check the camera is powered, CAM U0T (GPIO1) -> 470 ohm -> "
                   "S3 GPIO%d, and the grounds joined\n", st.rx_gpio);
        } else {
            printf("bytes arrive but no row parses: a wrong baud, or not target_cam's rows; "
                   "'cam --raw' shows what arrives\n");
        }
    } else if (r.detected) {
        printf("latest: DETECTED  az %+.1f deg  el %+.1f deg  cx %u  cy %u  area %u px\n",
               (double)r.az_deg, (double)r.el_deg, r.cx_px, r.cy_px, r.area_px);
        const float b = (float)cam_link_bearing_sign() * r.az_deg;
        printf("core/'s bearing: %+.1f deg, to the blimp's %s (sign %+d: 'cam --sign' to change)\n",
               (double)b, b > 0.0f ? "LEFT" : b < 0.0f ? "RIGHT" : "nose", cam_link_bearing_sign());
    } else {
        printf("latest: nothing detected\n");
    }
    if (st.streaming_off) {
        printf("the camera said '# streaming off': 'cam --tx s' turns its rows back on\n");
    }
}

static void print_recent(void)
{
    kept_line_t copy[CAM_RECENT];
    uint32_t n;
    portENTER_CRITICAL(&s.mux);
    memcpy(copy, s.recent, sizeof(copy));
    n = s.recent_n;
    portEXIT_CRITICAL(&s.mux);
    if (n == 0) {
        printf("no line received yet\n");
        return;
    }
    const uint32_t first = n > CAM_RECENT ? n - CAM_RECENT : 0;
    const uint32_t t = now_us();
    for (uint32_t i = first; i < n; i++) {
        const kept_line_t *k = &copy[i % CAM_RECENT];
        printf("%6" PRIu32 " ms ago  %s\n", (t - k->t_us) / 1000u, k->text);
    }
}

static int send_line(int argc, char **argv)
{
    if (CONFIG_CAM_FWD_UART_TX_GPIO < 0) {
        printf("no TX pin configured (CONFIG_CAM_FWD_UART_TX_GPIO = -1)\n");
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
    uart_write_bytes(CAM_UART, out, len);
    vTaskDelay(pdMS_TO_TICKS(CAM_TX_WAIT_MS));

    kept_line_t copy[CAM_INFO];
    uint32_t n;
    portENTER_CRITICAL(&s.mux);
    memcpy(copy, s.info, sizeof(copy));
    n = s.info_n;
    portEXIT_CRITICAL(&s.mux);
    int shown = 0;
    for (uint32_t i = n > CAM_INFO ? n - CAM_INFO : 0; i < n; i++) {
        if ((int32_t)(copy[i % CAM_INFO].t_us - t0) >= 0) {
            printf("%s\n", copy[i % CAM_INFO].text);
            shown++;
        }
    }
    printf("(sent %u bytes; %d '#' line%s back in %d ms)\n", (unsigned)len, shown, shown == 1 ? "" : "s",
           CAM_TX_WAIT_MS);
    return 0;
}

static int cam_usage(void)
{
    printf("cam                  the forward camera's link: rows/s, the latest detection, core/'s bearing\n"
           "cam --raw            the last %d lines exactly as they arrived\n"
           "cam --tx <text>      send one line to the camera ('?' lists its settings; 's' toggles its rows)\n"
           "cam --sign <-1|1>    core/'s bearing = sign x the camera's az, until the next boot\n",
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
        return send_line(argc, argv);
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
        .help = "The forward camera's link (target_cam over UART1): rows, the latest detection and the "
                "bearing core/ gets. 'cam --raw' shows the last lines, 'cam --tx ?' asks the camera.",
        .hint = "[--raw | --tx <text> | --sign <-1|1>]",
        .func = &do_cam_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

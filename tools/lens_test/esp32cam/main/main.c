/* tools/lens_test/esp32cam/main/main.c - lens-sorting server for the AI-Thinker ESP32-CAM.
 *
 * Bench tool, not flight code. Whatever module sits in the ESP32-CAM's 24-pin
 * connector is probed by the esp32-camera driver and served two ways:
 *
 * Over the USB serial cable (UART0, the one used for flashing), which is what
 * tools/lens_test/lens_test.py --port uses. Text commands in, one reply each:
 *
 *   ping                 -> PONG
 *   quiet                -> OK          (silences the ESP_LOG output for good)
 *   info                 -> JSON {...}  sensor identity: name, PID, VER, max size, JPEG?
 *   status               -> JSON {...}  current sensor settings
 *   set <var> <value>    -> OK | ERR <why>   any esp32-camera setter (see apply_setting)
 *   cap <size>           -> FRM <bytes> <w> <h>\n <jpeg bytes> \nEND\n
 *                           size is MAX, a name (QVGA, VGA, UXGA) or WxH
 *   reboot               -> OK, then restarts (re-probes the camera)
 *
 * (No "enter download mode" command: unlike the S2/S3/C3, the original ESP32 has
 * no RTC flag the ROM honours for that, so flashing always needs IO0 held while
 * RST is pressed when the adapter lacks DTR/RTS wiring.)
 *
 * Over Wi-Fi (soft AP "lens-test" by default, http://192.168.4.1/):
 *
 *   /                     browser page: live stream and a few controls
 *   /info /status         the same JSON as above
 *   /set?var=lenc&val=0   the same setters
 *   /capture?size=MAX     one still
 *   :81/stream?size=VGA   MJPEG live stream
 *
 * WHY SERIAL AS WELL: the laptop's only uplink is usually its Wi-Fi card, so
 * joining the board's access point cuts it off from everything else. The USB
 * cable is already there for flashing and carries ~90 kB/s at 921600 baud,
 * enough for a QVGA live view and full-size stills in a couple of seconds.
 *
 * WHY TWO HTTP SERVERS: esp_http_server runs handlers one at a time per server
 * and a stream handler never returns while a client watches. Putting the stream
 * on port 81 keeps /set and /capture answering during a live view.
 *
 * One mutex (s_cam_lock) serialises everything that touches the sensor or the
 * frame queue across the serial task and the two HTTP tasks.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "img_converters.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "lens_test";

/* AI-Thinker ESP32-CAM pin map, the only board this tool targets. */
#define CAM_PIN_PWDN   32
#define CAM_PIN_RESET  -1   /* not wired out on this board: the driver resets through PWDN */
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD   26
#define CAM_PIN_SIOC   27
#define CAM_PIN_D7     35
#define CAM_PIN_D6     34
#define CAM_PIN_D5     39
#define CAM_PIN_D4     36
#define CAM_PIN_D3     21
#define CAM_PIN_D2     19
#define CAM_PIN_D1     18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC  25
#define CAM_PIN_HREF   23
#define CAM_PIN_PCLK   22
#define FLASH_LED_GPIO  4   /* the white LED next to the connector; kept off */

#define STREAM_PORT     81
#define PART_BOUNDARY   "lenstestframe"
#define SERIAL_UART     UART_NUM_0
#define JSON_BUF_LEN    768

static SemaphoreHandle_t s_cam_lock;
static bool              s_cam_ok;
static esp_err_t         s_cam_err;
static framesize_t       s_max_fs;          /* largest size the frame buffers were sized for */
static int               s_stream_clients;  /* guarded by s_cam_lock */
static httpd_handle_t    s_ctrl_httpd;
static httpd_handle_t    s_stream_httpd;

/* ------------------------------------------------------------------------- */
/* Camera                                                                    */
/* ------------------------------------------------------------------------- */

static esp_err_t camera_start(void)
{
    camera_config_t cfg = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6, .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2, .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
        .pin_vsync    = CAM_PIN_VSYNC,
        .pin_href     = CAM_PIN_HREF,
        .pin_pclk     = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        /* Ask for more than any supported sensor delivers: the driver clamps to
         * the detected sensor's maximum and sizes the frame buffers for it, so
         * "cap MAX" always fits (UXGA on an OV2640, QSXGA on an OV5640). */
        .frame_size   = FRAMESIZE_QSXGA,
        .jpeg_quality = CONFIG_LENS_TEST_JPEG_QUALITY,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err == ESP_ERR_NOT_SUPPORTED) {
        /* Either nothing answered on the SCCB bus, or the sensor has no JPEG
         * encoder (OV7670/OV7725 class). One retry in RGB565 tells them apart;
         * the handlers then convert to JPEG in software, slowly but usably. */
        ESP_LOGW(TAG, "JPEG init failed; retrying in RGB565 for sensors without a JPEG encoder");
        cfg.pixel_format = PIXFORMAT_RGB565;
        cfg.frame_size   = FRAMESIZE_VGA;
        cfg.fb_count     = 1;
        err = esp_camera_init(&cfg);
    }
    if (err == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        s_max_fs = s->status.framesize;   /* what the driver settled on after clamping */
    }
    return err;
}

/* A frame as JPEG bytes, whatever the sensor produces. */
typedef struct {
    camera_fb_t *fb;
    uint8_t     *jpg;
    size_t       len;
    bool         owned;   /* jpg was malloc'd by frame2jpg and must be freed */
} frame_t;

static bool frame_acquire(frame_t *f)
{
    memset(f, 0, sizeof *f);
    f->fb = esp_camera_fb_get();
    if (!f->fb) {
        return false;
    }
    if (f->fb->format == PIXFORMAT_JPEG) {
        f->jpg = f->fb->buf;
        f->len = f->fb->len;
        return true;
    }
    f->owned = frame2jpg(f->fb, 80, &f->jpg, &f->len);
    if (!f->owned) {
        esp_camera_fb_return(f->fb);
        f->fb = NULL;
        return false;
    }
    return true;
}

static void frame_release(frame_t *f)
{
    if (f->owned && f->jpg) {
        free(f->jpg);
    }
    if (f->fb) {
        esp_camera_fb_return(f->fb);
    }
    memset(f, 0, sizeof *f);
}

/* "VGA", "UXGA", "MAX" or "640x480" -> framesize_t index, or -1. Matched through
 * the driver's own resolution table so the enum order of the installed
 * esp32-camera version never matters. */
static int framesize_from_name(const char *name)
{
    static const struct { const char *name; uint16_t w, h; } alias[] = {
        { "QQVGA", 160, 120 },   { "QCIF", 176, 144 },    { "HQVGA", 240, 176 },
        { "QVGA", 320, 240 },    { "CIF", 400, 296 },     { "HVGA", 480, 320 },
        { "VGA", 640, 480 },     { "SVGA", 800, 600 },    { "XGA", 1024, 768 },
        { "HD", 1280, 720 },     { "SXGA", 1280, 1024 },  { "UXGA", 1600, 1200 },
        { "FHD", 1920, 1080 },   { "QXGA", 2048, 1536 },  { "QHD", 2560, 1440 },
        { "QSXGA", 2560, 1920 },
    };
    unsigned w = 0, h = 0;

    if (!strcasecmp(name, "MAX")) {
        return (int)s_max_fs;
    }
    for (size_t i = 0; i < sizeof alias / sizeof alias[0]; i++) {
        if (!strcasecmp(name, alias[i].name)) {
            w = alias[i].w;
            h = alias[i].h;
            break;
        }
    }
    if (w == 0 && sscanf(name, "%ux%u", &w, &h) != 2) {
        return -1;
    }
    for (int i = 0; i < FRAMESIZE_INVALID; i++) {
        if (resolution[i].width == w && resolution[i].height == h) {
            return i;
        }
    }
    return -1;
}

/* Caller holds s_cam_lock. */
static esp_err_t set_size_locked(framesize_t fs)
{
    sensor_t *s = esp_camera_sensor_get();

    if (fs > s_max_fs) {
        fs = s_max_fs;   /* the frame buffers were sized for s_max_fs; larger would overflow */
    }
    if (s->status.framesize == fs) {
        return ESP_OK;
    }
    if (s->set_framesize(s, fs) != 0) {
        return ESP_FAIL;
    }
    /* The queue still holds frames at the old size and auto-exposure needs a
     * frame or two to settle: drop three before anyone measures anything. */
    for (int i = 0; i < 3; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* Shared answers: JSON builders and the setter table                        */
/* ------------------------------------------------------------------------- */

static int json_info(char *buf, size_t n)
{
    if (!s_cam_ok) {
        return snprintf(buf, n,
                        "{\"firmware\":\"lens_test\",\"camera_ok\":false,\"error\":\"%s\","
                        "\"hint\":\"no supported sensor answered: reseat the flex, gold contacts toward the board\","
                        "\"stream_port\":%d}",
                        esp_err_to_name(s_cam_err), STREAM_PORT);
    }
    sensor_t *s = esp_camera_sensor_get();
    camera_sensor_info_t *si = esp_camera_sensor_get_info(&s->id);
    framesize_t cur = s->status.framesize;

    return snprintf(buf, n,
                    "{\"firmware\":\"lens_test\",\"camera_ok\":true,"
                    "\"sensor\":\"%s\",\"pid\":\"0x%04x\",\"ver\":\"0x%02x\",\"midh\":\"0x%02x\",\"midl\":\"0x%02x\","
                    "\"sccb_addr\":\"0x%02x\",\"support_jpeg\":%s,\"pixformat\":\"%s\","
                    "\"max_width\":%u,\"max_height\":%u,\"width\":%u,\"height\":%u,"
                    "\"xclk_hz\":%d,\"psram_bytes\":%u,\"free_heap\":%u,\"uptime_s\":%lld,\"stream_port\":%d}",
                    si ? si->name : "unknown", (unsigned)s->id.PID, (unsigned)s->id.VER,
                    (unsigned)s->id.MIDH, (unsigned)s->id.MIDL, (unsigned)s->slv_addr,
                    (si && si->support_jpeg) ? "true" : "false",
                    s->pixformat == PIXFORMAT_JPEG ? "JPEG" : "RGB565",
                    (unsigned)resolution[s_max_fs].width, (unsigned)resolution[s_max_fs].height,
                    (unsigned)resolution[cur].width, (unsigned)resolution[cur].height,
                    s->xclk_freq_hz,
                    (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
                    (unsigned)esp_get_free_heap_size(),
                    (long long)(esp_timer_get_time() / 1000000), STREAM_PORT);
}

static int json_status(char *buf, size_t n)
{
    sensor_t *s = esp_camera_sensor_get();
    camera_status_t *st = &s->status;

    return snprintf(buf, n,
                    "{\"width\":%u,\"height\":%u,\"quality\":%u,\"brightness\":%d,\"contrast\":%d,"
                    "\"saturation\":%d,\"sharpness\":%d,\"special_effect\":%u,\"wb_mode\":%u,\"awb\":%u,"
                    "\"awb_gain\":%u,\"aec\":%u,\"aec2\":%u,\"ae_level\":%d,\"aec_value\":%u,\"agc\":%u,"
                    "\"agc_gain\":%u,\"gainceiling\":%u,\"bpc\":%u,\"wpc\":%u,\"raw_gma\":%u,\"lenc\":%u,"
                    "\"hmirror\":%u,\"vflip\":%u,\"dcw\":%u,\"colorbar\":%u,\"stream_clients\":%d}",
                    (unsigned)resolution[st->framesize].width, (unsigned)resolution[st->framesize].height,
                    st->quality, st->brightness, st->contrast, st->saturation, st->sharpness,
                    st->special_effect, st->wb_mode, st->awb, st->awb_gain, st->aec, st->aec2,
                    st->ae_level, st->aec_value, st->agc, st->agc_gain, st->gainceiling, st->bpc,
                    st->wpc, st->raw_gma, st->lenc, st->hmirror, st->vflip, st->dcw, st->colorbar,
                    s_stream_clients);
}

#define SETTER(field) ((s->field) ? s->field(s, v) : -1)

/* 0 ok, -1 the sensor rejected it, -2 unknown variable, -3 camera busy/missing.
 * Takes s_cam_lock itself. */
static int apply_setting(const char *var, const char *val)
{
    int res;

    if (!s_cam_ok) {
        return -3;
    }
    if (xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(10000)) != pdTRUE) {
        return -3;
    }
    sensor_t *s = esp_camera_sensor_get();
    int v = atoi(val);

    if (!strcmp(var, "framesize")) {
        int fs = framesize_from_name(val);
        res = (fs < 0) ? -1 : (set_size_locked((framesize_t)fs) == ESP_OK ? 0 : -1);
    } else if (!strcmp(var, "quality"))        { res = SETTER(set_quality);
    } else if (!strcmp(var, "brightness"))     { res = SETTER(set_brightness);
    } else if (!strcmp(var, "contrast"))       { res = SETTER(set_contrast);
    } else if (!strcmp(var, "saturation"))     { res = SETTER(set_saturation);
    } else if (!strcmp(var, "sharpness"))      { res = SETTER(set_sharpness);
    } else if (!strcmp(var, "special_effect")) { res = SETTER(set_special_effect);
    } else if (!strcmp(var, "wb_mode"))        { res = SETTER(set_wb_mode);
    } else if (!strcmp(var, "awb"))            { res = SETTER(set_whitebal);
    } else if (!strcmp(var, "awb_gain"))       { res = SETTER(set_awb_gain);
    } else if (!strcmp(var, "aec"))            { res = SETTER(set_exposure_ctrl);
    } else if (!strcmp(var, "aec2"))           { res = SETTER(set_aec2);
    } else if (!strcmp(var, "ae_level"))       { res = SETTER(set_ae_level);
    } else if (!strcmp(var, "aec_value"))      { res = SETTER(set_aec_value);
    } else if (!strcmp(var, "agc"))            { res = SETTER(set_gain_ctrl);
    } else if (!strcmp(var, "agc_gain"))       { res = SETTER(set_agc_gain);
    } else if (!strcmp(var, "gainceiling"))    { res = s->set_gainceiling ? s->set_gainceiling(s, (gainceiling_t)v) : -1;
    } else if (!strcmp(var, "bpc"))            { res = SETTER(set_bpc);
    } else if (!strcmp(var, "wpc"))            { res = SETTER(set_wpc);
    } else if (!strcmp(var, "raw_gma"))        { res = SETTER(set_raw_gma);
    } else if (!strcmp(var, "lenc"))           { res = SETTER(set_lenc);
    } else if (!strcmp(var, "hmirror"))        { res = SETTER(set_hmirror);
    } else if (!strcmp(var, "vflip"))          { res = SETTER(set_vflip);
    } else if (!strcmp(var, "dcw"))            { res = SETTER(set_dcw);
    } else if (!strcmp(var, "colorbar"))       { res = SETTER(set_colorbar);
    } else {
        res = -2;
    }
    xSemaphoreGive(s_cam_lock);
    return res;
}

/* ------------------------------------------------------------------------- */
/* Serial protocol on UART0                                                  */
/* ------------------------------------------------------------------------- */

static void serial_write(const void *data, size_t len)
{
    uart_write_bytes(SERIAL_UART, data, len);
}

static void serial_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) {
        serial_write(buf, (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1);
    }
}

static void serial_send_json(int (*build)(char *, size_t))
{
    char *buf = malloc(JSON_BUF_LEN);
    if (!buf) {
        serial_printf("ERR no memory\n");
        return;
    }
    int n = build(buf, JSON_BUF_LEN);
    serial_write("JSON ", 5);
    serial_write(buf, (size_t)n);
    serial_write("\n", 1);
    free(buf);
}

static void serial_capture(const char *size)
{
    int want = size ? framesize_from_name(size) : (int)s_max_fs;

    if (!s_cam_ok) {
        serial_printf("ERR no camera\n");
        return;
    }
    if (want < 0) {
        serial_printf("ERR unknown size\n");
        return;
    }
    if (xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(10000)) != pdTRUE) {
        serial_printf("ERR camera busy\n");
        return;
    }
    if (set_size_locked((framesize_t)want) != ESP_OK) {
        serial_printf("ERR could not set size\n");
    } else {
        frame_t f;
        if (!frame_acquire(&f)) {
            serial_printf("ERR no frame\n");
        } else {
            /* Header, payload, trailer: the PC reads exactly <bytes> after the
             * header, so nothing in the JPEG can be mistaken for a command reply. */
            serial_printf("FRM %u %u %u\n", (unsigned)f.len, (unsigned)f.fb->width, (unsigned)f.fb->height);
            serial_write(f.jpg, f.len);
            serial_write("\nEND\n", 5);
            frame_release(&f);
        }
    }
    xSemaphoreGive(s_cam_lock);
}

static void serial_handle(char *line)
{
    char *cmd = strtok(line, " \t");

    if (!cmd) {
        return;
    }
    if (!strcmp(cmd, "ping")) {
        serial_printf("PONG\n");
    } else if (!strcmp(cmd, "quiet")) {
        esp_log_level_set("*", ESP_LOG_NONE);
        serial_printf("OK\n");
    } else if (!strcmp(cmd, "info")) {
        serial_send_json(json_info);
    } else if (!strcmp(cmd, "status")) {
        if (!s_cam_ok) {
            serial_printf("ERR no camera\n");
        } else {
            serial_send_json(json_status);
        }
    } else if (!strcmp(cmd, "set")) {
        char *var = strtok(NULL, " \t");
        char *val = strtok(NULL, " \t");
        if (!var || !val) {
            serial_printf("ERR usage: set <var> <value>\n");
            return;
        }
        int r = apply_setting(var, val);
        if (r == 0) {
            serial_printf("OK\n");
        } else {
            serial_printf("ERR %s\n", r == -2 ? "unknown var" : r == -3 ? "camera busy or missing"
                                                              : "sensor rejected the value");
        }
    } else if (!strcmp(cmd, "cap")) {
        serial_capture(strtok(NULL, " \t"));
    } else if (!strcmp(cmd, "reboot")) {
        serial_printf("OK rebooting\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    } else {
        serial_printf("ERR unknown command\n");
    }
}

static void serial_task(void *arg)
{
    uart_config_t cfg = {
        .baud_rate  = CONFIG_LENS_TEST_SERIAL_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    char line[128];
    size_t len = 0;
    bool quiet = false;

    ESP_LOGI(TAG, "serial protocol on UART0 at %d baud from here on (lens_test.py --port)",
             CONFIG_LENS_TEST_SERIAL_BAUD);
    vTaskDelay(pdMS_TO_TICKS(50));   /* let that line leave at the old rate */
    ESP_ERROR_CHECK(uart_driver_install(SERIAL_UART, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SERIAL_UART, &cfg));
    /* Route printf/ESP_LOG through the same driver so log lines and frames
     * queue in order instead of interleaving in the FIFO. */
    uart_vfs_dev_use_driver(SERIAL_UART);

    for (;;) {
        uint8_t c;
        if (uart_read_bytes(SERIAL_UART, &c, 1, pdMS_TO_TICKS(100)) <= 0) {
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            line[len] = '\0';
            if (len) {
                if (!quiet) {
                    /* The first command means a program is listening: logs
                     * would only corrupt its parsing from now on. */
                    esp_log_level_set("*", ESP_LOG_NONE);
                    quiet = true;
                }
                serial_handle(line);
            }
            len = 0;
            continue;
        }
        if (len < sizeof line - 1) {
            line[len++] = (char)c;
        } else {
            len = 0;   /* line noise: start over */
        }
    }
}

/* ------------------------------------------------------------------------- */
/* HTTP                                                                      */
/* ------------------------------------------------------------------------- */

static esp_err_t send_text(httpd_req_t *req, const char *status, const char *body)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_json(httpd_req_t *req, int (*build)(char *, size_t))
{
    char buf[JSON_BUF_LEN];
    int n = build(buf, sizeof buf);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t query_value(httpd_req_t *req, const char *key, char *out, size_t out_len)
{
    char qry[160];
    size_t len = httpd_req_get_url_query_len(req);

    if (len == 0 || len >= sizeof qry) {
        return ESP_ERR_NOT_FOUND;
    }
    if (httpd_req_get_url_query_str(req, qry, sizeof qry) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    return httpd_query_key_value(qry, key, out, out_len);
}

static const char INDEX_HTML[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>lens test</title><style>body{font:14px system-ui;margin:12px;background:#15181c;color:#e8e4dc}"
    "img{max-width:100%;background:#000;display:block;margin-top:8px}button{margin:2px 6px 2px 0;padding:6px 10px}"
    "pre{white-space:pre-wrap;color:#9c9;font-size:12px}</style></head><body>"
    "<h3>ESP32-CAM lens test</h3><pre id=\"i\">loading /info ...</pre>"
    "<div><button onclick=\"go('lenc',1)\">LENC on</button><button onclick=\"go('lenc',0)\">LENC off</button>"
    "<button onclick=\"lock(1)\">AE lock</button><button onclick=\"lock(0)\">AE auto</button>"
    "<button onclick=\"go('colorbar',1)\">colour bars</button><button onclick=\"go('colorbar',0)\">live</button>"
    "<a href=\"/capture?size=MAX\" target=\"_blank\"><button>full-size still</button></a></div>"
    "<img id=\"s\" alt=\"stream\"><pre id=\"o\"></pre>"
    "<script>var h=location.hostname;document.getElementById('s').src='http://'+h+':81/stream?size=VGA';"
    "fetch('/info').then(function(r){return r.text()}).then(function(t){document.getElementById('i').textContent=t});"
    "function go(v,x){fetch('/set?var='+v+'&val='+x).then(function(r){return r.text()})"
    ".then(function(t){document.getElementById('o').textContent=v+'='+x+' -> '+t})}"
    "function lock(l){go('aec',l?0:1);go('agc',l?0:1)}</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t info_handler(httpd_req_t *req)
{
    return send_json(req, json_info);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    if (!s_cam_ok) {
        return send_text(req, "503 Service Unavailable", "no camera");
    }
    return send_json(req, json_status);
}

static esp_err_t set_handler(httpd_req_t *req)
{
    char var[24], val[24];

    if (query_value(req, "var", var, sizeof var) != ESP_OK ||
        query_value(req, "val", val, sizeof val) != ESP_OK) {
        return send_text(req, "400 Bad Request", "need ?var=<name>&val=<value>");
    }
    switch (apply_setting(var, val)) {
    case 0:  return send_text(req, "200 OK", "ok");
    case -2: return send_text(req, "400 Bad Request", "unknown var");
    case -3: return send_text(req, "503 Service Unavailable", "camera busy or missing");
    default: return send_text(req, "500 Internal Server Error", "sensor rejected the value");
    }
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    char val[24];
    char w_hdr[16], h_hdr[16];
    int want = (int)s_max_fs;
    bool started = false;

    if (!s_cam_ok) {
        return send_text(req, "503 Service Unavailable", "no camera");
    }
    if (query_value(req, "size", val, sizeof val) == ESP_OK) {
        want = framesize_from_name(val);
        if (want < 0) {
            return send_text(req, "400 Bad Request", "unknown size");
        }
    }
    if (xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(10000)) != pdTRUE) {
        return send_text(req, "503 Service Unavailable", "camera busy");
    }
    sensor_t *s = esp_camera_sensor_get();
    framesize_t prev = s->status.framesize;
    esp_err_t res = set_size_locked((framesize_t)want);

    if (res == ESP_OK) {
        frame_t f;
        if (!frame_acquire(&f)) {
            res = ESP_FAIL;
        } else {
            snprintf(w_hdr, sizeof w_hdr, "%u", (unsigned)f.fb->width);
            snprintf(h_hdr, sizeof h_hdr, "%u", (unsigned)f.fb->height);
            httpd_resp_set_type(req, "image/jpeg");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_set_hdr(req, "X-Width", w_hdr);
            httpd_resp_set_hdr(req, "X-Height", h_hdr);
            started = true;
            res = httpd_resp_send(req, (const char *)f.jpg, f.len);
            frame_release(&f);
        }
    }
    /* Put a running stream back at its own size, so the browser page and the
     * live view carry on after a full-size still. */
    if (s_stream_clients > 0 && s->status.framesize != prev) {
        set_size_locked(prev);
    }
    xSemaphoreGive(s_cam_lock);

    if (res != ESP_OK && !started) {
        return send_text(req, "500 Internal Server Error", "capture failed");
    }
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    char val[24];
    int want = -1;
    int misses = 0;
    esp_err_t res = ESP_OK;

    if (!s_cam_ok) {
        return send_text(req, "503 Service Unavailable", "no camera");
    }
    if (query_value(req, "size", val, sizeof val) == ESP_OK) {
        want = framesize_from_name(val);
    }
    if (xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(10000)) != pdTRUE) {
        return send_text(req, "503 Service Unavailable", "camera busy");
    }
    s_stream_clients++;
    if (want >= 0) {
        set_size_locked((framesize_t)want);
    }
    xSemaphoreGive(s_cam_lock);

    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" PART_BOUNDARY);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (res == ESP_OK) {
        frame_t f;
        char hdr[160];
        int n;

        if (xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(2000)) != pdTRUE) {
            continue;
        }
        if (!frame_acquire(&f)) {
            xSemaphoreGive(s_cam_lock);
            if (++misses > 20) {
                ESP_LOGE(TAG, "stream: no frames from the sensor");
                res = ESP_FAIL;
            } else {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            continue;
        }
        misses = 0;
        n = snprintf(hdr, sizeof hdr,
                     "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\n"
                     "Content-Length: %u\r\nX-Timestamp: %lld.%06ld\r\n\r\n",
                     (unsigned)f.len, (long long)f.fb->timestamp.tv_sec, (long)f.fb->timestamp.tv_usec);
        res = httpd_resp_send_chunk(req, hdr, n);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)f.jpg, f.len);
        }
        frame_release(&f);
        xSemaphoreGive(s_cam_lock);
    }

    xSemaphoreTake(s_cam_lock, portMAX_DELAY);
    s_stream_clients--;
    xSemaphoreGive(s_cam_lock);
    return res;   /* an error here only means the viewer went away */
}

static void start_servers(void)
{
    static const httpd_uri_t ctrl_uris[] = {
        { .uri = "/",        .method = HTTP_GET, .handler = index_handler },
        { .uri = "/info",    .method = HTTP_GET, .handler = info_handler },
        { .uri = "/status",  .method = HTTP_GET, .handler = status_handler },
        { .uri = "/set",     .method = HTTP_GET, .handler = set_handler },
        { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler },
    };
    static const httpd_uri_t stream_uri = {
        .uri = "/stream", .method = HTTP_GET, .handler = stream_handler
    };
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();

    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    ESP_ERROR_CHECK(httpd_start(&s_ctrl_httpd, &cfg));
    for (size_t i = 0; i < sizeof ctrl_uris / sizeof ctrl_uris[0]; i++) {
        httpd_register_uri_handler(s_ctrl_httpd, &ctrl_uris[i]);
    }

    cfg.server_port = STREAM_PORT;
    cfg.ctrl_port  += 1;   /* each server needs its own control socket */
    ESP_ERROR_CHECK(httpd_start(&s_stream_httpd, &cfg));
    httpd_register_uri_handler(s_stream_httpd, &stream_uri);
}

/* ------------------------------------------------------------------------- */
/* Wi-Fi                                                                     */
/* ------------------------------------------------------------------------- */

#if CONFIG_LENS_TEST_WIFI_STA

#define WIFI_GOT_IP  BIT0
#define WIFI_FAILED  BIT1
static EventGroupHandle_t s_wifi_events;
static int s_retries;

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retries++ < 5) {
            ESP_LOGW(TAG, "retrying \"%s\" (%d)", CONFIG_LENS_TEST_WIFI_SSID, s_retries);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_events, WIFI_FAILED);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "joined \"%s\": open http://" IPSTR "/", CONFIG_LENS_TEST_WIFI_SSID, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_GOT_IP);
    }
}

static void wifi_start(void)
{
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t cfg = { 0 };

    s_wifi_events = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event, NULL));

    strlcpy((char *)cfg.sta.ssid, CONFIG_LENS_TEST_WIFI_SSID, sizeof cfg.sta.ssid);
    strlcpy((char *)cfg.sta.password, CONFIG_LENS_TEST_WIFI_PASSWORD, sizeof cfg.sta.password);
    cfg.sta.threshold.authmode = strlen(CONFIG_LENS_TEST_WIFI_PASSWORD) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);   /* latency over battery life: this is a bench tool */

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_GOT_IP | WIFI_FAILED,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
    if (!(bits & WIFI_GOT_IP)) {
        ESP_LOGE(TAG, "could not join \"%s\"; check the password or rebuild in access-point mode",
                 CONFIG_LENS_TEST_WIFI_SSID);
    }
}

#else   /* access point */

static void wifi_start(void)
{
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t cfg = { 0 };
    esp_netif_ip_info_t ip;
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_wifi_init(&init));
    strlcpy((char *)cfg.ap.ssid, CONFIG_LENS_TEST_WIFI_SSID, sizeof cfg.ap.ssid);
    strlcpy((char *)cfg.ap.password, CONFIG_LENS_TEST_WIFI_PASSWORD, sizeof cfg.ap.password);
    cfg.ap.ssid_len = strlen(CONFIG_LENS_TEST_WIFI_SSID);
    cfg.ap.channel = 6;
    cfg.ap.max_connection = 2;
    cfg.ap.authmode = strlen(CONFIG_LENS_TEST_WIFI_PASSWORD) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    cfg.ap.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_netif_get_ip_info(ap, &ip);
    ESP_LOGI(TAG, "access point \"%s\" (password \"%s\"): open http://" IPSTR "/",
             CONFIG_LENS_TEST_WIFI_SSID, CONFIG_LENS_TEST_WIFI_PASSWORD, IP2STR(&ip.ip));
}

#endif

/* ------------------------------------------------------------------------- */

void app_main(void)
{
    /* The flash LED shares GPIO4 with the SD card lines and glows faintly when
     * the pin floats; drive it low so it does not light the lens test target. */
    gpio_reset_pin(FLASH_LED_GPIO);
    gpio_set_direction(FLASH_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FLASH_LED_GPIO, 0);

    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_cam_lock = xSemaphoreCreateMutex();
    s_cam_err = camera_start();
    s_cam_ok = (s_cam_err == ESP_OK);
    if (s_cam_ok) {
        sensor_t *s = esp_camera_sensor_get();
        camera_sensor_info_t *si = esp_camera_sensor_get_info(&s->id);
        ESP_LOGI(TAG, "sensor %s PID 0x%04x VER 0x%02x MIDH 0x%02x MIDL 0x%02x, max %ux%u, %s",
                 si ? si->name : "unknown", (unsigned)s->id.PID, (unsigned)s->id.VER,
                 (unsigned)s->id.MIDH, (unsigned)s->id.MIDL,
                 (unsigned)resolution[s_max_fs].width, (unsigned)resolution[s_max_fs].height,
                 s->pixformat == PIXFORMAT_JPEG ? "hardware JPEG" : "RGB565 + software JPEG");
    } else {
        ESP_LOGE(TAG, "no supported camera (%s). Power off, reseat the flex with the gold "
                 "contacts toward the board and the latch closed, then reset.",
                 esp_err_to_name(s_cam_err));
    }

    wifi_start();
    start_servers();
    ESP_LOGI(TAG, "control server on :80, stream on :%d", STREAM_PORT);

    xTaskCreate(serial_task, "lens_serial", 8192, NULL, 5, NULL);
}

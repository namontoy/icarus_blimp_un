/*
 * Forward-looking colour-target detection on an ESP32-CAM.
 *
 * Grabs paced RGB565 frames, finds the largest blob of one configured colour,
 * and streams its bearing, size and (optionally) range as CSV on UART0. Written
 * for the blimp's nose camera: point it at a balloon, ball or painted tag and
 * the flight board gets "target is 6.2 deg right, 1.4 deg up, ~3.1 m away".
 *
 * Built on the same rules as optflow_cam, and sharing its hard-won board
 * handling (boot report before the camera powers up, the GPIO0 reflash window,
 * flash LED driven low, radios never started):
 *
 *  1. HEAT. QQVGA, paced frames, no WiFi. The idle time is the point.
 *
 *  2. HONEST FAILURE. ok=0 means NO TARGET, never "target dead ahead". A blob
 *     touching the frame edge is flagged clipped and gets no range, because
 *     its apparent size is an underestimate. The runner-up blob's size is
 *     printed so an ambiguous scene - two red things - is visible downstream.
 *
 *  3. COLOUR IS NOT FREE. Hue survives shadow far better than RGB does, but
 *     auto white balance actively works against a detector when the target
 *     fills much of the frame: it shifts the whole image to make that colour
 *     greyer. AWB is left on by default and can be frozen with 'w0' once the
 *     scene is lit; which is better on this airframe is a measurement, not a
 *     calculation.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"

#include "camera_pins.h"
#include "blob_detect.h"

static const char *TAG = "target";

#define FRAME_SIZE      FRAMESIZE_QQVGA     /* 160x120 */
#define FRAME_W         160
#define FRAME_H         120

/* Seconds of reflash window before GPIO0 becomes XCLK. See
   escape_hatch_delay(). */
#define TARGETCAM_ESCAPE_SECONDS  5

/* Side of the centre patch the 'c' command calibrates from. 16 px is ~5.4 deg
   at QQVGA: a 300 mm balloon fills it out to about 3 m. */
#define CAL_PATCH       16

/* Kconfig bools are undefined when off; turn them into 0/1 for the sensor. */
#ifdef CONFIG_TARGETCAM_VFLIP
#define CAM_VFLIP       1
#else
#define CAM_VFLIP       0
#endif
#ifdef CONFIG_TARGETCAM_HMIRROR
#define CAM_HMIRROR     1
#else
#define CAM_HMIRROR     0
#endif

/* Runtime-adjustable state. Single-threaded, so no locking. */
static int   s_target_fps = CONFIG_TARGETCAM_TARGET_FPS;
static float s_hfov_deg   = (float)CONFIG_TARGETCAM_HFOV_DEG;
static float s_target_mm  = (float)CONFIG_TARGETCAM_TARGET_MM;
static bool  s_streaming  = true;
static bool  s_dump_mask;           /* set by 'm', served on the next frame */
static bool  s_calibrate;           /* set by 'c', served on the next frame */
static bool  s_awb        = true;

static void *s_work;                /* blob_detect workspace, allocated once */

/* The colour preset in use, or -1 once thresholds are hand-tuned (H/T/S/V or
   'c'), so the banner never claims "red" for settings that are not red's. */
#if defined(CONFIG_TARGETCAM_PRESET_BLUE)
static int s_preset = BLOB_PRESET_BLUE;
#elif defined(CONFIG_TARGETCAM_PRESET_GREEN)
static int s_preset = BLOB_PRESET_GREEN;
#elif defined(CONFIG_TARGETCAM_PRESET_ORANGE)
static int s_preset = BLOB_PRESET_ORANGE;
#else
static int s_preset = BLOB_PRESET_RED;
#endif

static blob_cfg_t s_cfg = {
    .width      = FRAME_W,
    .height     = FRAME_H,
    .cell       = 2,
    /* esp32-camera hands RGB565 over high byte first. If a first power-up
       detects nothing at all, even after 'c', try 'o' to flip this. */
    .big_endian = 1,
    /* hue, hue_tol, sat_min, val_min come from the preset in app_main() */
    .min_area   = CONFIG_TARGETCAM_MIN_AREA,
};

/* ------------------------------------------------------------------ */

/* Focal length in pixels, from the pinhole model. Bearing and range both go
   through this, so an uncalibrated FOV scales every angle and every range. */
static float focal_px(void)
{
    float hfov_rad = s_hfov_deg * (float)M_PI / 180.0f;
    return ((float)FRAME_W * 0.5f) / tanf(hfov_rad * 0.5f);
}

static void quiesce_heat_sources(void)
{
#if CAM_PIN_FLASH_LED >= 0
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << CAM_PIN_FLASH_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led));
    ESP_ERROR_CHECK(gpio_set_level(CAM_PIN_FLASH_LED, 0));
    ESP_LOGI(TAG, "flash LED on GPIO%d driven low", CAM_PIN_FLASH_LED);
#endif
    /* Nothing initialises WiFi or Bluetooth anywhere in this firmware. */
}

static esp_err_t camera_init_at(camera_fb_location_t where)
{
    camera_config_t config = {
        .pin_pwdn       = CAM_PIN_PWDN,
        .pin_reset      = CAM_PIN_RESET,
        .pin_xclk       = CAM_PIN_XCLK,
        .pin_sccb_sda   = CAM_PIN_SIOD,
        .pin_sccb_scl   = CAM_PIN_SIOC,
        .pin_d7         = CAM_PIN_D7,
        .pin_d6         = CAM_PIN_D6,
        .pin_d5         = CAM_PIN_D5,
        .pin_d4         = CAM_PIN_D4,
        .pin_d3         = CAM_PIN_D3,
        .pin_d2         = CAM_PIN_D2,
        .pin_d1         = CAM_PIN_D1,
        .pin_d0         = CAM_PIN_D0,
        .pin_vsync      = CAM_PIN_VSYNC,
        .pin_href       = CAM_PIN_HREF,
        .pin_pclk       = CAM_PIN_PCLK,

        .xclk_freq_hz   = CONFIG_TARGETCAM_XCLK_MHZ * 1000 * 1000,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,

        /* RGB565 straight from the sensor: two bytes per pixel, no JPEG
           encode or decode anywhere. Colour is the whole signal here, so
           unlike optflow_cam we cannot drop to grayscale. */
        .pixel_format   = PIXFORMAT_RGB565,
        .frame_size     = FRAME_SIZE,

        /* 2 x 38400 B. Internal DRAM first: it keeps the SPI PSRAM unclocked
           for pixel traffic (less heat) and runs on a board without PSRAM. */
        .fb_location    = where,
        .fb_count       = 2,
        .grab_mode      = CAMERA_GRAB_LATEST,

        .jpeg_quality   = 12,   /* unused in RGB565, must still be sane */
    };
    return esp_camera_init(&config);
}

static esp_err_t camera_start(void)
{
    esp_err_t err = camera_init_at(CAMERA_FB_IN_DRAM);
    if (err != ESP_OK && heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
        ESP_LOGW(TAG, "init with buffers in DRAM failed (%s); retrying in PSRAM",
                 esp_err_to_name(err));
        esp_camera_deinit();
        err = camera_init_at(CAMERA_FB_IN_PSRAM);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "check the ribbon seating, and that the board is fed 5 V "
                      "- a camera that browns out looks exactly like this");
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "no sensor handle after init");
        return ESP_FAIL;
    }

    camera_sensor_info_t *info = esp_camera_sensor_get_info(&s->id);
    ESP_LOGI(TAG, "sensor PID 0x%02x (%s)", s->id.PID,
             info ? info->name : "unknown");

    /* Colour pipeline ON: white balance and its gains, a little extra
       saturation to pull markers away from walls, lens correction so the
       frame corners are not darker (and so less saturated) than the centre. */
    if (s->set_whitebal)    { s->set_whitebal(s, 1); }
    if (s->set_awb_gain)    { s->set_awb_gain(s, 1); }
    if (s->set_saturation)  { s->set_saturation(s, 1); }
    if (s->set_lenc)        { s->set_lenc(s, 1); }
    if (s->set_bpc)         { s->set_bpc(s, 1); }
    if (s->set_wpc)         { s->set_wpc(s, 1); }
    if (s->set_raw_gma)     { s->set_raw_gma(s, 1); }
    if (s->set_dcw)         { s->set_dcw(s, 1); }
    if (s->set_vflip)       { s->set_vflip(s, CAM_VFLIP); }
    if (s->set_hmirror)     { s->set_hmirror(s, CAM_HMIRROR); }
    /* Same gain cap as optflow: high gain buys brightness with chroma noise,
       and chroma noise is exactly what scatters hue. */
    if (s->set_gain_ctrl)   { s->set_gain_ctrl(s, 1); }
    if (s->set_agc_gain)    { s->set_agc_gain(s, 0); }
    if (s->set_gainceiling) { s->set_gainceiling(s, GAINCEILING_4X); }
    if (s->set_exposure_ctrl) { s->set_exposure_ctrl(s, 1); }
    if (s->set_aec2)        { s->set_aec2(s, 0); }

    return ESP_OK;
}

static void set_awb(bool on)
{
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->set_whitebal) { s->set_whitebal(s, on ? 1 : 0); }
    s_awb = on;
}

/* ------------------------------------------------------------------ */

static void print_banner(void)
{
    float f = focal_px();
    printf("\n# ESP32-CAM forward colour target\n");
    printf("# frame        %dx%d RGB565 (%s-endian), XCLK %d MHz, paced to %d FPS\n",
           FRAME_W, FRAME_H, s_cfg.big_endian ? "big" : "little",
           CONFIG_TARGETCAM_XCLK_MHZ, s_target_fps);
    printf("# colour       %s: hue %d +/-%d deg, sat >= %d, val >= %d, AWB %s\n",
           s_preset >= 0 ? blob_preset_name(s_preset) : "custom",
           s_cfg.hue, s_cfg.hue_tol, s_cfg.sat_min, s_cfg.val_min,
           s_awb ? "auto" : "frozen");
    printf("# blobs        %dx%d cells, min area %d px, up to %d reported\n",
           s_cfg.cell, s_cfg.cell, s_cfg.min_area, BLOB_MAX_REPORT);
    printf("# geometry     h-FOV %.1f deg -> f %.1f px, %.3f deg/px at centre\n",
           s_hfov_deg, f, atanf(1.0f / f) * 180.0f / (float)M_PI);
    if (s_target_mm > 0.0f) {
        printf("# range        target %.0f mm; lost below min area beyond ~%.1f m\n",
               s_target_mm,
               s_target_mm * f / (2.0f * sqrtf((float)s_cfg.min_area / (float)M_PI))
               / 1000.0f);
    } else {
        printf("# range        off (set the target size with d<mm>)\n");
    }
    printf("# commands     p<red|blue|green|orange> preset | c calibrate on centre | "
           "H<deg> hue | T<deg> tol | S<0-255> sat | V<0-255> val\n");
    printf("#              a<px> area | d<mm> size |");
    printf(" f<fps> rate | w<0|1> AWB | o byte order | m mask | "
           "s stream | ? info\n");
    printf("# columns      t_ms,ok,cx_px,cy_px,az_deg,el_deg,area_px,w_px,h_px,"
           "fill,range_mm,clipped,blobs,matched_px,b2_az,b2_el,b2_range_mm,"
           "b3_az,b3_el,b3_range_mm,b4_az,b4_el,b4_range_mm,dt_ms,proc_us\n");
    fflush(stdout);
}

/* Parse an integer argument and apply it if it is in range. */
static bool arg_in(const char *s, int lo, int hi, int *out)
{
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s || v < lo || v > hi) {
        printf("# out of range (%d..%d)\n", lo, hi);
        return false;
    }
    *out = (int)v;
    return true;
}

/* Commands are case-sensitive: 'S' sets saturation, 's' toggles the stream.
   Read without blocking so pacing is never disturbed. */
static void poll_commands(void)
{
    static char line[32];
    static int  len;
    uint8_t ch;

    while (uart_read_bytes(UART_NUM_0, &ch, 1, 0) == 1) {
        if (ch != '\r' && ch != '\n') {
            if (len < (int)sizeof(line) - 1) {
                line[len++] = (char)ch;
            }
            continue;
        }
        line[len] = '\0';
        if (len == 0) {
            continue;
        }
        len = 0;
        int v;
        switch (line[0]) {
        case 'c':
            s_calibrate = true;
            break;
        case 'p': {
            /* 'p red', 'pred', 'p0' and 'pr' all select red. */
            const char *a = line + 1;
            while (*a == ' ') a++;
            int p = (*a >= '0' && *a <= '9') ? atoi(a) : blob_preset_find(a);
            if (blob_preset_apply(&s_cfg, p)) {
                s_preset = p;
                printf("# preset %s: hue %d +/-%d, sat >= %d, val >= %d\n",
                       blob_preset_name(p), s_cfg.hue, s_cfg.hue_tol,
                       s_cfg.sat_min, s_cfg.val_min);
            } else {
                printf("# unknown preset (red, blue, green, orange)\n");
            }
            break;
        }
        case 'H':
            if (arg_in(line + 1, 0, 359, &v)) { s_cfg.hue = v; s_preset = -1; printf("# hue %d\n", v); }
            break;
        case 'T':
            if (arg_in(line + 1, 1, 90, &v)) { s_cfg.hue_tol = v; s_preset = -1; printf("# hue tol %d\n", v); }
            break;
        case 'S':
            if (arg_in(line + 1, 0, 255, &v)) { s_cfg.sat_min = v; s_preset = -1; printf("# sat min %d\n", v); }
            break;
        case 'V':
            if (arg_in(line + 1, 0, 255, &v)) { s_cfg.val_min = v; s_preset = -1; printf("# val min %d\n", v); }
            break;
        case 'a':
            if (arg_in(line + 1, 1, FRAME_W * FRAME_H, &v)) {
                s_cfg.min_area = v;
                printf("# min area %d px\n", v);
            }
            break;
        case 'd':
            if (arg_in(line + 1, 0, 5000, &v)) {
                s_target_mm = (float)v;
                printf("# target size %d mm%s\n", v, v ? "" : " (range off)");
            }
            break;
        case 'f':
            if (arg_in(line + 1, 1, 30, &v)) { s_target_fps = v; printf("# target %d FPS\n", v); }
            break;
        case 'w':
            if (arg_in(line + 1, 0, 1, &v)) {
                set_awb(v != 0);
                printf("# AWB %s\n", s_awb ? "auto" : "frozen at current gains");
            }
            break;
        case 'o':
            s_cfg.big_endian = !s_cfg.big_endian;
            printf("# RGB565 byte order now %s-endian\n",
                   s_cfg.big_endian ? "big" : "little");
            break;
        case 'm':
            s_dump_mask = true;
            break;
        case 's':
            s_streaming = !s_streaming;
            printf("# streaming %s\n", s_streaming ? "on" : "off");
            break;
        case '?':
            print_banner();
            break;
        default:
            printf("# unknown command '%c'\n", line[0]);
            break;
        }
        fflush(stdout);
    }
}

/*
 * 'c': take the colour of whatever is in the centre patch and make it the
 * target. Thresholds are set relative to what was seen, not to fixed numbers,
 * so a pastel target and a fluorescent one both calibrate sensibly - and the
 * warnings say when the target is a poor choice rather than failing later.
 */
static void calibrate(const uint8_t *frame)
{
    blob_color_t c;
    blob_sample(&s_cfg, frame, (FRAME_W - CAL_PATCH) / 2, (FRAME_H - CAL_PATCH) / 2,
                CAL_PATCH, CAL_PATCH, &c);

    int tol = 3 * c.hue_spread;
    if (tol < 10) tol = 10;
    if (tol > 40) tol = 40;
    int sat = c.sat / 2;
    if (sat < 40) sat = 40;
    int val = c.val * 2 / 5;
    if (val < 25) val = 25;

    printf("# calibrate    centre %dx%d: hue %d +/-%d, sat %d, val %d\n",
           CAL_PATCH, CAL_PATCH, c.hue, c.hue_spread, c.sat, c.val);
    if (c.sat < 70) {
        printf("# calibrate    REFUSED: saturation %d is too low to separate from "
               "walls. Fill the centre with the target, or use a more vivid one.\n",
               c.sat);
        return;
    }
    if (c.hue_spread > 20) {
        printf("# calibrate    WARNING: hue spread %d deg - the patch is not one "
               "colour. Is the target filling the centre?\n", c.hue_spread);
    }
    s_preset = -1;
    s_cfg.hue = c.hue;
    s_cfg.hue_tol = tol;
    s_cfg.sat_min = sat;
    s_cfg.val_min = val;
    printf("# calibrate    set H%d T%d S%d V%d\n", s_cfg.hue, s_cfg.hue_tol,
           s_cfg.sat_min, s_cfg.val_min);
}

/* 'm': the mask exactly as the detector sees it, 4x4 pixels per character.
   '#' at least half the pixels match, '+' some do, '.' none; 'X' marks the
   reported centroid. The only way to see what is going on without WiFi. */
static void dump_mask(const uint8_t *frame, const blob_result_t *r)
{
    int xc = r->found ? (int)r->b[0].cx / 4 : -1;
    int yc = r->found ? (int)r->b[0].cy / 4 : -1;
    printf("# mask (4x4 px per char)\n");
    for (int by = 0; by < FRAME_H / 4; by++) {
        char row[FRAME_W / 4 + 1];
        for (int bx = 0; bx < FRAME_W / 4; bx++) {
            int k = 0;
            for (int y = by * 4; y < by * 4 + 4; y++)
                for (int x = bx * 4; x < bx * 4 + 4; x++)
                    k += blob_pixel_matches(&s_cfg, blob_pixel(&s_cfg, frame, x, y));
            row[bx] = (bx == xc && by == yc) ? 'X' : k >= 8 ? '#' : k ? '+' : '.';
        }
        row[FRAME_W / 4] = '\0';
        printf("# %s\n", row);
    }
    fflush(stdout);
}

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "PANIC (crash)";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "other watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply collapsed)";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    case ESP_RST_EXT:      return "external pin";
    default:               return "unknown";
    }
}

/* Printed before anything draws meaningful current, and before GPIO0 becomes
   an output - see optflow_cam for the brownout that taught us this. */
static void boot_report(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    esp_reset_reason_t rr = esp_reset_reason();

    printf("\n\n# ---- target_cam boot ----\n");
    printf("# reset reason  %s\n", reset_reason_str(rr));
    printf("# chip          ESP32 rev %d, %d core(s)\n",
           chip.revision, chip.cores);
    printf("# heap          %u B internal free\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    printf("# psram         %u B %s\n", (unsigned)psram,
           psram ? "detected" : "absent (not required; buffers live in DRAM)");

    if (rr == ESP_RST_BROWNOUT) {
        printf("#\n");
        printf("# *** The last reset was a BROWNOUT. The supply could not hold\n");
        printf("#     up. An ESP32-CAM needs 5 V able to source ~500 mA; a\n");
        printf("#     USB-serial adapter's 3.3 V pin cannot do it, and the\n");
        printf("#     camera's power-up is what pushes it over. Fix the supply\n");
        printf("#     before reading anything below as a sensor result. ***\n");
    }
    fflush(stdout);
}

/* On an AI-Thinker ESP32-CAM, XCLK is GPIO0 - the pin esptool pulls low for
   download mode. Once the camera clock starts, the board can no longer be put
   into download mode from DTR. This window keeps it reflashable. Do not
   remove it. */
static void escape_hatch_delay(void)
{
    printf("#\n# GPIO0 is XCLK on this board: once the camera starts, esptool\n");
    printf("# can no longer force download mode. Reflash window open:\n");
    for (int i = TARGETCAM_ESCAPE_SECONDS; i > 0; i--) {
        printf("#   %d...\n", i);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("# window closed, starting the camera\n");
    fflush(stdout);
}

/* Bearing of a blob from the pinhole model, about the optical axis: +az right
   of the nose, +el above it (image +y is down, hence the sign on el). Range
   from the box's larger side, withheld (-1) when off or clipped.
 *
 * Why the box and not the area: on a shaded mylar balloon the white specular
 * highlight and the dark far side both fail the colour test, so the area
 * undercounts the balloon and the range read 10-13% long on the host suite.
 * The box spans the highlight hole and read 5-11% long. Either way the residual
 * is systematic, so calibrate s_target_mm against a balloon at a measured
 * distance rather than trusting 550 mm. A clipped target reads small, so its
 * range would read long; that is why it gets none. */
static void blob_geom(const blob_t *b, float *az, float *el, float *range_mm)
{
    float f = focal_px();
    *az = atanf((b->cx - FRAME_W * 0.5f) / f) * 180.0f / (float)M_PI;
    *el = atanf((FRAME_H * 0.5f - b->cy) / f) * 180.0f / (float)M_PI;
    int w = b->x1 - b->x0 + 1, h = b->y1 - b->y0 + 1;
    int d_px = w > h ? w : h;
    *range_mm = (s_target_mm > 0.0f && !b->clipped && d_px > 0)
              ? s_target_mm * f / (float)d_px : -1.0f;
}

void app_main(void)
{
    boot_report();
    quiesce_heat_sources();
    blob_preset_apply(&s_cfg, s_preset);

    if (!blob_cfg_valid(&s_cfg)) {
        ESP_LOGE(TAG, "detector configuration is invalid for a %dx%d frame",
                 FRAME_W, FRAME_H);
        return;
    }

    size_t work_bytes = blob_workspace_bytes(&s_cfg);
    s_work = heap_caps_malloc(work_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_work == NULL) {
        ESP_LOGE(TAG, "could not allocate the %u-byte detector workspace",
                 (unsigned)work_bytes);
        return;
    }

    /* Take over UART0 so reads can be non-blocking. stdout keeps working. */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    escape_hatch_delay();

    /* Powering up the OV2640 is the largest current step this firmware takes.
       If the output stops here, it is the supply, not the code. */
    printf("# esp_camera_init: powering up the sensor now\n");
    fflush(stdout);
    if (camera_start() != ESP_OK) {
        printf("# camera init FAILED - see the error above\n");
        fflush(stdout);
        return;
    }
    printf("# esp_camera_init: ok\n");
    fflush(stdout);

    /* Let exposure and white balance settle before the first detection; the
       first frames after power-up have the wrong colour cast. */
    ESP_LOGI(TAG, "waiting for exposure and white balance to settle");
    for (int i = 0; i < 20; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    print_banner();

    int64_t prev_capture_us = 0;
    TickType_t next_wake = xTaskGetTickCount();

    while (1) {
        /* Pace first, work second. The idle time is the thermal budget. */
        TickType_t period_ticks = pdMS_TO_TICKS(1000 / s_target_fps);
        if (period_ticks == 0) {
            period_ticks = 1;
        }
        xTaskDelayUntil(&next_wake, period_ticks);

        poll_commands();

        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "frame grab failed");
            continue;
        }
        int64_t capture_us = esp_timer_get_time();

        if (fb->format != PIXFORMAT_RGB565 ||
            fb->width != FRAME_W || fb->height != FRAME_H) {
            ESP_LOGE(TAG, "unexpected frame: %dx%d format %d",
                     fb->width, fb->height, fb->format);
            esp_camera_fb_return(fb);
            continue;
        }

        if (s_calibrate) {
            s_calibrate = false;
            calibrate(fb->buf);
        }

        int64_t t0 = esp_timer_get_time();
        blob_result_t r;
        int ok = blob_detect(&s_cfg, fb->buf, s_work, &r);
        int64_t proc_us = esp_timer_get_time() - t0;

        if (s_dump_mask) {
            s_dump_mask = false;
            dump_mask(fb->buf, &r);
        }
        esp_camera_fb_return(fb);

        float dt_ms = prev_capture_us ? (float)(capture_us - prev_capture_us) / 1000.0f
                                      : 0.0f;
        prev_capture_us = capture_us;

        float az = 0.0f, el = 0.0f, range_mm = -1.0f, fill = 0.0f;
        int w = 0, h = 0;
        if (ok) {
            blob_geom(&r.b[0], &az, &el, &range_mm);
            w = r.b[0].x1 - r.b[0].x0 + 1;
            h = r.b[0].y1 - r.b[0].y0 + 1;
            fill = (float)r.b[0].area / (float)(w * h);
        }

        if (s_streaming) {
            printf("%lld,%d,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%.2f,%.0f,%d,%d,%d",
                   capture_us / 1000, ok, r.b[0].cx, r.b[0].cy, az, el,
                   r.b[0].area, w, h, fill, range_mm, r.b[0].clipped,
                   r.n_blobs, r.matched);
            /* The other balloons in view, largest (usually nearest) first.
               Absent ones print as 0,0,-1. */
            for (int k = 1; k < BLOB_MAX_REPORT; k++) {
                float a2 = 0.0f, e2 = 0.0f, r2 = -1.0f;
                if (k < r.n_report) {
                    blob_geom(&r.b[k], &a2, &e2, &r2);
                }
                printf(",%.2f,%.2f,%.0f", a2, e2, r2);
            }
            printf(",%.1f,%lld\n", dt_ms, proc_us);
            fflush(stdout);
        }
    }
}

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
#include "link.h"
#include "settings.h"

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
static bool  s_image;               /* 'v1': send every frame to the PC viewer */
static bool  s_grab;                /* 'g': send the next frame only */
static int   s_pending_baud;        /* 'B': switched once its ACK is out */

static void *s_work;                /* blob_detect workspace, allocated once */

#if defined(CONFIG_TARGETCAM_PRESET_BLUE)
#define DEFAULT_PRESET  BLOB_PRESET_BLUE
#elif defined(CONFIG_TARGETCAM_PRESET_GREEN)
#define DEFAULT_PRESET  BLOB_PRESET_GREEN
#elif defined(CONFIG_TARGETCAM_PRESET_ORANGE)
#define DEFAULT_PRESET  BLOB_PRESET_ORANGE
#else
#define DEFAULT_PRESET  BLOB_PRESET_RED
#endif

/* The colour preset in use, or -1 once thresholds are hand-tuned (H/T/S/V or
   'c'), so the banner never claims "red" for settings that are not red's. */
static int s_preset = DEFAULT_PRESET;

/* What the next boot would run: the record in flash, or the Kconfig defaults
   when nothing is saved. #!CFG compares the live settings against it, so the
   viewer can say "modified, not saved" - a calibration that dies with the next
   power cycle should never look finished. */
static tc_settings_t s_ref;
static bool          s_ref_saved;
static bool          s_nvs_ok;

/* Why the last command was refused, for its #!NAK line. */
static char s_why[64];

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
/* Live settings <-> the record kept in flash                          */

static void live_defaults(void)
{
    s_target_fps     = CONFIG_TARGETCAM_TARGET_FPS;
    s_hfov_deg       = (float)CONFIG_TARGETCAM_HFOV_DEG;
    s_target_mm      = (float)CONFIG_TARGETCAM_TARGET_MM;
    s_cfg.min_area   = CONFIG_TARGETCAM_MIN_AREA;
    s_cfg.big_endian = 1;
    s_preset         = DEFAULT_PRESET;
    blob_preset_apply(&s_cfg, s_preset);
    /* Before the camera starts there is no sensor yet; set_awb() then only
       records the wish, and app_main() applies it once the sensor is up. */
    set_awb(true);
}

/* Zeroed first, so the struct's padding is zero too and two captures of the
   same settings compare equal with memcmp(). */
static tc_settings_t live_capture(void)
{
    tc_settings_t t;
    memset(&t, 0, sizeof t);
    t.hue        = (int16_t)s_cfg.hue;
    t.hue_tol    = (int16_t)s_cfg.hue_tol;
    t.sat_min    = (int16_t)s_cfg.sat_min;
    t.val_min    = (int16_t)s_cfg.val_min;
    t.min_area   = s_cfg.min_area;
    t.target_mm  = (int32_t)s_target_mm;
    t.hfov_deg   = s_hfov_deg;
    t.fps        = (int16_t)s_target_fps;
    t.awb        = s_awb ? 1 : 0;
    t.big_endian = (int8_t)s_cfg.big_endian;
    t.preset     = (int8_t)s_preset;
    return t;
}

/* Apply a stored record, or change nothing and return false if any field is
   outside the range its command accepts: a record from a buggy build must not
   become a detector that silently matches nothing. */
static bool live_apply(const tc_settings_t *t)
{
    if (t->hue < 0 || t->hue > 359 || t->hue_tol < 1 || t->hue_tol > 90 ||
        t->sat_min < 0 || t->sat_min > 255 || t->val_min < 0 || t->val_min > 255 ||
        t->min_area < 1 || t->min_area > FRAME_W * FRAME_H ||
        t->target_mm < 0 || t->target_mm > 5000 ||
        !(t->hfov_deg >= 10.0f && t->hfov_deg <= 150.0f) ||
        t->fps < 1 || t->fps > 30 ||
        (t->awb != 0 && t->awb != 1) || (t->big_endian != 0 && t->big_endian != 1) ||
        t->preset < -1 || t->preset >= BLOB_PRESET_COUNT) {
        return false;
    }
    s_cfg.hue        = t->hue;
    s_cfg.hue_tol    = t->hue_tol;
    s_cfg.sat_min    = t->sat_min;
    s_cfg.val_min    = t->val_min;
    s_cfg.min_area   = t->min_area;
    s_cfg.big_endian = t->big_endian;
    s_target_mm      = (float)t->target_mm;
    s_hfov_deg       = t->hfov_deg;
    s_target_fps     = t->fps;
    s_preset         = t->preset;
    /* Keep the preset's name only if its numbers still match: a firmware
       update may have retuned the preset since this record was saved. */
    if (s_preset >= 0) {
        blob_cfg_t p = s_cfg;
        blob_preset_apply(&p, s_preset);
        if (p.hue != s_cfg.hue || p.hue_tol != s_cfg.hue_tol ||
            p.sat_min != s_cfg.sat_min || p.val_min != s_cfg.val_min) {
            s_preset = -1;
        }
    }
    set_awb(t->awb != 0);
    return true;
}

/* "saved" and "defaults" both mean a power cycle gives back exactly what is
   running now; "modified" means it would not. */
static const char *settings_state(void)
{
    tc_settings_t now = live_capture();
    if (memcmp(&now, &s_ref, sizeof now) != 0) {
        return "modified";
    }
    return s_ref_saved ? "saved" : "defaults";
}

/* The machine-readable config. Printed after every accepted command, so the
   viewer's controls always show what the camera really uses rather than what
   the PC last asked for. */
static void print_cfg(void)
{
    printf("#!CFG w=%d h=%d cell=%d hue=%d tol=%d sat=%d val=%d area=%d mm=%.0f "
           "fov=%.1f fps=%d awb=%d be=%d preset=%s baud=%d image=%d stream=%d "
           "settings=%s\n",
           FRAME_W, FRAME_H, s_cfg.cell, s_cfg.hue, s_cfg.hue_tol, s_cfg.sat_min,
           s_cfg.val_min, s_cfg.min_area, s_target_mm, s_hfov_deg, s_target_fps,
           s_awb ? 1 : 0, s_cfg.big_endian,
           s_preset >= 0 ? blob_preset_name(s_preset) : "custom",
           link_baud(), s_image ? 1 : 0, s_streaming ? 1 : 0, settings_state());
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
    const char *st = settings_state();
    printf("# settings     %s\n",
           strcmp(st, "saved") == 0    ? "saved (loaded from flash)" :
           strcmp(st, "defaults") == 0 ? "defaults (nothing saved in flash)" :
                                         "modified (not saved: !save keeps them)");
    printf("# commands     p<red|blue|green|orange> preset | c calibrate on centre | "
           "H<deg> hue | T<deg> tol | S<0-255> sat | V<0-255> val\n");
    printf("#              a<px> area | d<mm> size | F<deg> h-FOV |");
    printf(" f<fps> rate | w<0|1> AWB | o byte order | m mask | "
           "s stream | ? info\n");
    printf("#              k config | g grab frame | v<0|1> every frame | "
           "B<baud> link speed | !save | !load | !reset\n");
    printf("# columns      t_ms,ok,cx_px,cy_px,az_deg,el_deg,area_px,w_px,h_px,"
           "fill,range_mm,clipped,blobs,matched_px,b2_az,b2_el,b2_range_mm,"
           "b3_az,b3_el,b3_range_mm,b4_az,b4_el,b4_range_mm,dt_ms,proc_us\n");
    fflush(stdout);
}

/* Parse an integer argument. Trailing junk is refused, not ignored: on the
   bench a burst of commands arrived with bytes dropped and changed, and
   "H33x" must not quietly become hue 33. */
static bool arg_in(const char *s, int lo, int hi, int *out)
{
    char *end;
    long v = strtol(s, &end, 10);
    while (*end == ' ') end++;
    if (end == s || *end != '\0' || v < lo || v > hi) {
        snprintf(s_why, sizeof s_why, "out of range (%d..%d)", lo, hi);
        return false;
    }
    *out = (int)v;
    return true;
}

/* How a command went, for the protocol line poll_commands() prints after it. */
typedef enum {
    CMD_NAK,        /* refused; s_why says why                                 */
    CMD_OK,         /* done: #!ACK, then #!CFG                                 */
    CMD_LATER,      /* accepted, finishes later: #!ACK now, no #!CFG yet       */
} cmd_res_t;

/* The '!' commands touch flash, so they are whole words: no single corrupted
   byte can turn into a write. */
static cmd_res_t run_store_command(const char *line)
{
    if (!s_nvs_ok) {
        snprintf(s_why, sizeof s_why, "flash storage (NVS) did not initialise");
        return CMD_NAK;
    }
    if (strcmp(line, "!save") == 0) {
        tc_settings_t t = live_capture();
        esp_err_t err = settings_save(&t);
        if (err != ESP_OK) {
            snprintf(s_why, sizeof s_why, "save failed: %s", esp_err_to_name(err));
            return CMD_NAK;
        }
        s_ref = t;
        s_ref_saved = true;
        printf("# settings saved to flash; every boot now starts with them\n");
        return CMD_OK;
    }
    if (strcmp(line, "!load") == 0) {
        tc_settings_t t;
        if (!settings_load(&t)) {
            snprintf(s_why, sizeof s_why, "nothing saved in flash");
            return CMD_NAK;
        }
        if (!live_apply(&t)) {
            snprintf(s_why, sizeof s_why, "saved record is out of range; ignored");
            return CMD_NAK;
        }
        s_ref = live_capture();
        s_ref_saved = true;
        printf("# settings reloaded from flash\n");
        return CMD_OK;
    }
    if (strcmp(line, "!reset") == 0) {
        esp_err_t err = settings_erase();
        if (err != ESP_OK) {
            snprintf(s_why, sizeof s_why, "erase failed: %s", esp_err_to_name(err));
            return CMD_NAK;
        }
        live_defaults();
        s_ref = live_capture();
        s_ref_saved = false;
        printf("# saved settings erased; running the Kconfig defaults\n");
        return CMD_OK;
    }
    snprintf(s_why, sizeof s_why, "unknown command (!save, !load, !reset)");
    return CMD_NAK;
}

static cmd_res_t run_command(const char *line)
{
    int v;
    switch (line[0]) {
    case '!':
        return run_store_command(line);
    case 'c':
        s_calibrate = true;
        return CMD_LATER;               /* #!CFG follows the calibration */
    case 'p': {
        /* 'p red', 'pred', 'p0' and 'pr' all select red. */
        const char *a = line + 1;
        while (*a == ' ') a++;
        int p = (*a >= '0' && *a <= '9') ? atoi(a) : blob_preset_find(a);
        if (!blob_preset_apply(&s_cfg, p)) {
            snprintf(s_why, sizeof s_why, "unknown preset (red, blue, green, orange)");
            return CMD_NAK;
        }
        s_preset = p;
        printf("# preset %s: hue %d +/-%d, sat >= %d, val >= %d\n",
               blob_preset_name(p), s_cfg.hue, s_cfg.hue_tol,
               s_cfg.sat_min, s_cfg.val_min);
        return CMD_OK;
    }
    case 'H':
        if (!arg_in(line + 1, 0, 359, &v)) return CMD_NAK;
        s_cfg.hue = v; s_preset = -1; printf("# hue %d\n", v);
        return CMD_OK;
    case 'T':
        if (!arg_in(line + 1, 1, 90, &v)) return CMD_NAK;
        s_cfg.hue_tol = v; s_preset = -1; printf("# hue tol %d\n", v);
        return CMD_OK;
    case 'S':
        if (!arg_in(line + 1, 0, 255, &v)) return CMD_NAK;
        s_cfg.sat_min = v; s_preset = -1; printf("# sat min %d\n", v);
        return CMD_OK;
    case 'V':
        if (!arg_in(line + 1, 0, 255, &v)) return CMD_NAK;
        s_cfg.val_min = v; s_preset = -1; printf("# val min %d\n", v);
        return CMD_OK;
    case 'a':
        if (!arg_in(line + 1, 1, FRAME_W * FRAME_H, &v)) return CMD_NAK;
        s_cfg.min_area = v;
        printf("# min area %d px\n", v);
        return CMD_OK;
    case 'd':
        if (!arg_in(line + 1, 0, 5000, &v)) return CMD_NAK;
        s_target_mm = (float)v;
        printf("# target size %d mm%s\n", v, v ? "" : " (range off)");
        return CMD_OK;
    case 'F': {
        /* Fractional, because the FOV is measured, and 1 deg of h-FOV is ~2%
           of every bearing and range. */
        char *end;
        float f = strtof(line + 1, &end);
        while (*end == ' ') end++;
        if (end == line + 1 || *end != '\0' || !(f >= 10.0f && f <= 150.0f)) {
            snprintf(s_why, sizeof s_why, "out of range (10..150)");
            return CMD_NAK;
        }
        s_hfov_deg = f;
        printf("# h-FOV %.1f deg -> f %.1f px\n", f, focal_px());
        return CMD_OK;
    }
    case 'f':
        if (!arg_in(line + 1, 1, 30, &v)) return CMD_NAK;
        s_target_fps = v; printf("# target %d FPS\n", v);
        return CMD_OK;
    case 'w':
        if (!arg_in(line + 1, 0, 1, &v)) return CMD_NAK;
        set_awb(v != 0);
        printf("# AWB %s\n", s_awb ? "auto" : "frozen at current gains");
        return CMD_OK;
    case 'o':
        s_cfg.big_endian = !s_cfg.big_endian;
        printf("# RGB565 byte order now %s-endian\n",
               s_cfg.big_endian ? "big" : "little");
        return CMD_OK;
    case 'm':
        s_dump_mask = true;
        return CMD_OK;
    case 's':
        s_streaming = !s_streaming;
        printf("# streaming %s\n", s_streaming ? "on" : "off");
        return CMD_OK;
    case 'k':
        return CMD_OK;                  /* the #!CFG line is the answer */
    case 'g':
        s_grab = true;
        return CMD_OK;
    case 'v':
        if (!arg_in(line + 1, 0, 1, &v)) return CMD_NAK;
        s_image = v != 0;
        printf("# image %s\n", s_image ? "every frame (timing is NOT flight-representative)"
                                       : "off");
        return CMD_OK;
    case 'B':
        if (!arg_in(line + 1, 1, 4000000, &v)) return CMD_NAK;
        if (!link_baud_allowed(v)) {
            snprintf(s_why, sizeof s_why,
                     "unsupported (115200 230400 460800 921600 1500000 2000000)");
            return CMD_NAK;
        }
        s_pending_baud = v;
        return CMD_LATER;               /* the viewer confirms with 'k' */
    case '?':
        print_banner();
        return CMD_OK;
    default:
        snprintf(s_why, sizeof s_why, "unknown command");
        return CMD_NAK;
    }
}

/*
 * Commands are case-sensitive: 'S' sets saturation, 's' toggles the stream.
 * Read without blocking so pacing is never disturbed.
 *
 * Every line gets "#!ACK <line>" or "#!NAK <line> -- <reason>", so the viewer
 * knows each command arrived intact instead of assuming it did; the human
 * "# ..." lines are unchanged, for a plain terminal and target_monitor.py.
 */
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
        cmd_res_t res = run_command(line);
        if (res != CMD_NAK) {
            /* Only a line that parsed as a command proves the PC can talk at
               this baud; line noise can end in a newline by chance. */
            link_note_command();
        }
        if (res == CMD_NAK) {
            printf("# %s\n", s_why);
            printf("#!NAK %s -- %s\n", line, s_why);
        } else {
            printf("#!ACK %s\n", line);
            if (res == CMD_OK) {
                print_cfg();
            }
        }
        fflush(stdout);
        if (s_pending_baud) {
            /* After the ACK, which has to go out at the speed the PC is
               still listening on. */
            link_set_baud(s_pending_baud);
            s_pending_baud = 0;
        }
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
    printf("#!CAL %s hue=%d spread=%d sat=%d val=%d\n",
           c.sat < 70 ? "refused" : "ok", c.hue, c.hue_spread, c.sat, c.val);
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
   reported centroid. The text fallback for when the PC viewer is not to hand.
   It tests pixels one by one rather than through blob_mask(), which would
   need a 19 kB buffer in DRAM that is already holding two frames. */
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

    /* Kconfig defaults first; a saved calibration, if any, replaces them. */
    live_defaults();
    s_ref = live_capture();
    s_ref_saved = false;
    esp_err_t nvs_err = settings_init();
    s_nvs_ok = nvs_err == ESP_OK;
    if (!s_nvs_ok) {
        printf("# settings      NVS init failed (%s): running defaults, !save "
               "unavailable\n", esp_err_to_name(nvs_err));
    } else {
        tc_settings_t t;
        if (settings_load(&t)) {
            if (live_apply(&t)) {
                s_ref = live_capture();
                s_ref_saved = true;
                printf("# settings      loaded from flash\n");
            } else {
                printf("# settings      saved record out of range, IGNORED: "
                       "running defaults\n");
            }
        } else {
            printf("# settings      nothing saved: running Kconfig defaults\n");
        }
    }
    fflush(stdout);

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
    link_init();

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
    /* A saved "AWB frozen" freezes here, at the gains the settle just found -
       the camera had no sensor to talk to when the settings were loaded. */
    set_awb(s_awb);

    print_banner();
    print_cfg();
    fflush(stdout);

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
        /* After poll_commands(), so a confirming command that queued up while
           a slow frame was going out counts before the revert deadline. */
        link_poll();

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
            print_cfg();
            fflush(stdout);
        }

        int64_t t0 = esp_timer_get_time();
        blob_result_t r;
        int ok = blob_detect(&s_cfg, fb->buf, s_work, &r);
        int64_t proc_us = esp_timer_get_time() - t0;

        if (s_dump_mask) {
            s_dump_mask = false;
            dump_mask(fb->buf, &r);
        }
        /* The frame goes out before its CSV row, with the same t_ms, so the PC
           can draw this detection on these pixels. The buffer is held while
           it is sent; with two buffers and GRAB_LATEST the driver keeps
           filling the other. */
        if (s_grab || s_image) {
            s_grab = false;
            link_send_frame(fb->buf, fb->len, FRAME_W, FRAME_H,
                            s_cfg.big_endian, capture_us / 1000);
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

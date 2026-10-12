/*
 * Downward-looking optical flow on an ESP32-CAM.
 *
 * Grabs paced grayscale frames, block-matches each against the previous one,
 * and streams the result as CSV. Written for a blimp hovering at ~2 m with the
 * camera pointed at the floor.
 *
 * Three things drive every design choice here:
 *
 *  1. HEAT. An ESP32-CAM running flat out gets hot enough to brown-out, and the
 *     AMS1117 regulator on the AI-Thinker board is a large part of why. So: the
 *     radios are never initialised (not merely disabled - never started), XCLK
 *     is halved from the usual 20 MHz, the CPU runs at 160 MHz, the frame size
 *     is the smallest useful one, the 1 W flash LED is driven low explicitly,
 *     and frames are PACED rather than taken as fast as they arrive. The idle
 *     time between frames is the point, not a side effect.
 *
 *  2. HONEST FAILURE. A flow sensor that reports plausible numbers over a blank
 *     floor is worse than one that reports nothing, because a state estimator
 *     will believe it. Every rejection reason is counted and printed, and a
 *     frame with too few good blocks emits a row with ok=0 instead of a guess.
 *
 *  3. NO IMU HERE. Camera rotation produces apparent translation that cannot be
 *     separated from real motion using the image alone. Worse, the two height
 *     terms cancel exactly - a rotation of theta shifts the image by f*theta
 *     pixels, and f * mm_per_pixel == height identically - so the phantom ground
 *     velocity is just height * omega: 34.9 mm/s per deg/s at 2 m, and no
 *     better at any other altitude. This firmware therefore reports raw pixel
 *     flow ALONGSIDE the velocity, so whichever board owns the gyro can
 *     de-rotate properly. Do not use vx/vy as a velocity measurement while the
 *     airframe is turning.
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
#include "flow_match.h"

static const char *TAG = "optflow";

#define FRAME_SIZE      FRAMESIZE_QQVGA     /* 160x120 */
#define FRAME_W         160
#define FRAME_H         120

/* Seconds of reflash window before GPIO0 becomes XCLK. See
   escape_hatch_delay(). */
#define OPTFLOW_ESCAPE_SECONDS  5

/* Runtime-adjustable state. Single-threaded, so no locking. */
static int   s_target_fps   = CONFIG_OPTFLOW_TARGET_FPS;
static float s_height_mm    = (float)CONFIG_OPTFLOW_HEIGHT_MM;
static float s_hfov_deg     = (float)CONFIG_OPTFLOW_HFOV_DEG;
static bool  s_streaming    = true;

/* Integrated displacement. Dead reckoning, so it drifts without bound - it is
   here to make a bench test readable ("I slid it 300 mm and it says 290"), not
   as a position estimate. */
static float s_pos_x_mm = 0.0f;
static float s_pos_y_mm = 0.0f;

static uint8_t *s_prev;     /* our own copy of the previous frame */
static bool     s_have_prev;

static flow_cfg_t s_cfg = {
    .width   = FRAME_W,
    .height  = FRAME_H,
    .block   = CONFIG_OPTFLOW_BLOCK,
    .search  = CONFIG_OPTFLOW_SEARCH,
    .grid_x  = CONFIG_OPTFLOW_GRID,
    .grid_y  = CONFIG_OPTFLOW_GRID,
    .min_var = CONFIG_OPTFLOW_MIN_VAR,
    /* V-fit, not the parabola. A SAD minimum over real texture is a cusp, and
       fitting a parabola to a cusp loses half the displacement below 0.5 px -
       exactly where a 1.5 m hover lives. Measured on the host: 91% of true
       distance recovered at 0.12 px/frame against the parabola's 51%. See the
       report, section 8.13. */
    .subpix  = FLOW_SUBPIX_VFIT,
};

static const char *subpix_name(int m)
{
    switch (m) {
    case FLOW_SUBPIX_GAUSSIAN: return "gaussian";
    case FLOW_SUBPIX_VFIT:     return "vfit";
    default:                   return "parabola";
    }
}

/* ------------------------------------------------------------------ */

/* Ground sample distance: how many millimetres of floor one pixel covers.
   Pinhole model, so this scales linearly with height - which is why the height
   input matters as much as the image does. */
static float mm_per_pixel(void)
{
    float hfov_rad = s_hfov_deg * (float)M_PI / 180.0f;
    float footprint_mm = 2.0f * s_height_mm * tanf(hfov_rad * 0.5f);
    return footprint_mm / (float)FRAME_W;
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
    /* Nothing initialises WiFi or Bluetooth anywhere in this firmware. That is
       deliberate and is the largest single thermal saving available; esp_wifi
       is not even linked. */
}

static esp_err_t camera_start(void)
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

        .xclk_freq_hz   = CONFIG_OPTFLOW_XCLK_MHZ * 1000 * 1000,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,

        /* Grayscale straight from the sensor. The OV2640 driver routes
           GRAYSCALE through its YUV422 register set and hands us the Y plane,
           so there is no colour conversion and no JPEG decode on our side -
           one byte per pixel, exactly what the matcher wants. */
        .pixel_format   = PIXFORMAT_GRAYSCALE,
        .frame_size     = FRAME_SIZE,

        /* QQVGA grayscale is 19200 bytes, so two buffers fit in internal RAM.
           Keeping them out of PSRAM means this runs on a board with no PSRAM,
           and avoids clocking the SPI RAM at all - less power, less heat. */
        .fb_location    = CAMERA_FB_IN_DRAM,
        .fb_count       = 2,
        .grab_mode      = CAMERA_GRAB_LATEST,

        .jpeg_quality   = 12,   /* unused in grayscale, must still be sane */
    };

    esp_err_t err = esp_camera_init(&config);
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

    /* Auto-exposure and auto-gain stay ON: floor brightness varies and a fixed
       exposure would blow out or black out the texture the matcher needs.
       Everything that costs cycles or invents detail is turned off. */
    if (s->set_whitebal)    { s->set_whitebal(s, 0); }   /* meaningless in gray */
    if (s->set_awb_gain)    { s->set_awb_gain(s, 0); }
    if (s->set_bpc)         { s->set_bpc(s, 1); }        /* dead-pixel fix: keep */
    if (s->set_wpc)         { s->set_wpc(s, 1); }
    if (s->set_lenc)        { s->set_lenc(s, 1); }       /* flattens vignetting */
    if (s->set_raw_gma)     { s->set_raw_gma(s, 1); }
    if (s->set_dcw)         { s->set_dcw(s, 1); }
    if (s->set_hmirror)     { s->set_hmirror(s, 0); }
    if (s->set_vflip)       { s->set_vflip(s, 0); }
    if (s->set_sharpness)   { s->set_sharpness(s, 0); }  /* no invented edges */
    if (s->set_denoise)     { s->set_denoise(s, 0); }    /* no smeared texture */
    /* Cap the auto-gain. High gain on a dim floor buys brightness by adding
       noise, and noise is what makes a block match to the wrong place. */
    if (s->set_gain_ctrl)   { s->set_gain_ctrl(s, 1); }
    if (s->set_agc_gain)    { s->set_agc_gain(s, 0); }
    if (s->set_gainceiling) { s->set_gainceiling(s, GAINCEILING_4X); }
    if (s->set_exposure_ctrl) { s->set_exposure_ctrl(s, 1); }
    if (s->set_aec2)        { s->set_aec2(s, 0); }

    return ESP_OK;
}

/* ------------------------------------------------------------------ */

static void print_banner(void)
{
    float mmpx = mm_per_pixel();
    /* A displacement of exactly `search` px puts the SAD minimum on the edge of
       the window, where the boundary check rejects it. Usable radius is one
       pixel less - verified on the host suite. */
    float max_px_per_s = (float)(s_cfg.search - 1) * (float)s_target_fps;

    printf("\n# ESP32-CAM downward optical flow\n");
    printf("# frame        %dx%d grayscale, XCLK %d MHz, paced to %d FPS\n",
           FRAME_W, FRAME_H, CONFIG_OPTFLOW_XCLK_MHZ, s_target_fps);
    printf("# matcher      %d px blocks, +/-%d px search, %dx%d grid, min_var %u, "
           "sub-pixel %s\n",
           s_cfg.block, s_cfg.search, s_cfg.grid_x, s_cfg.grid_y,
           (unsigned)s_cfg.min_var, subpix_name(s_cfg.subpix));
    printf("# geometry     height %.0f mm, h-FOV %.1f deg -> %.2f mm/px, "
           "footprint %.2f m\n",
           s_height_mm, s_hfov_deg, mmpx, mmpx * FRAME_W / 1000.0f);
    printf("# range        +/-%.0f mm/s max, %.1f mm/s per 0.1 px\n",
           max_px_per_s * mmpx, 0.1f * mmpx * s_target_fps);
    printf("# commands     h<mm> height | f<fps> rate | r<px> search | "
           "e<0|1|2> sub-pixel | z zero | s stream | ? info\n");
    printf("# WARNING      vx/vy include rotation-induced flow (~%.0f mm/s per "
           "deg/s). De-rotate with a gyro before using them.\n",
           s_height_mm * (float)M_PI / 180.0f);
    printf("# columns      t_ms,ok,dx_px,dy_px,vx_mm_s,vy_mm_s,x_mm,y_mm,"
           "spread_px,valid,total,flat,edge,ambig,dt_ms,proc_us\n");
    fflush(stdout);
}

/* Single-character commands, read without blocking so the frame pacing is
   never disturbed by a console that nobody is typing at. */
static void poll_commands(void)
{
    static char line[32];
    static int  len;
    uint8_t ch;

    while (uart_read_bytes(UART_NUM_0, &ch, 1, 0) == 1) {
        if (ch == '\r' || ch == '\n') {
            line[len] = '\0';
            if (len > 0) {
                switch (line[0]) {
                case 'h': {
                    float v = strtof(line + 1, NULL);
                    if (v >= 100.0f && v <= 10000.0f) {
                        s_height_mm = v;
                        printf("# height %.0f mm -> %.2f mm/px\n",
                               s_height_mm, mm_per_pixel());
                    } else {
                        printf("# height out of range (100..10000 mm)\n");
                    }
                    break;
                }
                case 'f': {
                    int v = atoi(line + 1);
                    if (v >= 1 && v <= 30) {
                        s_target_fps = v;
                        printf("# target %d FPS\n", s_target_fps);
                    } else {
                        printf("# fps out of range (1..30)\n");
                    }
                    break;
                }
                case 'r': {
                    /* Tunable at runtime because the match cost scales as
                       (2*search+1)^2 and is what decides whether the loop
                       keeps up with the sensor's frame rate. Reflashing this
                       board needs the BOOT button held, so make it a knob. */
                    int v = atoi(line + 1);
                    flow_cfg_t trial = s_cfg;
                    trial.search = v;
                    if (v >= 1 && v <= FLOW_MAX_SEARCH && flow_cfg_valid(&trial)) {
                        s_cfg = trial;
                        s_have_prev = false;   /* old pair was matched differently */
                        printf("# search +/-%d px (usable +/-%d px) -> cost x%.2f\n",
                               s_cfg.search, s_cfg.search - 1,
                               (float)((2 * v + 1) * (2 * v + 1)) / 225.0f);
                    } else {
                        printf("# search out of range (1..%d) or invalid for this grid\n",
                               FLOW_MAX_SEARCH);
                    }
                    break;
                }
                case 'e': {
                    /* Same reasoning as 'r': the BOOT button makes reflashing
                       expensive, and the estimator is the one thing we want to
                       A/B on the *same* physical slide rather than two. */
                    int v = atoi(line + 1);
                    flow_cfg_t trial = s_cfg;
                    trial.subpix = v;
                    if (flow_cfg_valid(&trial)) {
                        s_cfg = trial;
                        printf("# sub-pixel %d (%s)\n", v, subpix_name(v));
                    } else {
                        printf("# sub-pixel out of range (0 parabola, "
                               "1 gaussian, 2 vfit)\n");
                    }
                    break;
                }
                case 'z':
                    s_pos_x_mm = s_pos_y_mm = 0.0f;
                    printf("# position zeroed\n");
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
            len = 0;
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = (char)ch;
        }
    }
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

/*
 * Printed before anything draws meaningful current, and before GPIO0 becomes
 * an output. Learned the hard way: the first version of this firmware printed
 * its banner only AFTER esp_camera_init(), so a board that browned out during
 * camera power-up produced zero bytes and was indistinguishable from a dead
 * chip. Diagnostics must come before the step that can kill the board.
 */
static void boot_report(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    esp_reset_reason_t rr = esp_reset_reason();

    printf("\n\n# ---- optflow_cam boot ----\n");
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

/*
 * A deliberate pause before the camera starts.
 *
 * On an AI-Thinker ESP32-CAM, XCLK is GPIO0 - the same pin esptool must pull
 * low to enter download mode. The moment esp_camera_init() starts the LEDC
 * clock, GPIO0 becomes a strong push-pull output and the adapter's DTR line
 * can no longer win against it, so the board stops being flashable without
 * manually jumpering GPIO0 to GND.
 *
 * This window is the escape hatch: during it GPIO0 is still an input, so a
 * reflash started now will succeed. Do not remove it.
 */
static void escape_hatch_delay(void)
{
    printf("#\n# GPIO0 is XCLK on this board: once the camera starts, esptool\n");
    printf("# can no longer force download mode. Reflash window open:\n");
    for (int i = OPTFLOW_ESCAPE_SECONDS; i > 0; i--) {
        printf("#   %d...\n", i);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("# window closed, starting the camera\n");
    fflush(stdout);
}

void app_main(void)
{
    boot_report();
    quiesce_heat_sources();

    if (!flow_cfg_valid(&s_cfg)) {
        ESP_LOGE(TAG, "matcher configuration is impossible for a %dx%d frame: "
                      "block %d + 2*search %d must fit, and grid %dx%d must be "
                      "at most %d blocks",
                 FRAME_W, FRAME_H, s_cfg.block, s_cfg.search,
                 s_cfg.grid_x, s_cfg.grid_y, FLOW_MAX_BLOCKS);
        return;
    }

    s_prev = heap_caps_malloc(FRAME_W * FRAME_H, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_prev == NULL) {
        ESP_LOGE(TAG, "could not allocate the %d-byte previous-frame buffer",
                 FRAME_W * FRAME_H);
        return;
    }

    /* Take over UART0 so reads can be non-blocking. stdout keeps working. */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    escape_hatch_delay();

    /* The next line is where a board with an inadequate supply dies: powering
       up the OV2640 is the largest current step this firmware ever takes. If
       the output stops here, it is the supply, not the code. */
    printf("# esp_camera_init: powering up the sensor now\n");
    fflush(stdout);
    if (camera_start() != ESP_OK) {
        printf("# camera init FAILED - see the error above\n");
        fflush(stdout);
        return;
    }
    printf("# esp_camera_init: ok\n");
    fflush(stdout);

    /* Let auto-exposure settle before the first match; a frame pair taken
       across an exposure step looks like a global brightness change, which is
       exactly what SAD block matching is worst at. */
    ESP_LOGI(TAG, "waiting for auto-exposure to settle");
    for (int i = 0; i < 10; i++) {
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
        /* Pace first, work second. The delay is computed from the target rate
           and taken with vTaskDelayUntil, so the period does not drift with
           however long the matching took. This idle time is the thermal
           budget. */
        int period_ms = 1000 / s_target_fps;
        TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
        if (period_ticks == 0) {
            period_ticks = 1;
        }
        xTaskDelayUntil(&next_wake, period_ticks);

        poll_commands();

        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "frame grab failed");
            s_have_prev = false;      /* the time gap invalidates the pair */
            continue;
        }

        int64_t capture_us = esp_timer_get_time();

        if (fb->format != PIXFORMAT_GRAYSCALE ||
            fb->width != FRAME_W || fb->height != FRAME_H) {
            ESP_LOGE(TAG, "unexpected frame: %dx%d format %d",
                     fb->width, fb->height, fb->format);
            esp_camera_fb_return(fb);
            continue;
        }

        if (!s_have_prev) {
            memcpy(s_prev, fb->buf, FRAME_W * FRAME_H);
            s_have_prev = true;
            prev_capture_us = capture_us;
            esp_camera_fb_return(fb);
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        flow_result_t r;
        int ok = flow_match(&s_cfg, s_prev, fb->buf, &r);
        int64_t proc_us = esp_timer_get_time() - t0;

        /* Measure dt from the real capture timestamps, not the target period.
           A dropped or late frame makes the true interval longer, and dividing
           by the wrong dt turns an accurate pixel measurement into an
           inaccurate velocity. */
        float dt_s = (float)(capture_us - prev_capture_us) * 1e-6f;

        float mmpx = mm_per_pixel();
        float vx = 0.0f, vy = 0.0f;

        if (ok && dt_s > 0.0f) {
            /* The ground moving left in the image means the airframe moved
               right, so body velocity is the negative of image flow. */
            vx = -r.dx * mmpx / dt_s;
            vy = -r.dy * mmpx / dt_s;
            s_pos_x_mm += vx * dt_s;
            s_pos_y_mm += vy * dt_s;
        }

        if (s_streaming) {
            printf("%lld,%d,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.3f,"
                   "%d,%d,%d,%d,%d,%.1f,%lld\n",
                   capture_us / 1000, ok, r.dx, r.dy, vx, vy,
                   s_pos_x_mm, s_pos_y_mm, r.spread,
                   r.n_valid, r.n_total, r.n_flat, r.n_edge, r.n_ambig,
                   dt_s * 1000.0f, proc_us);
            fflush(stdout);
        }

        memcpy(s_prev, fb->buf, FRAME_W * FRAME_H);
        prev_capture_us = capture_us;
        esp_camera_fb_return(fb);
    }
}

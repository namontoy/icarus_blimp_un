/* main.c - RTOS shell around blimp_step(). NO CONTROL LOGIC LIVES HERE.
 *
 * Everything this file does: bring up NVS, the shared I2C bus, the battery
 * ADC, the MPU-6050 IMU driver (plus the mag/baro drivers, compiled out via
 * BLIMP_BOARD_HAS_MAG/HAS_BARO since neither is populated on this PCB), the
 * four-motor LEDC PWM driver, and the ESP-NOW link; then create three
 * FreeRTOS tasks and get out of the way. The entire flight controller is
 * core/src/blimp.c's blimp_step() - this file's only job is to call it at a
 * fixed rate with real (or honestly-stubbed) sensor data and hand its output
 * to the actuator drivers.
 *
 * ===========================================================================
 * CONTROL RATE: 200 Hz (BLIMP_CONTROL_PERIOD_US = 5000 us below)
 * ===========================================================================
 * docs/07-control-design.md section 4.2 and docs/03-software-architecture.md
 * section 1.1 both flag that the reference corpus gives THREE different
 * numbers for "the" control rate: the MochiSwarm paper's own claimed 200 Hz,
 * the real shipped BlimpSwarm firmware's TIME_STEP_MICRO=4000us (250 Hz),
 * and mochi-swarm-sim's MuJoCo control callback at 100 Hz. Both documents'
 * own conclusion is the same: treat 200 Hz as a validated, achievable
 * TARGET CLASS for this MCU doing real sensor fusion + control + PWM, but
 * MEASURE the actually-achieved rate on the real PCB and sensor stack during
 * bring-up (docs/07 section 7, bring-up step 7 onward) rather than trusting
 * any of the three numbers as-is. This firmware follows that: 200 Hz is the
 * STARTING value, chosen to sit at the validated end of the range rather
 * than the unverified 250 Hz figure. Re-measure control_task's actual loop
 * period (log esp_timer_get_time() deltas, or watch for the "control tick
 * overrun/missed" warning below) once real sensors are attached, and adjust
 * BLIMP_CONTROL_PERIOD_US if blimp_step() plus every driver call cannot
 * actually keep up.
 *
 * WHY esp_timer + a semaphore, NOT vTaskDelayUntil, drives this loop:
 * ESP-IDF's default FreeRTOS tick rate is CONFIG_FREERTOS_HZ=100 (10 ms
 * granularity) - vTaskDelayUntil() cannot produce a 5 ms (200 Hz) period
 * against a 10 ms tick; the best it could do is round to the nearest tick,
 * silently giving 100 Hz instead of the intended 200 Hz. esp_timer runs off
 * a dedicated hardware timer at microsecond resolution, independent of
 * CONFIG_FREERTOS_HZ, so it gives a genuinely fixed period regardless of
 * tick-rate configuration. (Raising CONFIG_FREERTOS_HZ to 1000 in sdkconfig
 * would also narrow the gap and is worth doing anyway for general RTOS
 * responsiveness, but esp_timer is the correct fix for the control period
 * itself, not a workaround for not having done that.)
 */
#include "blimp/blimp.h"

#include "board_config.h"
#include "drivers/imu.h"
#include "drivers/baro.h"
#include "drivers/mag.h"
#include "drivers/motor.h"
#include "net/espnow_link.h"
#include "net/protocol.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <string.h>

static const char *TAG = "main";

/* ---- RTOS scheduling constants ----------------------------------------
 * Deliberately NOT in blimp_params_t (core/include/blimp/params.h): these
 * are operating-system scheduling facts (task priority, core affinity, tick
 * period), not control gains, and core/ has no concept of an RTOS at all
 * (blimp/blimp.h's own contract: no FreeRTOS dependency in core/).
 * ------------------------------------------------------------------------- */
#define BLIMP_CONTROL_PERIOD_US     5000u   /* 200 Hz - see header comment */

#define BLIMP_CORE_CONTROL          0        /* estimator+control+mixer+
                                              * guidance, matching docs/03
                                              * section 1.1's own core split */
#define BLIMP_CORE_OTHER            1        /* comms, telemetry           */

#define BLIMP_TASK_PRIO_CONTROL     20       /* highest of the three: this
                                              * is the hard-real-time chain  */
#define BLIMP_TASK_PRIO_COMMS       10
#define BLIMP_TASK_PRIO_TELEMETRY    5       /* lowest: a dropped/delayed
                                              * telemetry frame costs a log
                                              * gap, never flight safety     */

#define BLIMP_TASK_STACK_CONTROL    4096
#define BLIMP_TASK_STACK_COMMS      4096
#define BLIMP_TASK_STACK_TELEMETRY  3072

/* ---- shared state between tasks ----------------------------------------
 * s_blimp is owned EXCLUSIVELY by control_task: every blimp_*() call that
 * touches it (blimp_init, blimp_step, blimp_notify_link) happens only from
 * that one task. No other task calls into core/ directly. Cross-task
 * communication happens only through the two primitives below, both plain
 * data handoffs with no core API call inside the lock - this sidesteps any
 * question of whether blimp_t's own functions are safe to call from two
 * tasks at once (core/ makes no such promise, and needs none, if only one
 * task ever calls into it).
 * ------------------------------------------------------------------------- */
static blimp_t s_blimp;

typedef struct {
    blimp_target_obs_t target;
    bool     arm_request;
    uint32_t rx_seq;      /* incremented by comms_task on each valid command;
                           * control_task compares against its own last-seen
                           * value to decide whether to call
                           * blimp_notify_link() this tick                   */
} link_snapshot_t;

static SemaphoreHandle_t s_link_mutex;
static link_snapshot_t   s_link_snapshot;

static QueueHandle_t s_telemetry_queue;   /* length 1, holds one
                                          * blimp_telemetry_packet_t -
                                          * xQueueOverwrite'd by control_task,
                                          * consumed by telemetry_task        */

static SemaphoreHandle_t s_control_tick_sem;
static adc_oneshot_unit_handle_t s_adc_handle;

/* ---- board bring-up ----------------------------------------------------- */

static esp_err_t board_i2c_init(void)
{
    const i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = BLIMP_BOARD_I2C_SDA_GPIO,
        .scl_io_num    = BLIMP_BOARD_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master        = { .clk_speed = BLIMP_BOARD_I2C_CLK_HZ },
    };
    esp_err_t err = i2c_param_config((i2c_port_t)BLIMP_BOARD_I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install((i2c_port_t)BLIMP_BOARD_I2C_PORT, conf.mode, 0, 0, 0);
}

/* No drivers/battery.{h,c} pair exists in this skeleton (out of the task's
 * explicit driver file list - board_config.h's battery section explains
 * this) - the ADC read lives inline here. Promote to its own driver file if
 * it grows past this. */
static esp_err_t battery_adc_init(void)
{
    const adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        return err;
    }
    /* CONFIDENCE FLAG: ESP-IDF renamed `ADC_ATTEN_DB_11` to `ADC_ATTEN_DB_12`
     * in a v5.x release (same physical ~0-3.3V attenuation, a naming
     * correction) - if the installed IDF predates that rename, substitute
     * ADC_ATTEN_DB_11 here. Check esp_adc/adc_oneshot.h's `adc_atten_t` enum
     * for the version actually installed. */
    const adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    return adc_oneshot_config_channel(
        s_adc_handle, (adc_channel_t)BLIMP_BOARD_BATT_ADC_CHANNEL, &chan_cfg);
}

/* Returns a NEGATIVE value on failure (a raw ADC read failing) - -1.0f is
 * unambiguous since a real pack voltage is never negative. NOT
 * calibration-corrected (no adc_cali_* curve) - see board_config.h's
 * warning on ESP32-S3's known ADC nonlinearity around 2.5-3.0V. Good enough
 * for a coarse low-battery failsafe threshold; not for a logged "precise"
 * pack voltage. */
static float battery_read_voltage(void)
{
    if (!s_adc_handle) {
        return -1.0f;
    }
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, (adc_channel_t)BLIMP_BOARD_BATT_ADC_CHANNEL,
                         &raw) != ESP_OK) {
        return -1.0f;
    }
    const float vadc = ((float)raw / 4095.0f) * 3.3f;   /* 12-bit, ~3.3V FSR
                                                         * at 12dB atten -
                                                         * both assumed, see
                                                         * comment above */
    return vadc * BLIMP_BOARD_BATT_DIVIDER_RATIO;
}

/* Runs in the esp_timer service task (dispatch_method = ESP_TIMER_TASK, the
 * default), NOT in an ISR - so this is an ordinary xSemaphoreGive(), not the
 * FromISR variant. (Using the FromISR API here would be a real bug: it is
 * only correct when the caller is genuinely an interrupt context, which the
 * default esp_timer dispatch method is not.) */
static void control_timer_cb(void *arg)
{
    (void)arg;
    xSemaphoreGive(s_control_tick_sem);
}

/* ---- telemetry assembly --------------------------------------------------
 * Reads s_blimp directly - safe ONLY because this is called from
 * control_task, the struct's sole owner (see the header comment on
 * s_blimp above). Do not call this from any other task. */
static void build_telemetry_packet(const blimp_actuators_t *act,
                                   blimp_telemetry_packet_t *pkt)
{
    static uint8_t s_seq;

    memset(pkt, 0, sizeof(*pkt));
    pkt->magic   = BLIMP_PROTO_MAGIC;
    pkt->version = BLIMP_PROTO_VERSION;
    pkt->seq     = s_seq++;
    pkt->t_us    = s_blimp.state.t_us;

    pkt->roll  = s_blimp.state.roll;
    pkt->pitch = s_blimp.state.pitch;
    pkt->yaw   = s_blimp.state.yaw;
    pkt->altitude = s_blimp.state.altitude;
    pkt->vz       = s_blimp.state.vz;
    pkt->yaw_rate = s_blimp.state.yaw_rate;

    pkt->fx = s_blimp.demand.fx;
    pkt->fz = s_blimp.demand.fz;
    pkt->tz = s_blimp.demand.tz;

    pkt->thrust_fl = act->thrust_fl;
    pkt->thrust_fr = act->thrust_fr;
    pkt->thrust_rl = act->thrust_rl;
    pkt->thrust_rr = act->thrust_rr;
    pkt->clamped_total = act->clamped_total ? 1u : 0u;
    pkt->clamped_yaw   = act->clamped_yaw   ? 1u : 0u;
    pkt->clamped_heave = act->clamped_heave ? 1u : 0u;

    pkt->guid_state = (uint8_t)s_blimp.guid.state;
    pkt->armed      = s_blimp.armed ? 1u : 0u;

    pkt->crc32 = 0;
    pkt->crc32 = blimp_proto_crc32(pkt, sizeof(*pkt));
}

/* ---- tasks ---------------------------------------------------------------- */

static void control_task(void *arg)
{
    (void)arg;
    blimp_init(&s_blimp);
    uint32_t last_seen_rx_seq = 0;

    ESP_LOGI(TAG, "control_task started on core %d, period %u us",
             (int)xPortGetCoreID(), (unsigned)BLIMP_CONTROL_PERIOD_US);

    for (;;) {
        if (xSemaphoreTake(s_control_tick_sem, pdMS_TO_TICKS(50)) != pdTRUE) {
            ESP_LOGW(TAG, "control tick overrun/missed - timer not firing, "
                          "or blimp_step()+drivers took longer than one "
                          "period. If this repeats, BLIMP_CONTROL_PERIOD_US "
                          "is optimistic for this build - measure and "
                          "increase it (see this file's header comment).");
            continue;
        }

        blimp_input_t in = {0};
        in.t_us = (uint32_t)esp_timer_get_time();

        blimp_imu_sample_t sample = {0};
        sample.t_us = in.t_us;
        /* Each driver fills only its own slice of the sample and its own
         * *_valid flag - see imu.h/baro.h/mag.h. range_alt/range_valid stay
         * unset: no ToF driver exists in this skeleton (board_config.h).
         * mag/baro reads are compiled out entirely when the corresponding
         * BLIMP_BOARD_HAS_* flag is 0 (neither sensor is populated on this
         * PCB) - not just skipped, so an absent sensor is never probed on
         * the shared I2C bus at all. */
        (void)blimp_imu_read(&sample);
#if BLIMP_BOARD_HAS_BARO
        (void)blimp_baro_read(&sample);
#endif
#if BLIMP_BOARD_HAS_MAG
        (void)blimp_mag_read(&sample);
#endif
        in.imu = sample;

        if (xSemaphoreTake(s_link_mutex, 0) == pdTRUE) {
            in.target      = s_link_snapshot.target;
            in.arm_request = s_link_snapshot.arm_request;
            if (s_link_snapshot.rx_seq != last_seen_rx_seq) {
                last_seen_rx_seq = s_link_snapshot.rx_seq;
                /* The ONLY place blimp_notify_link() is called - keeps every
                 * write to s_blimp inside this one task. */
                blimp_notify_link(&s_blimp, in.t_us);
            }
            xSemaphoreGive(s_link_mutex);
        }

        /* TODO: onboard contact detector (docs/05-race-strategy.md section
         * 4 / docs/07-control-design.md section 6.2) - IMU deceleration-
         * spike primary check, vision-proximity fallback. Not implemented
         * in this skeleton; guidance's own APPROACH_TIMEOUT fallback (see
         * guidance.h) is the only thing standing in for it today, so the
         * vehicle will currently always time out an approach rather than
         * ever confirming a touch. */
        in.contact_detected = false;

        blimp_actuators_t out = {0};
        blimp_step(&s_blimp, &in, &out);

        (void)blimp_motor_apply(&s_blimp.params, &out);

        blimp_telemetry_packet_t tpkt;
        build_telemetry_packet(&out, &tpkt);
        (void)xQueueOverwrite(s_telemetry_queue, &tpkt);
    }
}

static void comms_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "comms_task started on core %d", (int)xPortGetCoreID());

    for (;;) {
        blimp_cmd_packet_t cmd;
        if (blimp_espnow_link_get_command(&cmd)) {
            if (xSemaphoreTake(s_link_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_link_snapshot.target.bearing_rad   = cmd.bearing_rad;
                s_link_snapshot.target.elevation_rad = cmd.elevation_rad;
                s_link_snapshot.target.range_m       = cmd.range_m;
                s_link_snapshot.target.confidence    = cmd.confidence;
                s_link_snapshot.target.visible       = (cmd.visible != 0);
                s_link_snapshot.target.hint_yaw_rad  = cmd.hint_yaw_rad;
                s_link_snapshot.target.hint_valid    = (cmd.hint_valid != 0);
                s_link_snapshot.target.alt_fix_m     = cmd.alt_fix_m;
                s_link_snapshot.target.alt_fix_valid = (cmd.alt_fix_valid != 0);
                s_link_snapshot.target.t_us          = (uint32_t)esp_timer_get_time();
                s_link_snapshot.arm_request          = (cmd.arm_request != 0);
                s_link_snapshot.rx_seq++;
                xSemaphoreGive(s_link_mutex);
            }
        }
        /* 100 Hz poll of the ESP-NOW RX mailbox - NOT the fixed control
         * period, so an ordinary vTaskDelay() is fine here; nothing about
         * this loop needs to be jitter-free. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void telemetry_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "telemetry_task started on core %d", (int)xPortGetCoreID());

    blimp_telemetry_packet_t pkt;
    for (;;) {
        if (xQueueReceive(s_telemetry_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            (void)blimp_espnow_link_send_telemetry(&pkt);
            /* TODO: also mirror to a UART/console log ring buffer per
             * docs/03-software-architecture.md section 1.5's param/log
             * service plan - not implemented in this skeleton. */
        }
    }
}

/* ---- entry point ----------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "DroneUN firmware boot - blimp core %s "
                 "(XIAO ESP32S3 + ESP-FLY PCB: MPU-6050 IMU, 4x brushed "
                 "motor via low-side MOSFET/LEDC PWM, ESP-NOW ground link)",
             blimp_version());

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(board_i2c_init());
    ESP_ERROR_CHECK(battery_adc_init());

    /* Every *_init() below is designed to return ESP_OK even if its sensor
     * turns out to be absent (see each driver's own header comment) - a
     * missing sensor should not stop the vehicle from booting into a state
     * where its absence can at least be observed over telemetry/logs.
     * blimp_params_validate() and the guidance PREPARE phase's own
     * pre-flight gates (docs/07 section 4.4, section 7 step 1) are the
     * actual place a missing REQUIRED sensor should block arming - not
     * app_main. */
    ESP_ERROR_CHECK(blimp_imu_init());
#if BLIMP_BOARD_HAS_BARO
    ESP_ERROR_CHECK(blimp_baro_init());
#endif
#if BLIMP_BOARD_HAS_MAG
    ESP_ERROR_CHECK(blimp_mag_init());
#endif
    ESP_ERROR_CHECK(blimp_motor_init());

#if BLIMP_BOOT_MOTOR_ID_MODE
    blimp_motor_identify_forever(); /* never returns */
#endif

    ESP_ERROR_CHECK(blimp_espnow_link_init());

    s_link_mutex       = xSemaphoreCreateMutex();
    s_telemetry_queue  = xQueueCreate(1, sizeof(blimp_telemetry_packet_t));
    s_control_tick_sem = xSemaphoreCreateBinary();
    if (!s_link_mutex || !s_telemetry_queue || !s_control_tick_sem) {
        ESP_LOGE(TAG, "RTOS primitive allocation failed - halting boot");
        return;
    }

    const esp_timer_create_args_t timer_args = {
        .callback       = &control_timer_cb,
        .arg            = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name           = "ctrl_tick",
    };
    esp_timer_handle_t control_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &control_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(control_timer, BLIMP_CONTROL_PERIOD_US));

    ESP_LOGI(TAG, "battery: %.2f V (uncalibrated estimate)",
             (double)battery_read_voltage());

    xTaskCreatePinnedToCore(control_task, "blimp_ctrl", BLIMP_TASK_STACK_CONTROL,
                            NULL, BLIMP_TASK_PRIO_CONTROL, NULL, BLIMP_CORE_CONTROL);
    xTaskCreatePinnedToCore(comms_task, "blimp_comms", BLIMP_TASK_STACK_COMMS,
                            NULL, BLIMP_TASK_PRIO_COMMS, NULL, BLIMP_CORE_OTHER);
    xTaskCreatePinnedToCore(telemetry_task, "blimp_telem", BLIMP_TASK_STACK_TELEMETRY,
                            NULL, BLIMP_TASK_PRIO_TELEMETRY, NULL, BLIMP_CORE_OTHER);
}

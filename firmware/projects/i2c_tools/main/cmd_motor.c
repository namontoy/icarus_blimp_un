/*
 * 'motor' console command and thrust API: two DRV8833 dual H-bridges.
 *
 * Four brushed thrusters in a quad layout, two per driver, eight logic lines
 * in total. As wired:
 *
 *                      +-----------------+
 *     step-up 5V ----->| VM         OUT1 |---- motor_RL
 *            GND ----->| GND        OUT2 |---'
 *        GPIO 13 ----->| IN1             |
 *        GPIO 12 ----->| IN2        OUT3 |---- motor_FL
 *        GPIO 11 ----->| IN3        OUT4 |---'
 *        GPIO 10 ----->| IN4             |
 *                      |         nSLEEP  |<--- 3V3 (or a GPIO, see Kconfig)
 *                      |         nFAULT  |---- open drain, unused
 *                      +-----------------+  DRV8833 #1  (the left side)
 *
 *                      +-----------------+
 *        GPIO  9 ----->| IN1        OUT1 |---- motor_RR
 *        GPIO  8 ----->| IN2        OUT2 |---'
 *        GPIO  7 ----->| IN3        OUT3 |---- motor_FR
 *        GPIO 14 ----->| IN4        OUT4 |---'
 *                      +-----------------+  DRV8833 #2  (the right side)
 *
 * IN1/IN2 are the datasheet's AIN1/AIN2 and IN3/IN4 its BIN1/BIN2; breakout
 * boards relabel them, the silicon does not care. Since the remount of
 * 2026-10-29 each motor's own IN1 is the second line of its pair (FL's IN1
 * is driver #1 IN4, RL's is IN2, FR's is driver #2 IN4, RR's is IN2): that
 * swap is what reversed all four. Before it, FL and RL were the other way
 * round on driver #1, FR and RR on driver #2, each with IN1 first. The
 * numbers are Kconfig's (MOTOR_*_IN*_GPIO); this drawing follows them.
 *
 * The split falls along the airframe rather than across it: driver #1 carries
 * the whole left side, driver #2 the whole right. Worth knowing before it
 * happens - one driver in thermal shutdown or fault does not cost a thruster,
 * it costs a side, which arrives as a yaw and roll upset rather than as a
 * loss of thrust.
 *
 * Two things about this arrangement are worth stating plainly, because both
 * are easy to get wrong and expensive to debug in the air:
 *
 * 1. The DRV8833 has no logic supply pin. Its inputs are specified down to
 *    VIH = 1.5 V, so 3.3 V drives them directly with margin, and VM can sit
 *    at 5 V independently. There is nothing to level-shift.
 *
 * 2. nSLEEP must be high or the outputs stay high-impedance and the motors
 *    simply never turn. If it is strapped to 3V3 in hardware there is nothing
 *    to do; if it is on a GPIO, set CONFIG_MOTOR_NSLEEP_GPIO so this driver
 *    raises it - and so that a true "everything off" state exists.
 *
 * Power, and the one failure this layout invites: the motors and the AMS1117
 * that feeds the sensors both hang off the same 5 V step-up. A brushed motor
 * draws several times its running current at break-away, the step-up sags
 * under it, and the AMS1117 needs about 1.1 V of headroom - so once the rail
 * dips near 4.4 V the 3.3 V sensor rail follows it down and the IMU and the
 * barometer brown out at exactly the moment thrust is applied. Bulk
 * capacitance at each DRV8833 VM pin, and a separately decoupled branch for
 * the AMS1117 input, is what keeps a thrust step from looking like a sensor
 * fault. If the IMU drops out when the motors start, this is why.
 *
 * PWM: every one of the eight lines gets its own LEDC channel. The ESP32-S3
 * has exactly eight, so this consumes all of them - nothing else in this
 * firmware may use LEDC. (The status LED is safe; it is driven by RMT.)
 * Spending all eight buys glitch-free reversal: in both decay modes one pin
 * of a pair is static and the other is modulated, but which one is which
 * swaps with direction, so binding a channel to only one pin would mean
 * reconfiguring a pin on every sign change. A blimp reverses thrust
 * constantly, so that trade is the wrong way round.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "cmd_motor.h"
#include "net_console.h"

static const char *TAG = "cmd_motor";

/* The fastest clock LEDC can reach in low-speed mode on this chip. Used only
   to work out how fine the duty resolution is allowed to be. */
#define MOTOR_LEDC_SRC_HZ       80000000ULL

/* Below this the thrust steps become visible as surges at low throttle. */
#define MOTOR_MIN_RES_BITS      8

/* Treated as "stop". Smaller commands than this are noise, not intent. */
#define MOTOR_ZERO_EPS          1e-4f

/* How often the failsafe looks at the clock. Fine enough that the overshoot
   past the configured timeout is small, coarse enough to be free. */
#define MOTOR_FAILSAFE_TICK_MS  50

#define MOTOR_CONSOLE_MAX_MS    30000

typedef struct {
    const char    *name;
    const char    *where;       // driver and input pair, for status output
    int            gpio_in1;
    int            gpio_in2;
    ledc_channel_t ch_in1;
    ledc_channel_t ch_in2;
} motor_pins_t;

/* Order matches motor_id_t and the array taken by motors_set_all(). */
static const motor_pins_t s_pins[MOTOR_COUNT] = {
    { "FL", "#1 IN4/IN3", CONFIG_MOTOR_FL_IN1_GPIO, CONFIG_MOTOR_FL_IN2_GPIO,
      LEDC_CHANNEL_0, LEDC_CHANNEL_1 },
    { "RL", "#1 IN2/IN1", CONFIG_MOTOR_RL_IN1_GPIO, CONFIG_MOTOR_RL_IN2_GPIO,
      LEDC_CHANNEL_2, LEDC_CHANNEL_3 },
    { "FR", "#2 IN4/IN3", CONFIG_MOTOR_FR_IN1_GPIO, CONFIG_MOTOR_FR_IN2_GPIO,
      LEDC_CHANNEL_4, LEDC_CHANNEL_5 },
    { "RR", "#2 IN2/IN1", CONFIG_MOTOR_RR_IN1_GPIO, CONFIG_MOTOR_RR_IN2_GPIO,
      LEDC_CHANNEL_6, LEDC_CHANNEL_7 },
};

static bool              s_ready;
static uint32_t          s_duty_full;        // 2^bits: a true 100%, see below
static int               s_res_bits;
static int               s_freq_hz;
static motor_decay_t     s_decay = MOTOR_DECAY_SLOW;
static float             s_deadband = CONFIG_MOTOR_DEADBAND_PERMILLE / 1000.0f;
static float             s_cmd[MOTOR_COUNT];
/* The throttle each pin pair was last written for by motor_write(), so
   motors_set_all() can leave a bridge alone when its command has not changed.
   NAN after anything that writes the pins some other way (coast, brake, the
   failsafe, init): NAN equals nothing, so the next command always lands. */
static float             s_out[MOTOR_COUNT];
static SemaphoreHandle_t s_lock;

static esp_timer_handle_t s_fs_timer;
static int                s_fs_timeout_ms = CONFIG_MOTOR_FAILSAFE_MS;
static volatile int64_t   s_fs_last_feed_us;
static volatile bool      s_fs_tripped;
/*
 * The failsafe is armed if and only if at least one motor carries a non-zero
 * command. That is the whole of the rule, and both halves matter:
 *
 * - Nothing driven, nothing to guard. At boot, and after any deliberate stop,
 *   the bridges are already coasting; a timer that fired there would log an
 *   error after every boot and after every bounded console run, which is how
 *   a safety mechanism gets trained into background noise - and this is the
 *   one mechanism whose entire value is that it still gets believed.
 * - Something driven, something to guard. A latched duty with no controller
 *   behind it is the failure that accelerates the airship into a wall, and
 *   the PWM peripheral will hold it forever.
 */
static volatile bool      s_fs_armed;

/* ------------------------------------------------------------------ duty */

/*
 * LEDC divides its source clock by (2^bits * N), N >= 1, so the resolution is
 * capped by freq * 2^bits <= source. Take the finest that still fits.
 */
static int pick_resolution_bits(int freq_hz)
{
    for (int bits = SOC_LEDC_TIMER_BIT_WIDTH; bits >= MOTOR_MIN_RES_BITS; bits--) {
        if ((uint64_t)freq_hz << bits <= MOTOR_LEDC_SRC_HZ) {
            return bits;
        }
    }
    return MOTOR_MIN_RES_BITS;
}

/*
 * Write one motor's pin pair from a signed command.
 *
 * The duty register is one bit wider than the resolution - ledc.c documents
 * the range as [0, 2^duty_res] - so 2^bits is a genuine, notch-free 100%.
 * Slow decay depends on that: the static-high pin has to be actually static,
 * not 2047/2048, or every PWM period puts a short glitch into an H-bridge
 * input and the bridge spends part of its life in a state nobody asked for.
 *
 * Call with the lock held.
 */
static void motor_write(motor_id_t m, float throttle)
{
    const motor_pins_t *p = &s_pins[m];
    uint32_t d1, d2;

    float mag = fabsf(throttle);
    if (mag < MOTOR_ZERO_EPS) {
        /* Coast, not brake. A zero command should stop driving the prop, not
           hold it against the airflow: on an airship the freewheeling prop is
           a far smaller disturbance than a braked one, and braking all four
           on every zero crossing would heat the drivers for nothing. */
        d1 = d2 = 0;
    } else {
        if (mag > 1.0f) {
            mag = 1.0f;
        }
        /* Map (0,1] onto [deadband,1]. Without this the bottom slice of the
           controller's output range produces duty that only warms the
           winding, and the plant looks dead until the command clears
           break-away - a nonlinearity the integrator would wind up against. */
        float duty = s_deadband + (1.0f - s_deadband) * mag;
        uint32_t d = (uint32_t)lrintf(duty * (float)s_duty_full);
        if (d > s_duty_full) {
            d = s_duty_full;
        }

        bool fwd = throttle > 0.0f;
        if (s_decay == MOTOR_DECAY_FAST) {
            /* Alternate drive (1,0) and coast (0,0): modulate the leading
               input, hold the other low. */
            d1 = fwd ? d : 0;
            d2 = fwd ? 0 : d;
        } else {
            /* Alternate drive (1,0) and brake (1,1): hold the leading input
               high and give the other the complement, so a larger command
               spends less of the period braking. */
            d1 = fwd ? s_duty_full : s_duty_full - d;
            d2 = fwd ? s_duty_full - d : s_duty_full;
        }
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, p->ch_in1, d1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, p->ch_in2, d2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, p->ch_in1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, p->ch_in2);
    s_out[m] = throttle;
}

/* Both inputs to the same level on every motor: 0 coasts, full brakes. */
static void motor_write_all_static(uint32_t duty)
{
    for (int m = 0; m < MOTOR_COUNT; m++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, s_pins[m].ch_in1, duty);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, s_pins[m].ch_in2, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, s_pins[m].ch_in1);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, s_pins[m].ch_in2);
        s_cmd[m] = 0.0f;
        s_out[m] = NAN;     /* a brake is "0" in s_cmd but not on the pins */
    }
}

/* -------------------------------------------------------------- failsafe */

static void failsafe_tick(void *arg)
{
    (void)arg;
    if (s_fs_timeout_ms <= 0 || s_fs_tripped || !s_fs_armed) {
        return;
    }
    int64_t idle_ms = (esp_timer_get_time() - s_fs_last_feed_us) / 1000;
    if (idle_ms < s_fs_timeout_ms) {
        return;
    }
    /* Write the bridges directly rather than through motors_coast(), which
       re-derives the armed state and would clear the flag we are about to
       set. Disarm first: once the commands are zeroed there is nothing
       latched, so a second tick must not report this again. */
    s_fs_armed = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_write_all_static(0);
    xSemaphoreGive(s_lock);
    s_fs_tripped = true;

    ESP_LOGE(TAG, "failsafe: no thrust command for %lld ms - all motors coasting",
             (long long)idle_ms);
}

/*
 * Restart the clock and re-derive whether there is anything to guard. Called
 * after every path that changes a command, so the armed state is a function
 * of what the bridges are actually doing rather than of which call happened
 * to run last.
 */
static void failsafe_sync(void)
{
    bool driven = false;
    for (int m = 0; m < MOTOR_COUNT; m++) {
        if (fabsf(s_cmd[m]) >= MOTOR_ZERO_EPS) {
            driven = true;
            break;
        }
    }
    s_fs_last_feed_us = esp_timer_get_time();
    s_fs_tripped = false;
    s_fs_armed = driven;
}

void motors_failsafe_feed(void)
{
    failsafe_sync();
}

bool motors_failsafe_tripped(void)
{
    return s_fs_tripped;
}

motor_failsafe_state_t motors_failsafe_state(void)
{
    if (s_fs_timeout_ms <= 0) {
        return MOTOR_FAILSAFE_OFF;
    }
    if (s_fs_tripped) {
        return MOTOR_FAILSAFE_TRIPPED;
    }
    return s_fs_armed ? MOTOR_FAILSAFE_ARMED : MOTOR_FAILSAFE_IDLE;
}

esp_err_t motors_failsafe_set_timeout_ms(int timeout_ms)
{
    if (timeout_ms < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_fs_timeout_ms = timeout_ms;
    failsafe_sync();
    return ESP_OK;
}

/* ------------------------------------------------------------------ init */

esp_err_t motors_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    for (int m = 0; m < MOTOR_COUNT; m++) {
        int pin[2] = { s_pins[m].gpio_in1, s_pins[m].gpio_in2 };
        for (int i = 0; i < 2; i++) {
            if (!GPIO_IS_VALID_OUTPUT_GPIO(pin[i])) {
                ESP_LOGE(TAG, "motor %s: GPIO %d cannot drive an output",
                         s_pins[m].name, pin[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (int m = 0; m < MOTOR_COUNT; m++) {
        s_out[m] = NAN;
    }

    /*
     * Drive all eight low as plain GPIOs before the PWM peripheral touches
     * them. From chip reset to this line the pins are high-impedance inputs
     * and the DRV8833 inputs float - with VM already live off the step-up,
     * that is an undefined bridge state during every boot. Software cannot
     * close that window; only a pulldown on each of the eight IN lines can.
     * Fit them. This code just makes sure the window ends in a known state.
     */
    for (int m = 0; m < MOTOR_COUNT; m++) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_pins[m].gpio_in1) | (1ULL << s_pins[m].gpio_in2),
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "motor %s: gpio_config failed: %s",
                     s_pins[m].name, esp_err_to_name(err));
            return err;
        }
        gpio_set_level(s_pins[m].gpio_in1, 0);
        gpio_set_level(s_pins[m].gpio_in2, 0);
    }

    s_freq_hz = CONFIG_MOTOR_PWM_FREQ_HZ;
    s_res_bits = pick_resolution_bits(s_freq_hz);
    s_duty_full = 1UL << s_res_bits;

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)s_res_bits,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = s_freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config(%d Hz, %d bit) failed: %s",
                 s_freq_hz, s_res_bits, esp_err_to_name(err));
        return err;
    }

    for (int m = 0; m < MOTOR_COUNT; m++) {
        int pin[2] = { s_pins[m].gpio_in1, s_pins[m].gpio_in2 };
        ledc_channel_t ch[2] = { s_pins[m].ch_in1, s_pins[m].ch_in2 };
        for (int i = 0; i < 2; i++) {
            ledc_channel_config_t ccfg = {
                .gpio_num = pin[i],
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = ch[i],
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = LEDC_TIMER_0,
                .duty = 0,                  // coast
                .hpoint = 0,
            };
            err = ledc_channel_config(&ccfg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "motor %s: ledc_channel_config on GPIO %d failed: %s",
                         s_pins[m].name, pin[i], esp_err_to_name(err));
                return err;
            }
        }
    }

    /*
     * Wake the drivers only now that every input is a defined low, so the
     * bridges come out of sleep coasting rather than into whatever the inputs
     * happened to be doing.
     */
#if CONFIG_MOTOR_NSLEEP_GPIO >= 0
    gpio_config_t slp = {
        .pin_bit_mask = 1ULL << CONFIG_MOTOR_NSLEEP_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&slp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nSLEEP on GPIO %d: %s",
                 CONFIG_MOTOR_NSLEEP_GPIO, esp_err_to_name(err));
        return err;
    }
    gpio_set_level(CONFIG_MOTOR_NSLEEP_GPIO, 1);
#endif

    const esp_timer_create_args_t fs_args = {
        .callback = failsafe_tick,
        .name = "motor_fs",
        .dispatch_method = ESP_TIMER_TASK,
    };
    err = esp_timer_create(&fs_args, &s_fs_timer);
    if (err != ESP_OK) {
        return err;
    }
    failsafe_sync();
    err = esp_timer_start_periodic(s_fs_timer, MOTOR_FAILSAFE_TICK_MS * 1000);
    if (err != ESP_OK) {
        return err;
    }

    s_ready = true;

    /* Report the frequency LEDC actually produced, not the one requested -
       the divider is integer and LEDC_AUTO_CLK may not have picked the source
       this code assumed, either of which silently shifts the real carrier. */
    uint32_t actual = ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    for (int m = 0; m < MOTOR_COUNT; m++) {
        ESP_LOGI(TAG, "motor_%s on GPIO %d/%d  (DRV8833 %s)", s_pins[m].name,
                 s_pins[m].gpio_in1, s_pins[m].gpio_in2, s_pins[m].where);
    }
    ESP_LOGI(TAG, "PWM %" PRIu32 " Hz (asked %d), %d bit, %s decay, deadband %.1f%%",
             actual, s_freq_hz, s_res_bits,
             s_decay == MOTOR_DECAY_SLOW ? "slow" : "fast",
             s_deadband * 100.0f);
    if (s_fs_timeout_ms > 0) {
        ESP_LOGI(TAG, "failsafe armed only while a motor is driven, then coasts "
                      "all of them after %d ms without a command", s_fs_timeout_ms);
    } else {
        ESP_LOGW(TAG, "failsafe disabled: a latched command will run forever");
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------- API */

const char *motor_name(motor_id_t m)
{
    return (m < MOTOR_COUNT) ? s_pins[m].name : "??";
}

const char *motor_where(motor_id_t m)
{
    return (m < MOTOR_COUNT) ? s_pins[m].where : "?";
}

esp_err_t motor_set(motor_id_t m, float throttle)
{
    if (m >= MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    /* A NaN or an infinity out of a controller must not reach the duty
       arithmetic, where every comparison below would go false and leave the
       bridge latched at whatever it was last told. */
    if (!isfinite(throttle)) {
        ESP_LOGE(TAG, "motor %s: non-finite throttle rejected", s_pins[m].name);
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (throttle > 1.0f) {
        throttle = 1.0f;
    } else if (throttle < -1.0f) {
        throttle = -1.0f;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cmd[m] = throttle;
    motor_write(m, throttle);
    xSemaphoreGive(s_lock);

    motors_failsafe_feed();
    return ESP_OK;
}

esp_err_t motors_set_all(const float throttle[MOTOR_COUNT])
{
    if (throttle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    for (int m = 0; m < MOTOR_COUNT; m++) {
        if (!isfinite(throttle[m])) {
            ESP_LOGE(TAG, "motor %s: non-finite throttle rejected", s_pins[m].name);
            return ESP_ERR_INVALID_ARG;
        }
    }

    /* One lock for the whole set, so a reader of s_cmd never sees half of a
       four-motor mix applied. The four pin pairs still reach the hardware a
       few microseconds apart, which is far inside one PWM period.
       A bridge whose command has not changed is not written again. The link
       repeats an unchanged command 20 times a second as its keep-alive
       (net_link.c); rewriting the same duties changes nothing on the pins,
       since the LEDC only latches a duty at the next period, so it was work
       for nothing on the link task. The failsafe is fed either way: a repeat
       is still a command. */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int m = 0; m < MOTOR_COUNT; m++) {
        float t = throttle[m];
        if (t > 1.0f) {
            t = 1.0f;
        } else if (t < -1.0f) {
            t = -1.0f;
        }
        s_cmd[m] = t;
        if (t != s_out[m]) {
            motor_write((motor_id_t)m, t);
        }
    }
    xSemaphoreGive(s_lock);

    motors_failsafe_feed();
    return ESP_OK;
}

float motor_get(motor_id_t m)
{
    return (m < MOTOR_COUNT) ? s_cmd[m] : 0.0f;
}

void motors_coast(void)
{
    if (!s_ready) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_write_all_static(0);
    xSemaphoreGive(s_lock);
    failsafe_sync();        // nothing driven now, so nothing to guard
}

void motors_brake(void)
{
    if (!s_ready) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_write_all_static(s_duty_full);
    xSemaphoreGive(s_lock);
    motors_failsafe_feed();
}

esp_err_t motors_set_decay(motor_decay_t mode)
{
    if (mode != MOTOR_DECAY_SLOW && mode != MOTOR_DECAY_FAST) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        s_decay = mode;
        return ESP_OK;
    }
    /* Re-apply the standing commands so the change is visible immediately
       rather than at whatever time the next command happens to arrive. */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_decay = mode;
    for (int m = 0; m < MOTOR_COUNT; m++) {
        motor_write((motor_id_t)m, s_cmd[m]);
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t motors_set_deadband(float deadband)
{
    if (!isfinite(deadband) || deadband < 0.0f || deadband > 0.9f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        s_deadband = deadband;
        return ESP_OK;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_deadband = deadband;
    for (int m = 0; m < MOTOR_COUNT; m++) {
        motor_write((motor_id_t)m, s_cmd[m]);
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

float motors_get_deadband(void)
{
    return s_deadband;
}

/* --------------------------------------------------------------- console */

static struct {
    struct arg_str *motor;
    struct arg_dbl *throttle;
    struct arg_int *duration;
    struct arg_lit *hold;
    struct arg_lit *stop;
    struct arg_lit *brake;
    struct arg_lit *sweep;
    struct arg_str *decay;
    struct arg_dbl *deadband;
    struct arg_int *failsafe;
    struct arg_lit *status;
    struct arg_end *end;
} motor_args;

/* Accepts "fl", "rl", "fr", "rr" and plain indices 0..3. */
static int parse_motor_id(const char *s)
{
    for (int m = 0; m < MOTOR_COUNT; m++) {
        if (strcasecmp(s, s_pins[m].name) == 0) {
            return m;
        }
    }
    if (s[0] >= '0' && s[0] <= '3' && s[1] == '\0') {
        return s[0] - '0';
    }
    return -1;
}

static void print_status(void)
{
    printf("PWM %" PRIu32 " Hz, %d bit (%" PRIu32 " steps), %s decay, "
           "deadband %.1f%%\n",
           ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0), s_res_bits,
           s_duty_full, s_decay == MOTOR_DECAY_SLOW ? "slow" : "fast",
           s_deadband * 100.0f);
    if (s_fs_timeout_ms <= 0) {
        printf("failsafe off\n");
    } else if (s_fs_tripped) {
        printf("failsafe %d ms  *** TRIPPED ***\n", s_fs_timeout_ms);
    } else if (s_fs_armed) {
        printf("failsafe %d ms, armed\n", s_fs_timeout_ms);
    } else {
        printf("failsafe %d ms, idle - nothing is being driven\n",
               s_fs_timeout_ms);
    }
#if CONFIG_MOTOR_NSLEEP_GPIO >= 0
    printf("nSLEEP driven high on GPIO %d\n", CONFIG_MOTOR_NSLEEP_GPIO);
#else
    printf("nSLEEP not on a GPIO - must be strapped high in hardware\n");
#endif
    printf("\n%-6s %-12s %-9s %10s %10s\n",
           "motor", "driver", "GPIO", "command", "duty");
    for (int m = 0; m < MOTOR_COUNT; m++) {
        char pins[12];
        snprintf(pins, sizeof(pins), "%d/%d", s_pins[m].gpio_in1, s_pins[m].gpio_in2);
        float mag = fabsf(s_cmd[m]);
        float duty = (mag < MOTOR_ZERO_EPS) ? 0.0f
                   : (s_deadband + (1.0f - s_deadband) * mag) * 100.0f;
        printf("%-6s %-12s %-9s %+10.3f %9.1f%%\n",
               s_pins[m].name, s_pins[m].where, pins, s_cmd[m], duty);
    }
}

/* Hold a command for ms, keeping the failsafe fed, then coast. */
static void run_for(int ms)
{
    const int slice = 100;
    for (int t = 0; t < ms; t += slice) {
        vTaskDelay(pdMS_TO_TICKS((ms - t < slice) ? ms - t : slice));
        motors_failsafe_feed();
    }
    motors_coast();
}

/*
 * Ramp one motor from rest in equal duty steps, printing each, so the duty at
 * which the prop breaks away can simply be read off. That number is the
 * deadband: feed it back with --deadband.
 *
 * The deadband mapping is removed for the duration, because measuring
 * break-away through the correction that is supposed to compensate for it
 * would just return the correction.
 */
static void run_sweep(motor_id_t m, bool reverse, int dwell_ms)
{
    const int steps = 20;
    float saved = s_deadband;

    motors_set_deadband(0.0f);
    printf("Ramping motor_%s %s in %d steps, %d ms each.\n",
           s_pins[m].name, reverse ? "reverse" : "forward", steps, dwell_ms);
    printf("Watch the prop and note the first step where it turns on its own.\n\n");
    printf("%6s %8s\n", "step", "duty");

    for (int i = 1; i <= steps; i++) {
        float frac = (float)i / steps;
        motor_set(m, reverse ? -frac : frac);
        printf("%6d %7.1f%%\n", i, frac * 100.0f);
        fflush(stdout);
        for (int t = 0; t < dwell_ms; t += 100) {
            vTaskDelay(pdMS_TO_TICKS(100));
            motors_failsafe_feed();
        }
    }
    motors_coast();
    motors_set_deadband(saved);
    printf("\nStopped. Set the break-away duty with: motor --deadband <percent>\n");
}

static int do_motor_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&motor_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, motor_args.end, argv[0]);
        return 1;
    }

    esp_err_t err = motors_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor driver not available: %s", esp_err_to_name(err));
        return 1;
    }

    /* Over the air the failsafe is the one timer a console-driven motor has:
       the link timeout watches MOTOR_SET, not this console (net_link.c). So
       it is changed at the desk only, and --hold over the air needs it on.
       Before 2026-10-29 'motor --failsafe 0' and then '--hold' over UDP ran
       a motor with no timer at all (docs/link-robustness-review.md, F4). */
    const bool remote = net_console_is_caller();
    if (remote && motor_args.failsafe->count) {
        printf("--failsafe is refused over the air; change it on the USB console\n");
        return 1;
    }
    if (remote && motor_args.hold->count && s_fs_timeout_ms <= 0) {
        printf("--hold is refused over the air while the failsafe is off\n");
        return 1;
    }

    /* Settings first, so they can be combined with a run in one line. */
    if (motor_args.decay->count) {
        const char *d = motor_args.decay->sval[0];
        motor_decay_t mode;
        if (strcasecmp(d, "slow") == 0) {
            mode = MOTOR_DECAY_SLOW;
        } else if (strcasecmp(d, "fast") == 0) {
            mode = MOTOR_DECAY_FAST;
        } else {
            ESP_LOGE(TAG, "--decay takes 'slow' or 'fast'");
            return 1;
        }
        motors_set_decay(mode);
        printf("decay mode: %s\n", d);
    }
    if (motor_args.deadband->count) {
        float db = (float)motor_args.deadband->dval[0] / 100.0f;
        if (motors_set_deadband(db) != ESP_OK) {
            ESP_LOGE(TAG, "--deadband takes 0..90 (percent)");
            return 1;
        }
        printf("deadband: %.1f%%\n", db * 100.0f);
    }
    if (motor_args.failsafe->count) {
        int ms = motor_args.failsafe->ival[0];
        if (motors_failsafe_set_timeout_ms(ms) != ESP_OK) {
            ESP_LOGE(TAG, "--failsafe takes 0 (off) or a positive ms");
            return 1;
        }
        printf("failsafe: %d ms%s\n", ms, ms == 0 ? " (off)" : "");
    }

    if (motor_args.stop->count) {
        motors_coast();
        printf("all motors coasting\n");
        return 0;
    }
    if (motor_args.brake->count) {
        motors_brake();
        printf("all motors braked\n");
        return 0;
    }

    int duration = motor_args.duration->count ? motor_args.duration->ival[0] : 1500;
    if (duration < 0 || duration > MOTOR_CONSOLE_MAX_MS) {
        ESP_LOGE(TAG, "--duration must be 0..%d ms", MOTOR_CONSOLE_MAX_MS);
        return 1;
    }

    if (motor_args.sweep->count || motor_args.throttle->count) {
        if (!motor_args.motor->count) {
            ESP_LOGE(TAG, "which motor? use -m FL|RL|FR|RR");
            return 1;
        }
        int m = parse_motor_id(motor_args.motor->sval[0]);
        if (m < 0) {
            ESP_LOGE(TAG, "unknown motor '%s'; use FL, RL, FR or RR",
                     motor_args.motor->sval[0]);
            return 1;
        }

        if (motor_args.sweep->count) {
            bool rev = motor_args.throttle->count && motor_args.throttle->dval[0] < 0;
            run_sweep((motor_id_t)m, rev, duration ? duration : 1000);
            return 0;
        }

        float t = (float)motor_args.throttle->dval[0];
        if (!isfinite(t) || t < -1.0f || t > 1.0f) {
            ESP_LOGE(TAG, "--throttle must be -1.0 .. 1.0");
            return 1;
        }
        if (motor_set((motor_id_t)m, t) != ESP_OK) {
            return 1;
        }

        if (motor_args.hold->count) {
            /* The REPL cannot interrupt a running command, so a held motor
               can only be stopped by the next command line - which is fine at
               a desk and is why it is not the default. */
            printf("motor_%s at %+.3f, held. Stop it with 'motor -s'.\n",
                   s_pins[m].name, t);
            if (s_fs_timeout_ms > 0) {
                printf("note: the %d ms failsafe will coast it; "
                       "disable with --failsafe 0 to really hold it.\n",
                       s_fs_timeout_ms);
            }
        } else {
            printf("motor_%s at %+.3f for %d ms...\n", s_pins[m].name, t, duration);
            fflush(stdout);
            run_for(duration);
            printf("stopped\n");
        }
        return 0;
    }

    /* Nothing to do but report - also what a bare 'motor' should print. */
    print_status();
    return 0;
}

void register_motor(void)
{
    motor_args.motor    = arg_str0("m", "motor", "<id>", "Motor: FL, RL, FR or RR");
    motor_args.throttle = arg_dbl0("t", "throttle", "<-1..1>", "Signed thrust; 0 coasts");
    motor_args.duration = arg_int0("d", "duration", "<ms>", "Run time then auto-stop (default 1500)");
    motor_args.hold     = arg_lit0(NULL, "hold", "Leave it running instead of auto-stopping");
    motor_args.stop     = arg_lit0("s", "stop", "Coast all four motors");
    motor_args.brake    = arg_lit0("b", "brake", "Brake all four motors");
    motor_args.sweep    = arg_lit0(NULL, "sweep", "Ramp one motor to find its break-away duty");
    motor_args.decay    = arg_str0(NULL, "decay", "<slow|fast>", "PWM decay mode (default slow)");
    motor_args.deadband = arg_dbl0(NULL, "deadband", "<percent>", "Break-away duty, 0..90");
    motor_args.failsafe = arg_int0(NULL, "failsafe", "<ms>", "Coast if idle this long; 0 disables");
    motor_args.status   = arg_lit0(NULL, "status", "Print pins, PWM and standing commands");
    motor_args.end      = arg_end(2);

    const esp_console_cmd_t motor_cmd = {
        .command = "motor",
        .help = "Drive the two DRV8833 motor drivers",
        .hint = NULL,
        .func = &do_motor_cmd,
        .argtable = &motor_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&motor_cmd));
}

/* motor.c - rotor thrust driver, LEDC PWM into four low-side MOSFET gates.
 *
 * All four channels share one LEDC timer (board_config.h: timer 0, 20 kHz,
 * 10-bit) and get one ledc_channel_config_t each (channels 0..3, duty 0,
 * hpoint 0 at init - commanded zero thrust, the safe power-up state).
 *
 * UNIDIRECTIONAL ONLY: these are brushed coreless motors driven low-side by
 * a single N-MOSFET gate each (no H-bridge) - duty 0..100% is thrust
 * 0..motor_max_thrust_n, there is no reverse and no bidirectional channel on
 * this airframe (contrast the old shared-tilt-bicopter design this replaces).
 *
 * THRUST -> DUTY CALIBRATION CURVE IS A PLACEHOLDER. A brushed motor's
 * static thrust is roughly quadratic in duty cycle (thrust ~ duty^2, since
 * thrust scales with speed^2 and speed is roughly proportional to applied
 * voltage/duty for a fixed-voltage PWM drive) - so duty = sqrt(thrust_frac)
 * is the naive inverse used below, offset by a deadband because these
 * motors do not turn at all below some small duty (static friction/cogging -
 * see board_config.h's BLIMP_MOTOR_DUTY_DEADBAND comment). This is an
 * UNMEASURED ENGINEERING GUESS, not a fit to real data. It MUST be replaced
 * by a measured lookup table from a thrust-stand sweep (duty -> thrust for
 * this exact motor/prop/battery-voltage combination) before this curve is
 * trusted for anything beyond "spins in roughly the right direction".
 */
#include "motor.h"

#include <math.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"

static const char *TAG = "motor";

/* Index convention throughout this file: 0=FL, 1=FR, 2=RL, 3=RR. */
static const int s_motor_gpio[4] = {
    BLIMP_MOTOR_FL_GPIO, BLIMP_MOTOR_FR_GPIO, BLIMP_MOTOR_RL_GPIO, BLIMP_MOTOR_RR_GPIO,
};
static const ledc_channel_t s_motor_channel[4] = {
    (ledc_channel_t)BLIMP_LEDC_MOTOR_CH_FL, (ledc_channel_t)BLIMP_LEDC_MOTOR_CH_FR,
    (ledc_channel_t)BLIMP_LEDC_MOTOR_CH_RL, (ledc_channel_t)BLIMP_LEDC_MOTOR_CH_RR,
};
static const char *s_motor_name[4] = { "FL", "FR", "RL", "RR" };

static bool s_ready;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* PLACEHOLDER calibration curve - see this file's header comment. Always
 * sanitises NaN and clamps to [0,1]; never returns more than 1.0 (100% duty)
 * regardless of input. */
static float thrust_to_duty(float thrust_n, float motor_max_thrust_n)
{
    if (!(thrust_n == thrust_n)) {   /* NaN != NaN, portable IEEE-754 test */
        return 0.0f;
    }
    if (thrust_n <= 0.0f) {
        return 0.0f;
    }
    if (!(motor_max_thrust_n > 0.0f)) {
        return 0.0f;
    }
    const float frac = clampf(thrust_n / motor_max_thrust_n, 0.0f, 1.0f);
    const float duty = BLIMP_MOTOR_DUTY_DEADBAND +
        (1.0f - BLIMP_MOTOR_DUTY_DEADBAND) * sqrtf(frac);
    return clampf(duty, 0.0f, 1.0f);
}

static uint32_t duty_frac_to_count(float duty_0_to_1)
{
    const uint32_t max_duty = (1u << BLIMP_LEDC_MOTOR_RES_BITS) - 1u;
    return (uint32_t)(duty_0_to_1 * (float)max_duty + 0.5f);
}

esp_err_t blimp_motor_init(void)
{
    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)BLIMP_LEDC_MOTOR_RES_BITS,
        .timer_num       = (ledc_timer_t)BLIMP_LEDC_MOTOR_TIMER,
        .freq_hz         = BLIMP_LEDC_MOTOR_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d "
                       "(20 kHz / 10-bit pair may not be achievable on this "
                       "target - see BLIMP_LEDC_MOTOR_* in board_config.h)",
                 (int)err);
        return err;
    }

    for (int i = 0; i < 4; i++) {
        const ledc_channel_config_t ch_cfg = {
            .gpio_num   = s_motor_gpio[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = s_motor_channel[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = (ledc_timer_t)BLIMP_LEDC_MOTOR_TIMER,
            .duty       = 0,
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ch_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed for motor %s (GPIO%d): %d",
                     s_motor_name[i], s_motor_gpio[i], (int)err);
            return err;
        }
    }

    s_ready = true;
    return ESP_OK;
}

esp_err_t blimp_motor_set_duty_raw(int motor_index, float duty_0_to_1)
{
    if (motor_index < 0 || motor_index > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!(duty_0_to_1 == duty_0_to_1)) {   /* NaN */
        duty_0_to_1 = 0.0f;
    }
    const float duty = clampf(duty_0_to_1, 0.0f, 1.0f);   /* never > 100% */
    const uint32_t duty_count = duty_frac_to_count(duty);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motor_channel[motor_index], duty_count);
    if (err != ESP_OK) { return err; }
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motor_channel[motor_index]);
}

esp_err_t blimp_motor_apply(const blimp_params_t *p, const blimp_actuators_t *act)
{
    if (!p || !act) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const float duties[4] = {
        thrust_to_duty(act->thrust_fl, p->motor_max_thrust_n),
        thrust_to_duty(act->thrust_fr, p->motor_max_thrust_n),
        thrust_to_duty(act->thrust_rl, p->motor_max_thrust_n),
        thrust_to_duty(act->thrust_rr, p->motor_max_thrust_n),
    };

    for (int i = 0; i < 4; i++) {
        const esp_err_t err = blimp_motor_set_duty_raw(i, duties[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t blimp_motor_all_stop(void)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < 4; i++) {
        const esp_err_t err = blimp_motor_set_duty_raw(i, 0.0f);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

void blimp_motor_identify_forever(void)
{
    ESP_LOGW(TAG, "=====================================================");
    ESP_LOGW(TAG, "MOTOR IDENTIFICATION MODE - REMOVE PROPELLERS OR");
    ESP_LOGW(TAG, "PHYSICALLY RESTRAIN THE VEHICLE BEFORE CONTINUING.");
    ESP_LOGW(TAG, "Each motor will be spun ALONE at 20%% duty for 1500 ms,");
    ESP_LOGW(TAG, "in order FL, FR, RL, RR, forever. Write down which");
    ESP_LOGW(TAG, "physical motor moves for each name.");
    ESP_LOGW(TAG, "=====================================================");
    vTaskDelay(pdMS_TO_TICKS(5000));   /* safety pause before the first motor
                                       * moves, so a person reading the log
                                       * over serial has time to react */

    for (;;) {
        for (int i = 0; i < 4; i++) {
            ESP_LOGW(TAG, "motor %s (GPIO%d) ON at 20%%", s_motor_name[i], s_motor_gpio[i]);
            (void)blimp_motor_set_duty_raw(i, 0.20f);
            vTaskDelay(pdMS_TO_TICKS(1500));

            (void)blimp_motor_set_duty_raw(i, 0.0f);
            ESP_LOGI(TAG, "motor %s (GPIO%d) OFF", s_motor_name[i], s_motor_gpio[i]);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

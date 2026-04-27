/*
 * hmi_app.c
 *
 * Single-page test HMI for IMU, motor, encoder and servo bring-up.
 */

#include "hmi_app.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "balance_app.h"
#include "imu_app.h"
#include "motor_app.h"
#include "scheduler.h"
#include "servo_app.h"

#define HMI_PAGE_SWITCH_PIN         (P33_11)
#define HMI_MOTOR_SWITCH_PIN        (P33_12)

#define HMI_TASK_PERIOD_MS          (10U)
#define HMI_DISPLAY_PERIOD_MS       (100U)
#define HMI_TELEMETRY_PERIOD_MS     (10U)
#define HMI_DRIVER_REFRESH_MS       (50U)

#define HMI_SPEED_STEP              (50)
#define HMI_SERVO_TEST_STEP         (5)

#define HMI_LINE_COUNT              (8U)
#define HMI_LINE_HEIGHT             (16U)
#define HMI_LINE_BUFFER_SIZE        (48U)
#define HMI_MAX_VISIBLE_CHARS       (30U)

/* 按住自动重复：首次按下立即触发，之后每 200ms 重发一次 */
#define HMI_KEY_DEBOUNCE_MS         (200U)

#define HMI_YAW_LOCK_DELAY_MS   (500U)

#define HMI_EXP_PHASE_MS        (2000U)
#define HMI_EXP_PHASE_S         (HMI_EXP_PHASE_MS / 1000U)
#define HMI_EXP_PHASE_COUNT     (8U)

/* Yaw rate (deg/s) for each phase. Positive = right turn (yaw increases).
 * Equivalent to "1° increment every N ms" where N = 1000/rate. */
static const float hmi_exp_yaw_rate_table[HMI_EXP_PHASE_COUNT] =
{
    0.0f, 10.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f, 130.0f
};

static uint8  hmi_display_mode = 0U;
static uint8  hmi_display_mode_last = 0xFFU;
static uint8  hmi_motor_enabled = 0U;
static int16  hmi_servo_test_offset = 0;
static uint32 hmi_last_display_tick = 0U;
static uint32 hmi_last_telemetry_tick = 0U;
static uint32 hmi_last_driver_refresh_tick = 0U;
static uint8  hmi_yaw_lock_pending = 0U;
static uint32 hmi_yaw_lock_tick = 0U;
static uint8  hmi_experiment_active = 0U;
static uint32 hmi_experiment_start_tick = 0U;
static uint8  hmi_experiment_phase = 0U;
static float  hmi_exp_yaw_integrated = 0.0f;
static uint32 hmi_exp_last_tick = 0U;
static uint8  hmi_exp_integrator_initialized = 0U;

static const gpio_pin_enum hmi_key_pins[KEY_NUMBER] = KEY_LIST;
static uint32 hmi_key_last_trigger[KEY_NUMBER] = {0U};

static uint8 hmi_switch_active(gpio_pin_enum pin)
{
    return (0 == gpio_get_level(pin)) ? 1U : 0U;
}

static void hmi_sendf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length > 0)
    {
        buffer[sizeof(buffer) - 1U] = '\0';
        wireless_uart_send_string(buffer);
    }
}

static void hmi_show_line(uint8 line, const char *format, ...)
{
    char line_buffer[HMI_LINE_BUFFER_SIZE];
    char padded_buffer[HMI_LINE_BUFFER_SIZE];
    va_list args;

    memset(line_buffer, 0, sizeof(line_buffer));
    memset(padded_buffer, ' ', sizeof(padded_buffer));
    padded_buffer[sizeof(padded_buffer) - 1U] = '\0';

    va_start(args, format);
    vsnprintf(line_buffer, sizeof(line_buffer), format, args);
    va_end(args);

    snprintf(padded_buffer, sizeof(padded_buffer), "%-30.30s", line_buffer);
    ips114_show_string(0, (uint16)(line * HMI_LINE_HEIGHT), padded_buffer);
}

static void hmi_limit_servo_test_offset(void)
{
    int16 max_positive = (int16)(l_max - mid);
    int16 max_negative = (int16)(r_max - mid);

    if (hmi_servo_test_offset > max_positive)
    {
        hmi_servo_test_offset = max_positive;
    }
    if (hmi_servo_test_offset < max_negative)
    {
        hmi_servo_test_offset = max_negative;
    }
}

static void hmi_apply_servo_test_mode(void)
{
    balance_set_servo_test_offset(hmi_servo_test_offset);

    if ((!hmi_motor_enabled) && (0 != hmi_servo_test_offset))
    {
        balance_set_servo_test_enabled(1U);
    }
    else
    {
        balance_set_servo_test_enabled(0U);
    }
}

/* 按键事件：按下 + 距上次触发至少 HMI_KEY_DEBOUNCE_MS 返回 1。
 * 换掉了 zf_device_key 的状态机——那套要在"松手那一瞬"才把 SHORT_PRESS 写进状态，
 * hmi scan 错过那 10ms 窗口事件就丢了（这是 "按键失灵" 的根源）。
 * 现在改成按下即触发，按住会每 200ms 自动重发一次，丢不了。*/
static uint8 hmi_key_edge(key_index_enum key)
{
    if ((0 == gpio_get_level(hmi_key_pins[key])) &&
        ((uwtick - hmi_key_last_trigger[key]) >= HMI_KEY_DEBOUNCE_MS))
    {
        hmi_key_last_trigger[key] = uwtick;
        return 1U;
    }
    return 0U;
}

static void hmi_handle_keys(void)
{
    if (hmi_key_edge(KEY_1))
    {
        motor_set_target_speed(motor_target_speed + HMI_SPEED_STEP);
    }

    if (hmi_key_edge(KEY_2))
    {
        int16 new_speed = motor_target_speed - HMI_SPEED_STEP;
        if (new_speed < 0) { new_speed = 0; }
        motor_set_target_speed(new_speed);
    }
}

static void hmi_update_inputs(void)
{
    uint8 new_motor_enabled;

    hmi_display_mode = hmi_switch_active(HMI_PAGE_SWITCH_PIN);

    new_motor_enabled = hmi_switch_active(HMI_MOTOR_SWITCH_PIN);

    if (new_motor_enabled && !hmi_motor_enabled)
    {
        motor_set_enabled(1U);
        if (motor_target_speed == 0)
        {
            motor_set_target_speed(559);
        }
        hmi_yaw_lock_pending = 1U;
        hmi_yaw_lock_tick = uwtick;

        hmi_experiment_active = 1U;
        hmi_experiment_start_tick = uwtick;
        hmi_experiment_phase = 0U;
        hmi_exp_integrator_initialized = 0U;
        balance_fallen = 0U;
        balance_set_steering_active(1U);
        balance_set_expect_angle(0.0f);
    }
    else if (!new_motor_enabled && hmi_motor_enabled)
    {
        motor_set_enabled(0U);
        hmi_experiment_active = 0U;
        balance_set_steering_active(1U);
        balance_set_expect_angle(0.0f);
    }

    hmi_motor_enabled = new_motor_enabled;
}

static void hmi_run_experiment(void)
{
    uint32 elapsed;
    uint32 phase;
    uint32 dt_ms;
    float dt_s;
    float rate;

    if (!hmi_experiment_active || balance_fallen)
    {
        return;
    }

    elapsed = uwtick - hmi_experiment_start_tick;
    phase = elapsed / HMI_EXP_PHASE_MS;
    if (phase >= HMI_EXP_PHASE_COUNT)
    {
        phase = HMI_EXP_PHASE_COUNT - 1U;
    }

    if ((uint8)phase != hmi_experiment_phase)
    {
        hmi_experiment_phase = (uint8)phase;
    }

    if (hmi_experiment_phase == 0U)
    {
        return;
    }

    if (!hmi_exp_integrator_initialized)
    {
        hmi_exp_yaw_integrated = yaw_target;
        hmi_exp_last_tick = uwtick;
        hmi_exp_integrator_initialized = 1U;
        return;
    }

    dt_ms = uwtick - hmi_exp_last_tick;
    hmi_exp_last_tick = uwtick;
    if (dt_ms > 100U)
    {
        return;
    }

    dt_s = (float)dt_ms * 0.001f;
    rate = hmi_exp_yaw_rate_table[hmi_experiment_phase];
    hmi_exp_yaw_integrated = normalize_angle(hmi_exp_yaw_integrated + rate * dt_s);
    balance_set_yaw_target_now(hmi_exp_yaw_integrated);
}

static void hmi_update_display(void)
{
    uint8 status_ok = (imu_ready && (!hmi_motor_enabled || motor_driver_online)) ? 1U : 0U;
    uint8 i;
    char marker;

    if (balance_fallen)
    {
        return;
    }

    if (hmi_display_mode != hmi_display_mode_last)
    {
        ips114_clear();
        hmi_display_mode_last = hmi_display_mode;
    }

    if (0U == hmi_display_mode)
    {
        hmi_show_line(0, "ROLL:%7.2f PIT:%7.2f", roll, pitch);
        hmi_show_line(1, "YAW:%6.2f YE:%6.2f YT:%5.1f", yaw, yaw_error, yaw_target);
        hmi_show_line(2, "M:%u TS:%5d AS:%5d", motor_enabled, motor_target_speed, motor_actual_speed);
        hmi_show_line(3, "ENC :%6d TOT:%6ld", encoder_physical, (long)encoder_physical_total);
        hmi_show_line(4, "AFB:%6.2f STR:%5.2f", balance_angle_feedback, steering_pid.out);
        hmi_show_line(5, "TG_A:%6.2f TG_G:%6.2f", target_angle, target_gyro);
        hmi_show_line(6, "OUT :%6.2f PWM:%5lu", servo_output, (unsigned long)servo_last_duty);
        hmi_show_line(7, "PH:%u R:%+6.1f L:%+5.1f %s",
                      hmi_experiment_phase,
                      hmi_exp_yaw_rate_table[hmi_experiment_phase],
                      steering_pid.out,
                      status_ok ? "OK" : "NG");
    }
    else
    {
        for (i = 0U; i < HMI_EXP_PHASE_COUNT; i++)
        {
            marker = (hmi_experiment_active && (i == hmi_experiment_phase)) ? '>' : ' ';
            hmi_show_line(i, "%cPH%u %+6.1f/s %u-%us",
                          marker, i, hmi_exp_yaw_rate_table[i],
                          (unsigned)(i * HMI_EXP_PHASE_S),
                          (unsigned)((i + 1U) * HMI_EXP_PHASE_S));
        }
    }
}

static void hmi_send_telemetry(void)
{
    hmi_sendf("t,ph,yr,pit,afb,tang,gy,fgy,tgy,out,pwm,ts,as,yaw,yt,ye,str:"
              "%lu,%u,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.0f,%lu,%d,%d,%.1f,%.1f,%.1f,%.2f\r\n",
              (unsigned long)uwtick, hmi_experiment_phase,
              hmi_exp_yaw_rate_table[hmi_experiment_phase],
              pitch, balance_angle_feedback,
              target_angle, gyro_y_rate, balance_filtered_gyro,
              target_gyro, servo_output, (unsigned long)servo_last_duty,
              motor_target_speed, motor_actual_speed,
              yaw, yaw_target, yaw_error, steering_pid.out);
}

void hmi_init(void)
{
    gpio_init(HMI_PAGE_SWITCH_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(HMI_MOTOR_SWITCH_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    key_init(HMI_TASK_PERIOD_MS);

    hmi_update_inputs();
    motor_set_enabled(0U);
    hmi_apply_servo_test_mode();
    hmi_last_display_tick = uwtick;
    hmi_last_telemetry_tick = uwtick;
    hmi_last_driver_refresh_tick = uwtick;
    hmi_display_mode_last = 0xFFU;

    hmi_update_display();
    hmi_send_telemetry();
}

void hmi_proc(void)
{
    hmi_update_inputs();
    hmi_handle_keys();
    hmi_apply_servo_test_mode();

    if (hmi_yaw_lock_pending && ((uwtick - hmi_yaw_lock_tick) >= HMI_YAW_LOCK_DELAY_MS))
    {
        balance_lock_yaw_target();
        hmi_yaw_lock_pending = 0U;
    }

    hmi_run_experiment();

    if ((uwtick - hmi_last_driver_refresh_tick) >= HMI_DRIVER_REFRESH_MS)
    {
        hmi_last_driver_refresh_tick = uwtick;
        motor_refresh_status();
    }

    if ((uwtick - hmi_last_display_tick) >= HMI_DISPLAY_PERIOD_MS)
    {
        hmi_last_display_tick = uwtick;
        hmi_update_display();
    }

    if ((uwtick - hmi_last_telemetry_tick) >= HMI_TELEMETRY_PERIOD_MS)
    {
        hmi_last_telemetry_tick = uwtick;
        hmi_send_telemetry();
    }
}

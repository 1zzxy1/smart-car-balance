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
#define HMI_ANGLE_KP_STEP           (0.1f)
#define HMI_ANGLE_KD_STEP           (0.05f)
#define HMI_ANGLE_KD_MIN            (-2.0f)
#define HMI_ANGLE_KD_MAX            (1.0f)
#define HMI_TURN_ANGLE_COARSE_STEP  (0.5f)
#define HMI_RUN_SPEED_STEP_MPS      (0.10f)

#define HMI_LINE_COUNT              (8U)
#define HMI_LINE_HEIGHT             (16U)
#define HMI_LINE_BUFFER_SIZE        (48U)
#define HMI_MAX_VISIBLE_CHARS       (30U)

/* Trigger once on press, then repeat every 200 ms while held. */
#define HMI_KEY_DEBOUNCE_MS         (200U)

static uint8  hmi_display_mode = 0U;
static uint8  hmi_display_mode_last = 0xFFU;
static uint8  hmi_motor_enabled = 0U;
static int16  hmi_servo_test_offset = 0;
static uint32 hmi_last_display_tick = 0U;
static uint32 hmi_last_telemetry_tick = 0U;
static uint32 hmi_last_driver_refresh_tick = 0U;

static const gpio_pin_enum hmi_key_pins[KEY_NUMBER] = KEY_LIST;
static uint32 hmi_key_last_trigger[KEY_NUMBER] = {0U};

float scheduler_get_mission_open_turn_angle(void);
float scheduler_get_mission_run_speed_mps(void);
void scheduler_adjust_mission_open_turn_angle(float delta);
void scheduler_adjust_mission_run_speed_mps(float delta);
float scheduler_get_mission_turn_delta(void);
float scheduler_get_mission_turn_progress(void);
float scheduler_get_mission_turn_remaining(void);

static uint8 hmi_switch_active(gpio_pin_enum pin)
{
    return (0 == gpio_get_level(pin)) ? 1U : 0U;
}

static void hmi_sendf(const char *format, ...)
{
    char buffer[384];
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
        if (0U == hmi_display_mode)
        {
            angle_pid.kp += HMI_ANGLE_KP_STEP;
        }
        else
        {
            scheduler_adjust_mission_open_turn_angle(HMI_TURN_ANGLE_COARSE_STEP);
        }
    }

    if (hmi_key_edge(KEY_2))
    {
        if (0U == hmi_display_mode)
        {
            angle_pid.kp -= HMI_ANGLE_KP_STEP;
            if (angle_pid.kp < 0.0f) { angle_pid.kp = 0.0f; }
        }
        else
        {
            scheduler_adjust_mission_open_turn_angle(-HMI_TURN_ANGLE_COARSE_STEP);
        }
    }

    if (hmi_key_edge(KEY_3))
    {
        if (0U == hmi_display_mode)
        {
            angle_pid.kd += HMI_ANGLE_KD_STEP;
            if (angle_pid.kd > HMI_ANGLE_KD_MAX) { angle_pid.kd = HMI_ANGLE_KD_MAX; }
        }
        else
        {
            scheduler_adjust_mission_run_speed_mps(HMI_RUN_SPEED_STEP_MPS);
        }
    }

    if (hmi_key_edge(KEY_4))
    {
        if (0U == hmi_display_mode)
        {
            angle_pid.kd -= HMI_ANGLE_KD_STEP;
            if (angle_pid.kd < HMI_ANGLE_KD_MIN) { angle_pid.kd = HMI_ANGLE_KD_MIN; }
        }
        else
        {
            scheduler_adjust_mission_run_speed_mps(-HMI_RUN_SPEED_STEP_MPS);
        }
    }
}

static void hmi_update_inputs(void)
{
    uint8 new_motor_enabled;

    hmi_display_mode = hmi_switch_active(HMI_PAGE_SWITCH_PIN);

    new_motor_enabled = hmi_switch_active(HMI_MOTOR_SWITCH_PIN);

    if (new_motor_enabled && !hmi_motor_enabled)
    {
        balance_lock_angle_zero();
        balance_lock_yaw_target();
        motor_set_enabled(1U);
        if (motor_target_speed == 0)
        {
            motor_set_target_speed(559);
        }
    }
    else if (!new_motor_enabled && hmi_motor_enabled)
    {
        motor_set_enabled(0U);
    }

    hmi_motor_enabled = new_motor_enabled;
}

static void hmi_update_display(void)
{
    if (hmi_display_mode != hmi_display_mode_last)
    {
        ips114_clear();
        hmi_display_mode_last = hmi_display_mode;
    }

    if (0U == hmi_display_mode)
    {
        hmi_show_line(0, "BAL PID  K1/2=AKP K3/4=AD");
        hmi_show_line(1, "AKP:%4.1f AD:%5.2f", angle_pid.kp, angle_pid.kd);
        hmi_show_line(2, "PIT:%5.1f AFB:%5.1f", pitch, balance_angle_feedback);
        hmi_show_line(3, "TG:%6.1f GY:%6.1f", target_gyro, gyro_y_rate);
        hmi_show_line(4, "GFB:%6.1f SO:%6.0f", balance_gyro_feedback, servo_output);
        hmi_show_line(5, "PWM:%5lu YE:%6.1f", (unsigned long)servo_last_duty, yaw_error);
        hmi_show_line(6, "SPD:%4.2f DST:%5.2f", motor_get_actual_speed_mps(), motor_get_total_distance_m());
    }
    else
    {
        hmi_show_line(0, "TURN K1/2 SPD K3/4");
        hmi_show_line(1, "TURN:%4.1f D:%6.1f",
                      scheduler_get_mission_open_turn_angle(),
                      scheduler_get_mission_turn_delta());
        hmi_show_line(2, "SPD T:%4.2f A:%4.2f",
                      scheduler_get_mission_run_speed_mps(),
                      motor_get_actual_speed_mps());
        hmi_show_line(3, "PROG:%6.1f REM:%6.1f",
                      scheduler_get_mission_turn_progress(),
                      scheduler_get_mission_turn_remaining());
        hmi_show_line(4, "YAW:%6.1f TGT:%6.1f", yaw, scheduler_get_mission_turn_target_yaw());
        hmi_show_line(5, "EXP:%5.1f TA:%5.1f", expect_angle, target_angle);
        hmi_show_line(6, "ST:%u HD:%u DST:%5.2f",
                      (unsigned int)scheduler_get_mission_state(),
                      (unsigned int)balance_heading_enabled,
                      motor_get_total_distance_m());
    }

    hmi_show_line(7, "T:%5lu IMU:%s M1:%s",
                  (unsigned long)uwtick,
                  imu_ready ? "OK" : "WAIT",
                  motor_driver_online ? "OK" : "WAIT");
}

static void hmi_send_telemetry(void)
{
    hmi_sendf("%lu,%u,%u,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,"
              "%.2f,%.2f,%.2f,%.2f,%lu,%.2f,%.4f,%.2f,%.2f\r\n",
              (unsigned long)uwtick,
              (unsigned int)scheduler_get_mission_state(),
              (unsigned int)balance_heading_enabled,
              motor_get_total_distance_m(),
              motor_get_actual_speed_mps(),
              scheduler_get_mission_start_yaw(),
              scheduler_get_mission_turn_target_yaw(),
              yaw_target,
              balance_get_target_yaw_smooth(),
              yaw,
              yaw_error,
              steering_pid.out,
              expect_angle,
              target_angle,
              scheduler_get_mission_open_turn_angle(),
              scheduler_get_mission_turn_delta(),
              scheduler_get_mission_turn_progress(),
              scheduler_get_mission_turn_remaining(),
              roll,
              pitch,
              gyro_y_rate,
              gyro_z_rate,
              balance_angle_feedback,
              target_gyro,
              balance_gyro_feedback,
              servo_output,
              (unsigned long)servo_last_duty,
              angle_pid.kp,
              angle_pid.kd,
              scheduler_get_mission_open_turn_angle(),
              scheduler_get_mission_run_speed_mps());
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

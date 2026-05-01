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
#define HMI_TURN_ANGLE_STEP         (0.5f)

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
void scheduler_adjust_mission_open_turn_angle(float delta);

static uint8 hmi_switch_active(gpio_pin_enum pin)
{
    return (0 == gpio_get_level(pin)) ? 1U : 0U;
}

static void hmi_sendf(const char *format, ...)
{
    char buffer[192];
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
        if (0U == hmi_display_mode) { angle_pid.kp += 0.1f; }
        else                        { gyro_pid.kp  += 0.1f; }
    }

    if (hmi_key_edge(KEY_2))
    {
        if (0U == hmi_display_mode)
        {
            angle_pid.kp -= 0.1f;
            if (angle_pid.kp < 0.0f) { angle_pid.kp = 0.0f; }
        }
        else
        {
            gyro_pid.kp -= 0.1f;
            if (gyro_pid.kp < 0.0f) { gyro_pid.kp = 0.0f; }
        }
    }

    if (hmi_key_edge(KEY_3))
    {
        if (0U == hmi_display_mode) { angle_pid.kd += 0.05f; }
        else                        { scheduler_adjust_mission_open_turn_angle(HMI_TURN_ANGLE_STEP); }
    }

    if (hmi_key_edge(KEY_4))
    {
        if (0U == hmi_display_mode)
        {
            angle_pid.kd -= 0.05f;
            if (angle_pid.kd < 0.0f) { angle_pid.kd = 0.0f; }
        }
        else
        {
            scheduler_adjust_mission_open_turn_angle(-HMI_TURN_ANGLE_STEP);
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

    (void)hmi_display_mode;

    hmi_show_line(0, "SPD T:%4.2f A:%4.2f", motor_get_target_speed_mps(), motor_get_actual_speed_mps());
    hmi_show_line(1, "DST :%6.2f m", motor_get_total_distance_m());
    hmi_show_line(2, "ST:%u TURN:%4.1f",
                  (unsigned int)scheduler_get_mission_state(),
                  scheduler_get_mission_open_turn_angle());
    hmi_show_line(3, "GYX :%7.2f", gyro_x_rate);
    hmi_show_line(4, "GYY :%7.2f", gyro_y_rate);
    hmi_show_line(5, "GYZ :%7.2f", gyro_z_rate);
    hmi_show_line(6, "YAW :%7.2f", yaw);
    hmi_show_line(7, "T:%5lu IMU:%s M1:%s",
                  (unsigned long)uwtick,
                  imu_ready ? "OK" : "WAIT",
                  motor_driver_online ? "OK" : "WAIT");
}

static void hmi_send_telemetry(void)
{
    hmi_sendf("%lu,%u,%u,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
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
              scheduler_get_mission_open_turn_angle());
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

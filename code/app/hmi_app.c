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

static uint8  hmi_display_mode = 0U;
static uint8  hmi_display_mode_last = 0xFFU;
static uint8  hmi_motor_enabled = 0U;
static int16  hmi_servo_test_offset = 0;
static uint32 hmi_last_display_tick = 0U;
static uint32 hmi_last_telemetry_tick = 0U;
static uint32 hmi_last_driver_refresh_tick = 0U;
static uint8  hmi_yaw_lock_pending = 0U;
static uint32 hmi_yaw_lock_tick = 0U;

static const gpio_pin_enum hmi_key_pins[KEY_NUMBER] = KEY_LIST;
static uint32 hmi_key_last_trigger[KEY_NUMBER] = {0U};

static uint8 hmi_switch_active(gpio_pin_enum pin)
{
    return (0 == gpio_get_level(pin)) ? 1U : 0U;
}

static void hmi_sendf(const char *format, ...)
{
    char buffer[320];
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
    }
    else if (!new_motor_enabled && hmi_motor_enabled)
    {
        /* SW2 下降沿：关电机，motor_set_enabled 内部会 reset PID + duty=0 */
        motor_set_enabled(0U);
    }

    hmi_motor_enabled = new_motor_enabled;
}

static void hmi_update_display(void)
{
    uint8 status_ok = (imu_ready && (!hmi_motor_enabled || motor_driver_online)) ? 1U : 0U;

    if (hmi_display_mode != hmi_display_mode_last)
    {
        ips114_clear();
        hmi_display_mode_last = hmi_display_mode;
    }

    if (0U == hmi_display_mode)
    {
        hmi_show_line(0, "ROLL:%7.2f PIT:%7.2f", roll, pitch);
        hmi_show_line(1, "YAW:%6.2f YE:%6.2f YT:%5.1f", yaw, yaw_error, yaw_target);
        hmi_show_line(2, "S:%u M:%u TS:%5d", (unsigned int)scheduler_get_mission_state(), motor_enabled, motor_target_speed);
        hmi_show_line(3, "DST:%6.2fm AS:%5d", motor_get_total_distance_m(), motor_actual_speed);
        hmi_show_line(4, "AFB:%6.2f STR:%5.2f", balance_angle_feedback, steering_pid.out);
        hmi_show_line(5, "TG_A:%6.2f TG_G:%6.2f", target_angle, target_gyro);
        hmi_show_line(6, "OUT :%6.2f PWM:%5lu", servo_output, (unsigned long)servo_last_duty);
        hmi_show_line(7, "T:%6lu DU:%5d %s", (unsigned long)uwtick, motor_last_duty, status_ok ? "OK" : "NG");
    }
    else
    {
        hmi_show_line(0, "GYX :%7.2f GYY:%7.2f", gyro_x_rate, gyro_y_rate);
        hmi_show_line(1, "GYZ :%7.2f YAW:%7.2f", gyro_z_rate, yaw);
        hmi_show_line(2, "LRPM:%6d RRPM:%6d", motor_driver_left_rpm, motor_driver_right_rpm);
        hmi_show_line(3, "EFB :%6d ETOT:%6ld", encoder_data_2, (long)encoder_physical_total);
        hmi_show_line(4, "FGYR:%6.2f MDU:%5d", balance_filtered_gyro, motor_last_duty);
        hmi_show_line(5, "TANG:%6.2f TGYR:%6.2f", target_angle, target_gyro);
        hmi_show_line(6, "SOUT:%6.2f SPWM:%5lu", servo_output, (unsigned long)servo_last_duty);
        hmi_show_line(7, "S:%u T:%6lu %s", (unsigned int)scheduler_get_mission_state(), (unsigned long)uwtick, motor_driver_online ? "OK" : "NG");
    }
}

static void hmi_send_telemetry(void)
{
    hmi_sendf("t,state,hd,dst,syaw,tyaw,yt,tys,yaw,ye,gz,ea,ta,str,pit,afb,gy,fgy,tgy,out,pwm,ts,as:"
              "%lu,%u,%u,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.0f,%lu,%d,%d\r\n",
              (unsigned long)uwtick,
              (unsigned int)scheduler_get_mission_state(),
              (unsigned int)balance_heading_enabled,
              motor_get_total_distance_m(),
              scheduler_get_mission_start_yaw(),
              scheduler_get_mission_turn_target_yaw(),
              yaw_target,
              balance_get_target_yaw_smooth(),
              yaw,
              yaw_error,
              gyro_z_rate,
              expect_angle,
              target_angle,
              steering_pid.out,
              pitch,
              balance_angle_feedback,
              gyro_y_rate,
              balance_filtered_gyro,
              target_gyro,
              servo_output,
              (unsigned long)servo_last_duty,
              motor_target_speed,
              motor_actual_speed);
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

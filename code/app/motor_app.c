/*
 * motor_app.c
 */

#include "motor_app.h"

#include "small_driver_uart_control.h"

#ifndef M_PI
#define M_PI (3.1415926f)
#endif

#define WHEEL_DIAMETER_M                    (0.070f)
#define ENCODER_PULSES_PER_ENCODER_TURN     (1024.0f)
#define ENCODER_TURNS_PER_WHEEL_TURN        (4.0f)
#define MOTOR_SPEED_LOOP_PERIOD_S           (0.020f)

#define PHYSICAL_ENCODER_SPEED_SIGN      (1)
#define PHYSICAL_ENCODER_SPEED_SCALE_NUM (1)
#define PHYSICAL_ENCODER_SPEED_SCALE_DEN (1)

int16 encoder_data_2 = 0;
int16 encoder_physical = 0;
int32 encoder_physical_total = 0;
int16 motor_last_duty = 0;
int16 motor_driver_left_rpm = 0;
int16 motor_driver_right_rpm = 0;
uint8 motor_driver_online = 0;

PID_T motor_speed_pid;
int16 motor_target_speed = 0;
int16 motor_actual_speed = 0;
uint8 motor_enabled = 0U;

static uint32 motor_last_frame_counter = 0U;
static uint8  motor_driver_stale_cycles = 0U;
static float  motor_speed_filt = 0.0f;

static float motor_speed_counts_to_wheel_rps(float speed_counts)
{
    return (speed_counts / ENCODER_PULSES_PER_ENCODER_TURN) /
           ENCODER_TURNS_PER_WHEEL_TURN /
           MOTOR_SPEED_LOOP_PERIOD_S;
}

/* 起步 kick：SW2 拨开后前 N 次 motor_speed_loop（20ms/次）强制高 duty，
 * 让电机越过静摩擦快速到速，期间不走 PID，防止慢慢爬半天启动 */
#define MOTOR_KICK_CYCLES   (15U)   /* 15 * 20ms = 300ms，够冲过静摩擦不让 PID 刹车 */
#define MOTOR_KICK_DUTY     (2500)  /* 25%，温和起步 */
static uint8 motor_kick_remaining = 0U;

static int16 encoder_scale_feedback(int16 raw_count)
{
    int32 scaled_value = (int32)raw_count * PHYSICAL_ENCODER_SPEED_SIGN;

    scaled_value = (scaled_value * PHYSICAL_ENCODER_SPEED_SCALE_NUM) /
                   PHYSICAL_ENCODER_SPEED_SCALE_DEN;

    if (scaled_value > 32767)
    {
        scaled_value = 32767;
    }
    if (scaled_value < -32768)
    {
        scaled_value = -32768;
    }

    return (int16)scaled_value;
}

static void encoder_capture_sample(void)
{
    encoder_physical = encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);
    encoder_physical_total += encoder_physical;
}

void motor_init(void)
{
    small_driver_uart_init();
    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
    encoder_feedback_reset();
    motor_set_duty(0);
    motor_refresh_status();
    motor_speed_pid_init();
}

void motor_set_duty(int16 duty)
{
    if (duty > 10000)
    {
        duty = 10000;
    }
    if (duty < -10000)
    {
        duty = -10000;
    }

    motor_last_duty = duty;
    small_driver_set_duty((int16)(-duty), 0);
}

void encoder_count(void)
{
    encoder_capture_sample();
    encoder_data_2 = encoder_scale_feedback(encoder_physical);
}

void encoder_feedback_reset(void)
{
    encoder_data_2 = 0;
    encoder_physical = 0;
    encoder_physical_total = 0;
    encoder_clear_count(ENCODER_2);
}

void motor_refresh_status(void)
{
    motor_driver_left_rpm = motor_value.receive_left_speed_data;
    motor_driver_right_rpm = motor_value.receive_right_speed_data;

    if (small_driver_frame_counter != motor_last_frame_counter)
    {
        motor_last_frame_counter = small_driver_frame_counter;
        motor_driver_stale_cycles = 0U;
        motor_driver_online = 1U;
    }
    else
    {
        if (motor_driver_stale_cycles < 255U)
        {
            motor_driver_stale_cycles++;
        }

        if (motor_driver_stale_cycles >= 5U)
        {
            motor_driver_online = 0U;
        }
    }

    small_driver_get_speed();
}

float motor_get_display_speed_mps(void)
{
    float wheel_circumference_m = (float)M_PI * WHEEL_DIAMETER_M;
    float wheel_rps = motor_speed_counts_to_wheel_rps((float)motor_actual_speed);

    return wheel_rps * wheel_circumference_m;
}

float motor_speed_counts_to_mps(float speed_counts)
{
    float wheel_circumference_m = (float)M_PI * WHEEL_DIAMETER_M;

    return motor_speed_counts_to_wheel_rps(speed_counts) * wheel_circumference_m;
}

int16 motor_speed_mps_to_counts(float speed_mps)
{
    float wheel_circumference_m = (float)M_PI * WHEEL_DIAMETER_M;
    float speed_counts = (speed_mps / wheel_circumference_m) *
                         MOTOR_SPEED_LOOP_PERIOD_S *
                         ENCODER_TURNS_PER_WHEEL_TURN *
                         ENCODER_PULSES_PER_ENCODER_TURN;
    int32 rounded_counts = (speed_counts >= 0.0f) ?
                           (int32)(speed_counts + 0.5f) :
                           (int32)(speed_counts - 0.5f);

    if (rounded_counts > 32767)
    {
        rounded_counts = 32767;
    }
    else if (rounded_counts < -32768)
    {
        rounded_counts = -32768;
    }

    return (int16)rounded_counts;
}

float motor_get_target_speed_mps(void)
{
    return motor_speed_counts_to_mps((float)motor_target_speed);
}

float motor_get_actual_speed_mps(void)
{
    return motor_speed_counts_to_mps((float)motor_actual_speed);
}

float motor_encoder_counts_to_m(float counts)
{
    float wheel_circumference_m = (float)M_PI * WHEEL_DIAMETER_M;

    return (counts / ENCODER_PULSES_PER_ENCODER_TURN) /
           ENCODER_TURNS_PER_WHEEL_TURN *
           wheel_circumference_m;
}

float motor_get_total_distance_m(void)
{
    return motor_encoder_counts_to_m((float)encoder_physical_total);
}

void motor_speed_pid_init(void)
{
    /* 参考 20260404 备份：位置式 PID */
    pid_init(&motor_speed_pid, 0.15f, 0.07f, 0.0f, 0.0f, 10000.0f);
    pid_app_limit_integral(&motor_speed_pid, -5000.0f, 5000.0f);
}

void motor_set_target_speed(int16 speed)
{
    if (speed > 2000)  { speed = 2000; }
    if (speed < -2000) { speed = -2000; }
    motor_target_speed = speed;
}

void motor_set_enabled(uint8 enabled)
{
    uint8 prev = motor_enabled;
    motor_enabled = enabled ? 1U : 0U;

    if (!motor_enabled)
    {
        /* 保留 motor_target_speed 让用户调的目标速度跨 SW2 持久化 */
        pid_reset(&motor_speed_pid);
        motor_speed_filt = 0.0f;
        motor_kick_remaining = 0U;
        motor_set_duty(0);
    }
    else if (!prev)
    {
        /* 上升沿：启动 kick 窗口，让电机 600ms 内冲到速 */
        motor_kick_remaining = MOTOR_KICK_CYCLES;
        pid_reset(&motor_speed_pid);
    }
}

void motor_speed_loop(void)
{
    /* 反馈：encoder 差分（20ms 周期由 CCU61_CH1 里 encoder_count 保证） */
    /* 一阶低通滤波 α=0.2，抑制编码器抖动 */
    float raw = (float)encoder_data_2;
    motor_speed_filt = motor_speed_filt * 0.8f + raw * 0.2f;
    motor_actual_speed = (int16)motor_speed_filt;

    if (!motor_enabled)
    {
        motor_set_duty(0);
        return;
    }

    /* 起步 kick 窗口：强制高 duty 越过静摩擦，期间跳过 PID */
    if (motor_kick_remaining > 0U)
    {
        motor_kick_remaining--;
        int16 kick_sign = (motor_target_speed >= 0) ? 1 : -1;
        motor_set_duty((int16)(kick_sign * MOTOR_KICK_DUTY));
        return;
    }

    pid_set_target(&motor_speed_pid, (float)motor_target_speed);
    float output = pid_calculate_positional(&motor_speed_pid, motor_speed_filt);
    motor_set_duty((int16)output);
}

/*
 * balance_app.c
 *
 * Three-stage cascade balance:
 * - 20 ms steering loop: yaw error -> expect_angle offset
 * - 5 ms angle loop: pitch error -> target gyro rate
 * - 1 ms gyro loop: gyro error -> servo output
 */

#include "balance_app.h"

#include <math.h>

#include "imu_app.h"
#include "servo_app.h"

#define BALANCE_EXPECT_ANGLE_DEFAULT (0.0f)

#define BALANCE_HEADING_STEP        (1.5f)

#define BALANCE_STEERING_KP         (0.15f)
#define BALANCE_STEERING_KI         (0.0f)
#define BALANCE_STEERING_KD         (0.05f)
#define BALANCE_STEERING_OUT_LIMIT  (3.5f)

#define BALANCE_ANGLE_KP            (22.0f)
#define BALANCE_ANGLE_KI            (0.005f)
#define BALANCE_ANGLE_KD            (-0.3f)
#define BALANCE_ANGLE_OUT_LIMIT     (150.0f)
#define BALANCE_ANGLE_INT_LIMIT     (30.0f)

#define BALANCE_GYRO_KP             (7.9f)
#define BALANCE_GYRO_KI             (0.0f)
#define BALANCE_GYRO_KD             (0.0f)
#define BALANCE_GYRO_OUT_LIMIT      (1300.0f)   /* 与 SAFE_LIMIT 对齐：原 2640 让 PID 看不到自己被夹 */
#define BALANCE_GYRO_INT_LIMIT      (200.0f)    /* anti-windup 上限，KI=0 时无效 */
#define BALANCE_GYRO_INPUT_LIMIT    (1500.0f)   /* 原 800 在快速倾倒时反馈失真 */
#define BALANCE_GYRO_LPF_ALPHA      (0.20f)

#define BALANCE_SERVO_SAFE_LIMIT    (1300.0f)

/* 静态零位锁定参数 */
#define BALANCE_ZERO_AVG_SAMPLES    (32U)    /* 开机平均采样数（32×5ms=160ms） */
#define BALANCE_ZERO_AVG_INTERVAL   (5U)     /* 每次采样间隔 ms */
#define BALANCE_ZERO_STATIC_GYRO    (30.0f)  /* SW2 触发锁零时的静止判定阈值 dps */

PID_T steering_pid;
PID_T angle_pid;
PID_T gyro_pid;

float expect_angle = BALANCE_EXPECT_ANGLE_DEFAULT;
float target_angle = BALANCE_EXPECT_ANGLE_DEFAULT;
float target_gyro = 0.0f;
float servo_output = 0.0f;
float balance_angle_zero = 0.0f;
float balance_angle_feedback = 0.0f;
float balance_gyro_feedback = 0.0f;
float balance_filtered_gyro = 0.0f;
float yaw_target = 0.0f;
float yaw_error = 0.0f;
uint8 balance_enabled = 0U;
uint8 balance_heading_enabled = 1U;
uint8 balance_zero_calibrated = 1U;

static float target_yaw_smooth = 0.0f;
static uint8 balance_servo_test_enabled = 0U;
static int16 balance_servo_test_offset = 0;
static float gyro_filtered = 0.0f;

static void balance_apply_servo_output(float output)
{
    int32 duty;

    servo_output = output;

    if (servo_output > BALANCE_SERVO_SAFE_LIMIT)
    {
        servo_output = BALANCE_SERVO_SAFE_LIMIT;
    }
    else if (servo_output < -BALANCE_SERVO_SAFE_LIMIT)
    {
        servo_output = -BALANCE_SERVO_SAFE_LIMIT;
    }

    if (balance_servo_test_enabled)
    {
        servo_set((uint32)((int32)mid + (int32)balance_servo_test_offset));
        return;
    }

    duty = (int32)mid + (int32)servo_output;
    if (duty < (int32)r_max)
    {
        duty = (int32)r_max;
    }
    if (duty > (int32)l_max)
    {
        duty = (int32)l_max;
    }

    servo_set((uint32)duty);
}

static void balance_reset_state(void)
{
    target_angle = expect_angle;
    target_gyro = 0.0f;
    servo_output = 0.0f;
    target_yaw_smooth = yaw_target;
    balance_angle_feedback = 0.0f;
    balance_gyro_feedback = 0.0f;
    balance_filtered_gyro = 0.0f;
    gyro_filtered = 0.0f;
    pid_reset(&steering_pid);
    pid_reset(&angle_pid);
    pid_reset(&gyro_pid);
}

void balance_init(void)
{
    pid_init(&steering_pid,
             BALANCE_STEERING_KP,
             BALANCE_STEERING_KI,
             BALANCE_STEERING_KD,
             0.0f,
             BALANCE_STEERING_OUT_LIMIT);

    pid_init(&angle_pid,
             BALANCE_ANGLE_KP,
             BALANCE_ANGLE_KI,
             BALANCE_ANGLE_KD,
             0.0f,
             BALANCE_ANGLE_OUT_LIMIT);

    pid_init(&gyro_pid,
             BALANCE_GYRO_KP,
             BALANCE_GYRO_KI,
             BALANCE_GYRO_KD,
             0.0f,
             BALANCE_GYRO_OUT_LIMIT);

    balance_enabled = 1U;
    /* balance_init 在 ISR 启动前调用，可阻塞采样 → 用静止平均锁零 */
    balance_capture_zero_static();
    balance_lock_yaw_target();
    balance_reset_state();
}

void balance_set_enabled(uint8 enabled)
{
    balance_enabled = enabled ? 1U : 0U;

    if (balance_enabled)
    {
        balance_lock_angle_zero();
        balance_lock_yaw_target();
    }

    balance_reset_state();
}

/* 开机静止采样平均：32×5ms=160ms 阻塞，须在 ISR 启动前调用。
 * 数据原因：SFLP pitch 单帧噪声 0.1-0.3°，KP=22 放大成 2-7° 输出偏置；
 * 转弯日志显示稳态 angle_pid 输出已偏 +9°，根因就是单帧锁零 */
void balance_capture_zero_static(void)
{
    float sum = 0.0f;
    uint16 i;

    for (i = 0U; i < BALANCE_ZERO_AVG_SAMPLES; i++)
    {
        system_delay_ms(BALANCE_ZERO_AVG_INTERVAL);
        imu_proc();
        sum += pitch;
    }

    balance_angle_zero = sum / (float)BALANCE_ZERO_AVG_SAMPLES;
    balance_angle_feedback = 0.0f;
    balance_zero_calibrated = 1U;
}

/* 运行时锁零（SW2 触发）：加静止守卫，剧烈摆动时拒绝更新，
 * 避免在车被颠簸或扶手抖动的瞬间锁错零位 */
void balance_lock_angle_zero(void)
{
    if (fabsf(gyro_y_rate) > BALANCE_ZERO_STATIC_GYRO)
    {
        /* 不静止：保留上次零位，不强制写入 */
        balance_angle_feedback = pitch - balance_angle_zero;
        return;
    }

    balance_angle_zero = pitch;
    balance_angle_feedback = 0.0f;
    balance_zero_calibrated = 1U;
}

void balance_set_expect_angle(float angle)
{
    expect_angle = angle;
    target_angle = expect_angle;
}

void balance_set_servo_test_enabled(uint8 enabled)
{
    balance_servo_test_enabled = enabled ? 1U : 0U;
}

void balance_set_servo_test_offset(int16 offset)
{
    int16 max_val = (int16)BALANCE_SERVO_SAFE_LIMIT;

    if (offset > max_val)
    {
        offset = max_val;
    }
    if (offset < -max_val)
    {
        offset = -max_val;
    }

    balance_servo_test_offset = offset;
}

void balance_lock_yaw_target(void)
{
    yaw_target = yaw;
    target_yaw_smooth = yaw;
    target_angle = expect_angle;
    pid_reset(&steering_pid);
}

void balance_set_yaw_target(float target)
{
    yaw_target = normalize_angle(target);
}

float balance_get_target_yaw_smooth(void)
{
    return target_yaw_smooth;
}

void balance_set_heading_enabled(uint8 enabled)
{
    balance_heading_enabled = enabled ? 1U : 0U;
    pid_reset(&steering_pid);

    if (!balance_heading_enabled)
    {
        yaw_error = 0.0f;
        target_angle = expect_angle;
    }
    else
    {
        target_yaw_smooth = yaw;
    }
}

void balance_steering_loop(void)
{
    float heading_error;

    if (!balance_enabled || !balance_heading_enabled)
    {
        yaw_error = 0.0f;
        target_yaw_smooth = yaw_target;
        target_angle = expect_angle;
        pid_reset(&steering_pid);
        return;
    }

    heading_error = normalize_angle(yaw_target - target_yaw_smooth);
    if (heading_error > BALANCE_HEADING_STEP)
    {
        target_yaw_smooth = normalize_angle(target_yaw_smooth + BALANCE_HEADING_STEP);
    }
    else if (heading_error < -BALANCE_HEADING_STEP)
    {
        target_yaw_smooth = normalize_angle(target_yaw_smooth - BALANCE_HEADING_STEP);
    }
    else
    {
        target_yaw_smooth = yaw_target;
    }

    yaw_error = normalize_angle(yaw - target_yaw_smooth);
    target_angle = expect_angle + pid_calculate_by_error(&steering_pid, yaw_error);
}

void balance_angle_loop(void)
{
    float angle_error;

    balance_angle_feedback = pitch - balance_angle_zero;

    if (!balance_enabled)
    {
        target_angle = expect_angle;
        target_gyro = 0.0f;
        angle_pid.integral = 0.0f;
        angle_pid.last_error = 0.0f;
        angle_pid.out = 0.0f;
        return;
    }

    angle_error = target_angle - balance_angle_feedback;

    angle_pid.integral += angle_error;
    if (angle_pid.integral > BALANCE_ANGLE_INT_LIMIT)
    {
        angle_pid.integral = BALANCE_ANGLE_INT_LIMIT;
    }
    else if (angle_pid.integral < -BALANCE_ANGLE_INT_LIMIT)
    {
        angle_pid.integral = -BALANCE_ANGLE_INT_LIMIT;
    }

    angle_pid.p_out = angle_pid.kp * angle_error;
    angle_pid.i_out = angle_pid.ki * angle_pid.integral;
    angle_pid.d_out = angle_pid.kd * (angle_error - angle_pid.last_error);
    angle_pid.out = angle_pid.p_out + angle_pid.i_out + angle_pid.d_out;

    if (angle_pid.out > BALANCE_ANGLE_OUT_LIMIT)
    {
        angle_pid.out = BALANCE_ANGLE_OUT_LIMIT;
    }
    else if (angle_pid.out < -BALANCE_ANGLE_OUT_LIMIT)
    {
        angle_pid.out = -BALANCE_ANGLE_OUT_LIMIT;
    }

    angle_pid.last_error = angle_error;
    target_gyro = angle_pid.out;
}

void balance_gyro_loop(void)
{
    float current_gyro = gyro_y_rate;
    float gyro_error;

    if (current_gyro > BALANCE_GYRO_INPUT_LIMIT)
    {
        current_gyro = BALANCE_GYRO_INPUT_LIMIT;
    }
    else if (current_gyro < -BALANCE_GYRO_INPUT_LIMIT)
    {
        current_gyro = -BALANCE_GYRO_INPUT_LIMIT;
    }

    gyro_filtered = BALANCE_GYRO_LPF_ALPHA * current_gyro
                  + (1.0f - BALANCE_GYRO_LPF_ALPHA) * gyro_filtered;

    balance_gyro_feedback = current_gyro;
    balance_filtered_gyro = gyro_filtered;

    if (!balance_enabled)
    {
        target_gyro = 0.0f;
        gyro_pid.integral = 0.0f;
        gyro_pid.last_error = 0.0f;
        gyro_pid.out = 0.0f;
        balance_apply_servo_output(0.0f);
        return;
    }

    gyro_error = target_gyro - gyro_filtered;

    /* 条件积分（anti-windup）：输出已撞限 + 误差仍在加大方向 → 不再累加积分。
     * 当前 KI=0 此分支冷藏，但调 KI≠0 时防止积分缠绕导致回中迟钝。 */
    {
        uint8 saturated_pos = (gyro_pid.out >=  BALANCE_GYRO_OUT_LIMIT) && (gyro_error > 0.0f);
        uint8 saturated_neg = (gyro_pid.out <= -BALANCE_GYRO_OUT_LIMIT) && (gyro_error < 0.0f);
        if (!saturated_pos && !saturated_neg)
        {
            gyro_pid.integral += gyro_error;
            if (gyro_pid.integral >  BALANCE_GYRO_INT_LIMIT)
            {
                gyro_pid.integral =  BALANCE_GYRO_INT_LIMIT;
            }
            else if (gyro_pid.integral < -BALANCE_GYRO_INT_LIMIT)
            {
                gyro_pid.integral = -BALANCE_GYRO_INT_LIMIT;
            }
        }
    }

    gyro_pid.p_out = gyro_pid.kp * gyro_error;
    gyro_pid.i_out = gyro_pid.ki * gyro_pid.integral;
    gyro_pid.d_out = gyro_pid.kd * (gyro_error - gyro_pid.last_error);
    gyro_pid.out = gyro_pid.p_out + gyro_pid.i_out + gyro_pid.d_out;

    if (gyro_pid.out > BALANCE_GYRO_OUT_LIMIT)
    {
        gyro_pid.out = BALANCE_GYRO_OUT_LIMIT;
    }
    else if (gyro_pid.out < -BALANCE_GYRO_OUT_LIMIT)
    {
        gyro_pid.out = -BALANCE_GYRO_OUT_LIMIT;
    }

    gyro_pid.last_error = gyro_error;
    balance_apply_servo_output(gyro_pid.out);
}

void pid_test(void)
{
    JustFloat_Test_five(balance_angle_feedback,
                        target_gyro,
                        balance_filtered_gyro,
                        gyro_pid.out,
                        servo_output);
}

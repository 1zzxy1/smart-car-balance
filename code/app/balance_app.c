/*
 * balance_app.c
 *
 * Three-stage cascade balance:
 * - 20 ms steering loop: yaw error → expect_angle offset
 * - 5 ms angle loop: pitch error → target gyro rate
 * - 1 ms gyro loop: gyro error → servo output
 */

#include "balance_app.h"

#include "imu_app.h"
#include "motor_app.h"
#include "servo_app.h"

#define BALANCE_EXPECT_ANGLE_DEFAULT (0.0f)
#define BALANCE_FALL_THRESHOLD      (25.0f)

#define BALANCE_HEADING_STEP        (1.5f)

#define BALANCE_STEERING_KP         (0.15f)
#define BALANCE_STEERING_KI         (0.0f)
#define BALANCE_STEERING_KD         (0.05f)
#define BALANCE_STEERING_OUT_LIMIT  (8.0f)
#define BALANCE_STEERING_INT_LIMIT  (20.0f)

#define BALANCE_ANGLE_KP            (8.3f)
#define BALANCE_ANGLE_KI            (0.02f)
#define BALANCE_ANGLE_KD            (-0.3f)
#define BALANCE_ANGLE_OUT_LIMIT     (150.0f)
#define BALANCE_ANGLE_INT_LIMIT     (30.0f)

#define BALANCE_GYRO_KP             (7.9f)
#define BALANCE_GYRO_KI             (0.0f)
#define BALANCE_GYRO_KD             (0.0f)
#define BALANCE_GYRO_OUT_LIMIT      (2640.0f)
#define BALANCE_GYRO_INPUT_LIMIT    (800.0f)
#define BALANCE_GYRO_LPF_ALPHA      (0.40f)

#define BALANCE_SERVO_SAFE_LIMIT    (1650.0f)

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
uint8 balance_zero_calibrated = 1U;

static float steering_output = 0.0f;
static float target_yaw_smooth = 0.0f;
static uint8 balance_steering_enabled = 1U;
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
    steering_output = 0.0f;
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
    balance_lock_angle_zero();
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

void balance_lock_angle_zero(void)
{
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
    steering_output = 0.0f;
    pid_reset(&steering_pid);
}

void balance_set_steering_enabled(uint8 enabled)
{
    balance_steering_enabled = enabled ? 1U : 0U;
    if (!balance_steering_enabled)
    {
        steering_output = 0.0f;
        yaw_error = 0.0f;
        target_yaw_smooth = yaw_target;
        pid_reset(&steering_pid);
    }
}

void balance_steering_loop(void)
{
    float heading_error;
    float steer_error;

    if (!balance_enabled || !balance_steering_enabled)
    {
        steering_output = 0.0f;
        yaw_error = 0.0f;
        target_yaw_smooth = yaw_target;
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
    steer_error = yaw_error;

    steering_pid.integral += steer_error;
    if (steering_pid.integral > BALANCE_STEERING_INT_LIMIT)
    {
        steering_pid.integral = BALANCE_STEERING_INT_LIMIT;
    }
    else if (steering_pid.integral < -BALANCE_STEERING_INT_LIMIT)
    {
        steering_pid.integral = -BALANCE_STEERING_INT_LIMIT;
    }

    steering_pid.p_out = steering_pid.kp * steer_error;
    steering_pid.i_out = steering_pid.ki * steering_pid.integral;
    steering_pid.d_out = steering_pid.kd * (steer_error - steering_pid.last_error);
    steering_pid.out = steering_pid.p_out + steering_pid.i_out + steering_pid.d_out;

    if (steering_pid.out > BALANCE_STEERING_OUT_LIMIT)
    {
        steering_pid.out = BALANCE_STEERING_OUT_LIMIT;
    }
    else if (steering_pid.out < -BALANCE_STEERING_OUT_LIMIT)
    {
        steering_pid.out = -BALANCE_STEERING_OUT_LIMIT;
    }

    steering_pid.last_error = steer_error;
    steering_output = steering_pid.out;
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

    target_angle = expect_angle + steering_output;
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

    if ((balance_angle_feedback > BALANCE_FALL_THRESHOLD) ||
        (balance_angle_feedback < -BALANCE_FALL_THRESHOLD))
    {
        motor_set_enabled(0U);
        balance_apply_servo_output(0.0f);
        return;
    }

    gyro_error = target_gyro - gyro_filtered;

    gyro_pid.integral += gyro_error;
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

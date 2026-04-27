/*
 * imu_app.c
 *
 * SFLP 480Hz 四元数模式：INT2 中断内读取芯片融合姿态 + gyro + acc，
 * imu_proc 只做数据拷贝（零 SPI），1ms 周期跟 CCU60_CH1 对齐。
 * 开机用 imu660rc_bias_calibrate(2000) 做裁剪均值校准 + SFLP 偏差注入。
 */

#include "imu_app.h"

#include <math.h>

#define PITCH_SPIKE_THRESH  (30.0f)

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float gyro_x_rate = 0.0f;
float gyro_y_rate = 0.0f;
float gyro_z_rate = 0.0f;
uint8 imu_ready = 0U;

static float yaw_offset = 0.0f;
static float pitch_last = 0.0f;
static uint8 spike_initialized = 0U;

float normalize_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

void imu_all_init(void)
{
    uint8 init_retry = 0U;

    while (imu660rc_init(IMU660RC_QUARTERNION_480HZ))
    {
        system_delay_ms(100);

        init_retry++;
        if (init_retry >= 20U)
        {
            imu_ready = 0U;
            ips114_clear();
            ips114_show_string(0, 0, "IMU INIT FAILED");
            ips114_show_string(0, 16, "CHECK HW / INT2");
            while (1);
            return;
        }
    }

    exti_disable(IMU660RC_INT2_PIN);
    imu660rc_bias_calibrate(2000);
    exti_enable(IMU660RC_INT2_PIN);

    system_delay_ms(200);
    yaw_offset = imu660rc_yaw;

    imu_proc();
    imu_ready = 1U;
}

void imu_proc(void)
{
    float raw_pitch;

    if (!imu_ready)
    {
        return;
    }

    raw_pitch = imu660rc_pitch;

    if (raw_pitch != raw_pitch)
    {
        return;
    }

    if (!spike_initialized)
    {
        pitch_last = raw_pitch;
        spike_initialized = 1U;
    }

    if (fabsf(raw_pitch - pitch_last) > PITCH_SPIKE_THRESH)
    {
        raw_pitch = pitch_last;
    }

    pitch_last = raw_pitch;

    pitch = raw_pitch;
    roll = imu660rc_roll;
    yaw = normalize_angle(imu660rc_yaw - yaw_offset);

    gyro_x_rate = imu660rc_gyro_transition(imu660rc_gyro_x);
    gyro_y_rate = imu660rc_gyro_transition(imu660rc_gyro_y);
    gyro_z_rate = imu660rc_gyro_transition(imu660rc_gyro_z);
}

void imu_test(void)
{
    printf("%f,%f,%f\r\n", roll, pitch, yaw);
}

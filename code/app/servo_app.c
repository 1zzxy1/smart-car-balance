/*
 * servo_app.c
 */

#include "servo_app.h"

uint32 servo_last_duty = mid;

void servo_set(uint32 duty)
{
    if (duty >= l_max)
    {
        duty = l_max;
    }
    if (duty <= r_max)
    {
        duty = r_max;
    }

    servo_last_duty = duty;
    pwm_set_duty(ATOM1_CH1_P33_9, duty);
}

void servo_test(void)
{
    static uint16 servo_motor_duty = mid;
    static uint8 servo_motor_dir = 0;

    if (servo_motor_dir)
    {
        servo_motor_duty++;
        if (servo_motor_duty >= l_max)
        {
            servo_motor_dir = 0U;
        }
    }
    else
    {
        servo_motor_duty--;
        if (servo_motor_duty <= r_max)
        {
            servo_motor_dir = 1U;
        }
    }

    servo_set(servo_motor_duty);
}

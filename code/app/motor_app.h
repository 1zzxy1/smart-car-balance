/*
 * motor_app.h
 *
 * Brushless driver + external encoder wrapper for the current car.
 */

#ifndef CODE_APP_MOTOR_APP_H_
#define CODE_APP_MOTOR_APP_H_

#include "zf_common_headfile.h"

#define ENCODER_2                   (TIM6_ENCODER)
#define ENCODER_2_A                 (TIM6_ENCODER_CH1_P20_3)
#define ENCODER_2_B                 (TIM6_ENCODER_CH2_P20_0)

extern int16 encoder_data_2;
extern int16 encoder_physical;
extern int32 encoder_physical_total;
extern int16 motor_last_duty;
extern int16 motor_driver_left_rpm;
extern int16 motor_driver_right_rpm;
extern uint8 motor_driver_online;

void motor_init(void);
void motor_set_duty(int16 duty);
void encoder_count(void);
void encoder_feedback_reset(void);
void motor_refresh_status(void);

extern PID_T  motor_speed_pid;
extern int16  motor_target_speed;
extern int16  motor_actual_speed;
extern uint8  motor_enabled;

void motor_speed_pid_init(void);
void motor_speed_loop(void);
void motor_set_target_speed(int16 speed);
void motor_set_enabled(uint8 enabled);

#endif /* CODE_APP_MOTOR_APP_H_ */
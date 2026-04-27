/*
 * balance_app.h
 */

#ifndef CODE_APP_BALANCE_APP_H_
#define CODE_APP_BALANCE_APP_H_

#include "zf_common_headfile.h"

extern PID_T steering_pid;
extern PID_T angle_pid;
extern PID_T gyro_pid;

extern float expect_angle;
extern float target_angle;
extern float target_gyro;
extern float servo_output;
extern float balance_angle_zero;
extern float balance_angle_feedback;
extern float balance_gyro_feedback;
extern float balance_filtered_gyro;
extern float yaw_target;
extern float yaw_error;
extern uint8 balance_enabled;
extern uint8 balance_zero_calibrated;

void balance_init(void);
void balance_set_enabled(uint8 enabled);
void balance_lock_angle_zero(void);
void balance_lock_yaw_target(void);
void balance_set_steering_enabled(uint8 enabled);
void balance_set_expect_angle(float angle);
void balance_set_servo_test_enabled(uint8 enabled);
void balance_set_servo_test_offset(int16 offset);
void balance_steering_loop(void);
void balance_angle_loop(void);
void balance_gyro_loop(void);
void pid_test(void);

#endif /* CODE_APP_BALANCE_APP_H_ */

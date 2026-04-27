/*
 * servo_app.h
 *
 * Servo output wrapper for the current car hardware.
 */

#ifndef CODE_APP_SERVO_APP_H_
#define CODE_APP_SERVO_APP_H_

#include "zf_common_headfile.h"

/* Current car servo calibration (330 Hz). */
#define l_max  8580
#define mid    5940
#define r_max  3500

extern uint32 servo_last_duty;

void servo_set(uint32 duty);
void servo_test(void);

#endif /* CODE_APP_SERVO_APP_H_ */

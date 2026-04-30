/*
 * scheduler.h
 *
 *  Created on: 2025Äê11ÔÂ11ÈÕ
 *      Author: suiyungui
 */

#ifndef CODE_APP_SCHEDULER_H_
#define CODE_APP_SCHEDULER_H_

#include "zf_common_headfile.h"

extern uint32_t uwtick;
void scheduler_init(void);
void scheduler_run(void);
uint8 scheduler_get_mission_state(void);
float scheduler_get_mission_start_yaw(void);
float scheduler_get_mission_turn_target_yaw(void);


#endif /* CODE_APP_SCHEDULER_H_ */

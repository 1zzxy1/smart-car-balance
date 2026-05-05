/*
 * schedule.c
 *
 * Cooperative scheduler for low-priority background tasks.
 */

#include "scheduler.h"

#include <math.h>

#include "balance_app.h"
#include "hmi_app.h"
#include "imu_app.h"
#include "motor_app.h"

/* 起步后先直行的距离，达到后开始执行转圈任务。 */
#define MISSION_GO_DISTANCE_M       (9.0f)
/* 任务全程目标速度，单位 m/s。 */
#define MISSION_RUN_SPEED_MPS       (1.00f)
#define MISSION_RUN_SPEED_MIN_MPS   (0.50f)
#define MISSION_RUN_SPEED_MAX_MPS   (2.00f)
/* 默认开环压弯角，越大转弯半径越小，但平衡风险越高。 */
#define MISSION_OPEN_TURN_ANGLE_DEFAULT (5.0f)
/* 按键调节压弯角的安全下限。 */
#define MISSION_OPEN_TURN_ANGLE_MIN     (1.0f)
/* 按键调节压弯角的安全上限。 */
#define MISSION_OPEN_TURN_ANGLE_MAX     (12.0f)
/* 距离本次目标转角还差多少度时，退出开环压弯并进入倾角溜泄。 */
#define MISSION_RELOCK_ERROR_DEG    (30.0f)
/* 倾角溜泄时间，期间 expect_angle 从当前压弯角线性降到 0。 */
#define MISSION_LEAN_BLEED_MS       (100U)
/* 溜泄后锁住目标航向的保持时间，结束后进入下一次开环转弯。 */
#define MISSION_BACK_HEADING_HOLD_MS (100U)
/* 第一次转弯目标角度，用于从直行切入绕圈轨迹。 */
#define MISSION_FIRST_TURN_DEG      (270.0f)
/* 后续每次完整绕圈的目标转角绝对值。 */
#define MISSION_FULL_TURN_DEG       (360.0f)
/* yaw 增量投影符号，若累计转角方向反了，只改这个符号。 */
#define MISSION_YAW_PROGRESS_SIGN   (-1.0f)

typedef enum
{
    MISSION_IDLE = 0,
    MISSION_GO_STRAIGHT,
    MISSION_OPEN_TURN,
    MISSION_LEAN_BLEED,
    MISSION_BACK_HEADING
} mission_state_enum;

uint8_t task_num;
uint32_t uwtick = 0;

static mission_state_enum mission_state = MISSION_IDLE;
static float mission_start_yaw = 0.0f;
static float mission_turn_target_yaw = 0.0f;
static float mission_turn_delta_deg = MISSION_FIRST_TURN_DEG;
static float mission_turn_progress_deg = 0.0f;
static float mission_turn_last_yaw = 0.0f;
static float mission_run_speed_mps = MISSION_RUN_SPEED_MPS;
static float mission_open_turn_angle = MISSION_OPEN_TURN_ANGLE_DEFAULT;
static float mission_lean_bleed_start_angle = 0.0f;
static uint32_t mission_lean_bleed_start_tick = 0U;
static uint32_t mission_back_heading_start_tick = 0U;
static uint8_t mission_next_full_turn_positive = 0U;

static void mission_proc(void);
static void mission_start_first_task(void);
static void mission_start_open_turn(void);
static void mission_set_turn_delta(float turn_delta_deg);
static void mission_reset(void);

typedef struct
{
    void (*task_func)(void);
    uint32_t rate_ms;
    uint32_t last_run;
} task_t;

static task_t scheduler_task[] =
{
    {mission_proc, 10U, 0U},
    {hmi_proc, 10U, 0U},
};

static void mission_start_first_task(void)
{
    mission_start_yaw = yaw;
    mission_set_turn_delta(MISSION_FIRST_TURN_DEG);
    mission_next_full_turn_positive = 0U;

    encoder_feedback_reset();
    balance_set_expect_angle(0.0f);
    balance_set_yaw_target(mission_start_yaw);
    balance_set_heading_enabled(1U);
    motor_set_target_speed(motor_speed_mps_to_counts(mission_run_speed_mps));

    mission_state = MISSION_GO_STRAIGHT;
}

static void mission_start_open_turn(void)
{
    float turn_sign = (mission_turn_delta_deg >= 0.0f) ? 1.0f : -1.0f;

    mission_turn_progress_deg = 0.0f;
    mission_turn_last_yaw = yaw;
    balance_set_heading_enabled(0U);
    balance_set_expect_angle(turn_sign * mission_open_turn_angle);
    mission_state = MISSION_OPEN_TURN;
}

static void mission_set_turn_delta(float turn_delta_deg)
{
    mission_turn_delta_deg = turn_delta_deg;
    mission_turn_target_yaw = normalize_angle(mission_start_yaw + mission_turn_delta_deg);
}

static void mission_reset(void)
{
    balance_set_expect_angle(0.0f);
    balance_set_heading_enabled(1U);
    mission_turn_delta_deg = MISSION_FIRST_TURN_DEG;
    mission_turn_progress_deg = 0.0f;
    mission_turn_last_yaw = 0.0f;
    mission_lean_bleed_start_angle = 0.0f;
    mission_lean_bleed_start_tick = 0U;
    mission_back_heading_start_tick = 0U;
    mission_next_full_turn_positive = 0U;
    mission_state = MISSION_IDLE;
}

static void mission_proc(void)
{
    float distance_m;
    float turn_step_deg;
    float turn_step_projected_deg;
    float turn_remaining_deg;
    float turn_sign;

    if (!motor_enabled)
    {
        mission_reset();
        return;
    }

    if (MISSION_IDLE == mission_state)
    {
        mission_start_first_task();
    }

    motor_set_target_speed(motor_speed_mps_to_counts(mission_run_speed_mps));

    switch (mission_state)
    {
        case MISSION_GO_STRAIGHT:
            distance_m = fabsf(motor_get_total_distance_m());
            if (distance_m >= MISSION_GO_DISTANCE_M)
            {
                mission_start_open_turn();
            }
            break;

        case MISSION_OPEN_TURN:
            turn_sign = (mission_turn_delta_deg >= 0.0f) ? 1.0f : -1.0f;
            turn_step_deg = normalize_angle(yaw - mission_turn_last_yaw);
            mission_turn_last_yaw = yaw;
            turn_step_projected_deg = MISSION_YAW_PROGRESS_SIGN * turn_sign * turn_step_deg;
            if (turn_step_projected_deg > 0.0f)
            {
                mission_turn_progress_deg += turn_step_projected_deg;
            }

            turn_remaining_deg = fabsf(mission_turn_delta_deg) - mission_turn_progress_deg;
            if (turn_remaining_deg <= MISSION_RELOCK_ERROR_DEG)
            {
                balance_set_heading_enabled(0U);
                mission_lean_bleed_start_angle = expect_angle;
                mission_lean_bleed_start_tick = uwtick;
                mission_state = MISSION_LEAN_BLEED;
            }
            break;

        case MISSION_LEAN_BLEED:
            if ((uint32_t)(uwtick - mission_lean_bleed_start_tick) >= MISSION_LEAN_BLEED_MS)
            {
                balance_set_expect_angle(0.0f);
                balance_set_yaw_target(mission_turn_target_yaw);
                balance_set_heading_enabled(1U);
                mission_back_heading_start_tick = uwtick;
                mission_state = MISSION_BACK_HEADING;
            }
            else
            {
                float bleed_ratio = (float)(uwtick - mission_lean_bleed_start_tick) /
                                    (float)MISSION_LEAN_BLEED_MS;
                balance_set_expect_angle(mission_lean_bleed_start_angle * (1.0f - bleed_ratio));
            }
            break;

        case MISSION_BACK_HEADING:
            if ((uint32_t)(uwtick - mission_back_heading_start_tick) >= MISSION_BACK_HEADING_HOLD_MS)
            {
                mission_start_yaw = mission_turn_target_yaw;
                if (mission_next_full_turn_positive)
                {
                    mission_set_turn_delta(MISSION_FULL_TURN_DEG);
                    mission_next_full_turn_positive = 0U;
                }
                else
                {
                    mission_set_turn_delta(-MISSION_FULL_TURN_DEG);
                    mission_next_full_turn_positive = 1U;
                }
                mission_start_open_turn();
            }
            break;

        default:
            break;
    }
}

void scheduler_init(void)
{
    task_num = (uint8_t)(sizeof(scheduler_task) / sizeof(task_t));
}

uint8 scheduler_get_mission_state(void)
{
    return (uint8)mission_state;
}

float scheduler_get_mission_start_yaw(void)
{
    return mission_start_yaw;
}

float scheduler_get_mission_turn_target_yaw(void)
{
    return mission_turn_target_yaw;
}

float scheduler_get_mission_open_turn_angle(void)
{
    return mission_open_turn_angle;
}

float scheduler_get_mission_run_speed_mps(void)
{
    return mission_run_speed_mps;
}

float scheduler_get_mission_turn_delta(void)
{
    return mission_turn_delta_deg;
}

float scheduler_get_mission_turn_progress(void)
{
    return mission_turn_progress_deg;
}

float scheduler_get_mission_turn_remaining(void)
{
    return fabsf(mission_turn_delta_deg) - mission_turn_progress_deg;
}

void scheduler_adjust_mission_open_turn_angle(float delta)
{
    mission_open_turn_angle += delta;

    if (mission_open_turn_angle < MISSION_OPEN_TURN_ANGLE_MIN)
    {
        mission_open_turn_angle = MISSION_OPEN_TURN_ANGLE_MIN;
    }
    if (mission_open_turn_angle > MISSION_OPEN_TURN_ANGLE_MAX)
    {
        mission_open_turn_angle = MISSION_OPEN_TURN_ANGLE_MAX;
    }
}

void scheduler_adjust_mission_run_speed_mps(float delta)
{
    mission_run_speed_mps += delta;

    if (mission_run_speed_mps < MISSION_RUN_SPEED_MIN_MPS)
    {
        mission_run_speed_mps = MISSION_RUN_SPEED_MIN_MPS;
    }
    if (mission_run_speed_mps > MISSION_RUN_SPEED_MAX_MPS)
    {
        mission_run_speed_mps = MISSION_RUN_SPEED_MAX_MPS;
    }
}

void scheduler_run(void)
{
    uint8_t i;
    uint32_t now_time = uwtick;

    for (i = 0U; i < task_num; i++)
    {
        if ((now_time - scheduler_task[i].last_run) >= scheduler_task[i].rate_ms)
        {
            scheduler_task[i].last_run = now_time;
            scheduler_task[i].task_func();
        }
    }
}

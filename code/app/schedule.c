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

#define MISSION_GO_DISTANCE_M       (9.0f)
#define MISSION_RUN_SPEED_MPS       (1.45f)
#define MISSION_OPEN_TURN_ANGLE     (10.0f)
#define MISSION_RELOCK_ERROR_DEG    (20.0f)
#define MISSION_LEAN_BLEED_MS       (300U)

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
static float mission_lean_bleed_start_angle = 0.0f;
static uint32_t mission_lean_bleed_start_tick = 0U;

static void mission_proc(void);
static void mission_start_first_task(void);
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
    mission_turn_target_yaw = normalize_angle(mission_start_yaw + 180.0f);

    encoder_feedback_reset();
    balance_set_expect_angle(0.0f);
    balance_set_yaw_target(mission_start_yaw);
    balance_set_heading_enabled(1U);
    motor_set_target_speed(motor_speed_mps_to_counts(MISSION_RUN_SPEED_MPS));

    mission_state = MISSION_GO_STRAIGHT;
}

static void mission_reset(void)
{
    balance_set_expect_angle(0.0f);
    balance_set_heading_enabled(1U);
    mission_lean_bleed_start_angle = 0.0f;
    mission_lean_bleed_start_tick = 0U;
    mission_state = MISSION_IDLE;
}

static void mission_proc(void)
{
    float distance_m;
    float turn_error_deg;

    if (!motor_enabled)
    {
        mission_reset();
        return;
    }

    if (MISSION_IDLE == mission_state)
    {
        mission_start_first_task();
    }

    motor_set_target_speed(motor_speed_mps_to_counts(MISSION_RUN_SPEED_MPS));

    switch (mission_state)
    {
        case MISSION_GO_STRAIGHT:
            distance_m = fabsf(motor_get_total_distance_m());
            if (distance_m >= MISSION_GO_DISTANCE_M)
            {
                balance_set_heading_enabled(0U);
                balance_set_expect_angle(MISSION_OPEN_TURN_ANGLE);
                mission_state = MISSION_OPEN_TURN;
            }
            break;

        case MISSION_OPEN_TURN:
            turn_error_deg = fabsf(normalize_angle(mission_turn_target_yaw - yaw));
            if (turn_error_deg <= MISSION_RELOCK_ERROR_DEG)
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

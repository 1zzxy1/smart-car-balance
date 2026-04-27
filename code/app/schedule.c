/*
 * schedule.c
 *
 * Cooperative scheduler for low-priority background tasks.
 */

#include "scheduler.h"

#include "hmi_app.h"

uint8_t task_num;
uint32_t uwtick = 0;

typedef struct
{
    void (*task_func)(void);
    uint32_t rate_ms;
    uint32_t last_run;
} task_t;

static task_t scheduler_task[] =
{
    {hmi_proc, 10U, 0U},
};

void scheduler_init(void)
{
    task_num = (uint8_t)(sizeof(scheduler_task) / sizeof(task_t));
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

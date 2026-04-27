/*
 * cpu0_main.c
 *
 * Minimal bring-up entry for the template-based bicycle project.
 */

#include "zf_common_headfile.h"
#include "hmi_app.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();

    pwm_init(ATOM1_CH1_P33_9, 330, mid);

    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    ips114_show_string(0, 0, "BOOT: LCD OK");
    ips114_show_string(0, 16, "STEP: IMU");

    imu_all_init();

    if (imu_ready)
    {
        ips114_show_string(0, 32, "IMU : OK");
    }
    else
    {
        ips114_show_string(0, 32, "IMU : FAIL");
    }

    ips114_show_string(0, 48, "STEP: BAL");
    balance_init();

    ips114_show_string(0, 64, "STEP: MOTOR");
    motor_init();

    ips114_show_string(0, 80, "STEP: HMI");

    wireless_uart_init();
    wireless_uart_send_string("BOOT OK\r\n");
    hmi_init();
    scheduler_init();

    pit_ms_init(CCU60_CH0, 1);
    pit_ms_init(CCU60_CH1, 1);   /* imu_proc + gyro 环 1ms */
    pit_ms_init(CCU61_CH0, 5);   /* angle 环 5ms */
    pit_ms_init(CCU61_CH1, 20);

    cpu_wait_event_ready();

    while (TRUE)
    {
        scheduler_run();
    }
}

#pragma section all restore

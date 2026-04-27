/*
 * small_driver_uart_control.h
 *
 * Brushless motor driver UART protocol wrapper.
 */

#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"

#define SMALL_DRIVER_UART_TEST_MODE (0)
#define SMALL_DRIVER_UART           (UART_3)
#define SMALL_DRIVER_BAUDRATE       (460800)
#define SMALL_DRIVER_RX             (UART3_TX_P15_6)
#define SMALL_DRIVER_TX             (UART3_RX_P15_7)

typedef struct
{
    uint8 send_data_buffer[7];
    uint8 receive_data_buffer[7];
    uint8 receive_data_count;
    uint8 sum_check_data;
    int16 receive_left_speed_data;
    int16 receive_right_speed_data;
} small_device_value_struct;

extern small_device_value_struct motor_value;
extern uint32 small_driver_frame_counter;

void uart_control_callback(void);
void small_driver_set_duty(int16 left_duty, int16 right_duty);
void small_driver_get_speed(void);
void small_driver_uart_init(void);

#endif /* SMALL_DRIVER_UART_CONTROL_H_ */

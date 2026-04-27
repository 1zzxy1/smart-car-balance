#include "small_driver_uart_control.h"

#include <string.h>

small_device_value_struct motor_value;
uint32 small_driver_frame_counter = 0U;

#if SMALL_DRIVER_UART_TEST_MODE
static const uint8 small_driver_test_message[] = "nihao\r\n";

static void small_driver_send_test_message(void)
{
    uart_write_buffer(SMALL_DRIVER_UART,
                      small_driver_test_message,
                      (uint32)(sizeof(small_driver_test_message) - 1U));
}
#endif

static void small_driver_reset_receive_state(void)
{
    motor_value.receive_data_count = 0U;
    memset(motor_value.receive_data_buffer, 0, sizeof(motor_value.receive_data_buffer));
}

static void small_driver_init(void)
{
    memset(&motor_value, 0, sizeof(motor_value));
    small_driver_frame_counter = 0U;
}

void uart_control_callback(void)
{
#if SMALL_DRIVER_UART_TEST_MODE
    uint8 receive_data;

    uart_query_byte(SMALL_DRIVER_UART, &receive_data);
    return;
#else
    uint8 receive_data;

    if (!uart_query_byte(SMALL_DRIVER_UART, &receive_data))
    {
        return;
    }

    if ((0xA5U == receive_data) && (0xA5U != motor_value.receive_data_buffer[0]))
    {
        motor_value.receive_data_count = 0U;
    }

    if (motor_value.receive_data_count >= sizeof(motor_value.receive_data_buffer))
    {
        small_driver_reset_receive_state();
    }

    motor_value.receive_data_buffer[motor_value.receive_data_count++] = receive_data;

    if (motor_value.receive_data_count < sizeof(motor_value.receive_data_buffer))
    {
        return;
    }

    if (0xA5U == motor_value.receive_data_buffer[0])
    {
        uint8 index;

        motor_value.sum_check_data = 0U;
        for (index = 0U; index < 6U; index++)
        {
            motor_value.sum_check_data += motor_value.receive_data_buffer[index];
        }

        if (motor_value.sum_check_data == motor_value.receive_data_buffer[6])
        {
            if (0x02U == motor_value.receive_data_buffer[1])
            {
                motor_value.receive_left_speed_data =
                    (int16)(((uint16)motor_value.receive_data_buffer[2] << 8) |
                            (uint16)motor_value.receive_data_buffer[3]);
                motor_value.receive_right_speed_data =
                    (int16)(((uint16)motor_value.receive_data_buffer[4] << 8) |
                            (uint16)motor_value.receive_data_buffer[5]);
                small_driver_frame_counter++;
            }
        }
    }

    small_driver_reset_receive_state();
#endif
}

void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
#if SMALL_DRIVER_UART_TEST_MODE
    (void)left_duty;
    (void)right_duty;
    return;
#else
    uint8 index;

    motor_value.send_data_buffer[0] = 0xA5U;
    motor_value.send_data_buffer[1] = 0x01U;
    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);
    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);
    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);
    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);
    motor_value.send_data_buffer[6] = 0U;

    for (index = 0U; index < 6U; index++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[index];
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7U);
#endif
}

void small_driver_get_speed(void)
{
#if SMALL_DRIVER_UART_TEST_MODE
    small_driver_send_test_message();
    return;
#else
    motor_value.send_data_buffer[0] = 0xA5U;
    motor_value.send_data_buffer[1] = 0x02U;
    motor_value.send_data_buffer[2] = 0x00U;
    motor_value.send_data_buffer[3] = 0x00U;
    motor_value.send_data_buffer[4] = 0x00U;
    motor_value.send_data_buffer[5] = 0x00U;
    motor_value.send_data_buffer[6] = 0xA7U;

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7U);
#endif
}

void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);

#if !SMALL_DRIVER_UART_TEST_MODE
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);
#endif

    small_driver_init();

#if SMALL_DRIVER_UART_TEST_MODE
    small_driver_send_test_message();
#else
    small_driver_set_duty(0, 0);
    small_driver_get_speed();
#endif
}

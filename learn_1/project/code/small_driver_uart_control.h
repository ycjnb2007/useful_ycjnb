#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"

////////////左侧无刷电机
#define SMALL_DRIVER_UART_LEFT                  (UART_2        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX_LEFT                    (UART2_TX_P10_1)

#define SMALL_DRIVER_TX_LEFT                    (UART2_RX_P10_0)


////////////右侧无刷电机
#define SMALL_DRIVER_UART_RIGHT                  (UART_4        )

#define SMALL_DRIVER_BAUDRATE                    (460800        )

#define SMALL_DRIVER_RX_RIGHT                    (UART4_TX_P14_1)

#define SMALL_DRIVER_TX_RIGHT                    (UART4_RX_P14_0)


////////////负压风扇
#define SMALL_DRIVER_UART_FAN                   (UART_1        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )
  
#define SMALL_DRIVER_RX_FAN                     (UART1_TX_P04_1)

#define SMALL_DRIVER_TX_FAN                     (UART1_RX_P04_0)




typedef struct
{
    uint8 send_data_buffer[7];                  // 发送缓冲数组

    uint8 receive_data_buffer[7];               // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_left_speed_data;              // 接收到的左电机速度数据

    int16 receive_right_speed_data;             // 接收到的右电机速度数据

}small_device_value_struct;

extern small_device_value_struct motor_value_left;      //左侧电机结构体
extern small_device_value_struct motor_value_right;     //右侧电机结构体
extern small_device_value_struct motor_value_fan;       //负压风扇结构体

////////////左侧无刷电机
void uart_control_callback_left(void);                                     // 无刷驱动 串口接收回调函数

void small_driver_set_duty_left(int16 left_duty, int16 right_duty);      // 无刷驱动 设置电机占空比

void small_driver_get_speed_left(void);                                   // 无刷驱动 获取速度信息

void small_driver_uart_init_left(void);                                  // 无刷驱动 串口通讯初始化




////////////右侧无刷电机
void uart_control_callback_right(void);                                     // 无刷驱动 串口接收回调函数

void small_driver_set_duty_right(int16 left_duty, int16 right_duty);      // 无刷驱动 设置电机占空比

void small_driver_get_speed_right(void);                                   // 无刷驱动 获取速度信息

void small_driver_uart_init_right(void);                                   // 无刷驱动 串口通讯初始化

////////////负压风扇
void small_driver_set_duty_fan(int16 left_duty, int16 right_duty);      // 无刷驱动 设置电机占空比
void small_driver_init_fan(void);
void small_driver_uart_init_fan(void);



#endif

#include "small_driver_uart_control.h"

small_device_value_struct motor_value_left;     // 定义通讯参数结构体
small_device_value_struct motor_value_right;    // 定义通讯参数结构体
small_device_value_struct motor_value_fan;      // 定义通讯参数结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_control_callback(1000, -1000);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback_left(void)
{
    uint8 receive_data;                                                                      // 定义临时变量

    if(uart_query_byte(SMALL_DRIVER_UART_LEFT, &receive_data))                                   // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value_left.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前接收内容中是否正确包含帧头
        {
            motor_value_left.receive_data_count = 0;                                             // 未收到帧头或者未正确包含帧头则重新接收
        }

        motor_value_left.receive_data_buffer[motor_value_left.receive_data_count ++] = receive_data;  // 保存串口数据

        if(motor_value_left.receive_data_count >= 7)                                             // 判断是否接收到指定数量的数据
        {
            if(motor_value_left.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {

                motor_value_left.sum_check_data = 0;                                             // 清除校验位数据

                for(int i = 0; i < 6; i ++)
                {
                    motor_value_left.sum_check_data += motor_value_left.receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value_left.sum_check_data == motor_value_left.receive_data_buffer[6])        // 校验数据准确性
                {

                    if(motor_value_left.receive_data_buffer[1] == 0x02)                          // 判断是否正确接收到 速度输出 功能字
                    {
                        motor_value_left.receive_left_speed_data  = (((int)motor_value_left.receive_data_buffer[2] << 8) | (int)motor_value_left.receive_data_buffer[3]);  // 拟合左侧电机转速数据

                        motor_value_left.receive_right_speed_data = (((int)motor_value_left.receive_data_buffer[4] << 8) | (int)motor_value_left.receive_data_buffer[5]);  // 拟合右侧电机转速数据
                    }

                    motor_value_left.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value_left.receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
                else
                {
                    motor_value_left.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value_left.receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
            }
            else
            {
                motor_value_left.receive_data_count = 0;                                         // 清除缓冲区计数值

                memset(motor_value_left.receive_data_buffer, 0, 7);                              // 清除缓冲区数据
            }
        }
    }
}





void uart_control_callback_right(void)
{
    uint8 receive_data;                                                                     // 定义临时变量

    if(uart_query_byte(SMALL_DRIVER_UART_RIGHT, &receive_data))                                   // 接收串口数据
    {
        if(receive_data == 0xA5 && motor_value_right.receive_data_buffer[0] != 0xA5)              // 判断是否收到帧头 并且 当前接收内容中是否正确包含帧头
        {
            motor_value_right.receive_data_count = 0;                                             // 未收到帧头或者未正确包含帧头则重新接收
        }

        motor_value_right.receive_data_buffer[motor_value_right.receive_data_count ++] = receive_data;  // 保存串口数据

        if(motor_value_right.receive_data_count >= 7)                                             // 判断是否接收到指定数量的数据
        {
            if(motor_value_right.receive_data_buffer[0] == 0xA5)                                  // 判断帧头是否正确
            {

                motor_value_right.sum_check_data = 0;                                             // 清除校验位数据

                for(int i = 0; i < 6; i ++)
                {
                    motor_value_right.sum_check_data += motor_value_right.receive_data_buffer[i];       // 重新计算校验位
                }

                if(motor_value_right.sum_check_data == motor_value_right.receive_data_buffer[6])        // 校验数据准确性
                {

                    if(motor_value_right.receive_data_buffer[1] == 0x02)                          // 判断是否正确接收到 速度输出 功能字
                    {
                        motor_value_right.receive_left_speed_data  = (((int)motor_value_right.receive_data_buffer[2] << 8) | (int)motor_value_right.receive_data_buffer[3]);  // 拟合左侧电机转速数据

                        motor_value_right.receive_right_speed_data = (((int)motor_value_right.receive_data_buffer[4] << 8) | (int)motor_value_right.receive_data_buffer[5]);  // 拟合右侧电机转速数据
                    }

                    motor_value_right.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value_right.receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
                else
                {
                    motor_value_right.receive_data_count = 0;                                     // 清除缓冲区计数值

                    memset(motor_value_right.receive_data_buffer, 0, 7);                          // 清除缓冲区数据
                }
            }
            else
            {
                motor_value_right.receive_data_count = 0;                                         // 清除缓冲区计数值

                memset(motor_value_right.receive_data_buffer, 0, 7);                              // 清除缓冲区数据
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置电机占空比
// 参数说明     left_duty       左侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 参数说明     right_duty      右侧电机占空比  范围 -10000 ~ 10000  负数为反转
// 返回参数     void
// 使用示例     small_driver_set_duty(1000, -1000);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty_left(int16 left_duty, int16 right_duty)
{
    motor_value_left.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value_left.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value_left.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 拆分 左侧占空比 的高八位

    motor_value_left.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 拆分 左侧占空比 的低八位

    motor_value_left.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 拆分 右侧占空比 的高八位

    motor_value_left.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 拆分 右侧占空比 的低八位

    motor_value_left.send_data_buffer[6] = 0;                                            // 和校验清除

    for(int i = 0; i < 6; i ++)
    {
        motor_value_left.send_data_buffer[6] += motor_value_left.send_data_buffer[i];         // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART_LEFT, motor_value_left.send_data_buffer, 7);                     // 发送设置占空比的 字节包 数据
}



void small_driver_set_duty_right(int16 left_duty, int16 right_duty)
{
    motor_value_right.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value_right.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value_right.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 拆分 左侧占空比 的高八位

    motor_value_right.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 拆分 左侧占空比 的低八位

    motor_value_right.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 拆分 右侧占空比 的高八位

    motor_value_right.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 拆分 右侧占空比 的低八位

    motor_value_right.send_data_buffer[6] = 0;                                            // 和校验清除

    for(int i = 0; i < 6; i ++)
    {
        motor_value_right.send_data_buffer[6] += motor_value_right.send_data_buffer[i];         // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART_RIGHT, motor_value_right.send_data_buffer, 7);                     // 发送设置占空比的 字节包 数据
}





void small_driver_set_duty_fan(int16 left_duty, int16 right_duty)
{
    motor_value_fan.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value_fan.send_data_buffer[1] = 0X01;                                         // 配置功能字

    motor_value_fan.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 拆分 左侧占空比 的高八位

    motor_value_fan.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 拆分 左侧占空比 的低八位

    motor_value_fan.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 拆分 右侧占空比 的高八位

    motor_value_fan.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 拆分 右侧占空比 的低八位

    motor_value_fan.send_data_buffer[6] = 0;                                            // 和校验清除

    for(int i = 0; i < 6; i ++)
    {
        motor_value_fan.send_data_buffer[6] += motor_value_fan.send_data_buffer[i];         // 计算校验位
    }

    uart_write_buffer(SMALL_DRIVER_UART_FAN, motor_value_fan.send_data_buffer, 7);                     // 发送设置占空比的 字节包 数据
}






//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取速度信息
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_get_speed();
// 备注信息     仅需发送一次 驱动将周期发出速度信息(默认10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed_left(void)
{
    motor_value_left.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value_left.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value_left.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value_left.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value_left.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value_left.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value_left.send_data_buffer[6] = 0xA7;                                         // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART_LEFT, motor_value_left.send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
}




void small_driver_get_speed_right(void)
{
    motor_value_right.send_data_buffer[0] = 0xA5;                                         // 配置帧头

    motor_value_right.send_data_buffer[1] = 0X02;                                         // 配置功能字

    motor_value_right.send_data_buffer[2] = 0x00;                                         // 数据位清空

    motor_value_right.send_data_buffer[3] = 0x00;                                         // 数据位清空

    motor_value_right.send_data_buffer[4] = 0x00;                                         // 数据位清空

    motor_value_right.send_data_buffer[5] = 0x00;                                         // 数据位清空

    motor_value_right.send_data_buffer[6] = 0xA7;                                         // 配置校验位

    uart_write_buffer(SMALL_DRIVER_UART_RIGHT, motor_value_right.send_data_buffer, 7);                     // 发送获取转速数据的 字节包 数据
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 参数初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init_left(void)
{
    memset(motor_value_left.send_data_buffer, 0, 7);                             // 清除缓冲区数据

    memset(motor_value_left.receive_data_buffer, 0, 7);                          // 清除缓冲区数据

    motor_value_left.receive_data_count          = 0;

    motor_value_left.sum_check_data              = 0;

    motor_value_left.receive_right_speed_data    = 0;

    motor_value_left.receive_left_speed_data     = 0;
}






void small_driver_init_right(void)
{
    memset(motor_value_right.send_data_buffer, 0, 7);                             // 清除缓冲区数据

    memset(motor_value_right.receive_data_buffer, 0, 7);                          // 清除缓冲区数据

    motor_value_right.receive_data_count          = 0;

    motor_value_right.sum_check_data              = 0;

    motor_value_right.receive_right_speed_data    = 0;

    motor_value_right.receive_left_speed_data     = 0;
}




void small_driver_init_fan(void)
{
    memset(motor_value_fan.send_data_buffer, 0, 7);                             // 清除缓冲区数据

    memset(motor_value_fan.receive_data_buffer, 0, 7);                          // 清除缓冲区数据

    motor_value_fan.receive_data_count          = 0;

    motor_value_fan.sum_check_data              = 0;

    motor_value_fan.receive_right_speed_data    = 0;

    motor_value_fan.receive_left_speed_data     = 0;
}





//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例 SMALL_DRIVER_UART_LEFTuart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------


////////////左侧无刷电机
void small_driver_uart_init_left(void)
{
    uart_init(SMALL_DRIVER_UART_LEFT, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX_LEFT, SMALL_DRIVER_TX_LEFT);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART_LEFT, 1);                                                    // 使能串口接收中断

    small_driver_init_left();                                                                        // 结构体参数初始化

    small_driver_set_duty_left(0, 0);                                                                // 设置0占空比

    small_driver_get_speed_left();                                                                   // 获取实时速度数据
}

////////////右侧无刷电机
void small_driver_uart_init_right(void)
{
    uart_init(SMALL_DRIVER_UART_RIGHT, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX_RIGHT, SMALL_DRIVER_TX_RIGHT);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART_RIGHT, 1);                                                    // 使能串口接收中断

    small_driver_init_right();                                                                        // 结构体参数初始化

    small_driver_set_duty_right(0, 0);                                                                // 设置0占空比

    small_driver_get_speed_right();                                                                   // 获取实时速度数据
}
////////////负压风扇//不需要数据回传
void small_driver_uart_init_fan(void)
{
    uart_init(SMALL_DRIVER_UART_FAN, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX_FAN, SMALL_DRIVER_TX_FAN);      // 串口初始化

//    uart_rx_interrupt(SMALL_DRIVER_UART_FAN, 1);                                                    // 使能串口接收中断

    small_driver_init_fan();                                                                        // 结构体参数初始化

    small_driver_set_duty_fan(0, 0);                                                                // 设置0占空比

//    small_driver_get_speed_fan();                                                                   // 获取实时速度数据
}


#include "Encoder.h"

// 编码器滤波参数
#define ENCODER_FILTER_ALPHA    0.90f    // 编码器低通滤波系数(0-1)，越大表示新数据权重越大
#define ENCODER_FILTER_LIMIT    50      // 编码器限幅滤波阈值

// 限幅低通滤波器
int16 encoder_filter(int16 new_value, int16 old_value) 
{
    // 限幅滤波
    if(func_abs(new_value - old_value) > ENCODER_FILTER_LIMIT) 
    {
        if(new_value > old_value)
        {
            new_value = old_value + ENCODER_FILTER_LIMIT;
        } else {
            new_value = old_value - ENCODER_FILTER_LIMIT;
        }
    }
    
    // 低通滤波
    return (int16)(ENCODER_FILTER_ALPHA * new_value + (1-ENCODER_FILTER_ALPHA) * old_value);
}

int16 SPEED,speed;
void encoder_add()
{
    // 对编码器速度进行滤波

    // 使用滤波后的速度计算平均速度
    SPEED = (func_abs(motor_value_left.receive_left_speed_data) + func_abs(motor_value_right.receive_right_speed_data)
             +func_abs(motor_value_left.receive_right_speed_data)+func_abs(motor_value_right.receive_left_speed_data)) / 4;
    total_pulses+=SPEED;
    
}

void speed_update()
{
        speed = (func_abs(motor_value_left.receive_left_speed_data) + func_abs(motor_value_right.receive_right_speed_data)
             +func_abs(motor_value_left.receive_right_speed_data)+func_abs(motor_value_right.receive_left_speed_data)) / 4;
}



/*
 * imu660.c
 *
 *  Created on: 2025年12月22日
 *      Author: ycj01
*/



#include "imu660.h"

//#define pi (3.14f)
uint8_t gyro_zero_flag = 0, acc_zero_flag=0; // 零飘标定完成标志位初始化
volatile float yaw_plus = 0; // 小车机体坐标系y轴与东天北or全局坐标系y轴夹角(逆时针为正，顺时针为负，范围[-pi,pi])

volatile float yaw = 0;                    // 实时航偏角初始化
volatile float yaw_last = 0;               // 中值积分航偏角前后状态初始化
volatile float yaw_now;                    // 中值积分航偏角前后状态初始化

volatile float acc_y_speed = 0;      // 实时小车实际速度初始化
volatile float acc_last = 0;         // 中值积分加速度前后状态初始化
volatile float acc_now;      // 中值积分加速度前后状态初始化

//float yaw_memery[FLASH_PAGE_LENGTH] = {0}; // 记录的航偏角
//float yaw_store[FLASH_PAGE_LENGTH] = {0};  // 打点储存的历史航偏角
gyro_zero_paramTypedef gyro_zero_param;    // 零飘参数
acc_zero_paramTypedef acc_zero_param;      // 零飘参数

volatile gyro_paramTypedef gyro_param = {
    .gyro_x = 0,
    .gyro_y = 0,
    .gyro_z = 0

}; // 角加速度
volatile acc_paramTypedef acc_param = {
    .acc_x = 0,
    .acc_y = 0,
    .acc_z = 0
}; // 加速度计

// 零飘标定
void gyro_zero_param_init(void)
{

    gyro_zero_param.Xdata = 0;
    gyro_zero_param.Ydata = 0; // 零飘参数
    gyro_zero_param.Zdata = 0;
    for (uint16_t i = 0; i < 100; i++)
    {
        imu660rb_get_gyro();
        gyro_zero_param.Xdata += imu660rb_gyro_x;
        gyro_zero_param.Ydata += imu660rb_gyro_y;
        gyro_zero_param.Zdata += imu660rb_gyro_z;
        system_delay_ms(5);
    }
    gyro_zero_param.Xdata /= 100;
    gyro_zero_param.Ydata /= 100;
    gyro_zero_param.Zdata /= 100;
    gyro_zero_flag = 1; // 零飘标定完成,大概需要1s左右
}

void acc_zero_param_init(void)
{

    acc_zero_param.Xdata = 0;
    acc_zero_param.Ydata = 0; // 零飘参数
    acc_zero_param.Zdata = 0;
    for (uint16_t i = 0; i < 100; i++)
    {
        imu660rb_get_acc();
        acc_zero_param.Xdata += imu660rb_acc_x;
        acc_zero_param.Ydata += imu660rb_acc_y;
        acc_zero_param.Zdata += imu660rb_acc_z;
        system_delay_ms(5);
    }
    acc_zero_param.Xdata /= 100;
    acc_zero_param.Ydata /= 100;
    acc_zero_param.Zdata /= 100;
    acc_zero_flag = 1; // 零飘标定完成,大概需要1s左右
}

// 单位转化为度数
void gyro_transform_value(void)
{
    gyro_param.gyro_x = imu660rb_gyro_transition((float)imu660rb_gyro_x - gyro_zero_param.Xdata);
    gyro_param.gyro_y = imu660rb_gyro_transition((float)imu660rb_gyro_y - gyro_zero_param.Ydata);
    gyro_param.gyro_z = imu660rb_gyro_transition((float)imu660rb_gyro_z - gyro_zero_param.Zdata);
    // 转为实际物理值，单位度
}

// 单位转化为度数
void acc_transform_value(void)
{
    acc_param.acc_x = imu660rb_acc_transition((float)imu660rb_acc_x - acc_zero_param.Xdata);
    acc_param.acc_y = imu660rb_acc_transition((float)imu660rb_acc_y - acc_zero_param.Ydata);
    acc_param.acc_z = imu660rb_acc_transition((float)imu660rb_acc_z - acc_zero_param.Zdata);
    // 转为实际物理值，单位m^2/s
}

// 中值积分算角度
void gyro_yaw_integral(void)
{

    if (fabsf(gyro_param.gyro_z) < 5.0f)
        gyro_param.gyro_z = 0;

    yaw_now = gyro_param.gyro_z;
    yaw += -(yaw_last + yaw_now) / 2 * 0.005f * 0.0633f; // 中值积分法



    yaw_last = yaw_now;

    yaw_plus += ((yaw_last + yaw_now) / 2 * 0.005f * 0.0633f) * pi / 180.00f; // 逆时针为正，顺时针为负

    if (yaw_plus >= pi)
    {
        yaw_plus -= 2 * pi;
    }
    else if (yaw_plus < -pi)
    {
        yaw_plus += 2 * pi;
    }
// 规范到[-180,180]度
}


// 中值积分算速度
void acc_y_integral(void)
{

    if (fabsf(acc_param.acc_y) < 50.0f)
        acc_param.acc_y = 0;

    acc_param.acc_y = LPF_Update(&velocity_filter, acc_param.acc_y);

    acc_now = acc_param.acc_y;
    acc_y_speed += (acc_last + acc_now) / 2 * 0.005f*0.01f; // 中值积分法

    acc_last = acc_now;


}

// 偏航角重置函数 - 在需要重置偏航角时调用此函数
void reset_yaw_integral(void)
{
    // 重置所有相关变量
    yaw = 0.0f;
    yaw_plus = 0.0f;
    yaw_last = 0.0f;
    yaw_now = 0.0f;

    // 重置积分中间状态（关键！）
    // 获取当前陀螺仪值作为新的起点
    imu660rb_get_gyro();  // 确保已读取最新数据
    gyro_transform_value();  // 确保已转换值
    yaw_last = gyro_param.gyro_z;  // 设置为当前值
}







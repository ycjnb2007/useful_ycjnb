

/*
 * imu660.c
 *
 *  Created on: 2025年12月22日
 *      Author: ycj01
*/



#include "imu660.h"

//#define pi (3.14f)
uint8_t gyro_zero_flag = 0, acc_zero_flag=0; // 零飘标定完成标志位初始化
volatile float yaw_plus = 0; // 绝对偏航角（电子指南针），标定车头为0度

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
    .gyro_z = 0   //Z 轴角速度（转弯瞬时速度）

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
    // 【强制加括号】防止宏定义展开时运算优先级出错
    gyro_param.gyro_x = imu660rb_gyro_transition( ((float)imu660rb_gyro_x - gyro_zero_param.Xdata) );
    gyro_param.gyro_y = imu660rb_gyro_transition( ((float)imu660rb_gyro_y - gyro_zero_param.Ydata) );
    gyro_param.gyro_z = -imu660rb_gyro_transition( ((float)imu660rb_gyro_z - gyro_zero_param.Zdata) );//板子上z轴是反的，加了个负号
}
// 单位转化为度数
void acc_transform_value(void)
{
    acc_param.acc_x = imu660rb_acc_transition((float)imu660rb_acc_x - acc_zero_param.Xdata);
    acc_param.acc_y = imu660rb_acc_transition((float)imu660rb_acc_y - acc_zero_param.Ydata);
    acc_param.acc_z = imu660rb_acc_transition((float)imu660rb_acc_z - acc_zero_param.Zdata);
    // 转为实际物理值，单位m^2/s
}



// 中值积分算角度 (结合逐飞思路重写版)
void gyro_yaw_integral(void)
{
    // 【直接注释掉死区】相信我们前面正确的零偏标定
        // if (fabsf(gyro_param.gyro_z) < 1.0f) {
        //     gyro_param.gyro_z = 0;
        // }

    yaw_now = gyro_param.gyro_z;

    // 2. 纯净的中值积分：(上次角速度 + 本次角速度)/2 * 中断时间 dt
    // 注意：假设你的中断是 5ms，所以 dt = 0.005f。千万别再乘别的奇怪常数了！
    // 前面的负号取决于你的陀螺仪安装方向，如果打反了，去掉负号即可。
    float delta_yaw = (yaw_last + yaw_now) / 2.0f * 0.005f;

    // 3. 相对偏航角 (无限累加)
    yaw += delta_yaw;

    // 4. 绝对电子罗盘角 yaw_plus (强制在 -180 到 180 度之间)
    // 专门用来配合逐飞推文做 90 度直角转弯判断！
    yaw_plus += delta_yaw;

    if (yaw_plus > 180.0f) {
        yaw_plus -= 360.0f;
    } else if (yaw_plus < -180.0f) {
        yaw_plus += 360.0f;
    }

    yaw_last = yaw_now;
}

// 偏航角重置函数 (发车时，或者识别到特征点时可以重置，消除累计误差)
void reset_yaw_integral(void)
{
    yaw = 0.0f;
    yaw_plus = 0.0f;
    yaw_last = 0.0f;
    yaw_now = 0.0f;

    imu660rb_get_gyro();
    gyro_transform_value();
    yaw_last = gyro_param.gyro_z;
}





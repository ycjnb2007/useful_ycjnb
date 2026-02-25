#include "gyroscope.h"


// 定义参数
#define SENSITIVITY   0.061035f  // 示例：度/秒每LSB (±250dps范围)
#define DT            0.005f     // 采样间隔5ms
#define ALPHA         0.8f      // 低通滤波系数

float offset_z = 0.0f;         // 零偏校准值
float filtered_omega_z = 0.0f; // 滤波后的角速度
float theta_total = 0.0f;      // 累计角度
//
//// 初始化：校准零偏（需在静止时执行）
//void calibrate_gyro() 
//{
//    int32_t sum = 0;
//    for (int i = 0; i < 1000; i++) 
//    {
//        sum += imu660ra_gyro_z; // 读取原始Z轴数据
//    }
//    offset_z = sum / 1000.0f;
//}

// 主循环或定时中断处理
void update_angle()
{
    // 读取原始数据
    int16_t raw_z = imu660ra_gyro_z;
    
    // 转换为角速度（度/秒）
    float omega_z = (raw_z - offset_z) * SENSITIVITY;
    
    // 低通滤波
    filtered_omega_z = ALPHA * filtered_omega_z + (1 - ALPHA) * omega_z;
    
    // 计算角度增量并累加
    theta_total += filtered_omega_z * DT;
    
}
#include "quaternion.h"
#include "zf_device_imu660ra.h"
#include <math.h>

#define DEG_TO_RAD       (0.01745329251994329576923690768489f)
#define RAD_TO_DEG       (57.295779513082320876798154814105f)
#define CALIBRATION_SAMPLES 2000  // 校准采样次数（约1秒）

static Quaternion quat = 
{
    .q0 = 1.0f, 
    .calibrated = 0,
    .calibration_samples = 0
};

void quaternion_init(void) 
{
    quat.q0 = 1.0f;
    quat.q1 = 0.0f;
    quat.q2 = 0.0f;
    quat.q3 = 0.0f;
    quat.gx_bias = 0;
    quat.gy_bias = 0;
    quat.gz_bias = 0;
    quat.calibrated = 0;
    quat.calibration_samples = 0;
}

void quaternion_calibrate(void) 
{
    quat.calibration_samples = CALIBRATION_SAMPLES;
    quat.gx_bias = 0;
    quat.gy_bias = 0;
    quat.gz_bias = 0;
    quat.calibrated = 0;
}

void quaternion_update(int16 imu660ra_gyro_x, int16 imu660ra_gyro_y, int16 imu660ra_gyro_z, float dt) 
{
  
  
  
  
    // 获取原始数据并转换为°/s
    float gx = imu660ra_gyro_transition((imu660ra_gyro_x/20)*20);//丢掉一点精度去零飘
    float gy = imu660ra_gyro_transition((imu660ra_gyro_y/20)*20);
    float gz = imu660ra_gyro_transition((imu660ra_gyro_z/20)*20);
    // 校准阶段数据采集
    if(quat.calibration_samples > 0) 
    {
        quat.gx_bias += gx;
        quat.gy_bias += gy;
        quat.gz_bias += gz;
        quat.calibration_samples--;
        
        if(quat.calibration_samples == 0)
        {
            quat.gx_bias /= CALIBRATION_SAMPLES;
            quat.gy_bias /= CALIBRATION_SAMPLES;
            quat.gz_bias /= CALIBRATION_SAMPLES;
            quat.calibrated = 1;
        }
        return;
    }

    // 应用零偏校准
    if(quat.calibrated) 
    {
        gx -= quat.gx_bias;
        gy -= quat.gy_bias;
        gz -= quat.gz_bias;
    }

    // 转换为rad/s并计算半角
    gx *= 0.5f * DEG_TO_RAD;
    gy *= 0.5f * DEG_TO_RAD;
    gz *= 0.5f * DEG_TO_RAD;

    // 四元数微分方程（一阶龙格库塔法）
    float q0i = quat.q0 + (-quat.q1*gx - quat.q2*gy - quat.q3*gz)*dt;
    float q1i = quat.q1 + (quat.q0*gx + quat.q2*gz - quat.q3*gy)*dt;
    float q2i = quat.q2 + (quat.q0*gy - quat.q1*gz + quat.q3*gx)*dt;
    float q3i = quat.q3 + (quat.q0*gz + quat.q1*gy - quat.q2*gx)*dt;

    // 归一化处理
    float norm = sqrtf(q0i*q0i + q1i*q1i + q2i*q2i + q3i*q3i);
    quat.q0 = q0i / norm;
    quat.q1 = q1i / norm;
    quat.q2 = q2i / norm;
    quat.q3 = q3i / norm;

    // 转换为欧拉角（ZYX顺序）
    quat.roll  = atan2f(2*(quat.q0*quat.q1 + quat.q2*quat.q3), 
                       1 - 2*(quat.q1*quat.q1 + quat.q2*quat.q2)) * RAD_TO_DEG;
    quat.pitch = asinf(2*(quat.q0*quat.q2 - quat.q3*quat.q1)) * RAD_TO_DEG;
// 修改为（添加角度连续化处理）：
    float raw_yaw = atan2f(2*(quat.q0*quat.q3 + quat.q1*quat.q2), 
                   1 - 2*(quat.q2*quat.q2 + quat.q3*quat.q3)) * RAD_TO_DEG;

// 将yaw角转换为0-360度范围
quat.yaw = fmodf(raw_yaw + 360.0f, 360.0f);
}

void quaternion_reset(void)
{
    // 重置四元数到初始状态
    quat.q0 = 1.0f;
    quat.q1 = 0.0f;
    quat.q2 = 0.0f;
    quat.q3 = 0.0f;
    
    // 重置欧拉角
    quat.roll = 0.0f;
    quat.pitch = 0.0f;
    quat.yaw = 0.0f;
    
    // 不重置校准状态和零偏值，保留开机时的校准结果
}


Quaternion* get_euler_angles(void)
{
    return &quat;
}
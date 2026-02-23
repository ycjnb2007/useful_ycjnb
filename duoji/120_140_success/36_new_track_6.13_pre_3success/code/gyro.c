#include "zf_common_headfile.h"


EulerAngles angles;
AttitudeEstimator estimator;


float final_yaw[YAW_NUM] =  {0};
uint8 final_yaw_num = 0;

/** 消除零偏
 * 
 * 
 * 
 * 
 * 
 **/

float gyro_bias_x = 0;
float gyro_bias_y = 0;
float gyro_bias_z = 0;

void calibrate_gyro_bias(float* bias_x, float* bias_y, float* bias_z) 
{
    const int samples = 500; // 采集500个样本
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    for(int i=0; i<samples; i++) {
        imu660ra_get_gyro();
        sum_x += imu660ra_gyro_transition(imu660ra_gyro_x);
        sum_y += imu660ra_gyro_transition(imu660ra_gyro_y);
        sum_z += imu660ra_gyro_transition(imu660ra_gyro_z);
        system_delay_ms(10); // 10ms间隔
    }
    
    *bias_x = sum_x / samples;
    *bias_y = sum_y / samples;
    *bias_z = sum_z / samples;
}




/** 初始化姿态估计器
 * 参数：
 *   estimator: 姿态估计器对象指针
 *   Kp: 比例增益（建议值2.0-10.0）
 *   Ki: 积分增益（建议值0.001-0.1）
 *   dt: 采样周期（单位秒，需与实际调用周期一致）
    示例：    attitude_estimator_init(&estimator, 2.0f, 0.005f, 0.01f); // Kp=2, Ki=0.005, dt=10ms

 **/

void attitude_estimator_init(AttitudeEstimator* estimator, 
                            float Kp, float Ki, 
                            float dt) 
{
    estimator->quaternion.q0 = 1.0f;
    estimator->quaternion.q1 = 0.0f;
    estimator->quaternion.q2 = 0.0f;
    estimator->quaternion.q3 = 0.0f;
    
    estimator->Kp = Kp;
    estimator->Ki = Ki;
    estimator->dt = dt;
    
    estimator->integral_error.ex = 0.0f;
    estimator->integral_error.ey = 0.0f;
    estimator->integral_error.ez = 0.0f;
}


/** 姿态更新函数（需按固定周期调用）
 * 流程：
 * 1. 读取加速度计数据并归一化
 * 2. 读取陀螺仪数据并转换为rad/s
 * 3. 通过加速度计计算姿态误差
 * 4. 使用PI控制器补偿陀螺仪漂移
 * 5. 更新四元数
 * 6. 四元数归一化处理
 **/
void attitude_update(AttitudeEstimator* estimator) 
{
    // 获取加速度计数据
    imu660ra_get_acc();
    float ax = imu660ra_acc_transition(imu660ra_acc_x);
    float ay = imu660ra_acc_transition(imu660ra_acc_y);
    float az = imu660ra_acc_transition(imu660ra_acc_z);

    // 归一化加速度
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    // 获取陀螺仪数据（转换为rad/s）
    imu660ra_get_gyro();
    float gx = imu660ra_gyro_transition(imu660ra_gyro_x) * M_PI / 180.0f;
    float gy = imu660ra_gyro_transition(imu660ra_gyro_y) * M_PI / 180.0f;
    float gz = imu660ra_gyro_transition(imu660ra_gyro_z) * M_PI / 180.0f;
//      float gx = (imu660ra_gyro_transition(imu660ra_gyro_x) - gyro_bias_x) * M_PI / 180.0f;
//      float gy = (imu660ra_gyro_transition(imu660ra_gyro_y) - gyro_bias_y) * M_PI / 180.0f;
//      float gz = (imu660ra_gyro_transition(imu660ra_gyro_z) - gyro_bias_z) * M_PI / 180.0f;
    /* 姿态解算核心算法（同原始代码逻辑）*/
    // 计算预测重力方向
     // ----- 核心算法：Mahony互补滤波 -----
    // 步骤1：根据当前四元数计算预测的重力方向（vx, vy, vz）
    // 重力向量在世界坐标系为[0,0,1]，通过四元数旋转到机体坐标系
    float vx = 2.0f * (estimator->quaternion.q1*estimator->quaternion.q3 - estimator->quaternion.q0*estimator->quaternion.q2);
    float vy = 2.0f * (estimator->quaternion.q0*estimator->quaternion.q1 + estimator->quaternion.q2*estimator->quaternion.q3);
    float vz = estimator->quaternion.q0*estimator->quaternion.q0 
             - estimator->quaternion.q1*estimator->quaternion.q1 
             - estimator->quaternion.q2*estimator->quaternion.q2 
             + estimator->quaternion.q3*estimator->quaternion.q3;

    // 计算误差
   // 步骤2：计算加速度计测量值与预测值的误差（叉乘）
  // 误差方向即为需要旋转来校正的方向
    float ex = (ay * vz) - (az * vy);// 绕X轴误差
    float ey = (az * vx) - (ax * vz);// 绕Y轴误差
    float ez = (ax * vy) - (ay * vx);// 绕Z轴误差

    // 步骤3：积分误差用于补偿陀螺仪零漂
    estimator->integral_error.ex += ex * estimator->Ki * estimator->dt;
    estimator->integral_error.ey += ey * estimator->Ki * estimator->dt;
    estimator->integral_error.ez += ez * estimator->Ki * estimator->dt;

    // 步骤4：应用PI补偿（调整陀螺仪读数）
    gx += estimator->Kp * ex + estimator->integral_error.ex;
    gy += estimator->Kp * ey + estimator->integral_error.ey;
    gz += estimator->Kp * ez + estimator->integral_error.ez;

    //保存当前四元数用于后续计算（避免更新过程中值被修改）
    float q0 = estimator->quaternion.q0;
    float q1 = estimator->quaternion.q1;
    float q2 = estimator->quaternion.q2;
    float q3 = estimator->quaternion.q3;

    // 四元数更新
    // 步骤5：四元数微分方程更新（一阶龙格库塔法）
    // 四元数导数公式：dq/dt = 0.5 * q 叉乘 [0, ω]
    estimator->quaternion.q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * estimator->dt;
    estimator->quaternion.q1 += 0.5f * ( q0*gx + q2*gz - q3*gy) * estimator->dt;
    estimator->quaternion.q2 += 0.5f * ( q0*gy - q1*gz + q3*gx) * estimator->dt;
    estimator->quaternion.q3 += 0.5f * ( q0*gz + q1*gy - q2*gx) * estimator->dt;

    // 归一化处理
     // 步骤6：四元数归一化（防止数值发散）
    norm = sqrtf(estimator->quaternion.q0*estimator->quaternion.q0 +
                estimator->quaternion.q1*estimator->quaternion.q1 +
                estimator->quaternion.q2*estimator->quaternion.q2 +
                estimator->quaternion.q3*estimator->quaternion.q3);
    if (norm > 0.0f) {
        estimator->quaternion.q0 /= norm;
        estimator->quaternion.q1 /= norm;
        estimator->quaternion.q2 /= norm;
        estimator->quaternion.q3 /= norm;
    }
}


/**从四元数获取欧拉角（单位：度）**/
EulerAngles get_euler_angles(const AttitudeEstimator* estimator) 
{
    EulerAngles angles;
    
    // 计算横滚角（绕X轴）
    angles.roll = atan2f(2.0f * (estimator->quaternion.q0*estimator->quaternion.q1 + 
                               estimator->quaternion.q2*estimator->quaternion.q3),
                       1.0f - 2.0f * (estimator->quaternion.q1*estimator->quaternion.q1 + 
                                    estimator->quaternion.q2*estimator->quaternion.q2));
    
    // 计算俯仰角（绕Y轴）
    angles.pitch = asinf(2.0f * (estimator->quaternion.q0*estimator->quaternion.q2 - 
                               estimator->quaternion.q3*estimator->quaternion.q1));
    
    // 计算偏航角（绕Z轴）
    angles.yaw = atan2f(2.0f * (estimator->quaternion.q0*estimator->quaternion.q3 + 
                              estimator->quaternion.q1*estimator->quaternion.q2),
                      1.0f - 2.0f * (estimator->quaternion.q2*estimator->quaternion.q2 + 
                                    estimator->quaternion.q3*estimator->quaternion.q3));
    
    // 转换为角度
    angles.roll *= 180.0f / M_PI;
    angles.pitch *= 180.0f / M_PI;
    angles.yaw *= 180.0f / M_PI;
    
    return angles;
}

// 四元数乘法 q_out = q * r
static void quaternion_multiply(Quaternion* q_out, 
                               const Quaternion* q, 
                               const Quaternion* r) 
{
    q_out->q0 = q->q0 * r->q0 - q->q1 * r->q1 - q->q2 * r->q2 - q->q3 * r->q3;
    q_out->q1 = q->q0 * r->q1 + q->q1 * r->q0 + q->q2 * r->q3 - q->q3 * r->q2;
    q_out->q2 = q->q0 * r->q2 - q->q1 * r->q3 + q->q2 * r->q0 + q->q3 * r->q1;
    q_out->q3 = q->q0 * r->q3 + q->q1 * r->q2 - q->q2 * r->q1 + q->q3 * r->q0;
}

// 生成绕Z轴旋转四元数
static void quaternion_from_yaw(Quaternion* q, float yaw_rad) 
{
    float half_yaw = yaw_rad * 0.5f;
    q->q0 = cosf(half_yaw);
    q->q1 = 0.0f;
    q->q2 = 0.0f;
    q->q3 = sinf(half_yaw);
}


void reset_yaw_to_zero(AttitudeEstimator* estimator) 
{
    // 获取当前偏航角（度）
    EulerAngles current_angles = get_euler_angles(estimator);
    float yaw_rad = current_angles.yaw * (M_PI / 180.0f);

    // 生成反向旋转四元数
    Quaternion inverse_yaw_quat;
    quaternion_from_yaw(&inverse_yaw_quat, -yaw_rad);

    // 应用旋转到当前四元数
    Quaternion new_quat;
    quaternion_multiply(&new_quat, 
                       &(estimator->quaternion), 
                       &inverse_yaw_quat);

    // 更新四元数并归一化
    float norm = sqrtf(new_quat.q0*new_quat.q0 + 
                      new_quat.q1*new_quat.q1 + 
                      new_quat.q2*new_quat.q2 + 
                      new_quat.q3*new_quat.q3);
    if (norm > 0.0f) {
        estimator->quaternion.q0 = new_quat.q0 / norm;
        estimator->quaternion.q1 = new_quat.q1 / norm;
        estimator->quaternion.q2 = new_quat.q2 / norm;
        estimator->quaternion.q3 = new_quat.q3 / norm;
    }

    // 清除积分误差（防止残留误差影响）
    estimator->integral_error.ex = 0.0f;
    estimator->integral_error.ey = 0.0f;
    estimator->integral_error.ez = 0.0f;
}



/*
 徐州工程学院
 */
#include "imu.h"

static float NormAcc;

typedef volatile struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;
Quaternion NumQ = {1, 0, 0, 0};

struct V
{
    float x;
    float y;
    float z;
};

volatile struct V GyroIntegError = {0};

void imu_rest(void)
{
    NumQ.q0 = 1;
    NumQ.q1 = 0;
    NumQ.q2 = 0;
    NumQ.q3 = 0;
    GyroIntegError.x = 0;
    GyroIntegError.y = 0;
    GyroIntegError.z = 0;
    Angle.pitch = 0;
    Angle.roll = 0;
}

void GetAngle(const _st_Mpu *pMpu, _st_AngE *pAngE, float dt)
{

    volatile struct V Gravity, Acc, Gyro, AccGravity;

    static float KpDef = 0.5f;
    static float KiDef = 0.0003f;

    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 提取等效旋转矩阵中的重力分量
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // 加速度归一化
    NormAcc = Q_rsqrt(squa(pMpu->accX) + squa(pMpu->accY) + squa(pMpu->accZ));

    Acc.x = pMpu->accX * NormAcc;
    Acc.y = pMpu->accY * NormAcc;
    Acc.z = pMpu->accZ * NormAcc;
    // 向量差乘得出的值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // 再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    // 角速度融合加速度积分补偿值（互补滤波）
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; // 弧度制
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
    // 一阶龙格库塔法, 更新四元数

    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    // 四元数归一化
    NormQuat = Q_rsqrt(
        squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // 四元数转欧拉角
    {

#ifdef YAW_GYRO
        *(
            float *)pAngE = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA; // yaw
#else
        float yaw_G = pMpu->gyroZ * Gyro_G;
        if ((yaw_G > 1.0f) || (yaw_G < -1.0f)) // 数据太小可以认为是干扰，不是偏航动作
        {
            pAngE->yaw += yaw_G * dt;
        }
#endif
        pAngE->pitch = asin(2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;

        pAngE->roll = atan2(2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1,
                            1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) *
                      RtA; // PITCH
    }
}

//======================================================================
//======================================================================

float Kp = 0.5f;    /*比例增益*/
float Ki = 0.0003f; /*积分增益*/

float q0 = 1.0f; /*四元数*/
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float rMat[3][3] = {{0, 0}, {0, 0}, {0, 0}}; /*旋转矩阵*/

/*计算旋转矩阵*/
void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuUpdate(const _st_Mpu *pMpu, _st_AngE *pAngE, float dt) /*数据融合 互补滤波*/
{
    // 需要单独的变量来保存计算值，数据才不会出错
    volatile struct V Acc, Gyro, AccGravity, GyroIntegError1 = {0, 0, 0};

    float halfT = 0.5f * dt;
    float normalise = 0;

    /* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
    /*单位化加速计测量值*/
    normalise = Q_rsqrt(
        pMpu->accX * pMpu->accX + pMpu->accY * pMpu->accY + pMpu->accZ * pMpu->accZ);

    Acc.x = pMpu->accX * normalise;
    Acc.y = pMpu->accY * normalise;
    Acc.z = pMpu->accZ * normalise;

    /*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
    AccGravity.x = (Acc.y * rMat[2][2] - Acc.z * rMat[2][1]);
    AccGravity.y = (Acc.z * rMat[2][0] - Acc.x * rMat[2][2]);
    AccGravity.z = (Acc.x * rMat[2][1] - Acc.y * rMat[2][0]);

    /*误差累计，与积分常数相乘*/
    GyroIntegError1.x += Ki * AccGravity.x;
    GyroIntegError1.y += Ki * AccGravity.y;
    GyroIntegError1.z += Ki * AccGravity.z;

    /*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
    Gyro.x = pMpu->gyroX * Gyro_Gr + Kp * AccGravity.x + GyroIntegError1.x;
    Gyro.y = pMpu->gyroY * Gyro_Gr + Kp * AccGravity.y + GyroIntegError1.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + Kp * AccGravity.z + GyroIntegError1.z;

    /* 一阶近似算法（一阶龙格库塔法），四元数运动学方程的离散化形式和积分 */
    float q0Last = q0;
    float q1Last = q1;
    float q2Last = q2;
    float q3Last = q3;
    q0 += (-q1Last * Gyro.x - q2Last * Gyro.y - q3Last * Gyro.z) * halfT;
    q1 += (q0Last * Gyro.x + q2Last * Gyro.z - q3Last * Gyro.y) * halfT;
    q2 += (q0Last * Gyro.y - q1Last * Gyro.z + q3Last * Gyro.x) * halfT;
    q3 += (q0Last * Gyro.z + q1Last * Gyro.y - q2Last * Gyro.x) * halfT;

    /*单位化四元数*/
    normalise = Q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= normalise;
    q1 *= normalise;
    q2 *= normalise;
    q3 *= normalise;

    imuComputeRotationMatrix(); /*计算旋转矩阵*/

    {
        /*计算roll pitch yaw 欧拉角*/
        pAngE->pitch = -asinf(rMat[2][0]) * RtA;
        pAngE->roll = atan2f(rMat[2][1], rMat[2][2]) * RtA;

        float yaw_G = pMpu->gyroZ * Gyro_G;
        if ((yaw_G > 1.0f) || (yaw_G < -1.0f)) // 数据太小可以认为是干扰，不是偏航动作
        {
            pAngE->yaw += yaw_G * dt;
        }
    }
}

/***************************************************END OF FILE***************************************************/

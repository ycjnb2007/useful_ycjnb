/*
 徐州工程学院
 */
#include "IMU660.h"

void IMU660RAReadAcc(short *accData)
{
   imu660ra_get_acc();
    accData[0] = imu660ra_acc_x;
    accData[1] = imu660ra_acc_y;
    accData[2] = imu660ra_acc_z;
}

void IMU660RAReadGyro(short *gyroData)
{

    imu660ra_get_gyro();
    gyroData[0] = imu660ra_gyro_x;
    gyroData[1] = imu660ra_gyro_y;
    gyroData[2] = imu660ra_gyro_z;
}

short g_acc_avg[3]; /* 加速度平均值 */
short g_gyro_avg[3]; /* 陀螺仪平均值 */

/**
 * @brief       数据校准
 * @note        对数据减去初值, 加速度减去重力加速度影响
 * @param       gx, gy, gz  : 3轴陀螺仪数据指针
 * @param       ax, ay, az  : 3轴加速度数据指针
 * @retval      无
 */
void imu_data_calibration(short *gx, short *gy, short *gz, short *ax, short *ay, short *az) {
    /* 减去初值(去掉零飘) */
    *gx -= g_gyro_avg[0];
    *gy -= g_gyro_avg[1];
    *gz -= g_gyro_avg[2];
    *ax -= g_acc_avg[0];
    *ay -= g_acc_avg[1];
    *az -= (g_acc_avg[2] - 8192); /* 8192是重力加速度理论值 */
}

/**
 * @brief       姿态解算初始化
 * @note        该函数必须在传感器初始化之后再调用
 * @param       无
 * @retval      无
 */
void imu_init(void) {
    uint16_t i = 0, avg = 256;
    int acc_sum[3] = { 0 }, gyro_sum[3] = { 0 };

    short acc_data[3]; /* 加速度传感器原始数据 */
    short gyro_data[3]; /* 陀螺仪原始数据 */

    const int8_t MAX_GYRO_QUIET = 5;
    const int8_t MIN_GYRO_QUIET = -5;

    int16_t LastGyro[3] = {0};
    int16_t ErrorGyro[3];
    uint8_t k=30;

    while(k--)//30次静止则判定飞行器处于静止状态
    {
        do
        {
            system_delay_ms(10);
            IMU660RAReadGyro(gyro_data);
            for(i=0;i<3;i++)
            {
                ErrorGyro[i] = gyro_data[i] - LastGyro[i];
                LastGyro[i] = gyro_data[i];
            }
        }while ((ErrorGyro[0] >  MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET)//标定静止
                    ||(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET)
                    ||(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
                        );
    }



    system_delay_ms(10);
    for (i = 0; i < 356; i++) /* 循环读取256次 取平均 */
    {
        IMU660RAReadAcc(acc_data);
        IMU660RAReadGyro(gyro_data);

        if (i >= 100) { //前面100次的数据不要
            acc_sum[0] += acc_data[0];
            acc_sum[1] += acc_data[1];
            acc_sum[2] += acc_data[2];
            gyro_sum[0] += gyro_data[0];
            gyro_sum[1] += gyro_data[1];
            gyro_sum[2] += gyro_data[2];
        }

        system_delay_ms(3);
    }

    g_acc_avg[0] = acc_sum[0] / avg;
    g_acc_avg[1] = acc_sum[1] / avg;
    g_acc_avg[2] = acc_sum[2] / avg;
    g_gyro_avg[0] = gyro_sum[0] / avg;
    g_gyro_avg[1] = gyro_sum[1] / avg;
    g_gyro_avg[2] = gyro_sum[2] / avg;
}




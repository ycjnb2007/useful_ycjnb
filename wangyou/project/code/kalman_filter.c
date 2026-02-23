/*
 徐州工程学院
 */
#include "kalman_filter.h"
#include "math.h"

struct _1_ekf_filter {
    float LastP;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;
};

void kalman_1(struct _1_ekf_filter *ekf, float input)  //一维卡尔曼
{
    ekf->Now_P = ekf->LastP + ekf->Q;
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
    ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;
}

//下面参数根据调试经验得出
static struct _1_ekf_filter ekf[3] = { { 0.02, 0, 0, 0, 0.001, 0.543 }, {
            0.02, 0, 0, 0, 0.001, 0.543 }, { 0.02, 0, 0, 0, 0.001, 0.543 } };

const float factor = 0.15f;  //滤波因素
static float tBuff[3]={0.0,0.0,0.0};

//一维卡尔曼
//void kalman_filter_new(Axis3f *acc, Axis3f *gyro) {
//
//
//  kalman_1(&ekf[0], acc->x);
//  acc->x =  ekf[0].out;
//
//  kalman_1(&ekf[1], acc->y);
//  acc->y =  ekf[1].out;
//
//  kalman_1(&ekf[2], acc->z);
//  acc->z =  ekf[2].out;
//
//  //一阶低通滤波
//  gyro->x = tBuff[0] = tBuff[0] * (1 - factor) + gyro->x * factor;
//  gyro->y = tBuff[1] = tBuff[1] * (1 - factor) + gyro->y * factor;
//  gyro->z = tBuff[2] = tBuff[2] * (1 - factor) + gyro->z * factor;
//
//}

void kalman_filter_new(_st_Mpu *pMpu) {


    kalman_1(&ekf[0], pMpu->accX);
    pMpu->accX =  ekf[0].out;

    kalman_1(&ekf[1], pMpu->accY);
    pMpu->accY =  ekf[1].out;

    kalman_1(&ekf[2], pMpu->accZ);
    pMpu->accZ =  ekf[2].out;

    //一阶低通滤波
    pMpu->gyroX = tBuff[0] = tBuff[0] * (1 - factor) + pMpu->gyroX * factor;
    pMpu->gyroY = tBuff[1] = tBuff[1] * (1 - factor) + pMpu->gyroY * factor;
    pMpu->gyroZ = tBuff[2] = tBuff[2] * (1 - factor) + pMpu->gyroZ * factor;

}



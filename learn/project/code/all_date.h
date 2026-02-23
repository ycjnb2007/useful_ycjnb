/*
 徐州工程学院
 */
#ifndef ALL_DATE_H_
#define ALL_DATE_H_
#include "zf_common_function.h"


extern volatile uint32_t SysTick_count;


typedef struct{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
}_st_Mpu;




typedef struct{
    float roll;
    float pitch;
    float yaw;
}_st_AngE;

typedef volatile struct
{
    float desired;     //< set point
    float offset;      //
    float prevError;    //< previous error
    float integ;        //< integral
    float kp;           //< proportional gain
    float ki;           //< integral gain
    float kd;           //< derivative gain
    float IntegLimitHigh;       //< integral limit
    float IntegLimitLow;
    float measured;
    float out;
    float OutLimitHigh;
    float OutLimitLow;
}PidObject;




extern _st_Mpu MPU6050;
extern _st_AngE Angle;




extern  PidObject pidRateX;
extern  PidObject pidRateY;
extern  PidObject pidRateZ;

extern  PidObject pidPitch;
extern  PidObject pidRoll;
extern  PidObject pidYaw;



extern int16_t motor_PWM_Value[4];



#endif /* ALL_DATE_H_ */

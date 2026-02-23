#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR_L_PIN1    ATOM0_CH5_P02_5
#define MOTOR_L_PIN2    ATOM0_CH4_P02_4

#define MOTOR_R_PIN1     ATOM2_CH4_P11_9
#define MOTOR_R_PIN2     ATOM2_CH5_P11_10


#define MOTOR_PWM_FREQ    17000
#define MOTOR_MAX_DUTY    10000

// ================= 函数声明 =================
void Motor_Init(void);
void Motor_Control(int16 l_speed, int16 r_speed);
void Motor_Brake(void); // 新增：主动刹车函数

#endif

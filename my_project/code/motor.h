#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

// ================= 电机引脚定义 (AT8236 双PWM模式) =================
#define MOTOR_L_PIN1    ATOM0_CH5_P02_5
#define MOTOR_L_PIN2    ATOM0_CH4_P02_4

#define MOTOR_R_PIN1     ATOM2_CH4_P11_9
#define MOTOR_R_PIN2     ATOM2_CH5_P11_10

#define MOTOR_PWM_FREQ    17000     // PWM频率
#define MOTOR_MAX_DUTY    10000     // 满占空比 10000

// ================= 编码器引脚定义 =================
#define ENCODER_L         TIM6_ENCODER
#define ENCODER_L_A       TIM6_ENCODER_CH1_P20_3
#define ENCODER_L_B       TIM6_ENCODER_CH2_P20_0

#define ENCODER_R         TIM2_ENCODER
#define ENCODER_R_A       TIM2_ENCODER_CH1_P33_7
#define ENCODER_R_B       TIM2_ENCODER_CH2_P33_6

#define MOTOR_DEAD_ZONE 300
// ================= 全局变量声明 =================
// 供你的 PID 算法文件外部调用 (极简，只有速度)
extern float Actual_Speed[2];  // 滤波后的真实车速(脉冲/周期)，[0]左轮 [1]右轮

// ================= 函数声明 =================
void Motor_Init(void);                               // 电机底层初始化
void Motor_Control(int16 l_speed, int16 r_speed);    // 电机PWM控制
void Motor_Brake(void);                              // 电机主动刹车

void Encoder_Init(void);                             // 编码器初始化
void Encoder_Update_Speed(void);                     // 【核心】定时器中断里调用，更新车速

#endif

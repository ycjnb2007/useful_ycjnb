/*
 * pid.h
 *
 *  Created on: 2026年2月26日
 *      Author: ycj01
 */

#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

// ---------------- 核心参数宏定义 (需根据实车调参) ----------------
// 【基础速度】单位：编码器每周期脉冲数
#define SPEED_STRAIGHT_L 250   // 长直道极速
#define SPEED_STRAIGHT_S 180   // 短直道速度
#define SPEED_CURVE      120   // 弯道速度

// 【位置式 PD】外环：图像偏差 -> 目标角速度
#define TURN_KP_BASE 1.5f      // 基础比例系数
#define TURN_KP_VAR  0.05f     // 变参数增益 (国特核心)
#define TURN_KD      0.8f      // 微分系数

// 【位置式 PD】中环：角速度偏差 -> 速度差
#define GYRO_KP      1.2f
#define GYRO_KD      0.1f

// 【增量式 PI】内环：速度闭环
#define SPEED_KP     18.0f
#define SPEED_KI     1.5f

// ---------------- 数据结构定义 ----------------
// 位置式 PD 结构体 (用于外环和中环)
typedef struct {
    float Kp;
    float Kd;
    float error;
    float last_error;
    float output;
} Positional_PD_t;

// 增量式 PI 结构体 (用于内环电机速度)
typedef struct {
    float Kp;
    float Ki;
    float error;
    float last_error;
    float output;     // 积分累加的PWM值
    float out_max;    // 限幅上限
    float out_min;    // 限幅下限
} Incremental_PI_t;

// 全局控制状态 (方便上位机监控)
typedef struct {
    float camera_error;
    float angular_rate_target;
    float angular_rate_current;
    float speed_diff;

    int16 base_speed;
    float target_left_speed;
    float target_right_speed;

    float current_left_speed;
    float current_right_speed;

    int16 output_left_pwm;
    int16 output_right_pwm;
} System_Control_State_t;

extern System_Control_State_t ctrl_state;

void PID_Init(void);
void Control_Loop(void);

#endif

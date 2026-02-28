/*
 * pid.c
 *
 *  Created on: 2026年2月26日
 *      Author: ycj01
 */

#include "pid.h"
#include <math.h> // 需要用到 fabs 函数

System_Control_State_t ctrl_state;

// 实例化控制器
Positional_PD_t  pid_turn_outer; // 图像外环
Positional_PD_t  pid_gyro_middle;// 陀螺仪中环
Incremental_PI_t pid_speed_L;    // 左轮内环
Incremental_PI_t pid_speed_R;    // 右轮内环

/**
 * @brief  PID 参数初始化
 */
void PID_Init(void) {
    // 初始化外环与中环参数
    pid_turn_outer.Kd = TURN_KD;
    pid_gyro_middle.Kp = GYRO_KP;
    pid_gyro_middle.Kd = GYRO_KD;

    // 初始化内环 PI 参数及限幅 (限幅设为最大占空比 10000 减去死区，防止溢出)
    float max_pwm = 9500.0f;
    pid_speed_L.Kp = SPEED_KP; pid_speed_L.Ki = SPEED_KI;
    pid_speed_L.out_max = max_pwm; pid_speed_L.out_min = -max_pwm;

    pid_speed_R.Kp = SPEED_KP; pid_speed_R.Ki = SPEED_KI;
    pid_speed_R.out_max = max_pwm; pid_speed_R.out_min = -max_pwm;
}

/**
 * @brief 【国特秘籍2】一阶低通滤波
 * @note  传感器数据有高频噪声，直接丢进 PID 计算会导致 D 作用被异常放大，电机发抖。
 */
float Low_Pass_Filter(float current_val, float last_val, float alpha) {
    // alpha 越小，滤波越强，延迟越大。一般取 0.3 - 0.7
    return alpha * current_val + (1.0f - alpha) * last_val;
}

/**
 * @brief 增量式 PI 计算 (仅针对内环速度)
 */
float Calc_Incremental_PI(Incremental_PI_t *pid, float target, float current) {
    float increment;
    pid->error = target - current;

    // 增量公式: OUT += Kp*(E(k) - E(k-1)) + Ki*E(k)
    increment = pid->Kp * (pid->error - pid->last_error) +
                pid->Ki * pid->error;

    pid->output += increment; // 累加得到绝对 PWM

    // 积分抗饱和限幅 (Anti-Windup)
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    pid->last_error = pid->error;
    return pid->output;
}

/**
 * @brief 位置式 PD 计算 (带变参数机制，用于外环)
 */
float Outer_Loop_Camera(void) {
    // 动态前瞻获取
    uint8 aim_row = Deal_Bottom + 15;
    if(IF == straightlineL) aim_row = Deal_Bottom + 25;
    else if(IF == curve) aim_row = Deal_Bottom + 10;
    if(aim_row > Deal_Top) aim_row = Deal_Top;

    // 计算偏差
    ctrl_state.camera_error = 70.0f - (float)mid_line[aim_row];
    pid_turn_outer.error = ctrl_state.camera_error;

    // 【国特秘籍3】变参数 PD 控制
    // 误差越小，Kp 越小（直道稳，不画龙）；误差越大，Kp 越大（弯道转得猛，不冲出赛道）
    pid_turn_outer.Kp = TURN_KP_BASE + TURN_KP_VAR * fabs(pid_turn_outer.error);

    // 位置式公式: OUT = Kp*E(k) + Kd*(E(k) - E(k-1))
    pid_turn_outer.output = pid_turn_outer.Kp * pid_turn_outer.error +
                            pid_turn_outer.Kd * (pid_turn_outer.error - pid_turn_outer.last_error);

    pid_turn_outer.last_error = pid_turn_outer.error;

    // 目标角速度限幅防翻车
    if(pid_turn_outer.output > 300.0f) pid_turn_outer.output = 300.0f;
    if(pid_turn_outer.output < -300.0f) pid_turn_outer.output = -300.0f;

    return pid_turn_outer.output;
}

/**
 * @brief 核心中断控制循环 (需放在定时器中断中，周期推荐 5ms)
 */
void Control_Loop(void) {
    // 1. 基础速度设定
    switch (IF) {
        case straightlineL: ctrl_state.base_speed = SPEED_STRAIGHT_L; break;
        case straightlineS: ctrl_state.base_speed = SPEED_STRAIGHT_S; break;
        case curve:         ctrl_state.base_speed = SPEED_CURVE;      break;
        default:            ctrl_state.base_speed = SPEED_STRAIGHT_S; break;
    }

    // 2. 图像外环算角速度
    ctrl_state.angular_rate_target = Outer_Loop_Camera();

    // 3. 陀螺仪中环算差速
    static float last_gyro = 0;
    // 【替换函数】获取IMU Z轴角速度，并使用一阶滤波滤除震动噪声
    float raw_gyro = imu660_get_z_gyro(); // 注意极性！车体向左转该值为正
    ctrl_state.angular_rate_current = Low_Pass_Filter(raw_gyro, last_gyro, 0.6f);
    last_gyro = ctrl_state.angular_rate_current;

    pid_gyro_middle.error = ctrl_state.angular_rate_target - ctrl_state.angular_rate_current;

    // 中环位置式 PD
    pid_gyro_middle.output = pid_gyro_middle.Kp * pid_gyro_middle.error +
                             pid_gyro_middle.Kd * (pid_gyro_middle.error - pid_gyro_middle.last_error);
    pid_gyro_middle.last_error = pid_gyro_middle.error;
    ctrl_state.speed_diff = pid_gyro_middle.output;

    // 4. 计算目标速度 (差速模型，注意加减号极性需根据实车测试反转)
    ctrl_state.target_left_speed  = ctrl_state.base_speed - ctrl_state.speed_diff;
    ctrl_state.target_right_speed = ctrl_state.base_speed + ctrl_state.speed_diff;

    // 5. 内环闭环计算
    static float last_enc_L = 0, last_enc_R = 0;
    // 【替换函数】读取编码器值，并滤波
    float raw_enc_L = encoder_get_count(TIM6_ENCODER);
    float raw_enc_R = encoder_get_count(TIM2_ENCODER);
    encoder_clear_count(TIM6_ENCODER);
    encoder_clear_count(TIM2_ENCODER);

    ctrl_state.current_left_speed = Low_Pass_Filter(raw_enc_L, last_enc_L, 0.4f);
    ctrl_state.current_right_speed = Low_Pass_Filter(raw_enc_R, last_enc_R, 0.4f);
    last_enc_L = ctrl_state.current_left_speed;
    last_enc_R = ctrl_state.current_right_speed;

    // 计算增量式 PI，获取目标 PWM
    ctrl_state.output_left_pwm  = (int16)Calc_Incremental_PI(&pid_speed_L, ctrl_state.target_left_speed, ctrl_state.current_left_speed);
    ctrl_state.output_right_pwm = (int16)Calc_Incremental_PI(&pid_speed_R, ctrl_state.target_right_speed, ctrl_state.current_right_speed);

    // 6. 执行电机输出
    Motor_Control(ctrl_state.output_left_pwm, ctrl_state.output_right_pwm);
}



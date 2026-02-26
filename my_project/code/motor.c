#include "motor.h"

// ================= 变量定义 =================
int16 Raw_Pulse[2];            // 存放单次读取到的原始脉冲
float Actual_Speed[2] = {0,0}; // 供外部PID读取的平滑速度

// ==============================
// 第一部分：AT8236 底层驱动逻辑
// ==============================

/**
 * @brief 电机PWM初始化
 */
void Motor_Init(void)
{
    pwm_init(MOTOR_L_PIN1, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_L_PIN2, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_R_PIN1, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_R_PIN2, MOTOR_PWM_FREQ, 0);
}

/**
 * @brief 单个电机的正反转控制
 * @note  AT8236 控制逻辑: 正转 PIN1波 PIN2低; 反转 PIN1低 PIN2波
 */
static void Motor_Drive_Single(pwm_channel_enum pin1, pwm_channel_enum pin2, int16 speed)
{
    // 软件限幅，防止占空比越界
    if(speed > MOTOR_MAX_DUTY)  speed = MOTOR_MAX_DUTY;
    if(speed < -MOTOR_MAX_DUTY) speed = -MOTOR_MAX_DUTY;

    if (speed > 0)
    {
        pwm_set_duty(pin1, speed);
        pwm_set_duty(pin2, 0);
    }
    else if (speed < 0)
    {
        pwm_set_duty(pin1, 0);
        pwm_set_duty(pin2, -speed);
    }
    else
    {
        pwm_set_duty(pin1, 0);
        pwm_set_duty(pin2, 0);
    }
}

/**
 * @brief 左右电机整体控制
 */
void Motor_Control(int16 l_speed, int16 r_speed)
{
    Motor_Drive_Single(MOTOR_L_PIN1, MOTOR_L_PIN2, l_speed);
    Motor_Drive_Single(MOTOR_R_PIN1, MOTOR_R_PIN2, r_speed);
}

/**
 * @brief 电机主动刹车 (遇障或停车使用)
 */
void Motor_Brake(void)
{
    pwm_set_duty(MOTOR_L_PIN1, MOTOR_MAX_DUTY);
    pwm_set_duty(MOTOR_L_PIN2, MOTOR_MAX_DUTY);
    pwm_set_duty(MOTOR_R_PIN1, MOTOR_MAX_DUTY);
    pwm_set_duty(MOTOR_R_PIN2, MOTOR_MAX_DUTY);
}

// ==============================
// 第二部分：编码器读取与滤波测速
// ==============================

/**
 * @brief 编码器底层硬件初始化
 */
void Encoder_Init(void)
{
    // 配置 TC264 的硬件正交解码功能
    encoder_dir_init(ENCODER_L, ENCODER_L_A, ENCODER_L_B);
    encoder_dir_init(ENCODER_R, ENCODER_R_A, ENCODER_R_B);
}

/**
 * @brief 【核心测速函数】请放在 5ms 或 10ms 的定时器中断中调用！
 * @note  逐句解析：
 * 1. 读取脉冲。
 * 2. 必须立刻清零！这样下次读到的就是“单位时间内的脉冲变化量”，也就是速度。
 * 3. 进行一阶低通滤波，消除机械抖动。
 */
void Encoder_Update_Speed(void)
{
    // 1. 读取原始脉冲
    Raw_Pulse[0] = encoder_get_count(ENCODER_L);
    Raw_Pulse[1] = -encoder_get_count(ENCODER_R); // 注意: 如果右轮向前转脉冲是负的，这里加负号将其正规化

    // 2. 硬件计数器清零 (极其重要！不清零没法做速度闭环)
    encoder_clear_count(ENCODER_L);
    encoder_clear_count(ENCODER_R);

    // 3. 一阶低通滤波提取平滑速度
    // 公式: 当前输出 = (本次真实测量的脉冲) * 0.3 + (上次计算得出的速度) * 0.7
    // 这里的 0.3 和 0.7 是滤波权重，如果你觉得速度响应太慢，可以改成 0.5 和 0.5，甚至0.7和0.3
    Actual_Speed[0] = (float)Raw_Pulse[0] * 0.3f + Actual_Speed[0] * 0.7f;
    Actual_Speed[1] = (float)Raw_Pulse[1] * 0.3f + Actual_Speed[1] * 0.7f;
}

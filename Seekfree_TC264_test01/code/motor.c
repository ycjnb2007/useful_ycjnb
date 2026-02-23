#include "motor.h"

/**
 * @brief  电机初始化
 * @note   将4个引脚全部初始化为PWM模式
 */
void Motor_Init(void)
{
    // 初始化引脚，初始占空比全部为0（防暴转）
    pwm_init(MOTOR_L_PIN1, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_L_PIN2, MOTOR_PWM_FREQ, 0);

    pwm_init(MOTOR_R_PIN1, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_R_PIN2, MOTOR_PWM_FREQ, 0);
}

/**
 * @brief  单电机控制内部函数
 */
static void Motor_Drive_Single(pwm_channel_enum pin1, pwm_channel_enum pin2, int16 speed)
{
    // 限幅
    if(speed > MOTOR_MAX_DUTY)  speed = MOTOR_MAX_DUTY;
    if(speed < -MOTOR_MAX_DUTY) speed = -MOTOR_MAX_DUTY;

    if (speed > 0)
    {
        // 正转：PIN1 有波，PIN2 低电平
        pwm_set_duty(pin1, speed);
        pwm_set_duty(pin2, 0);
    }
    else if (speed < 0)
    {
        // 反转：PIN1 低电平，PIN2 有波
        pwm_set_duty(pin1, 0);
        pwm_set_duty(pin2, -speed); // 取绝对值
    }
    else
    {
        // 停止 (滑行)：两个都为 0
        pwm_set_duty(pin1, 0);
        pwm_set_duty(pin2, 0);
    }
}

/**
 * @brief  电机控制
 * @param  l_speed 左轮速度 (-10000 到 10000)
 * @param  r_speed 右轮速度 (-10000 到 10000)
 */
void Motor_Control(int16 l_speed, int16 r_speed)
{
    Motor_Drive_Single(MOTOR_L_PIN1, MOTOR_L_PIN2, l_speed);
    Motor_Drive_Single(MOTOR_R_PIN1, MOTOR_R_PIN2, r_speed);
}

/**
 * @brief  主动刹车 (立刻停止)
 * @note   AT8236 特性：两个输入端都为高电平时刹车
 */
void Motor_Brake(void)
{
    // 左轮刹车：两个脚都拉满
    pwm_set_duty(MOTOR_L_PIN1, MOTOR_MAX_DUTY);
    pwm_set_duty(MOTOR_L_PIN2, MOTOR_MAX_DUTY);

    // 右轮刹车：两个脚都拉满
    pwm_set_duty(MOTOR_R_PIN1, MOTOR_MAX_DUTY);
    pwm_set_duty(MOTOR_R_PIN2, MOTOR_MAX_DUTY);
}

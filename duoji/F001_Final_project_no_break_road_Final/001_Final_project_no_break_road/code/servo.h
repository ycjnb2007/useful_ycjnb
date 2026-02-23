#ifndef _servo_h_
#define _servo_h_

//按键调试舵机定时器通道

//舵机控制引脚
#define PWM3                (TCPWM_CH52_P18_5)

#define SERVO_PWM_MID           745     
#define SERVO_PWM_MAX           800             //左转
#define SERVO_PWM_MIN           680             //右转

extern uint16 pwm;
extern uint16 real;

void servo_init(void);                          //舵机初始化
void servo_pwm_control(uint16 pwm);             //PWM控制舵机
void servo_set_angle(double angle);             //舵机角度设置
        

#endif
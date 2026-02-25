#ifndef _motor_h_
#define _motor_h_

//定义电机引脚
#define PWM_CH1             (TCPWM_CH14_P00_2)                          //右轮
#define PWM_CH2             (TCPWM_CH13_P00_3)

#define PWM_CH3             (TCPWM_CH51_P18_6)                          //左轮
#define PWM_CH4             (TCPWM_CH50_P18_7)

#define MAX_SPEED_TOLERANCE          200                             // 允许的速度偏差值
#define MIN_SPEED_TOLERANCE          150                             // 允许的速度偏差值
#define MAX_CONSECUTIVE_FAULTS       10                                  // 连续异常次数阈值
#define ZERO_SPEED_THRESHOLD         5                

//#define DIR1                (P00_2)
//#define PWM1                (TCPWM_CH13_P00_3)
//#define DIR2                (P18_6)
//#define PWM2                (TCPWM_CH50_P18_7)


#define motor_max_duty          6500
#define motor_min_duty          -6500

//声明电机信息
typedef struct motor
{
  
  int32 target_speed;     //电机设置的速度
  int32 duty;             //电机占空比
  double encoder_speed;    //编码器测得的电机速度
  int32 encoder_raw;              
  int32 total_encoder;

}motor;//类型名 


extern motor L_motor;           //左电机结构体
extern motor R_motor;           //右电机结构体


void motor_init(void);                          //左右电机初始化
void L_motor_set_speed(float Out);             //左电机设置占空比
void R_motor_set_speed(float Out);             //右电机设置占空比
uint8 protect_speed_range(void);                 //电机保护


// 全局状态变量
typedef struct 
{
    uint8 speed_reached_target; // 速度达到目标标志
    uint8 protection_enabled;   // 保护启用标志
    uint8 consecutive_faults;   //多次验证

    
} MotorProtectionState;
extern MotorProtectionState motor_protection;

#endif
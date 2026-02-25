#include "zf_common_headfile.h"

/**左右电机初始化**/
void motor_init(void)
{
    pwm_init(PWM_CH1, 17000, 0);                                                // 右轮PWM 通道 1 初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM_CH2, 17000, 0);                                                // 右轮PWM 通道 2 初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM_CH3, 17000, 0);                                                // 左轮PWM 通道 3 初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM_CH4, 17000, 0);                                                // 左轮PWM 通道 4 初始化频率 17KHz 占空比初始为 0

//    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO 初始化为输出 默认上拉输出高
//    gpio_init(DIR2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO 初始化为输出 默认上拉输出高
//    pwm_init(PWM1, 17000, 0);                                                   // PWM 通道初始化频率 17KHz 占空比初始为 0
//    pwm_init(PWM2, 17000, 0);                                                   // PWM 通道初始化频率 17KHz 占空比初始为 0
  
}


/**设置左电机速度(左电机PWM占空比)**/
void L_motor_set_speed(float Out)      
{
      if(Out >= 0.0f)
        {                                                                      // PWM_DUTY_MAX为10000
          pwm_set_duty(PWM_CH3, (uint32)Out);
          pwm_set_duty(PWM_CH4,0);
//          gpio_set_level(DIR2, GPIO_HIGH);                                    // DIR输出高电平
//          pwm_set_duty(PWM2, (uint32)Out);                    // 计算占空比
         
          
          
        }else
        {
          pwm_set_duty(PWM_CH3, 0);
          pwm_set_duty(PWM_CH4,(uint32)(-Out));
//          gpio_set_level(DIR2, GPIO_LOW);                                     // DIR输出低电平
//          pwm_set_duty(PWM2, (uint32)(-Out));                 // 计算占空比
          
          
        }
}


/**设置右电机速度(右电机PWM占空比)**/
void R_motor_set_speed(float Out)      
{
      if(Out >= 0.0f)
        {                                                                      // PWM_DUTY_MAX为10000
          pwm_set_duty(PWM_CH1, (uint32)Out);
          pwm_set_duty(PWM_CH2,0);
//            gpio_set_level(DIR1, GPIO_HIGH);                                    // DIR输出高电平
//            pwm_set_duty(PWM1,(uint32)(Out));                    // 计算占空比
          
                                                                            
        }else
        {
          pwm_set_duty(PWM_CH1, 0);
          pwm_set_duty(PWM_CH2,(uint32)(-Out));
//            gpio_set_level(DIR1, GPIO_LOW);                                    // DIR输出高电平
//            pwm_set_duty(PWM1,(uint32)(-Out));                    // 计算占空比
          
          
        }
}





/** 电机堵转保护 **/
MotorProtectionState motor_protection = {0, 0, 0};
uint8 protect_speed_range(void)
{
    // 1. 检查速度是否达到目标范围
    if (!motor_protection.speed_reached_target) 
    {
        int normol_lower_limit = normol_speed - 5;
        int normol_upper_limit = normol_speed + 5;
        
        if (L_motor.encoder_speed > normol_lower_limit && 
            L_motor.encoder_speed < normol_upper_limit &&
            R_motor.encoder_speed > normol_lower_limit && 
            R_motor.encoder_speed < normol_upper_limit) 
        {
            motor_protection.speed_reached_target = 1;
            motor_protection.protection_enabled = 1;
            motor_protection.consecutive_faults = 0;  // 重置计数器
        }
        return 0;
    }
    
    // 2. 保护使能时的连续检测
    if (motor_protection.protection_enabled) 
    {    
        int normol_lower_limit = L_motor.target_speed - MIN_SPEED_TOLERANCE;    //-80
        int normol_upper_limit = R_motor.target_speed + MAX_SPEED_TOLERANCE;    //+120

        // 检测是否超出范围
        uint8 is_fault = (L_motor.encoder_speed < normol_lower_limit || 
                            L_motor.encoder_speed > normol_upper_limit ||
                            R_motor.encoder_speed < normol_lower_limit || 
                            R_motor.encoder_speed > normol_upper_limit ||
                            fabs(L_motor.encoder_speed) < ZERO_SPEED_THRESHOLD ||
                             fabs(R_motor.encoder_speed) < ZERO_SPEED_THRESHOLD   );
        
        if (is_fault) 
        {
            motor_protection.consecutive_faults++;  // 异常计数增加
            
            if (motor_protection.consecutive_faults >= MAX_CONSECUTIVE_FAULTS) 
            {
                motor_protection.consecutive_faults = 0;  // 重置计数器
                return 1;  // 连续多次异常，触发保护
            }
        } else 
        {
            motor_protection.consecutive_faults = 0;  // 正常时重置计数器
        }
    }
    
    return 0;
}
//uint8_t protect_speed_range(void)
//{
//    // 1. 检查速度是否达到目标范围
//    if (!motor_protection.speed_reached_target) 
//    {
//        // 计算目标速度范围
//        int normol_lower_limit = normol_speed - 5;
//        int normol_upper_limit = normol_speed + 5;
//        
//        // 检查左右电机是否都进入目标范围
//        if (L_motor.encoder_speed > normol_lower_limit && 
//            L_motor.encoder_speed < normol_upper_limit &&
//            R_motor.encoder_speed > normol_lower_limit && 
//            R_motor.encoder_speed < normol_upper_limit) 
//        {
//            // 标记速度已达到目标范围
//            motor_protection.speed_reached_target = 1;
//            motor_protection.protection_enabled = 1;
//        }
//        return 0; // 在达到目标速度前不进行保护
//    }
//    
//    // 2. 如果保护已启用，执行正常范围检测
//    if (motor_protection.protection_enabled) 
//    {
//        int lower_limit = normol_speed - MIN_SPEED_TOLERANCE;
//        int upper_limit = normol_speed + MAX_SPEED_TOLERANCE;
//        
//        if (L_motor.encoder_raw < lower_limit || 
//            L_motor.encoder_raw > upper_limit ||
//            R_motor.encoder_raw < lower_limit ||
//            R_motor.encoder_raw > upper_limit 
//             ) 
//        {
//            // 触发保护
////            zf_assert(0);
//            return 1;
//        }
//    }
//    
//    return 0;
//}


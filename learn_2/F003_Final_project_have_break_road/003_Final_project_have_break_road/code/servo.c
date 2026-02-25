#include "zf_common_headfile.h"

uint16 pwm = SERVO_PWM_MID;
uint16 real = SERVO_PWM_MID;


/**舵机初始化**/
void servo_init(void)
{
  pwm_init(PWM3,50,SERVO_PWM_MID);                                          //舵机初始化,频率50Hz,占空比初始为 0
}


/**pwm 控制舵机**/
void servo_pwm_control(uint16 pwm)
{
  
  if(SERVO_PWM_MAX < pwm )
  {
      real = SERVO_PWM_MAX;
  }else if (SERVO_PWM_MIN > pwm)
  {
      real = SERVO_PWM_MIN;
  }else
  {
      real = pwm;
  }
   
  pwm_set_duty(PWM3, real);

}



/**
函数：舵机旋转角度

说明：舵机初始化为50Hz,即周期是20ms
      舵机在0.5ms至2.5ms脉宽对应 0°至 180°
      占空比2.5% 到 12.5%  最大值为10000，即占空比变化在2.5% *10000 到 12.5% *10000之间
                                                            250          1250        
测试：角度为100时为正，90时往右转，110时往左转
    
**/
void servo_set_angle(double angle)
{
  
  pwm_set_duty(PWM3, (uint32)(250 + angle * (1000 / 180)));

}

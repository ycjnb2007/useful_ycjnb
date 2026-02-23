#include "zf_common_headfile.h"

uint8 start = 0;                //电机开始转动的按键控制
uint8 exchange = 0;             //按键选项
float differential_k = 0.7;       //差速系数
int normol_speed = 120;        //电机设定速度(编码器速度)

/**按键控制电机启动和停止**/
void car_start(void)
{
  if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
    start++;
    } 
  
  if(start != 0)
      {
        if(start % 2 == 1)
        {
          
//          Motor_Control(normol_speed, normol_speed);                    //单纯电机PI速度    

          Final_motor_control(normol_speed, differential_k);         //电机差速控制 
         
          //加入电机堵转保护
          if(protect_speed_range())
          {
            start = 2;
            motor_protection.speed_reached_target = 0;
            motor_protection.protection_enabled = 0;
            roundabout_dir = 0;
            exchange = 0;
            

            

          }
         
        }else
        {
          L_motor_set_speed(0);
          R_motor_set_speed(0);

        }
      
      
      }

  //舵机控制
  
  if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
  {
    exchange++;
    if(exchange >= 4)
    {
      exchange = 0;
    }
    
  }
  if(exchange == 0)             //舵机Kp系数
  {
      if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
      {
        Servo_pid.Kp += 0.1;      
      }
      if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
      {
        Servo_pid.Kp -= 0.1;
      }
  }else if(exchange == 1)       //舵机Kd系数
  {
      if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
      {
        Servo_pid.Kd += 0.1;     
      }
      if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
      {
        Servo_pid.Kd -= 0.1;
      }
  
  }else if(exchange == 2)       //差速误差系数
  {
     if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
          differential_k += 0.1;     
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
          differential_k -= 0.1;
        }

  }
  else if(exchange == 3)        //编码器设定速度
  {
     if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
          normol_speed += 5;     
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
          normol_speed -= 5;
        }
  
  }
  

}

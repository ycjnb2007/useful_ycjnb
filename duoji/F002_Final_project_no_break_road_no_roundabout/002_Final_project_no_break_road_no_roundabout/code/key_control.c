#include "zf_common_headfile.h"

uint8 exchange_show_page = 1;                   //屏幕显示不同页
uint8 exchange_choose_option = 1;               //选项的不同
float differential_k = 0.6;       //普通寻迹差速
float differential_k_normal = 0.6;
float differential_k_roundabout = 0.6;
int normol_speed = 85;        //基础速度
uint8 start_motor = 0;
uint8 start_servo = 0;



void car_start(void)
{

      if(start_motor != 0)
      {
        if(start_motor % 2 == 1)
        {
              encoder_clear_count(ENCODER_QUAD1);
              encoder_clear_count(ENCODER_QUAD2);
          
          
            start_servo = 1;
          
              Final_motor_control(normol_speed, differential_k);         //差速控制
        

         
          //保护
          if(protect_speed_range())
          {
              start_motor = 2;
              motor_protection.speed_reached_target = 0;
              motor_protection.protection_enabled = 0;
              roundabout_dir = 0;
              start_servo = 0;
              encoder_clear_count(ENCODER_QUAD1);
              encoder_clear_count(ENCODER_QUAD2);
            }
             
        }else
        {
          L_motor_set_speed(0);
          R_motor_set_speed(0);

        }
      
      
      }

}



void exchange_show_page_control(void)
{
    static uint8 key1_last_state = 0;
    static uint8 key2_last_state = 0;
    static uint8 key3_last_state = 0;
    static uint8 key4_last_state = 0;

    // KEY_1 防抖
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        if(key1_last_state == 0)
        {
            uint8 max_choose = 1;
            if(exchange_show_page == 1) 
            {
                max_choose = 2;
            }else if(exchange_show_page ==2)
            {
                max_choose = 4;
            }
            exchange_choose_option++;
            if(exchange_choose_option > max_choose)
            {
                exchange_choose_option = 1;
            }
        }
        key1_last_state = 1;
    }
    else
    {
        key1_last_state = 0;
    }

    // KEY_2 防抖
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        if(key2_last_state == 0)
        {
            if(exchange_show_page == 1)
            {
                start_motor++;
            }
            if(exchange_show_page == 2)
            {
                switch(exchange_choose_option)
                {
                    case 1:Servo_pid.Kp += 0.1f;  break;
                    case 2:Servo_pid.Kd += 0.1f;  break;
                    case 3:differential_k_normal += 0.1f;  break;
                    case 4:normol_speed += 5;  break;
                }
            }
        }
        key2_last_state = 1;
    }
    else
    {
        key2_last_state = 0;
    }

    // KEY_3 防抖
    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        if(key3_last_state == 0)
        {
            if(exchange_show_page == 1)
            {
                start_servo++;
            }
            if(exchange_show_page == 2)
            {
                switch(exchange_choose_option)
                {
                    case 1:Servo_pid.Kp -= 0.1f;  break;
                    case 2:Servo_pid.Kd -= 0.1f;  break;
                    case 3:differential_k_normal -= 0.1f;  break;
                    case 4:normol_speed -= 5;  break;
                }
            }
        }
        key3_last_state = 1;
    }
    else
    {
        key3_last_state = 0;
    }

    // KEY_4 防抖（原有逻辑）
    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        if(key4_last_state == 0)
        {
            ips114_clear();
            exchange_show_page++;       
            if(exchange_show_page > 2)
            {
                exchange_show_page = 1;
            }
            
            exchange_choose_option = 1;
        }
        key4_last_state = 1;
    }
    else
    {
        key4_last_state = 0;
    }
}



#ifdef ENABLE_BREAK
/************************************************断路************************************************/


/*******************************断路UI信息*******************************/
/**********采集断路区域**********/                                                     
uint8 area_memery = 1;                                   //断路区的采取区域选择（1-3）
uint8 write_flag = 0;                                    //写入标志位
uint8 write_flag_finish1 = 0;
uint8 write_flag_finish2 = 0;
uint8 write_flag_finish3 = 0;
uint16 servo_angle_break_road = SERVO_PWM_MID;
/**********采集断路区域**********/
/*******************************断路UI信息*******************************/

//专门写入断路区flash的函数
void Key_control_break_road(void)
{
    static uint8 key1_last_state1 = 0;
    static uint8 key2_last_state1 = 0;
    static uint8 key3_last_state1 = 0;
    static uint8 key4_last_state1 = 0;

    // KEY_1 防抖
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        if(key1_last_state1 == 0)
        {
            exchange_choose_option++;
            if(exchange_choose_option > 3)
            {
                exchange_choose_option = 1;
            }
        }
        key1_last_state1 = 1;
    }
    else
    {
        key1_last_state1 = 0;
    }
    
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        if(key2_last_state1 == 0)
        { 
          
            switch(exchange_choose_option)
            {
                case 1: area_memery = (area_memery >= 3) ? 1 : area_memery + 1; break;
                case 2:servo_angle_break_road += 1;  break;
                case 3:write_flag = 1;  break;
                
                
            }    
        }
    
        key2_last_state1 = 1;
    }else 
    {
         key2_last_state1 = 0;   
    }
    
    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        if(key3_last_state1 == 0)
        { 
          
            switch(exchange_choose_option)
            {
                case 1: area_memery = (area_memery >= 3) ? 1 : area_memery + 1; break;
                case 2:servo_angle_break_road -= 1;  break;
                case 3:write_flag = 1;  break;
                
                
            }   


        }
    
        key3_last_state1 = 1;
    }else 
    {
         key3_last_state1 = 0;   
    }
    

    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        if(key4_last_state1 == 0)
        {
          car_state_current = STATE_NORMAL;
          ips114_clear();
          exchange_show_page = 1;               // 重置显示页面
          exchange_choose_option = 1;           // 重置选项

        }
        key4_last_state1 = 1;
    
    
    }else 
    {
    
         key4_last_state1 = 0;     
    }





}


void Break_road_servo_show(void)
{
    servo_pwm_control(servo_angle_break_road);


}

void Write_break_road_angle(void)
{
    if(write_flag == 1)
    {
        switch(area_memery)
        {
          case 1:
                Break_road_write_select[0] = servo_angle_break_road;
                flash_break_road_1_memery_angle();
                write_flag_finish1 = 1;
                write_flag = 0;
                break;

          case 2:
                Break_road_write_select[1] = servo_angle_break_road;
                flash_break_road_2_memery_angle();
                write_flag_finish2 = 1;
                write_flag = 0;
                break;

          case 3:
                Break_road_write_select[2] = servo_angle_break_road;
                flash_break_road_3_memery_angle();
                write_flag_finish3 = 1;
                write_flag = 0;                
                
                break;                
        
        
        }
    
    
    
    }


}



/************************************函数：UI标志位功能实现************************************/


/************************************************断路************************************************/
#endif
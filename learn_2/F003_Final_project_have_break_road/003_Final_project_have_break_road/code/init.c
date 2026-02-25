#include "zf_common_headfile.h"


//初始化函数
void init(void)
{   
    /***屏幕初始化***/
     ips114_init();  
    
    /***摄像头初始化***/
     /***检验mt9v034摄像头是否成功初始化，异常初始化会导致主板LED1闪灯***/
    while(1)
    {
        if(mt9v03x_init())
            ips114_show_string(0, 7*16, "mt9v03x error!!!");
        else
            break;
        system_delay_ms(500);                                                   // 闪灯表示异常
    }
    ips114_show_string(0, 16, "init success.");//显示摄像头初始化成功
    
    
  
    /***舵机初始化***/
    servo_init();

    /***电机初始化***/
    motor_init();
    
    /***编码器初始化***/
    encoder_init();

#ifdef ENABLE_BREAK    

      flash_init();
      flash_road_read_get_all();
#endif
    
    /***按键初始化***/
    key_init(10);

    /***蜂鸣器初始化***/
    gpio_init(Buzzer, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    

    /***定时器中断初始化***/

    pit_ms_init(PIT_CH1, 10);           //电机控制     
    pit_ms_init(PIT_CH2, 20);           //舵机控制        
    pit_ms_init(PIT_CH10,10);           //按键控制   
    

}
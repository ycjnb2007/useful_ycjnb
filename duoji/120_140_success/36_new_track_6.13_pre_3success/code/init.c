#include "zf_common_headfile.h"

uint8 count = 0;                //用于定时中断计时通道内计数
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
    /***陀螺仪初始化***/
    
    if(imu660ra_init())
    {
      ips114_show_string(0,6*16,"imu660ra error!!!");
    }
    //四元数初始化
    attitude_estimator_init(&estimator, 2.0f, 0.005f, 0.01f); // Kp=2, Ki=0.005, dt=10ms

    /***FLASH初始化***/
    flash_init();
#endif
    
    /***按键初始化***/
    key_init(5);

    /***蜂鸣器初始化***/
    gpio_init(Buzzer, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    /***定时器中断初始化***/
//    pit_ms_init(PIT0, 20);
//    pit_enable(PIT0); 
    
    pit_ms_init(PIT_CH0, 10);                                                     // 初始化 PIT1 为周期中断 10ms 
    pit_ms_init(PIT_CH1, 20);
    
    

}
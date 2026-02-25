/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version



* 2024-6-12     增加可手动校准正常赛道白殿志并存入flash
********************************************************************************************************************/

#include "zf_common_headfile.h"

#if DEBUG_UART_USE_INTERRUPT
extern fifo_struct debug_uart_fifo;
#endif
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
uint8 fps_test=0;
uint8_t emergency_stop_flag = 0; // 急停标志
static uint8_t stop_match_state = 0; // 状态机指针

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    
    flash_init();                       //Flash初始化    
    load_params_from_flash();           //从flash读取参数
    trajectory_init();                  //惯导初始化，从flash读取所有断路区轨迹点
    
    mt9v03x_init ();                    //摄像头初始化        
    mykey_init();                       //按键初始化    
    imu660ra_init();                    //陀螺仪初始化
    
    quaternion_init();                  //四元数结构体初始化
    quaternion_calibrate();             // 陀螺仪校准初始化
    
    small_driver_uart_init_left( );     // 左侧无刷驱动 串口通讯初始化
    small_driver_uart_init_right( );    // 右侧无刷驱动 串口通讯初始化
    small_driver_uart_init_fan();       //负压风扇 串口通讯初始化

    ips200_init(IPS200_TYPE_SPI);       //屏幕初始化   
    
    adc_init(ADC0_CH21_P07_5, ADC_8BIT);        //  初始化电位器旋钮
    gpio_init(P19_4, GPO, 0, GPO_PUSH_PULL);    //蜂鸣器初始化
    
    pit_ms_init(PIT_CH0,1);                     // 初始化 PIT0 为周期中断 1ms 周期
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {

///////////////////////////////////////////////串口急停/////////////////////////////////////////////////////////// 
        static uint8_t uart_stop_buffer[64];
        uint32_t uart_stop_len = 64;
        fifo_read_buffer(&debug_uart_fifo, uart_stop_buffer, &uart_stop_len, FIFO_READ_AND_CLEAN);

        for (uint32_t i = 0; i < uart_stop_len; i++)
        {
            switch (stop_match_state)
            {
                case 0: stop_match_state = (uart_stop_buffer[i] == 'S') ? 1 : 0; break;
                case 1: stop_match_state = (uart_stop_buffer[i] == 'T') ? 2 : (uart_stop_buffer[i] == 'S' ? 1 : 0); break;
                case 2: stop_match_state = (uart_stop_buffer[i] == 'O') ? 3 : (uart_stop_buffer[i] == 'S' ? 1 : 0); break;
                case 3:
                    if (uart_stop_buffer[i] == 'P')
                    {
                        emergency_stop_flag = 1;
                        uart_write_string(DEBUG_UART_INDEX, "\r\n!!! EMERGENCY STOP !!!\r\n");
                        stop_match_state = 0;
                    }
                    else
                    {
                        stop_match_state = (uart_stop_buffer[i] == 'S') ? 1 : 0;
                    }
                    break;
            }
        }
///////////////////////////////////////////////串口急停/////////////////////////////////////////////////////////// 
                // 此处编写需要循环执行的代码
     
          if(ting<300||run_flag==0||(finish_flag>=2||emergency_stop_flag) )        
          {
              display( );                    //发车不显示
          }
        
        
                if((finish_flag<2)&&ting>300&&(!emergency_stop_flag))     //发车
          {
                fan_ctr();                             //负压控制
          }
        
//                 printf("%d,%d\r\n",speed,mtv9_position);                               //打印帧率
   

//    timer_init(TC_TIME2_CH0, TIMER_US);                                         // 定时器使用 TC_TIME2_CH0 使用微秒级计数
//    timer_start(TC_TIME2_CH0);                                                  // 启动定时  
              if(mt9v03x_finish_flag)
         {
              mt9v03x_finish_flag=0;
              image_processing();//图像处理流程
//              fps_test++;
         }    
//    timer_stop(TC_TIME2_CH0);                                                   // 停止定时器
//    printf("Timer count is %d us.\r\n", timer_get(TC_TIME2_CH0));               // 获取定时器的值并输出
//    timer_clear(TC_TIME2_CH0);                 
      

//        static uint32_t print_count = 0;
//        if(++print_count >= 5) 
//        {  
//            print_count = 0;
//            Quaternion* euler = get_euler_angles();
//            printf("Calib:%d Roll:%.2f Pitch:%.2f Yaw:%.2f \r\n", 
//            euler->calibrated, euler->roll, euler->pitch, euler->yaw); //打印姿态角       
//            printf("%d,%.2f,%.2f\r\n", outer_out, euler->yaw,target_yaw); //打印姿态角    
//        }   
    }
    
    

}

void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数      
{
    pit_isr_flag_clear(PIT_CH0);  
//////////  1ms中断   ////////// 
    imu660ra_get_gyro();                        //获取 IMU660RA 陀螺仪数据    
    quaternion_update(imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, 0.001f);  //  四元数更新 1ms周期
//////////  5ms中断   ////////// 
    static uint16 pit_counts_5ms = 0;
    pit_counts_5ms++;
    if(pit_counts_5ms==5)
    {
      pit_counts_5ms=0; 
      trajectory_processing();          //轨迹记录及回放    
      ting_circle++;                    //计数，防止连续判定环岛
      speed_update();
      
          if(run_flag)
            {
                ting++;                 //计数，用来延时发车
                tings++;                //计数，防止连续判定终点
                ting_check_break++;     //计数，防止连续判定断路
            }

          if(ting<300||run_flag==0)         //未发车
            {
                  menu();                  //菜单逻辑判断       
            }

          if((finish_flag<2)&&ting>300&&(!emergency_stop_flag))     //发车
            {
                  if(ting>500)pid_processing();          //PID控制过程
                  check_break_start=1;                   //启用断路检测
            } 
          else if(finish_flag>=2||emergency_stop_flag)          //停车
            {
                except_speed=0;
                except_speed_base=0;
                core_out=0;
                small_driver_set_duty_left(0, 0);                                                              // 设置0占空比
                small_driver_set_duty_right(0, 0);                                                             // 设置0占空比
                small_driver_set_duty_fan(0, 0);                                                               // 负压风扇设置0占空比
                run_flag=0;
            }
          
            if(start_angle_flag)         
           {
               update_angle();
           } 

//          static uint16 pit_counts = 0;
//          pit_counts++;
//          if(pit_counts==200)                                            
//          {
//            pit_counts=0;
//            printf("%d\r\n",fps_test);                               //打印帧率
//            fps_test=0;
//          }
          
    }
}
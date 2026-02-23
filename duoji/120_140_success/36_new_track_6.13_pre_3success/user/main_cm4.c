/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
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
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-11-19       pudding            first version
* 2025-5-22       sprite               
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************


int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口初始化
    
    init();                             //初始化
    
    for(;;)
    {

/************************************************图像处理*********************************************************/
                
         /******************************摄像头裁剪及二值化显示*******************************/      
        if(mt9v03x_finish_flag)
        {
           memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);    //将原始图像拷贝到 image_copy  这个数组中    
           crop_image_from_bottom();
//           uint8 th = otsu_threshold();
           uint8 th = 155;
           Show_binaryzation(th);                                           //二值化阈值                
           Draw_Black_Box(0);
           ips114_show_uint(190,7*16,th,3);
           
            mt9v03x_finish_flag = 0;
            
        if( start % 2 == 0)
        {        
         ips114_show_gray_image(0, 0, image[0], MT9V03X_W, CUT_HEIGHT, MT9V03X_W, CUT_HEIGHT, 0);//显示图像
        }
        
        /******************************寻迹状态处理*******************************/      
        Trackstate = determine_tracking_state();
        
        line_tracking_state_machine();            
            
            
            
              
        }

        
        
/************************************************图像处理*********************************************************/
        
 
        
        
        
/************************************************舵机pid及速度调节显示*********************************************************/
            
    
    if( start % 2 == 0)
        { 
        
//        printf("%f, %f\r\n",L_motor.encoder_speed,R_motor.encoder_speed);
          //圆环状态
          ips114_show_uint(190,2*16,roundabout_dir,2);
          ips114_show_uint(190,3*16,roundabout_current_state,2);
          
          
          //断路偏航角
          ips114_show_float(190,0,(double)angles.yaw,2,2);
          ips114_show_float(190,1*16,(double)flash_union_buffer[final_yaw_num].float_type,2,2);        
            
            
            //舵机pid及速度调节显示
          
           ips114_show_string(0*16,6*16,"P");
           ips114_show_float(1*16,6*16,(double)Servo_pid.Kp,2,1);  
          
           ips114_show_string(3*16,6*16,"D");
           ips114_show_float(4*16,6*16,(double)Servo_pid.Kd,2,1);          
          
           ips114_show_string(7*16,6*16,"K");
           ips114_show_float(8*16,6*16,(double)differential_k,2,1);          
          
           ips114_show_string(0*16,7*16,"E");
           ips114_show_uint(1*16,7*16,exchange,1);
           
           ips114_show_string(3*16,7*16,"S");
           ips114_show_uint(4*16,7*16,start,1);
              
           ips114_show_string(7*16,7*16,"V");
           ips114_show_uint(8*16,7*16,normol_speed,3);
  
        }
        
//        printf("%d, %f, %d, %f\r\n",L_motor.target_speed,L_motor.encoder_speed,R_motor.target_speed,R_motor.encoder_speed);

/************************************************圆环调试显示*********************************************************/
        
//        //显示按键选择
//        ips114_show_string(190,0*16,"S");
//        ips114_show_uint(190+1*16,0*16,start,2);
//        ips114_show_string(190,1*16,"E");
//        ips114_show_uint(190+1*16,1*16,exchange,2);        
//        ips114_show_string(190,2*16,"V");
//        ips114_show_uint(190+1*16,2*16,normol_speed,3);        
//        
//        
//        //显示圆环状态
//        ips114_show_string(0*16,5*16,"dir:");
//        ips114_show_uint(3*16,5*16,roundabout_dir,1);
//        ips114_show_string(0*16,6*16,"state");
//        ips114_show_uint(3*16,6*16,roundabout_current_state,1);
//        
//        //显示元素行和元素列
//        ips114_show_string(5*16,5*16,"Y");
//        ips114_show_uint(5*16,6*16,up_row_num,1);  
//        ips114_show_uint(6*16,6*16,down_row_num,1);  
//        ips114_show_uint(5*16,7*16,left_col_num,1);  
//        ips114_show_uint(6*16,7*16,right_col_num,1);  
//           
//        
//        ips114_show_string(8*16,5*16,"N");
//        ips114_show_uint(8*16,6*16,up_row_num1,1);
//        ips114_show_uint(9*16,6*16,down_row_num1,1);
//        ips114_show_uint(8*16,7*16,left_col_num1,1);
//        ips114_show_uint(9*16,7*16,right_col_num1,1);
//        
//        
//        //显示起始点
//        ips114_show_float(10*16,5*16,(L_start_point[0] + R_start_point[0])/2,3,1);
//        ips114_show_float(12*16,5*16,(L_start_point[1] + R_start_point[1])/2,3,1);
//        
//        ips114_show_uint(10*16,6*16,roundabout_line_points[0].x,3);
//        ips114_show_uint(12*16,6*16,roundabout_line_points[0].y,3);
//        
           
/************************************************舵机pid及速度调节显示*********************************************************/

           
           
           
           
// 此处编写需要循环执行的代码         
    }
}

// **************************** 代码区域 ****************************

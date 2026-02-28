/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
//摄像头一维数组设置
#define USER_IMAGE mt9v03x_image[0]
// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等


        mt9v03x_init();                     //总转风摄像头初始化
        tft180_set_dir(TFT180_CROSSWISE);
         tft180_init();
        // tft180_full(RGB565_YELLOW);

         //UI_Init();
        // DRV8701_init();
         imu660rb_init();
         // 滤波器初始化
        // 这里的 10.0f 是截止频率，越小越平滑但延迟越高；0.005f是采样时间(5ms)
        LPF_InitByFrequency(&velocity_filter, 10.0f, 0.005f);
        gyro_zero_param_init();
        acc_zero_param_init();
        pit_ms_init(CCU60_CH0, 5);
        Motor_Init();







        // 此处编写用户代码 例如外设初始化代码等
        cpu_wait_event_ready();         // 等待所有核心初始化完毕
        while (TRUE)
        {
            // 此处编写需要循环执行的代码
             Motor_Control(2800, 2800);




           // printf("%d,%d,%d\n", gyro_param.gyro_x,  gyro_param.gyro_y,  gyro_param.gyro_z);
           // printf("%d,%d,%d\n", acc_param.acc_x, acc_param.acc_y, acc_param.acc_z);
           // printf("%.2f,%.2f,%.2f,%.2f\n", gyro_param.gyro_x, gyro_param.gyro_y, gyro_param.gyro_z,yaw);
            tft180_show_int(0, 0, yaw_plus, 10);

                    //system_delay_ms(100);
                    // DRV8701_motor_driver(int L_SPEED,int R_SPEED)     //设置DRV的正反转


           //tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 150);
            if(1)
                    {


                //DRV8701_motor_go();
               // go();
                //tft180_show_gray_image(0, 0, imgGray[0], XM, YM, XM , YM , nowThreshold);
                //DRV8701_motor_go();
                //DRV8701_motor_driver(0,100);     //设置DRV的正反转












                        //UI_Task();
                       // mt9v03x_finish_flag = 0;



                    }

            // 此处编写需要循环执行的代码
        }
}

#pragma section all restore
// **************************** 代码区域 ****************************

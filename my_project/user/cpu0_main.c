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


    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口

    // === 2. 硬件外设初始化 ===
    mt9v03x_init();                 // 总转风摄像头初始化

    // 屏幕强制使用横屏，适配菜单
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();

    imu660rb_init();
    // 滤波器初始化
    LPF_InitByFrequency(&velocity_filter, 10.0f, 0.005f);
    gyro_zero_param_init();
    acc_zero_param_init();

    Motor_Init();

    // === 3. 菜单与按键初始化 ===
    UI_Menu_Init();

    // === 4. 最后开定时器中断，防止一上电还没初始化完就进中断跑飞 ===
    // 5ms触发一次 cc60_pit_ch0_isr
    pit_ms_init(CCU60_CH0, 5);

    cpu_wait_event_ready();         // 等待所有核心初始化完毕

    while (TRUE)
    {
        // === 5. 菜单调度任务 (发车前有效，发车后内部会自动直接退出) ===
        UI_Menu_Task();

        // === 6. 图像处理与寻线逻辑 ===
        // 这里假设你在其他地方或者用 mt9v03x_finish_flag 标志位判断图像是否采集完
        if (mt9v03x_finish_flag)
        {
            // 在这里放你的图像处理代码
            // image_copy();
            // Get_imgOSTU();
            // error_val = xxx; // 计算出中线偏差并更新 error_val 供 PID 和屏幕使用

            mt9v03x_finish_flag = 0; // 清空标志位
        }

        // === 7. 发车状态检测与安全锁 ===
        if (system_running == 1)
        {
            // 发车后主循环只全力算图像，绝对不管屏幕了！
            // 底层电机控制权完全交给 isr.c 里的 5ms 定时器中断
        }
        else
        {
            // 停车模式下：强制输出电机为 0，防止把车放在地上调参时车跑了伤人
            Motor_Control(0, 0);
        }
    }
}


#pragma section all restore
// **************************** 代码区域 ****************************

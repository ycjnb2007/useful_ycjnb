

// *
// *                             _ooOoo_
// *                            o8888888o
// *                            88" . "88
// *                            (| -_- |)
// *                            O\  =  /O
// *                         ____/`---'\____
// *                       .'  \\|     |//  `.
// *                      /  \\|||  :  |||//  \
// *                     /  _||||| -:- |||||-  \
// *                     |   | \\\  -  /// |   |
// *                     | \_|  ''\---/''  |   |
// *                     \  .-\__  `-`  ___/-. /
// *                   ___`. .'  /--.--\  `. . __
// *                ."" '<  `.___\_<|>_/___.'  >'"".
// *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |
// *               \  \ `-.   \_ __\ /__ _/   .-` /  /
// *          ======`-.____`-.___\_____/___.-`____.-'======
// *                             `=---='
// *          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// *                佛祖保佑      永无BUG     永不修改
// *         佛曰：
// *                实验室中调车人，调车代码两相缠；
// *                三更焊板星辰伴，五更调试月作弦。
// *                醒时参数调千遍，醉卧赛道枕电池；
// *                调参整定日复日，互补滤波年复年。
// *                但求算法无妄念，更祈电路皆周全；
// *                电机温润如春水，芯片清凉若山泉。
// *                陀仪不飘循迹稳，舵机灵敏转向翩；
// *                电容未炸晶振起，电池长续航无边。
// *                队友同心破万难，强敌皆抛锚路边；
// *                串口无言报捷讯，灯带流光庆凯旋。
// *                他日赛场锋芒现，代码风驰赛道燃；
// *                诸佛护我战车疾，捧得金杯笑开颜！

#include "zf_common_headfile.h"
#include "key.h"      // 菜单系统头文件
#include "my_flash.h" // Flash存储管理

// **************************** 代码区域 ****************************
// uint8 image_copy[60][100]; // 图像缓存
int main(void)
{
    clock_init(SYSTEM_CLOCK_160M); // 时钟配置及系统初始化<务必保留>
    debug_info_init();             // 调试串口信息初始化

    // 初始化蜂鸣器IO口
    gpio_init(BEE_IO, GPO, GPIO_LOW, GPO_PUSH_PULL);
    BEE_STA(1);
    system_delay_ms(1000);
    BEE_STA(0);
    mt9v03x_init();
    tft180_init();
    wireless_uart_init();
    timer_init(TC_TIME2_CH0, TIMER_MS);
    Encoder_Motor_Init();
    circle_init();
    flash_param_init();
    control_system_init();
    pit_ms_init(PIT_CH11, 10); // 1ms定时器

    if (imu660ra_init() == 0)
    {
        imu_init();
        pit_ms_init(PIT_CH10, 5);
    }

    // 增量式pid初始化
    Incremental_PID_Init(&pid_left, speed_kp, speed_ki, speed_kd, 7000, -7000);
    Incremental_PID_Init(&pid_right, speed_kp, speed_ki, speed_kd, 7000, -7000);
    // 初始化控制系统，确保所有变量都为0
    control_system_init();
    break_road_init();
    // 初始化菜单系统
    my_key_init(); // 按键初始化

    // set_image_process_mode(1);
    while (true)
    {
        // 根据system_running状态控制菜单模式和运行模式
        if (!system_running)   // 菜单模式
        {
            // Image_Process();
            key_scan(); // 按键扫描和处理
            
            // 处理Flash操作
            flash_write_params(); // 处理flash写入请求
            flash_read_params();  // 处理flash读取请求

            // 图像显示模式或断路记录模式下的特殊处理
            if (image_display_mode || current_page == MENU_BREAK_RECORD_PAGE)
            {
                // 需要先处理图像数据，然后显示
                Image_Process(); // 处理图像数据（但不进行控制逻辑）
                menu_display();  // 显示菜单

                if (image_display_mode)
                {
                    // 图像显示模式：显示摄像头图像和调试信息
                    tft180_show_gray_image(0, 0, image[0], 120, 70, 120, 70, 0);
                    uint8 time = timer_get(TC_TIME2_CH0);
                    // tft180_show_uint(0, 80, time, 3);
                    draw_line();

                    // 环岛参数检测
                    //  tft180_show_uint(70, 100, bottom_left_white, 3);
                    //  tft180_show_uint(90, 100, bottom_right_white, 3);
                    //  tft180_show_uint(20, 100, top_left_white, 3);
                    //  tft180_show_uint(40, 100, top_right_white, 3);

                    // 十字修复参数
                    // tft180_show_uint(20, 80, ImageFlag.straight_long, 3);
                    // tft180_show_uint(40, 80, judge.Right_Flag, 3);
                    // tft180_show_uint(75, 80, judge.Left_Flag, 3);

                    //车辆模块检查显示
                    // tft180_show_float(100, 80, Err, 3, 1);
                    tft180_show_uint(20, 80, Circle.Left_flag, 3);
                    tft180_show_uint(40, 80, Circle.Right_flag, 3);
                    tft180_show_uint(75, 80, ImageFlag.Cross, 3);
                    tft180_show_float(100, 80, eulerAngle.yaw, 3, 1);
                    tft180_show_uint(0, 100, outsthreshold, 3);
                    tft180_show_float(30, 100, Encoer_Speed[0], 3, 1);
                    tft180_show_float(75, 100, Encoer_Speed[1], 3, 1);
                }
            }
            else
            {
                // 普通菜单模式：正常显示菜单
                menu_display(); // 菜单显示
            }
        }
        else   // 运行模式
        {
                    
            // 图像处理
            Image_Process();
            key_scan();
            // printf("%f,%f,%f\n", Err, angular_rate_target, angular_rate_current);
            // printf("%f,%f,%f,%f\n", target_left_speed, Encoer_Speed[0], Err, angular_rate_target);
            // printf("%f,%f,%f\n", target_yaw, current_yaw, break_speed_diff);
            // printf("%f,%d\n", Circle.now_yaw, Circle.out_flag);
            // printf("%f,%f,%f,%f\n", target_left_speed, Encoer_Speed[0], target_right_speed, Encoer_Speed[1]);
            // printf("%d,%d,%d,%d,%d\n", Circle.consider_circle_top, Circle.consider_circle, Circle.consider_circle_down, top_left_white,  bottom_left_white);
            // printf("%d,%d,%d,%d\n", top_left_white, top_right_white, bottom_left_white, bottom_right_white);
                             
        }
    }
}
// **************************** 代码区域 ****************************

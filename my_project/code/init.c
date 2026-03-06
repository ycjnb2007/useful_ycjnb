#include "init.h"
#include "image_deal_best.h"
#include "imu660.h"
#include "motor.h"
#include "pid.h"
#include "filter.h"

/******************************************************************************
* 函数名称     : init
* 描述         : 系统总初始化，上电后调用一次
******************************************************************************/
void init(void)
{
    /* 1. 电机与编码器初始化 */
    Motor_Init();
    Encoder_Init();

    /* 2. IMU660RB 陀螺仪初始化 + 零漂校准（需要静止约1秒） */
    imu660rb_init();
    gyro_zero_param_init();
    acc_zero_param_init();

    /* 3. PID 参数初始化 */
    PID_Init();

    /* 4. 低通滤波器初始化（编码器速度用） */
    LPF_InitByAlpha(&velocity_filter, 0.3f);

    /* 5. 摄像头初始化（逐飞库自带，通常在 main 里已调用） */
    /* mt9v03x_init(); */

    /* 6. 图像处理初始状态 */
    standard();
    cur_state = STATE_NORMAL;
    is_blind_turning = 0;
    node_index = 0;
}

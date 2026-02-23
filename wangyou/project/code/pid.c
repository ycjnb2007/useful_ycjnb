#include "pid.h"
float error_left;
float error_right;

// 获取PID输出
float output_left;
float output_right;
// float angular_rate_current = 0; // 当前角速度

void Incremental_PID_Init(Incremental_PID_TypeDef *PID, float Kp, float Ki, float Kd, float Out_Put_Max, float Out_Put_Min)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->Error = 0;
    PID->Error_Last = 0;
    PID->Error_Pre = 0;
    PID->Out_Put_Min = Out_Put_Min;
    PID->Out_Put_Max = Out_Put_Max;
}

float Get_Incremental_PID_Value(Incremental_PID_TypeDef *PID, float Error)
{
    float P = 0.0f, I = 0.0f, D = 0.0f;
    static float Increment = 0; // 增量

    PID->Error = Error; // 获取误差

    P = PID->Error - PID->Error_Last; // 当前误差-上一次误差
    I = PID->Error;
    D = PID->Error - 2 * PID->Error_Last + PID->Error_Pre;

    Increment = PID->Kp * P + PID->Ki * I + PID->Kd * D;

    PID->Error_Pre = PID->Error_Last;
    PID->Error_Last = PID->Error;

    PID->Output = PID->Output + Increment;
    // 限幅
    if (PID->Output > PID->Out_Put_Max)
    {
        PID->Output = PID->Out_Put_Max;
    }
    else if (PID->Output < PID->Out_Put_Min)
    {
        PID->Output = PID->Out_Put_Min;
    }

    return PID->Output;
}

Incremental_PID_TypeDef pid_left, pid_right;
int target_speed = 60; // 标准速度

float target_left_speed, target_right_speed;
float angular_rate_target; // 角速度目标值

float angular_error;    // 角速度误差
float speed_difference; // 速度差值

// 外环控制器输出角速度
float outer_loop_control(float error)
{
    static float last_error = 0;
    // static uint8 reset_flag = 1; // 重置标志

    // // 检查是否需要重置静态变量
    // extern uint8 system_running;
    // if (!system_running)
    // {
    //     reset_flag = 1; // 系统停止时设置重置标志
    // }

    // if (reset_flag)
    // {
    //     last_error = 0;
    //     reset_flag = 0;
    // }

    float Kp_outer;
    float Kd_outer;
    // if (judge.Right_Flag == 1 || judge.Left_Flag == 1)
    // {
    //     Kp_outer = 3.8; // 外环PD参数，需要根据实际情况调整
    //     Kd_outer = 1.5;
    // }
    // else
    // {
    //     Kp_outer = 3.65; // 外环PD参数，需要根据实际情况调整
    //     Kd_outer = 1.0;
    // }
    Kp_outer = angle_kp; // 外环PD参数，需要根据实际情况调整
    Kd_outer = angle_kd;

    float derivative = error - last_error;
    angular_rate_target = Kp_outer * error + Kd_outer * derivative;
    last_error = error;
    return angular_rate_target;
}

// 中间环角速度环输出速度差值
float middle_loop_control()
{
    static float last_angular_error = 0;
    // static uint8 reset_flag = 1; // 重置标志

    // // 检查是否需要重置静态变量
    // extern uint8 system_running;
    // if (!system_running)
    // {
    //     reset_flag = 1; // 系统停止时设置重置标志
    // }

    // if (reset_flag)
    // {
    //     last_angular_error = 0;
    //     reset_flag = 0;
    // }

    angular_error = angular_rate_target - angular_rate_current;
    const float Kp_middle = gyro_kp; // ä¸­é—´çŽ¯PDå�‚æ•°ï¼Œéœ€è¦�æ ¹æ�®å®žé™…æƒ…å†µè°ƒæ•´//0.6
    const float Kd_middle = gyro_kd;

    float derivative = angular_error - last_angular_error;
    speed_difference = Kp_middle * angular_error + Kd_middle * derivative;
    last_angular_error = angular_error;
    return speed_difference;
}

// 修改Speed_ClosedLoop_Control函数，使用三环控制机制
void Speed_ClosedLoop_Control()
{
    // 外环
    angular_rate_target = outer_loop_control(Err);
    angular_rate_target = Limit_float(-140, angular_rate_target, 140);
    // 中间环
    speed_difference = middle_loop_control();
    // speed_difference = Limit_float(-70, speed_difference, 70); // 限制速度差范围
    // 内环差分式速度环 - 分段速度控制
    uint16 current_speed = normal_speed; // 默认正常速度

    // 根据路径状态选择对应速度

    if (ImageFlag.Bend_Road != 0)
    {
        current_speed = curve_speed; // 弯道速度
    }
    else if (ImageFlag.straight_long)
    {
        current_speed = straight_speed; // 直道正常速度
    }
    else
    {
        current_speed = normal_speed;
    }

    target_left_speed = current_speed - speed_difference;
    target_right_speed = current_speed + speed_difference;

    target_left_speed = Limit_float(-10, target_left_speed, 120);
    target_right_speed = Limit_float(-10, target_right_speed, 120);

    int16 error_left = (int16)target_left_speed - (int16)Encoer_Speed[0];
    int16 error_right = (int16)target_right_speed - (int16)Encoer_Speed[1];

    //     error_left = target_speed - Encoer_Speed[0];//------------速度闭环控制
    //     error_right = target_speed - Encoer_Speed[1];

    // 获坖PID输出
    output_left = Get_Incremental_PID_Value(&pid_left, error_left);
    output_right = Get_Incremental_PID_Value(&pid_right, error_right);
    // 在停止模式下将电机设置为0，只有运行模式下才计算PID
    // extern uint8 system_running; // 引用系统运行标志
}

// 控制系统初始化函数
void control_system_init(void)
{
    // 初始化所有静态变量
    angular_rate_current = 0.0f;
    angular_rate_target = 0.0f;
    angular_error = 0.0f;
    speed_difference = 0.0f;
    target_left_speed = 0.0f;
    target_right_speed = 0.0f;
    output_left = 0.0f;
    output_right = 0.0f;

    // 重置PID控制器
    pid_left.Output = 0.0f;
    pid_left.Error = 0.0f;
    pid_left.Error_Last = 0.0f;
    pid_left.Error_Pre = 0.0f;

    pid_right.Output = 0.0f;
    pid_right.Error = 0.0f;
    pid_right.Error_Last = 0.0f;
    pid_right.Error_Pre = 0.0f;
}

// 速度测试函数，用于调试电机响应，可以直接控制左右电机
static uint16 speed_test_timer = 0;
static uint8 speed_test_mode = 0; // 0=低速30, 1=高速50
float test_speed = -30;           // 初始目标速度为30
void Speed_Control_Test(void)
{
    // 增加计时器计数
    speed_test_timer++;

    // 假设中断周期为5ms，400次为2秒 (400 * 5ms = 2000ms = 2s)
    if (speed_test_timer >= 200)
    {
        speed_test_timer = 0;               // 重置计时器
        speed_test_mode = !speed_test_mode; // 切换模式

        // 根据模式设置目标速度
        if (speed_test_mode)
        {
            test_speed = 30; // 高速模式
        }
        else
        {
            test_speed = -30; // 低速模式
        }
    }
    float error_left = test_speed - Encoer_Speed[0]; //------------速度闭环控制
    float error_right = test_speed - Encoer_Speed[1];

    output_left = Get_Incremental_PID_Value(&pid_left, error_left);
    output_right = -output_left;
    // output_right = Get_Incremental_PID_Value(&pid_right, error_right);
}

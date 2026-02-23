#include "motor_control.h"

int16 Encoder_Pules_Buffer[2]; // 脉冲数组
float Encoer_Speed[2];         // 速度数组

int32 Left_motorpwm, Right_motorpwm; // pwm的占空比
int32 Left_motorpwm_num, Right_motorpwm_num;

// 断路距离记录功能 - 整合版 (*10记录)
#define PULSES_PER_METER 4500.0f // 前进一米的编码器脉冲数

float break_dis = 0.0f;               // 断路距离 (cm)
static int32 break_encoder_total = 0; // 断路期间编码器累计值
static uint8 break_recording = 0;     // 断路记录状态标志
/*
#define ENCODER1 (TC_CH09_ENCODER)                     // 编码器接口   左
#define ENCODER1_QUADDEC_A (TC_CH09_ENCODER_CH1_P05_0) // A引脚
#define ENCODER1_QUADDEC_B (TC_CH09_ENCODER_CH2_P05_1) // B引脚

#define ENCODER2 (TC_CH07_ENCODER)                     // 编码器接口   右
#define ENCODER2_QUADDEC_A (TC_CH07_ENCODER_CH1_P02_0) // A引脚
#define ENCODER2_QUADDEC_B (TC_CH07_ENCODER_CH2_P02_1) // B引脚
*/
void Encoder_Motor_Init()
{
    // 编码器初始化
    encoder_dir_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P02_0, TC_CH07_ENCODER_CH2_P02_1); // 右
    encoder_dir_init(TC_CH09_ENCODER, TC_CH09_ENCODER_CH1_P05_0, TC_CH09_ENCODER_CH2_P05_1); // 左
    // 电机初始化
    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    gpio_init(DIR2, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM1, 20000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM2, 20000, 0);                       // PWM 通道初始化频率 17KHz 占空比初始为 0

    // 断路距离记录初始化
    break_road_distance_init();
}

void Encoder_Getvalue()
{
    Encoder_Pules_Buffer[1] = -encoder_get_count(TC_CH07_ENCODER); // 变量接收右编码器的数据

    encoder_clear_count(TC_CH07_ENCODER); // 复位编码器1

    Encoder_Pules_Buffer[0] = encoder_get_count(TC_CH09_ENCODER); // 变量接收左编码器的数据

    encoder_clear_count(TC_CH09_ENCODER); // 复位编码器2
}

static float Get_LF_Speed()
{
    static float Filter;
    float Speed = (Encoder_Pules_Buffer[0] / 600.0f) * 100.0f;
    Filter = 0.25f * Speed + Filter * 0.75f;

    return Filter;
}

static float Get_RF_Speed()
{
    static float Filter;
    float Speed = (Encoder_Pules_Buffer[1] / 600.0f) * 100.0f;
    Filter = 0.25f * Speed + Filter * 0.75f;

    return Filter;
}

void encoder_process()
{
    Encoder_Getvalue();
    Encoer_Speed[0] = Get_LF_Speed();
    Encoer_Speed[1] = Get_RF_Speed();

    // 更新断路距离记录
    break_road_distance_update();
}

void Motor_Ctrl(int32 Left_PWM, int32 Right_PWM)
{
    if (Left_PWM >= 0)
    {
        Left_PWM = (Left_PWM > PWM_Motor_MAX) ? PWM_Motor_MAX : Left_PWM;
        gpio_set_level(DIR1, 1);
        pwm_set_duty(PWM1, Left_PWM);
    }
    else
    {
        Left_PWM = (-Left_PWM > PWM_Motor_MAX) ? PWM_Motor_MAX : -Left_PWM;
        gpio_set_level(DIR1, 0);
        pwm_set_duty(PWM1, Left_PWM);
    }
    if (Right_PWM >= 0)
    {
        Right_PWM = (Right_PWM > PWM_Motor_MAX) ? PWM_Motor_MAX : Right_PWM;
        gpio_set_level(DIR2, 0);
        pwm_set_duty(PWM2, Right_PWM);
    }
    else
    {
        Right_PWM = (-Right_PWM > PWM_Motor_MAX) ? PWM_Motor_MAX : -Right_PWM;
        gpio_set_level(DIR2, 1);
        pwm_set_duty(PWM2, Right_PWM);
    }
}

// ===== 断路距离记录功能=====
/**
 * @brief 断路距离初始化
 */
void break_road_distance_init(void)
{
    break_dis = 0.0f;
    break_encoder_total = 0;
    break_recording = 0;
}

/**
 * @brief 更新断路距离记录
 */
void break_road_distance_update(void)
{
    if (break_recording)
    {
        // 计算当前编码器增量 (左右轮平均)
        int32 current_encoder = (Encoder_Pules_Buffer[0] + Encoder_Pules_Buffer[1]) / 2;
        break_encoder_total += current_encoder;

        // 计算距离 (*10记录，直接输出cm)
        break_dis = ((float)break_encoder_total / PULSES_PER_METER) * 10.0f;
    }
}

/**
 * @brief 开始记录断路距离
 */
void start_break_road_recording(void)
{
    break_recording = 1;
    break_encoder_total = 0;
    break_dis = 0.0f;
}

/**
 * @brief 停止记录断路距离
 */
void stop_break_road_recording(void)
{
    break_recording = 0;
}



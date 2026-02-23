#include "zf_common_headfile.h"

uint8 encoder_distance_flag = 0;
//定义电机信息
motor L_motor;
motor R_motor;

/**编码器初始化**/
void encoder_init(void)
{
    encoder_dir_init(ENCODER_QUAD1, ENCODER_QUAD1_PHASE_A, ENCODER_QUAD1_PHASE_B);  // 初始化编码器模块与引脚 方向编码器模式
    encoder_dir_init(ENCODER_QUAD2, ENCODER_QUAD2_PHASE_A, ENCODER_QUAD2_PHASE_B);  // 初始化编码器模块与引脚 方向编码器模式
    encoder_clear_count(ENCODER_QUAD1);
    encoder_clear_count(ENCODER_QUAD2);
    
    
    
}


/**获取编码器值**/

void encoder_data_get(void)
{
    L_motor.encoder_raw = encoder_get_count(ENCODER_QUAD1);     //获取编码器数值
    L_motor.encoder_speed = L_motor.encoder_speed * 0.2 + L_motor.encoder_raw *0.8;//编码器数值经过滤波之后作为编码器速度
    L_motor.total_encoder += L_motor.encoder_raw;               //编码器数值累加
    encoder_clear_count(ENCODER_QUAD1);
    
    R_motor.encoder_raw = -encoder_get_count(ENCODER_QUAD2);
    R_motor.encoder_speed = R_motor.encoder_speed * 0.2 + R_motor.encoder_raw *0.8;
    R_motor.total_encoder += R_motor.encoder_raw;
    encoder_clear_count(ENCODER_QUAD2);
    

}


/**
 * @brief 轮子数据结构体，封装单个轮子的编码器相关数据
 *
 * @成员 total_pulses     累计脉冲数，用于虚拟清零功能
 * @成员 encoder_last     上一次读取的编码器值，用于计算增量
 * @成员 encoder_prev_speed 速度计算基准值，保存上一次中断时的编码器值
 * @成员 finish_flag;     达到设定阈值标志位
 * @成员 target_pulses    目标脉冲数，达到此值后触发虚拟清零（应设为正数）
 */

// 初始化左轮数据结构体
// - total_pulses: 初始累计脉冲数为0
// - last_total_pulses: 初始上一次脉冲数为0
// - encoder_last: 初始编码器值为0
// - encoder_prev_speed: 初始速度基准值为0
// - target_pulses: 虚拟清零阈值为120脉冲（需根据实际调整）1cm有多少个脉冲
WheelData wheel_left = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 120};

// 初始化右轮数据结构体（配置同左轮）
WheelData wheel_right = {
    .total_pulses = 0,
    .last_total_pulses = 0,
    .encoder_last = 0,
    .encoder_prev_speed = 0,
    .target_pulses = 120};


/**
 * @brief 计算编码器增量，自动处理int16溢出
 *
 * @param current 当前编码器值（int16类型）
 * @param last 上一次编码器值（int16类型）
 * @return int32_t 修正后的增量（实际物理脉冲变化）
 *
 * @说明
 * - 编码器计数器为16位有符号数（-32768~32767），
 *   当计数器溢出时（如从32767变为-32768），直接相减会得到错误增量。
 * - 此函数通过判断差值范围，修正正向和反向溢出。
 */
static inline int32_t calculate_delta(int16_t current, int16_t last)
{
    int32_t delta = (int32_t)current - (int32_t)last;

    // 处理正向溢出（例如从32767到-32768，实际增量为+1）
    if (delta > 32767)
    {
        delta -= 65536; // 65536 = 2^16
    }
    // 处理反向溢出（例如从-32768到32767，实际增量为-1）
    else if (delta < -32768)
    {
        delta += 65536;
    }

    return delta;
}

/**
 * @brief 更新轮子脉冲累计值并处理虚拟清零
 *
 * @param wheel 轮子数据指针（指向左轮或右轮的结构体）
 * @param current_encoder 当前编码器值
 *
 * @说明
 * - 此函数应在主循环中定期调用，确保实时更新脉冲累计值。
 * - 防御性检查过滤异常脉冲跳变（如信号干扰）。
 * - 使用"跨阈值检测法"同时支持正向和反向累计，解决前进后退交替时计数卡住问题。
 * - 添加智能delta处理，解决正反转切换时编码器值跳变导致的计数卡住问题。
 */
int32_t Update_Wheel_Pulses(WheelData *wheel, int16_t current_encoder)
{
    // 1. 计算增量 (包含溢出处理)
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_last);
    int32_t forward_crossings = 0; // 在函数开始时声明并初始化

    // 2. 智能跳变检测逻辑
    //    阈值需要根据实际情况调整，目标是捕获硬件跳变，但不误伤高速脉冲。
    if (abs(delta) > 1000)
    {
        // printf("Jump detected! Delta: %d, LastEnc: %d, CurrEnc: %d\n",
        //        (int)delta, (int)wheel->encoder_last, (int)current_encoder); // 调试输出

        // 重置累计脉冲，避免跳变干扰后续计数
        wheel->total_pulses = 0;
        // 本次调用不计算位移，因为发生了跳变重置
        forward_crossings = 0; // 确保跳变时返回0
    }
    else
    {
        // --- 正常脉冲累加和计数值计算 ---
        // 保存上一次的累计脉冲值，用于检测跨阈值 (这个变量在新的逻辑中实际未使用，但保留也无害)
        wheel->last_total_pulses = wheel->total_pulses;

        // 更新累计脉冲数（支持正反转，负值表示反向转动）
        wheel->total_pulses += delta;

        // 计算前向阈值穿越次数
        if (wheel->total_pulses >= wheel->target_pulses)
        {
            forward_crossings = wheel->total_pulses / wheel->target_pulses; // 计算穿越了多少次阈值
            wheel->total_pulses %= wheel->target_pulses;                    // 保留余数，用于下一次累计
        }
        // 处理反向穿越 (如果需要反向计数逻辑的话)
        else if (wheel->total_pulses <= -wheel->target_pulses)
        {
            wheel->total_pulses %= wheel->target_pulses; // 模拟取模，保持在阈值范围内
                                                         // 确保如果刚好是 -target_pulses，模运算后变为 0
            if (wheel->total_pulses == -wheel->target_pulses)
            {
                wheel->total_pulses = 0;
            }
        }
    }

    // 3. 保存当前编码器值供下次使用
    wheel->encoder_last = current_encoder;

    // 4. 返回本次计算出的前进厘米数
    return forward_crossings;
}


/**
 * @brief 获取轮子在固定时间窗口5ms内的速度（脉冲增量）
 *
 * @param wheel 轮子数据指针
 * @param current_encoder 当前编码器值
 * @return int32_t 时间窗口内的脉冲增量（正值为正向，负值为反向）
 *
 * @说明
 * - 此函数应在定时中断（如5ms）中调用，确保固定时间间隔。
 * - 每次调用会更新encoder_prev_speed为当前值，供下次计算使用。
 */
int32_t Get_Wheel_Speed(WheelData *wheel, int16_t current_encoder)
{
    // 计算当前编码器值与上一次中断时的基准值的增量
    int32_t delta = calculate_delta(current_encoder, wheel->encoder_prev_speed);

    // 更新基准值为当前值，用于下一次速度计算
    wheel->encoder_prev_speed = current_encoder;

    if (abs(delta) > 1000)
        delta = 0;
    return delta;
}

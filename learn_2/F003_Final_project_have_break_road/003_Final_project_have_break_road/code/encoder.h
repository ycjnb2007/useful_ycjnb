#ifndef _encoder_h
#define _encoder_h



#define ENCODER_QUAD1                    (TC_CH09_ENCODER)                      // 编码器接口(左轮编码器)
#define ENCODER_QUAD1_PHASE_A            (TC_CH09_ENCODER_CH1_P05_0)            // PHASE_A 对应的引脚                 
#define ENCODER_QUAD1_PHASE_B            (TC_CH09_ENCODER_CH2_P05_1)            // PHASE_B 对应的引脚                   
                                                                                
#define ENCODER_QUAD2                    (TC_CH07_ENCODER)                      // 编码器接口(右轮编码器)
#define ENCODER_QUAD2_PHASE_A            (TC_CH07_ENCODER_CH1_P02_0)            // PHASE_A 对应的引脚
#define ENCODER_QUAD2_PHASE_B            (TC_CH07_ENCODER_CH2_P02_1)            // PHASE_B 对应的引脚


extern uint8 encoder_distance_flag;


void encoder_init(void);                //方向编码器初始化
void encoder_data_get(void);            //读取编码器值


typedef struct
{
    volatile int32_t total_pulses;       // 累计脉冲数（可能为负，表示反向转动）
    volatile int32_t last_total_pulses;  // 上一次的累计脉冲数，用于检测跨阈值
    volatile int16_t encoder_last;       // 上一次编码器值（用于Update_Wheel_Pulses）
    volatile int16_t encoder_prev_speed; // 速度计算基准值（用于Get_Wheel_Speed）
    const int32_t target_pulses;         // 虚拟清零阈值（常量，初始化后不可修改）
} WheelData;
extern WheelData wheel_left, wheel_right;

extern volatile int32_t speed_left;  // 左轮速度（脉冲/5ms）
extern volatile int32_t speed_right; // 右轮速度（脉冲/5ms）
extern volatile int16 err_guandao_plus;         // 惯导差速



int32_t Update_Wheel_Pulses(WheelData *wheel, int16_t current_encoder);
int32_t Get_Wheel_Speed(WheelData *wheel, int16_t current_encoder);

#endif
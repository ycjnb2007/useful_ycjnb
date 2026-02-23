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
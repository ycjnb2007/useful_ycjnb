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


#endif
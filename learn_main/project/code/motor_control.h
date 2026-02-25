#ifndef _motor_control_h_
#define _motor_control_h_
#include "zf_common_headfile.h"

//============================ ?´ù¹µÀ©°úµÓÄê? ============================//
#define DIR2 (P00_2)            // ±¦?´ùÊı¸ş¹µÀ©°úµÓ
#define PWM2 (TCPWM_CH13_P00_3) // ±¦?´ùPWM?½Ğ°úµÓ
#define DIR1 (P18_6)            // º¸?´ùÊı¸ş¹µÀ©°úµÓ
#define PWM1 (TCPWM_CH50_P18_7) // º¸?´ùPWM?½Ğ°úµÓ
#define PWM_Motor_MAX 6000      // ?´ùPWMºÇÂç?¸ÂÀ©

void Encoder_Motor_Init();                      // ??´ïÏÂ?´ù½é»Ï²½È¡¿ô
extern void encoder_process();                 // ??´ï¿ô¿ø?ÍıÈ¡¿ô
void Motor_Ctrl(int32 Left_PWM, int32 Right_PWM); // ?´ù¹µÀ©È¡¿ô

//============================ Á´¶É?ÎÌÀ¼ÌÀ ============================//
extern int16 Encoder_Pules_Buffer[2]; // ??´ïæ÷ÑÕ?Â¸¿ô?
extern float Encoer_Speed[2];         // ??´ïÂ®ÅÙ¿ô?
extern int left_speed;                // º¸?Â®ÅÙ
extern int right_speed;               // ±¦?Â®ÅÙ
extern float break_dis;               // ÃÇÏ©µ÷Ğó

extern int32 Left_motorpwm,           // º¸?´ùPWM?½Ğ?
    Right_motorpwm;                   // ±¦?´ùPWM?½Ğ?
extern int32 Left_motorpwm_num, Right_motorpwm_num; // ?´ùPWM?¿ô?

//============================ ÃÇÏ©µ÷Ğó??¸ùÇ½ ============================//
void break_road_distance_init(void);   // ÃÇÏ©µ÷Ğó??½é»Ï²½È¡¿ô
void break_road_distance_update(void); // ÃÇÏ©µ÷Ğó¹¹¿·È¡¿ô
void start_break_road_recording(void); // ?»ÏÃÇÏ©µ÷Ğó??È¡¿ô
void stop_break_road_recording(void);  // Ää»ßÃÇÏ©µ÷Ğó??È¡¿ô
float get_break_road_distance(void);   // ?¼èÃÇÏ©µ÷ĞóÈ¡¿ô
uint8 is_break_road_recording(void);   // ÃÇÏ©??¾õ???È¡¿ô

#endif
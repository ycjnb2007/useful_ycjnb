#ifndef _MENU_H_
#define _MENU_H_

#include "zf_common_headfile.h"


typedef enum {
    
	PAGE_HOME,			
	PAGE_2,
	PAGE_3,
        PAGE_4
} page_flag;
extern page_flag page;

extern uint8 run_flag;               //开跑！


#define KEY_UP       1    // 上
#define KEY_DOWN     2    // 下
#define KEY_ADD      3    // 加
#define KEY_REDUCE   4    // 减

#define KEY_PAGE     1    // 页切换
#define KEY_Choice   2    // 选泽

extern uint8 Choice;
//////////////////////test
//extern float kp;
//extern float ki;
//extern float kd;
//extern uint8 save_flag;
//////////////////////test
extern uint8 buzzer_enable; // 蜂鸣器使能，1表示允许高电平，0表示禁止高电平
extern uint8 fan_enable;    // 负压风扇使能，1表示开启，0表示关闭

extern uint8_t record_mode;
extern uint8_t replay_mode;
extern uint16 MT9V03X_EXP_TIME_DEF;               // 曝光时间设置   摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
extern uint16 MT9V03X_FPS_DEF;


void menu( );
void display( );
uint16 get_knob_value( );
void encoder_knob_menu_control(void);
void menu_key_event(uint8_t key);
void menu_key_handler(uint8_t key);

#endif
#ifndef _Key_control_h
#define _key_control_h

extern uint8 start;             //电机是否启动
extern uint8 exchange;          //按键选项
extern float differential_k;    //差速系数
extern int normol_speed;      //编码器速度


void car_start();

#endif
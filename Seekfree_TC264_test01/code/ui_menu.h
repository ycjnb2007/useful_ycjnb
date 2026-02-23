/*
 * ui_menu.h
 *
 *  Created on: 2026年1月10日
 *      Author: ycj01
 */

#ifndef _UI_MENU_H_
#define _UI_MENU_H_

#include "zf_common_headfile.h"
extern float motor_kp;
extern float motor_ki;
extern float motor_kd;
extern int16_t target_speed;
extern int16_t error_val;
extern int16_t speed_left_enc;
extern int16_t speed_right_enc;
extern float   gyro_z_val;

void UI_Init(void);
void UI_Task(void); // 放在主循环中调用

#endif

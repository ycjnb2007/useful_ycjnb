// ==========================================
// 文件名: ui_menu.h
// 描述: 屏幕菜单系统头文件，包含外部控制变量声明
// ==========================================
#ifndef _UI_MENU_H_
#define _UI_MENU_H_

#include "zf_common_headfile.h"

// 屏幕尺寸约束 (强行设为横屏模式)
#define SCR_W 160
#define SCR_H 128

// --- 全局核心状态标志 ---
// 最重要的标志位。置 1 后，将关闭一切屏幕刷新操作，释放所有算力给控制。
extern uint8_t system_running;

// --- 传感器与控制变量声明 (供菜单显示或调节使用) ---
//extern float Yaw_plus;          // 陀螺仪积分角度或偏方差
extern int16_t error_val;       // 巡线误差
extern int16_t speed_left_enc;  // 左轮编码器速度
extern int16_t speed_right_enc; // 右轮编码器速度

// PID 与 速度参数声明
extern float motor_kp;
extern float motor_ki;
extern float motor_kd;
extern float servo_kp;
extern float servo_kd;

extern int16_t expect_speed_str; // 直道期望速度
extern int16_t expect_speed_cur; // 弯道期望速度

// 菜单系统初始化
void UI_Menu_Init(void);
// 菜单任务函数 (放在主循环内)
void UI_Menu_Task(void);

#endif // _UI_MENU_H_

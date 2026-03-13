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

// 菜单系统初始化
void UI_Menu_Init(void);
// 菜单任务函数 (放在主循环内)
void UI_Menu_Task(void);

#endif // _UI_MENU_H_

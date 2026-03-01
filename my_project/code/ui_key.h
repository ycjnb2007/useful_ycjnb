// ==========================================
// 文件名: ui_key.h
// 描述: 4按键驱动头文件，定义按键引脚与事件枚举
// ==========================================
#ifndef _UI_KEY_H_
#define _UI_KEY_H_

#include "zf_common_headfile.h"

// 1. 物理按键引脚定义 (根据你的硬件修改)
// 放弃5按键，只保留4个核心按键，足够完成所有交互
#define PIN_KEY_UP      P11_3  // 向上/加
#define PIN_KEY_DOWN    P13_1  // 向下/减
#define PIN_KEY_ENTER   P02_7  // 确认/进入/长按发车
#define PIN_KEY_BACK    P13_3  // 返回/取消/减(辅助)

// 2. 按键事件枚举
// 定义按键触发后的所有动作
typedef enum {
    KEY_NONE = 0,       // 无按键按下

    // 单击事件 (松手时触发)
    KEY_UP_CLICK,       // 上键单击
    KEY_DOWN_CLICK,     // 下键单击
    KEY_ENTER_CLICK,    // 确认键单击
    KEY_BACK_CLICK,     // 返回键单击

    // 长按事件 (按住到达时间阈值时立刻触发)
    KEY_ENTER_LONG,     // 确认键长按 (用于一键发车)

    // 连按事件 (按住不放时持续高频触发，用于快速调参)
    KEY_UP_REPEAT,      // 上键连按
    KEY_DOWN_REPEAT     // 下键连按

} Key_Event_t;

// 3. 外部函数声明
void UI_Key_Init(void);          // 按键引脚与状态初始化
Key_Event_t UI_Key_Scan(void);   // 按键扫描任务 (需放在主循环，非阻塞)

#endif // _UI_KEY_H_

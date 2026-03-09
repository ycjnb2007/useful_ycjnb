// ==========================================
// 文件名: ui_key.h
// 描述: 5按键驱动头文件，定义按键引脚与事件枚举
// ==========================================
#ifndef _UI_KEY_H_
#define _UI_KEY_H_

#include "zf_common_headfile.h"

// 1. 物理按键引脚定义 (根据你的硬件修改)
#define PIN_KEY_UP      P33_13  // 向上/加参数
#define PIN_KEY_DOWN    P33_9   // 向下/减参数
#define PIN_KEY_CAR     P33_12  // 短按确定 / 长按发车
#define PIN_KEY_BACK    P33_10  // 切换到上一页
#define PIN_KEY_NEXT    P33_11  // 切换到下一页 (原ENTER)

// 2. 按键事件枚举 (严格对应上层逻辑)
typedef enum {
    KEY_NONE = 0,       // 无按键按下

    // 单击事件 (松手时触发，保证菜单不乱跳)
    KEY_UP_CLICK,       // 上键单击
    KEY_DOWN_CLICK,     // 下键单击
    KEY_CAR_CLICK,      // 确定键单击 (进入/退出修改模式)
    KEY_BACK_CLICK,     // 上一页键单击
    KEY_NEXT_CLICK,     // 下一页键单击

    // 长按事件 (按住到达时间阈值时立刻触发)
    KEY_CAR_LONG,       // 发车键长按 (用于一键发车)

    // 连按事件 (按住不放时持续高频触发，用于快速调参)
    KEY_UP_REPEAT,      // 上键连按
    KEY_DOWN_REPEAT     // 下键连按
} Key_Event_t;

// 3. 外部函数声明
void UI_Key_Init(void);
Key_Event_t UI_Key_Scan(void);

#endif // _UI_KEY_H_

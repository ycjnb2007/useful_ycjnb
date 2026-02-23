#ifndef _UI_KEY_H_
#define _UI_KEY_H_

#include "zf_common_headfile.h"

// ==================== 1. 引脚配置 ====================
// 请务必确认这些引脚与你的原理图一致
#define PIN_KEY_UP      P11_3
#define PIN_KEY_DOWN    P13_1
#define PIN_KEY_LEFT    P13_3  // 减
#define PIN_KEY_RIGHT   P02_5  // 加
#define PIN_KEY_CENTER  P02_7  // 确认

// ==================== 2. 全局变量引用 ====================
extern float motor_kp;
extern float motor_ki;
extern float motor_kd;
extern int16_t target_speed;
extern int16_t error_val;
extern int16_t speed_left_enc;
extern int16_t speed_right_enc;
extern float   gyro_z_val;

// ==================== 3. 事件枚举 ====================
typedef enum {
    KEY_NONE = 0,

    // 单击
    KEY_UP_CLICK,
    KEY_DOWN_CLICK,
    KEY_LEFT_CLICK,
    KEY_RIGHT_CLICK,
    KEY_CENTER_CLICK,

    // 长按 (用于退出)
    KEY_DOWN_LONG_PRESS,

    // 连按 (用于调参)
    KEY_LEFT_REPEAT,
    KEY_RIGHT_REPEAT

} Key_Event_t;

void UI_Key_Init(void);
Key_Event_t UI_Key_Scan(void);

#endif

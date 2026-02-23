#include "ui_key.h"

// ==================== 手感微调区 ====================
// 这里的单位是“扫描次数”。如果你的 main 循环延时是 10ms：
#define KEY_DEBOUNCE_TIME   3    // 消抖 30ms (反应快且稳)
#define KEY_LONG_TIME       80   // 长按 0.8秒 (触发退出)
#define KEY_REPEAT_DELAY    30   // 连按延迟 0.3秒 (按住0.3秒后开始连跳)
#define KEY_REPEAT_RATE     5    // 连按间隔 50ms (数值跳动飞快)

// 按键索引
enum {K_UP=0, K_DOWN, K_LEFT, K_RIGHT, K_CENTER, KEY_NUM};
static const gpio_pin_enum key_pins[KEY_NUM] = {PIN_KEY_UP, PIN_KEY_DOWN, PIN_KEY_LEFT, PIN_KEY_RIGHT, PIN_KEY_CENTER};

typedef struct {
    uint8_t  state;      // 0:松开, 1:按下, 2:长按锁定
    uint16_t down_time;
} Key_Ctrl_t;

static Key_Ctrl_t keys[KEY_NUM];

void UI_Key_Init(void) {
    for(int i=0; i<KEY_NUM; i++) {
        // 务必开启上拉 (GPI_PULL_UP)，因为按键按下是接地(低电平)
        gpio_init(key_pins[i], GPI, 0, GPI_PULL_UP);
        keys[i].state = 0;
        keys[i].down_time = 0;
    }
}

Key_Event_t UI_Key_Scan(void) {
    Key_Event_t event = KEY_NONE;

    for(int i=0; i<KEY_NUM; i++) {
        // 低电平有效 (按下 == 0)
        if (gpio_get_level(key_pins[i]) == 0) {

            if (keys[i].state == 0) {
                // 初次按下，防抖
                keys[i].down_time++;
                if (keys[i].down_time >= KEY_DEBOUNCE_TIME) keys[i].state = 1;
            }
            else if (keys[i].state == 1) { // 持续按住
                keys[i].down_time++;

                // === 逻辑1: 长按 DOWN 键退出 ===
                if (i == K_DOWN) {
                    if (keys[i].down_time >= KEY_LONG_TIME) {
                        keys[i].state = 2; // 进入锁定状态，防止重复触发退出
                        return KEY_DOWN_LONG_PRESS;
                    }
                }

                // === 逻辑2: 左右键连按 (调参) ===
                else if (i == K_LEFT || i == K_RIGHT) {
                    if (keys[i].down_time >= KEY_REPEAT_DELAY) {
                        // 周期性触发连按
                        if ((keys[i].down_time - KEY_REPEAT_DELAY) % KEY_REPEAT_RATE == 0) {
                            // 【重点修复】这里保持 state=1，不要改成2，否则连按会断
                            if (i == K_LEFT)  return KEY_LEFT_REPEAT;
                            if (i == K_RIGHT) return KEY_RIGHT_REPEAT;
                        }
                    }
                }
            }
            // state==2 (长按锁定后) 死等松手，不产生事件
        }
        else { // 按键松开
            if (keys[i].state == 1) { // 如果没触发长按，松开这一下算单击
                switch(i) {
                    case K_UP:     event = KEY_UP_CLICK; break;
                    case K_DOWN:   event = KEY_DOWN_CLICK; break;
                    case K_LEFT:   event = KEY_LEFT_CLICK; break;
                    case K_RIGHT:  event = KEY_RIGHT_CLICK; break;
                    case K_CENTER: event = KEY_CENTER_CLICK; break;
                }
            }
            // 复位状态
            keys[i].state = 0;
            keys[i].down_time = 0;
        }
    }
    return event;
}

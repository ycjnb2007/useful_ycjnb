// ==========================================
// 文件名: ui_key.c
// 描述: 4按键消抖与状态机实现，支持单击、长按、连按
// ==========================================
#include "ui_key.h"

// ==================== 按键手感参数微调 ====================
// 假设主循环或定时器调用此扫描函数的周期为 10ms
#define DEBOUNCE_TICK   3     // 消抖时间 = 3次 * 10ms = 30ms
#define LONG_PRESS_TICK 80    // 长按阈值 = 80次 * 10ms = 0.8秒 (0.8秒后发车)
#define REPEAT_DELAY    40    // 连按延迟 = 40次 * 10ms = 0.4秒 (按住0.4秒后开始连调)
#define REPEAT_RATE     5     // 连按速率 = 5次 * 10ms = 50ms (每50ms数值跳动一次)

// 内部按键ID枚举，方便数组遍历
enum {K_UP = 0, K_DOWN, K_ENTER, K_BACK, KEY_NUM};
// 绑定引脚
static const gpio_pin_enum key_pins[KEY_NUM] = {PIN_KEY_UP, PIN_KEY_DOWN, PIN_KEY_ENTER, PIN_KEY_BACK};

// 按键控制块结构体 (用于状态机记录每个按键的状态)
typedef struct {
    uint8_t  state;      // 0=松开状态, 1=按下消抖状态, 2=长按/连按锁定状态
    uint16_t time_cnt;   // 按下持续时间计数器
} Key_Ctrl_t;

static Key_Ctrl_t keys[KEY_NUM];

// 初始化所有按键引脚
void UI_Key_Init(void) {
    for(int i = 0; i < KEY_NUM; i++) {
        // 调用逐飞库的GPIO初始化。
        // 参数: 引脚名, 输入模式(GPI), 默认电平0, 开启内部上拉(GPI_PULL_UP)
        // 开启上拉是因为通常按键按下是接地(即按下读到低电平0)
        gpio_init(key_pins[i], GPI, 0, GPI_PULL_UP);
        keys[i].state = 0;
        keys[i].time_cnt = 0;
    }
}

// 按键核心扫描机 (每次循环调用一次，绝不使用死循环延时)
Key_Event_t UI_Key_Scan(void) {
    Key_Event_t event = KEY_NONE; // 默认无事件

    for(int i = 0; i < KEY_NUM; i++) {
        // 读取引脚电平，0表示按下 (因为配置了上拉)
        if (gpio_get_level(key_pins[i]) == 0) {

            // 状态0: 刚检测到按下
            if (keys[i].state == 0) {
                keys[i].time_cnt++;
                // 达到消抖阈值，确认是真的按下了，跳转到状态1
                if (keys[i].time_cnt >= DEBOUNCE_TICK) keys[i].state = 1;
            }
            // 状态1: 持续按下中，处理长按和连按逻辑
            else if (keys[i].state == 1) {
                keys[i].time_cnt++;

                // 逻辑A: 处理 ENTER 键的长按发车
                if (i == K_ENTER) {
                    if (keys[i].time_cnt >= LONG_PRESS_TICK) {
                        keys[i].state = 2; // 进入锁定态，松手前不再触发其他事件
                        return KEY_ENTER_LONG; // 向上层抛出长按发车指令
                    }
                }
                // 逻辑B: 处理 UP/DOWN 键的连按调参
                else if (i == K_UP || i == K_DOWN) {
                    if (keys[i].time_cnt >= REPEAT_DELAY) {
                        // 利用取余运算，实现每隔 REPEAT_RATE 触发一次连按
                        if ((keys[i].time_cnt - REPEAT_DELAY) % REPEAT_RATE == 0) {
                            if (i == K_UP)   return KEY_UP_REPEAT;
                            if (i == K_DOWN) return KEY_DOWN_REPEAT;
                        }
                    }
                }
            }
            // 状态2(长按锁定): 什么都不做，死等用户松手
        }
        else { // 引脚高电平，按键已松开
            // 如果是在状态1(即没有触发长按就松手了)，说明是一次有效单击
            if (keys[i].state == 1) {
                switch(i) {
                    case K_UP:    event = KEY_UP_CLICK;    break;
                    case K_DOWN:  event = KEY_DOWN_CLICK;  break;
                    case K_ENTER: event = KEY_ENTER_CLICK; break;
                    case K_BACK:  event = KEY_BACK_CLICK;  break;
                }
            }
            // 松手后，所有状态全部清零复位
            keys[i].state = 0;
            keys[i].time_cnt = 0;
        }
    }

    // 如果一次扫描中有多个按键事件，先只返回第一个(由于上层循环很快，下一个事件会在下一次扫描抛出)
    return event;
}

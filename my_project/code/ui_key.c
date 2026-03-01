// ==========================================
// 文件名: ui_key.c
// 描述: 5按键消抖与状态机实现，非阻塞架构
// ==========================================
#include "ui_key.h"

// ==================== 按键手感参数微调 ====================
// 假设主循环或 PIT定时器 调用此函数的周期为 10ms
#define DEBOUNCE_TICK   3      // 消抖时间 = 3次 * 10ms = 30ms (过滤机械抖动)
#define LONG_PRESS_TICK 150    // 长按发车阈值 = 150次 * 10ms = 1.5秒 (防误触)
#define REPEAT_DELAY    40     // 连按延迟 = 40次 * 10ms = 0.4秒 (按住0.4秒后开始疯狂跳数)
#define REPEAT_RATE     5      // 连按速率 = 5次 * 10ms = 50ms (每50ms触发一次加减)

// 内部按键ID枚举，将5个引脚映射为 0-4 的索引，方便 for 循环遍历
enum {K_UP = 0, K_DOWN, K_CAR, K_BACK, K_NEXT, KEY_NUM};

// 将宏定义的引脚绑定到数组中
static const gpio_pin_enum key_pins[KEY_NUM] = {
    PIN_KEY_UP, PIN_KEY_DOWN, PIN_KEY_CAR, PIN_KEY_BACK, PIN_KEY_NEXT
};

// 按键控制块结构体
typedef struct {
    uint8_t  state;      // 状态机: 0=松开, 1=按下消抖中, 2=触发了长按/连按(死等松手)
    uint16_t time_cnt;   // 按下持续时间计数器
} Key_Ctrl_t;

static Key_Ctrl_t keys[KEY_NUM];

// 初始化所有按键引脚
void UI_Key_Init(void) {
    for(int i = 0; i < KEY_NUM; i++) {
        // 逐飞库 API: 初始化为通用输入(GPI)，默认低电平0，内部上拉(GPI_PULL_UP)
        // 开启上拉后，按键未按下时读到高电平(1)，按下接地时读到低电平(0)
        gpio_init(key_pins[i], GPI, 0, GPI_PULL_UP);
        keys[i].state = 0;
        keys[i].time_cnt = 0;
    }
}

// 按键核心扫描机 (必须放在主循环或定时器中断中高频调用，绝对不能有 delay)
Key_Event_t UI_Key_Scan(void) {
    Key_Event_t event = KEY_NONE; // 默认返回无事件

    for(int i = 0; i < KEY_NUM; i++) {
        // 读取引脚电平，== 0 表示物理按键被按下了
        if (gpio_get_level(key_pins[i]) == 0) {

            // 状态0: 之前是松开的，现在刚检测到按下
            if (keys[i].state == 0) {
                keys[i].time_cnt++;
                // 计数值达到消抖阈值，确认不是干扰，进入状态1
                if (keys[i].time_cnt >= DEBOUNCE_TICK) keys[i].state = 1;
            }
            // 状态1: 已经被确认按下，正在持续按住
            else if (keys[i].state == 1) {
                keys[i].time_cnt++;

                // 逻辑A: 处理 CAR 键的 长按发车
                if (i == K_CAR) {
                    if (keys[i].time_cnt >= LONG_PRESS_TICK) {
                        keys[i].state = 2; // 进入状态2(锁定)，必须松手才能重置
                        return KEY_CAR_LONG; // 立刻向上层抛出长按事件
                    }
                }
                // 逻辑B: 处理 UP/DOWN 键的 连按调参
                else if (i == K_UP || i == K_DOWN) {
                    // 如果按住的时间超过了连按的起步延迟...
                    if (keys[i].time_cnt >= REPEAT_DELAY) {
                        // 利用取余运算，时间每走过 REPEAT_RATE，就抛出一次连击事件
                        if ((keys[i].time_cnt - REPEAT_DELAY) % REPEAT_RATE == 0) {
                            if (i == K_UP)   return KEY_UP_REPEAT;
                            if (i == K_DOWN) return KEY_DOWN_REPEAT;
                        }
                    }
                }
            }
            // 状态2: 锁定态。按键还没松开，什么都不做，防止长按后又触发单击
        }
        else {
            // 引脚高电平，说明按键被【松开】了

            // 如果是在状态1松手的(没触发长按就松手了)，说明这是一次完美的【单击】
            if (keys[i].state == 1) {
                switch(i) {
                    case K_UP:    event = KEY_UP_CLICK;    break;
                    case K_DOWN:  event = KEY_DOWN_CLICK;  break;
                    case K_CAR:   event = KEY_CAR_CLICK;   break;
                    case K_BACK:  event = KEY_BACK_CLICK;  break;
                    case K_NEXT:  event = KEY_NEXT_CLICK;  break;
                }
            }
            // 无论刚才是什么状态，只要松手，全部清零复位，为下一次按下做准备
            keys[i].state = 0;
            keys[i].time_cnt = 0;
        }
    }

    // 每次扫描只抛出一个有效事件，优先处理阵列前面的按键
    return event;
}

#include "ui_menu.h"
#include <stdio.h>

uint8_t system_running = 0; // 默认开机停车

typedef enum {
    PAGE_MAIN = 0, // 图像监视页
    PAGE_PARAM,    // 调参页
    PAGE_MY_A,     // 备用A
    PAGE_MY_B,     // 备用B
    PAGE_MAX
} Page_Enum;

static Page_Enum curr_page = PAGE_MAIN;

// 菜单滚动与光标控制
static uint8_t cursor = 0;       // 当前光标指向的参数索引
static uint8_t show_start = 0;   // 屏幕顶端显示的参数索引(用于实现滑动列表)
#define MAX_SHOW 5               // 屏幕最多同时显示 5 行

static uint8_t is_editing = 0;   // 状态机：0=浏览光标模式, 1=修改参数模式

// 参数项结构体绑定
typedef struct {
    char *name;
    void *ptr_val;
    float step;
    uint8_t is_int;
} Param_Item_t;

// 将你的控制变量绑定到菜单字典中
static Param_Item_t params[] = {
    {"Spd_L", &speed_straight_l, 5.0f,  1},
    {"Spd_S", &speed_straight_s, 5.0f,  1},
    {"Spd_C", &speed_curve,      5.0f,  1},
    {"TrnKp", &turn_kp_base,     0.1f,  0},
    {"TrnKv", &turn_kp_var,      0.01f, 0},
    {"TrnKd", &turn_kd,          0.1f,  0},
    {"GyoKp", &gyro_kp,          0.1f,  0},
    {"GyoKd", &gyro_kd,          0.05f, 0},
    {"SpdKp", &speed_kp,         1.0f,  0},
    {"SpdKi", &speed_ki,         0.1f,  0}
};
#define PARAM_CNT (sizeof(params)/sizeof(params[0]))

// --- 页面切换安全机制 ---
static void UI_Switch_Page(Page_Enum new_page) {
    if(curr_page != new_page) {
        curr_page = new_page;
        is_editing = 0; // 切页强制退出修改模式
        tft180_clear(); // 仅切页时清屏，防止全局刷屏导致闪烁严重！
    }
}

// --- 通用数据叠加显示 ---
void Draw_Bottom_Dashboard(void) {
    char buf[32];
    int16_t y_start = 94; // 避开上层图像或菜单

    sprintf(buf, "E:%-5.1f G:%-5.1f   ", ctrl_state.camera_error, ctrl_state.angular_rate_current);
    tft180_show_string(0, y_start, buf);

    y_start += 16;
    sprintf(buf, "L:%-4.0f R:%-4.0f   ", ctrl_state.current_left_speed, ctrl_state.current_right_speed);
    tft180_show_string(0, y_start, buf);
}

// --- UI 绘图层 (保留你原本的绘图逻辑，精简代码) ---
static void UI_Draw_Page_Image(void) { /* 在这里写你之前的tft180_show_gray_image*/ Draw_Bottom_Dashboard(); }
static void UI_Draw_Page_Param(void) { /* 同你原来的代码，遍历 params 画出参数 */ Draw_Bottom_Dashboard(); }
static void UI_Draw_Page_My_A(void)  { /* ... */ }
static void UI_Draw_Page_My_B(void)  { /* ... */ }

// --- 参数加减执行器 ---
static void UI_Adjust_Param(int8_t direction) {
    Param_Item_t *p = &params[cursor];
    if(p->is_int) {
        int16_t *val = (int16_t*)p->ptr_val; // 注意这里强转必须与你变量的实际类型一致！
        *val += (int16_t)(direction * p->step);
    } else {
        float *val = (float*)p->ptr_val;
        *val += (direction * p->step);
    }
}

void UI_Menu_Init(void) {
    UI_Key_Init();
    tft180_clear();
}

// ==========================================
// 菜单主循环任务 (最核心的状态机分配逻辑)
// ==========================================
void UI_Menu_Task(void) {
    Key_Event_t key = UI_Key_Scan(); // 获取底层按键事件

    // 【全局最高优先级】：不管什么界面，只要长按CAR，直接发车！
    if (key == KEY_CAR_LONG) {
        system_running = 1; // 置标志位，主函数通过判断这个标志位来启动控制代码
        tft180_clear();
        tft180_show_string(40, 60, "CAR RUNNING!");
        return;
    }

    // 如果发车了，菜单不再响应任何操作，节省算力
    if (system_running) return;

    // ---【分页导航逻辑】(全局有效)---
    // 只有在非编辑模式下，才允许左右切页
    if (!is_editing) {
        if (key == KEY_BACK_CLICK) {
            Page_Enum prev = (curr_page == PAGE_MAIN) ? (PAGE_MAX - 1) : (curr_page - 1);
            UI_Switch_Page(prev);
        }
        else if (key == KEY_NEXT_CLICK) {
            Page_Enum next = (curr_page + 1) % PAGE_MAX;
            UI_Switch_Page(next);
        }
    }

    // ---【页面内部交互逻辑】---
    if (curr_page == PAGE_PARAM) {
        if (!is_editing) {
            // 【浏览模式】: 上下键用于移动光标，CAR短按用于进入编辑
            if (key == KEY_UP_CLICK || key == KEY_UP_REPEAT) {
                if (cursor > 0) cursor--;
                else cursor = PARAM_CNT - 1; // 到底循环
            }
            if (key == KEY_DOWN_CLICK || key == KEY_DOWN_REPEAT) {
                if (cursor < PARAM_CNT - 1) cursor++;
                else cursor = 0; // 到顶循环
            }
            // 视口自动跟随逻辑 (非常精妙，不用改)
            if (cursor < show_start) show_start = cursor;
            if (cursor >= show_start + MAX_SHOW) show_start = cursor - MAX_SHOW + 1;

            if (key == KEY_CAR_CLICK) is_editing = 1; // 按下确定，进入修改状态
        }
        else {
            // 【编辑模式】: 上下键用于加减参数，CAR短按用于保存退出
            if(key == KEY_UP_CLICK   || key == KEY_UP_REPEAT)   UI_Adjust_Param(1);  // 加
            if(key == KEY_DOWN_CLICK || key == KEY_DOWN_REPEAT) UI_Adjust_Param(-1); // 减
            if(key == KEY_CAR_CLICK) is_editing = 0; // 修改完毕，按下确定退回光标浏览
        }
    }

    // --- 页面渲染刷新调度 ---
    switch(curr_page) {
        case PAGE_MAIN:   UI_Draw_Page_Image(); break;
        case PAGE_PARAM:  UI_Draw_Page_Param(); break;
        case PAGE_MY_A:   UI_Draw_Page_My_A();  break;
        case PAGE_MY_B:   UI_Draw_Page_My_B();  break;
        default: break;
    }
}

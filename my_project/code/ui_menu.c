#include "ui_menu.h"
#include <stdio.h>

#define SCR_W 160
#define SCR_H 128

typedef enum {
    PAGE_MAIN = 0,
    PAGE_PARAM,
    PAGE_MY_A,
    PAGE_MY_B,
    PAGE_MAX
} Page_Enum;

static Page_Enum curr_page = PAGE_MAIN;

// 菜单滚动与光标控制
static uint8_t cursor = 0;       // 当前选中项索引
static uint8_t show_start = 0;   // 屏幕上显示的第一项索引
#define MAX_SHOW 5               // 屏幕最多同时显示 5 行参数，避免和仪表盘重叠

static uint8_t is_editing = 0;   // 0:浏览翻页模式, 1:修改参数模式

typedef struct {
    char *name;
    void *ptr_val;
    float step;
    uint8_t is_int;
} Param_Item_t;

// 【核心】全面绑定你的智能车实际调参变量！
static Param_Item_t params[] = {
    {"Spd_L", &speed_straight_l, 5.0f,  1}, // 长直道基准速度
    {"Spd_S", &speed_straight_s, 5.0f,  1}, // 短直道基准速度
    {"Spd_C", &speed_curve,      5.0f,  1}, // 弯道基准速度
    {"TrnKp", &turn_kp_base,     0.1f,  0}, // 图像外环基础比例
    {"TrnKv", &turn_kp_var,      0.01f, 0}, // 图像外环变增益系数
    {"TrnKd", &turn_kd,          0.1f,  0}, // 图像外环微分系数
    {"GyoKp", &gyro_kp,          0.1f,  0}, // 陀螺仪差速中环Kp
    {"GyoKd", &gyro_kd,          0.05f, 0}, // 陀螺仪差速中环Kd
    {"SpdKp", &speed_kp,         1.0f,  0}, // 内环速度Kp
    {"SpdKi", &speed_ki,         0.1f,  0}  // 内环速度Ki
};
#define PARAM_CNT (sizeof(params)/sizeof(params[0]))

// 页面切换安全清屏
static void UI_Switch_Page(Page_Enum new_page) {
    if(curr_page != new_page) {
        curr_page = new_page;
        is_editing = 0;
        tft180_clear(); // 仅切页时清屏，不再闪烁！
    }
}

void Safe_Draw_Point(int16_t x, int16_t y, uint16_t color) {
    if(x >= 0 && x < SCR_W && y >= 0 && y < SCR_H) {
        tft180_draw_point(x, y, color);
    }
}

// 通用底部仪表盘
void Draw_Bottom_Dashboard(void) {
    char buf[32];
    int16_t y_start = 94;

    // 显示真实图像偏差与陀螺仪真实角速度
    sprintf(buf, "E:%-5.1f G:%-5.1f   ", ctrl_state.camera_error, ctrl_state.angular_rate_current);
    tft180_show_string(0, y_start, buf);

    y_start += 16;
    // 显示左右轮当前速度，监控有没有闭环上
    sprintf(buf, "L:%-4.0f R:%-4.0f   ", ctrl_state.current_left_speed, ctrl_state.current_right_speed);
    tft180_show_string(0, y_start, buf);
}

// 页面1：跑车图像页面
static void UI_Draw_Page_Image(void) {
    int16_t offset_x = (SCR_W - XM) / 2;
    int16_t offset_y = 0;

    for(int y = 0; y < YM; y++) {
        for(int x = 0; x < XM; x++) {
            if(imgOSTU[y][x] == 255) Safe_Draw_Point(offset_x + x, offset_y + y, RGB565_WHITE);
        }
    }
    for(int i=0; i<YM; i++) {
        if(mid_line[i] < XM) Safe_Draw_Point(offset_x + mid_line[i], offset_y + i, RGB565_RED);
    }
    Draw_Bottom_Dashboard();
}

// 页面2：支持滚动的 PID 调参页面
static void UI_Draw_Page_Param(void) {
    tft180_show_string(30, 0, "== SETUP ==");
    char buff[32];

    for(int i = 0; i < MAX_SHOW; i++) {
        int idx = show_start + i;
        if(idx >= PARAM_CNT) break; // 防止越界

        uint16_t y_pos = 18 + i * 15;

        // 绘制交互光标
        if(idx == cursor) {
            if(is_editing) tft180_show_string(0, y_pos, "*"); // 正在修改
            else           tft180_show_string(0, y_pos, ">"); // 正在浏览
        } else {
            tft180_show_string(0, y_pos, " ");
        }

        // 格式化数据防重叠
        if(params[idx].is_int) {
            int16_t *val = (int16_t*)params[idx].ptr_val;
            sprintf(buff, "%-5s:%-6d   ", params[idx].name, *val);
        } else {
            float *val = (float*)params[idx].ptr_val;
            sprintf(buff, "%-5s:%-6.2f   ", params[idx].name, *val);
        }
        tft180_show_string(16, y_pos, buff);
    }
    Draw_Bottom_Dashboard();
}

// 页面3、4：你的备用页面
static void UI_Draw_Page_My_A(void) {
    tft180_show_string(40, 40, "== my_a ==");
    Draw_Bottom_Dashboard();
}
static void UI_Draw_Page_My_B(void) {
    tft180_show_string(40, 40, "== my_b ==");
    Draw_Bottom_Dashboard();
}

static void UI_Adjust_Param(int8_t direction) {
    Param_Item_t *p = &params[cursor];
    if(p->is_int) {
        int16_t *val = (int16_t*)p->ptr_val;
        *val += (int16_t)(direction * p->step);
    } else {
        float *val = (float*)p->ptr_val;
        *val += (direction * p->step);
    }
}

void UI_Init(void) {
    UI_Key_Init();
    tft180_clear();
}

void UI_Task(void) {
    Key_Event_t key = UI_Key_Scan();

    // 1. 全局急救按钮：无论在哪，长按向下键立刻退回主图像页面！
    if(key == KEY_DOWN_LONG_PRESS) {
        UI_Switch_Page(PAGE_MAIN);
        return;
    }

    if (!is_editing) {
        // ---【浏览翻页模式】---
        // 任何页面下：左右键纯负责切页
        if (key == KEY_LEFT_CLICK || key == KEY_LEFT_REPEAT) {
            Page_Enum prev = (curr_page == PAGE_MAIN) ? (PAGE_MAX - 1) : (curr_page - 1);
            UI_Switch_Page(prev);
        }
        else if (key == KEY_RIGHT_CLICK || key == KEY_RIGHT_REPEAT) {
            Page_Enum next = (curr_page + 1) % PAGE_MAX;
            UI_Switch_Page(next);
        }
        // 如果在参数页：上下移动光标，确定进入修改
        else if (curr_page == PAGE_PARAM) {
            if (key == KEY_UP_CLICK) {
                if (cursor > 0) cursor--;
                else cursor = PARAM_CNT - 1;
            }
            if (key == KEY_DOWN_CLICK) {
                if (cursor < PARAM_CNT - 1) cursor++;
                else cursor = 0;
            }

            // --- 计算菜单视口滚动逻辑 ---
            if (cursor < show_start) show_start = cursor;
            if (cursor >= show_start + MAX_SHOW) show_start = cursor - MAX_SHOW + 1;

            if (key == KEY_CENTER_CLICK) is_editing = 1;
        }
    }
    else {
        // ---【编辑调参模式】(仅参数页有效)---
        if (curr_page == PAGE_PARAM) {
            // 此时左右键不负责翻页，专职加减数值，支持长按连击起飞
            if(key == KEY_RIGHT_CLICK || key == KEY_RIGHT_REPEAT) UI_Adjust_Param(1);
            if(key == KEY_LEFT_CLICK  || key == KEY_LEFT_REPEAT)  UI_Adjust_Param(-1);

            // 改好了再按一次中心键确认退回浏览
            if(key == KEY_CENTER_CLICK) is_editing = 0;
        }
    }

    // 刷新页面
    switch(curr_page) {
        case PAGE_MAIN:   UI_Draw_Page_Image(); break;
        case PAGE_PARAM:  UI_Draw_Page_Param(); break;
        case PAGE_MY_A:   UI_Draw_Page_My_A();  break;
        case PAGE_MY_B:   UI_Draw_Page_My_B();  break;
        default: break;
    }
}

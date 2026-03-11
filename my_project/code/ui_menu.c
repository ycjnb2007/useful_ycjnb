// ==========================================
// 文件名: ui_menu.c
// ==========================================
#include "ui_menu.h"
#include "motor.h"
#include "imu660.h"
#include <stdio.h>
#include "pid.h"
#include "image_deal_best.h"
#include "run.h"

#define SCR_W 160
#define SCR_H 128

uint8_t system_running = 0;

// --- 页面枚举 ---
typedef enum {
    PAGE_IMAGE = 0,
    PAGE_SPEED,
    PAGE_TURN_PID,
    PAGE_MOTOR_PID,
    PAGE_IMG_THRES,    // <--- 新增阈值页面
    PAGE_MY_A,         // <--- 新增的备用页 A
    PAGE_MAX
} Page_Enum;

static Page_Enum curr_page = PAGE_IMAGE;

static uint8_t cursor = 0;
static uint8_t show_start = 0;
#define MAX_SHOW 5

static uint8_t is_editing = 0;

typedef struct {
    char *name;
    void *ptr_val;
    float step;
    uint8_t is_int;
} Param_Item_t;

static Param_Item_t speed_params[] = {
    {"Spd_L", &speed_straight_l, 10.0f, 1},
    {"Spd_S", &speed_straight_s, 10.0f, 1},
    {"Spd_C", &speed_curve,      10.0f, 1},
};

static Param_Item_t turn_params[] = {
    {"TrnKp", &turn_kp_base, 0.1f,  0},
    {"TrnKv", &turn_kp_var,  0.01f, 0},
    {"TrnKd", &turn_kd,      0.1f,  0},
    {"GyoKp", &gyro_kp,      0.1f,  0},
    {"GyoKd", &gyro_kd,      0.05f, 0},
    {"BldKp", &blind_turn_kp, 0.1f, 0}, 
    {"BldKd", &blind_turn_kd, 0.05f,0}
};

static Param_Item_t motor_params[] = {
    {"MotKp", &speed_kp, 1.0f, 0},
    {"MotKi", &speed_ki, 0.1f, 0},
    {"MotKff",&speed_kff, 0.5f, 0} 
};

static Param_Item_t img_params[] = {
    {"ClsTh", &close_Threshold, 1.0f, 2},
    {"MidTh", &mid_Threshold,   1.0f, 2},
    {"FarTh", &far_Threshold,   1.0f, 2}
};

static Param_Item_t *curr_params = NULL;
static uint8_t curr_param_cnt = 0;

// ==================== 底部仪表盘 (严格限制在 20 字符以内) ====================
void Draw_Bottom_Dashboard(void) {
    char buf[32];
    int16_t y_start = 96;

    sprintf(buf, "ST:%d ND:%d       ", cur_state, node_index);
    buf[19] = '\0';
    tft180_show_string(0, y_start, buf);

    y_start += 16;
    sprintf(buf, "T:%.0f R:%.0f       ", ctrl_state.angular_rate_target, ctrl_state.angular_rate_current);
    buf[19] = '\0';
    tft180_show_string(0, y_start, buf);
}

static inline void Safe_Draw_Point(int16_t x, int16_t y, uint16_t color) {
    if (x >= 0 && x < SCR_W && y >= 0 && y < SCR_H) {
        tft180_draw_point(x, y, color);
    }
}

static void UI_Draw_Page_Image(void) {
    int16_t offset_x = (SCR_W - XM) / 2;
    int16_t offset_y = 0;

    tft180_show_gray_image(offset_x, offset_y, (const uint8_t *)imgGray[0], IMG_W, IMG_H, XM, YM, nowThreshold);

    for (int i = 0; i < YM; i++) {
        if (mid_line[i] < XM) Safe_Draw_Point(offset_x + mid_line[i], offset_y + i, RGB565_RED);
    }
    for (int i = 0; i < l_data_statics; i++) {
        if (points_l[i][0] < XM && points_l[i][1] < YM) Safe_Draw_Point(offset_x + points_l[i][0], offset_y + points_l[i][1], RGB565_GREEN);
    }
    for (int i = 0; i < r_data_statics; i++) {
        if (points_r[i][0] < XM && points_r[i][1] < YM) Safe_Draw_Point(offset_x + points_r[i][0], offset_y + points_r[i][1], RGB565_BLUE);
    }
    Draw_Bottom_Dashboard();
}

static void UI_Draw_Page_Param(char *page_title) {
    char buf[32];
    tft180_show_string(0, 0, page_title);
    if(curr_param_cnt == 0 || curr_params == NULL) return;

    for (uint8_t i = 0; i < MAX_SHOW; i++) {
        uint8_t p_idx = show_start + i;
        if (p_idx >= curr_param_cnt) break;

        Param_Item_t *p = &curr_params[p_idx];
        int16_t y_pos = 16 + i * 16;

        char val_str[16];
        if (p->is_int == 2) sprintf(val_str, "%d", *(uint8*)p->ptr_val);
        else if (p->is_int == 1) sprintf(val_str, "%d", *(int16_t*)p->ptr_val);
        else           sprintf(val_str, "%.2f", *(float*)p->ptr_val);

        // 缩短光标前后填充，严防超长
        if (p_idx == cursor) {
            if (is_editing) sprintf(buf, ">[%s:%s]<       ", p->name, val_str);
            else            sprintf(buf, "->%s:%s       ", p->name, val_str);
        } else {
            sprintf(buf, "  %s:%s       ", p->name, val_str);
        }
        buf[19] = '\0';
        tft180_show_string(0, y_pos, buf);
    }
    Draw_Bottom_Dashboard();
}

// ==================== 新增的 MY_A 空白页 ====================
static void UI_Draw_Page_My_A(void) {
    char buf[32];
    tft180_show_string(0, 0, "=== SENSOR DEBUG ===");
    sprintf(buf, "SpdL:%d R:%d       ", (int)Actual_Speed[0], (int)Actual_Speed[1]);
    buf[19] = '\0';
    tft180_show_string(0, 20, buf);
    sprintf(buf, "PwmL:%d R:%d       ", ctrl_state.output_left_pwm, ctrl_state.output_right_pwm);
    buf[19] = '\0';
    tft180_show_string(0, 40, buf);
    sprintf(buf, "GyoZ:%.0f        ", gyro_param.gyro_z);
    buf[19] = '\0';
    tft180_show_string(0, 60, buf);
    sprintf(buf, "Yaw:%.1f          ", yaw_plus);
    buf[19] = '\0';
    tft180_show_string(0, 80, buf);
    Draw_Bottom_Dashboard();
}

static void UI_Switch_Page(Page_Enum new_page) {
    if(curr_page != new_page) {
        curr_page = new_page;
        is_editing = 0;
        cursor = 0;
        show_start = 0;
        tft180_clear();

        switch (curr_page) {
            case PAGE_SPEED:
                curr_params = speed_params;
                curr_param_cnt = sizeof(speed_params)/sizeof(speed_params[0]);
                break;
            case PAGE_TURN_PID:
                curr_params = turn_params;
                curr_param_cnt = sizeof(turn_params)/sizeof(turn_params[0]);
                break;
            case PAGE_MOTOR_PID:
                curr_params = motor_params;
                curr_param_cnt = sizeof(motor_params)/sizeof(motor_params[0]);
                break;
            case PAGE_IMG_THRES:
                curr_params = img_params;
                curr_param_cnt = sizeof(img_params)/sizeof(img_params[0]);
                break;
            default:
                curr_params = NULL;
                curr_param_cnt = 0;
                break;
        }
    }
}

static void UI_Adjust_Param(int8_t direction) {
    if(curr_params == NULL) return;
    Param_Item_t *p = &curr_params[cursor];
    if(p->is_int == 2) {
        uint8 *val = (uint8*)p->ptr_val;
        int16_t tmp = *val + (int16_t)(direction * p->step);
        if (tmp < 0) tmp = 0;
        if (tmp > 255) tmp = 255;
        *val = (uint8)tmp;
    } else if(p->is_int == 1) {
        int16_t *val = (int16_t*)p->ptr_val;
        *val += (int16_t)(direction * p->step);
    } else {
        float *val = (float*)p->ptr_val;
        *val += (direction * p->step);
    }
}

void UI_Menu_Init(void) {
    UI_Key_Init();
    tft180_clear();
    curr_page = PAGE_MAX;
    UI_Switch_Page(PAGE_IMAGE);
}

void UI_Menu_Task(void) {
    Key_Event_t key = UI_Key_Scan();

    if (key == KEY_CAR_LONG) {
        system_running = 1;
        tft180_clear();
        tft180_show_string(30, 60, "CAR RUNNING!");
        return;
    }
    if (system_running) return;

    if (!is_editing) {
        if (key == KEY_BACK_CLICK) {
            Page_Enum prev = (curr_page == PAGE_IMAGE) ? (PAGE_MAX - 1) : (curr_page - 1);
            UI_Switch_Page(prev);
        }
        else if (key == KEY_NEXT_CLICK) {
            Page_Enum next = (curr_page + 1) % PAGE_MAX;
            UI_Switch_Page(next);
        }
    }

    if (curr_page != PAGE_IMAGE && curr_page != PAGE_MY_A && curr_param_cnt > 0) {
        if (!is_editing) {
            if (key == KEY_UP_CLICK || key == KEY_UP_REPEAT) {
                if (cursor > 0) cursor--;
                else cursor = curr_param_cnt - 1;
            }
            if (key == KEY_DOWN_CLICK || key == KEY_DOWN_REPEAT) {
                if (cursor < curr_param_cnt - 1) cursor++;
                else cursor = 0;
            }
            if (cursor < show_start) show_start = cursor;
            if (cursor >= show_start + MAX_SHOW) show_start = cursor - MAX_SHOW + 1;

            if (key == KEY_CAR_CLICK) is_editing = 1;
        }
        else {
            if(key == KEY_UP_CLICK   || key == KEY_UP_REPEAT)   UI_Adjust_Param(1);
            if(key == KEY_DOWN_CLICK || key == KEY_DOWN_REPEAT) UI_Adjust_Param(-1);
            if(key == KEY_CAR_CLICK) is_editing = 0;
        }
    }

    // --- 【关键修复：防闪烁渲染器降频】 ---
    // 假设此函数在主循环或 10ms 的定时器中被极高频调用。
    // 我们强制只让它每调用 4 次（或更多次）才去刷一次屏幕，保护 SPI 通信带宽。
    static uint8_t render_delay = 0;
    render_delay++;
    if (render_delay < 4) return; // 小于4时直接退出，不画图
    render_delay = 0;             // 等于4时清零，并放行下面的画图逻辑

    // 渲染总路由
    switch(curr_page) {
        case PAGE_IMAGE:     UI_Draw_Page_Image(); break;
        case PAGE_SPEED:     UI_Draw_Page_Param("=== SPEED CFG ==="); break;
        case PAGE_TURN_PID:  UI_Draw_Page_Param("=== TURN PID ==="); break;
        case PAGE_MOTOR_PID: UI_Draw_Page_Param("=== MOTOR PID ==="); break;
        case PAGE_IMG_THRES: UI_Draw_Page_Param("=== IMG THRESH ==="); break;
        case PAGE_MY_A:      UI_Draw_Page_My_A(); break; // <--- 渲染新增的 MY_A
        default: break;
    }
}

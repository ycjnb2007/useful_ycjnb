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

uint8_t system_running = 0;

// 状态机字符串映射表 (最多2个字符，保持对齐)
static const char *State_Str[] = {
    "NM", // NORMAL (正常巡线)
    "CN", // CHECK_NODE (动态查框)
    "FI", // FALSE_IGNORE (假干扰强拉)
    "SO", // SMOOTH_OFFSET (平滑偏置)
    "BT", // BLIND_TURN_YAW (盲转)
    "CC", // CAPACITY_CHECK (容量检测)
    "WN", // WAIT_NODE (等节点沉下)
    "LC", // L_CORNER (左直角)
    "RC", // R_CORNER (右直角)
    "BL"  // BLEND (过渡)
};

// --- 页面枚举 ---
typedef enum {
    PAGE_IMAGE = 0,
    PAGE_SPEED,
    PAGE_TURN_PID,
    PAGE_MOTOR_PID,
    PAGE_MY_A,         // <--- 新增的备用页 A
    PAGE_MAX
} Page_Enum;

static Page_Enum curr_page = PAGE_IMAGE;

static uint8_t cursor = 0;
static uint8_t show_start = 0;

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

static Param_Item_t *curr_params = NULL;
static uint8_t curr_param_cnt = 0;

// ==================== 底部仪表盘 (严格限制在 20 字符以内) ====================
void Draw_Bottom_Dashboard(void) {
    char buf[32];
    int16_t y_start = 220;

    sprintf(buf, "ST:%d ND:%d       ", cur_state, node_index);
    buf[19] = '\0';
    ips200_show_string(0, y_start, buf);

    y_start += 16;
    sprintf(buf, "T:%.0f R:%.0f       ", ctrl_state.angular_rate_target, ctrl_state.angular_rate_current);
    buf[19] = '\0';
    ips200_show_string(0, y_start, buf);
}

static inline void Safe_Draw_Point(int16_t x, int16_t y, uint16_t color) {
    if (x >= 0 && x < SCR_W && y >= 0 && y < SCR_H) {
        ips200_draw_point(x, y, color);
    }
}

static void UI_Draw_Page_Image(void) {
    // 【分区1：左上角 140x90 原始图像块发送 (高性能)】
    // 为了解决底层 Y=0 在近处(倒置)的问题，我们在内存中快速翻转一次，直接调用块显示
    static uint8_t disp_buf[90][140]; // 静态申请，防止爆栈
    for (int y = 0; y < YM; y++) {
        for (int x = 0; x < XM; x++) {
            disp_buf[y][x] = imgOSTU[YM - 1 - y][x]; // Y轴镜像翻转
        }
    }
    // 假设阈值填128，255会显示为白，0显示为黑。利用逐飞库一次性推入显存！
    ips200_show_gray_image(0, 0, (const uint8_t *)disp_buf, XM, YM, XM, YM, 128);

    // 【分区2：左下角 纯净线特征提取区】
    // 在 Y=120 开始的下方区域，打点显示提取的寻线数组
    int16_t y_offset = 120;
    
    for (int i = 0; i < YM; i++) {
        if (mid_line[i] < XM) 
            Safe_Draw_Point(mid_line[i], y_offset + YM - 1 - i, RGB565_RED);
    }
    for (int i = 0; i < l_data_statics; i++) {
        if (points_l[i][0] < XM && points_l[i][1] < YM) 
            Safe_Draw_Point(points_l[i][0], y_offset + YM - 1 - points_l[i][1], RGB565_GREEN);
    }
    for (int i = 0; i < r_data_statics; i++) {
        if (points_r[i][0] < XM && points_r[i][1] < YM) 
            Safe_Draw_Point(points_r[i][0], y_offset + YM - 1 - points_r[i][1], RGB565_BLUE);
    }

    // 【分区3：右侧 核心数据看板】
    char buf[32];
    int16_t panel_x = 150; // 右侧看板起始X坐标
    int16_t py = 0;
    
    // 行1：大津法阈值 与 偏差 (假设用 start_center_x 算偏差)
    sprintf(buf, "Thres:%3d Err:%3d ", nowThreshold, (int)(XM/2 - start_center_x));
    ips200_show_string(panel_x, py, buf);
    py += 20;
    
    // 行2：状态机与节点
    // 严防 cur_state 越界导致死机
    uint8_t state_idx = (cur_state <= STATE_BLEND) ? cur_state : 0;
    sprintf(buf, "St: [%s] Node: %d   ", State_Str[state_idx], node_index);
    ips200_show_string(panel_x, py, buf);
    py += 20;

    // 行3：姿态核心
    sprintf(buf, "Yaw: %5.1f       ", yaw_plus); // 注意挂载你的 yaw 变量
    ips200_show_string(panel_x, py, buf);
    py += 20;
    
    sprintf(buf, "GyoZ:%5.0f       ", gyro_param.gyro_z);
    ips200_show_string(panel_x, py, buf);
    py += 20;

    // 行4：底层电机动作
    sprintf(buf, "L:%4d R:%4d    ", ctrl_state.output_left_pwm, ctrl_state.output_right_pwm);
    ips200_show_string(panel_x, py, buf);
}

static void UI_Draw_Page_Param(char *page_title) {
    char buf[32];
    ips200_show_string(0, 0, page_title);
    if(curr_param_cnt == 0 || curr_params == NULL) return;

    // 支持双栏，假设最多显示 12 个参数
    #define MAX_SHOW_NEW 12 
    
    for (uint8_t i = 0; i < MAX_SHOW_NEW; i++) {
        uint8_t p_idx = show_start + i;
        if (p_idx >= curr_param_cnt) break;

        Param_Item_t *p = &curr_params[p_idx];
        
        // 分栏逻辑：偶数在左，奇数在右
        int16_t x_pos = (i % 2 == 0) ? 0 : 160; 
        int16_t y_pos = 20 + (i / 2) * 20;

        char val_str[16];
        if (p->is_int == 2) sprintf(val_str, "%d", *(uint8*)p->ptr_val);
        else if (p->is_int == 1) sprintf(val_str, "%d", *(int16_t*)p->ptr_val);
        else           sprintf(val_str, "%.2f", *(float*)p->ptr_val);

        if (p_idx == cursor) {
            if (is_editing) sprintf(buf, ">[%s:%s]<  ", p->name, val_str);
            else            sprintf(buf, "->%s:%s    ", p->name, val_str);
        } else {
            sprintf(buf, "  %s:%s    ", p->name, val_str);
        }
        // 确保定长擦除
        buf[15] = '\0'; 
        ips200_show_string(x_pos, y_pos, buf);
    }
}

// ==================== 新增的 MY_A 空白页 ====================
static void UI_Draw_Page_My_A(void) {
    char buf[32];
    ips200_show_string(0, 0, "=== SENSOR DEBUG ===");
    sprintf(buf, "SpdL:%d R:%d       ", (int)Actual_Speed[0], (int)Actual_Speed[1]);
    buf[19] = '\0';
    ips200_show_string(0, 20, buf);
    sprintf(buf, "PwmL:%d R:%d       ", ctrl_state.output_left_pwm, ctrl_state.output_right_pwm);
    buf[19] = '\0';
    ips200_show_string(0, 40, buf);
    sprintf(buf, "GyoZ:%.0f        ", gyro_param.gyro_z);
    buf[19] = '\0';
    ips200_show_string(0, 60, buf);
    sprintf(buf, "Yaw:%.1f          ", yaw_plus);
    buf[19] = '\0';
    ips200_show_string(0, 80, buf);
    Draw_Bottom_Dashboard();
}

static void UI_Switch_Page(Page_Enum new_page) {
    if(curr_page != new_page) {
        curr_page = new_page;
        is_editing = 0;
        cursor = 0;
        show_start = 0;
        ips200_clear();

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
    ips200_clear();
    curr_page = PAGE_MAX;
    UI_Switch_Page(PAGE_IMAGE);
}

void UI_Menu_Task(void) {
    Key_Event_t key = UI_Key_Scan();

    if (key == KEY_CAR_LONG) {
        system_running = 1;
        ips200_clear();
        ips200_show_string(30, 60, "CAR RUNNING!");
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
            if (cursor >= show_start + MAX_SHOW_NEW) show_start = cursor - MAX_SHOW_NEW + 1;

            if (key == KEY_CAR_CLICK) is_editing = 1;
        }
        else {
            if(key == KEY_UP_CLICK   || key == KEY_UP_REPEAT)   UI_Adjust_Param(1);
            if(key == KEY_DOWN_CLICK || key == KEY_DOWN_REPEAT) UI_Adjust_Param(-1);
            if(key == KEY_CAR_CLICK) is_editing = 0;
        }
    }

    // --- 【强制性能锁】 ---
    static uint8_t render_delay = 0;
    render_delay++;
    // 如果主循环是 10ms，则 50ms 刷一次屏幕 (5次)
    // 绝对不能让 UI 刷新抢占电磁识别和 PID 的算力时间片！
    if (render_delay < 5) return; 
    render_delay = 0;             

    // 渲染总路由
    switch(curr_page) {
        case PAGE_IMAGE:     UI_Draw_Page_Image(); break;
        case PAGE_SPEED:     UI_Draw_Page_Param("=== SPEED CFG ==="); break;
        case PAGE_TURN_PID:  UI_Draw_Page_Param("=== TURN PID ==="); break;
        case PAGE_MOTOR_PID: UI_Draw_Page_Param("=== MOTOR PID ==="); break;
        case PAGE_MY_A:      UI_Draw_Page_My_A(); break; // <--- 渲染新增的 MY_A
        default: break;
    }
}

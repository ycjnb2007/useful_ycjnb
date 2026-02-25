#include "ui_menu.h"
#include "deal_img.h"
#include "ui_key.h"





// ==================== 屏幕参数 ====================
// 必须在 main.c 里调用 tft180_set_dir(2); 设置为横屏
#define SCR_W 160
#define SCR_H 128

// ==================== 菜单定义 ====================
typedef enum {
    PAGE_MAIN = 0, // 图像 + 底部数据
    PAGE_PARAM,    // 调参 + 底部数据
    PAGE_OTHER,    // 其他
    PAGE_MAX
} Page_Enum;

static Page_Enum curr_page = PAGE_MAIN;
static uint8_t cursor = 0;

// 全局变量定义
float motor_kp = 1.00f;
float motor_ki = 1.50f;
float motor_kd = 1.20f;
int16_t target_speed = 1000;

// 模拟的传感器变量 (请在 main 中更新这些值)
int16_t error_val = 0;
int16_t speed_left_enc = 0;
int16_t speed_right_enc = 0;
float   gyro_z_val = 0.0f;

// 调参项
typedef struct {
    char *name;
    void *ptr_val;
    float step;
    uint8_t is_int;
} Param_Item_t;

static Param_Item_t params[] = {
    {"KP",    &motor_kp,     0.1f, 0},
    {"KI",    &motor_ki,     0.01f, 0},
    {"KD",    &motor_kd,     0.1f, 0},
    {"Speed", &target_speed, 50.0f, 1} // 速度步长设为50，调得快
};
#define PARAM_CNT 4

// ==================== 【核心】安全绘图函数 ====================
// 所有的画点操作都必须经过这里！这是防止断言报错的最后一道防线。
void Safe_Draw_Point(int16_t x, int16_t y, uint16_t color) {
    // 只有坐标在屏幕范围内才画，否则直接忽略
    if(x >= 0 && x < SCR_W && y >= 0 && y < SCR_H) {
        tft180_draw_point(x, y, color);
    }
}

// ==================== 辅助函数: 绘制底部通用仪表盘 ====================
// 这个函数在所有页面通用，确保数据在哪都能看到
void Draw_Bottom_Dashboard(void) {
    char buf[32];
    // 从 Y=94 开始画，留出上方给图像或参数
    int16_t y_start = 94;

    // 画一条分割线
    // tft180_show_string(0, y_start-10, "----------------");

    // 第一行：误差 + 陀螺仪
    // 格式：%-4d 左对齐补空格，防重叠残影
    sprintf(buf, "Err:%-4d Gyro:%-5.1f   ", error_val, gyro_z_val);
    tft180_show_string(0, y_start, buf);

    // 第二行：左右速度
    y_start += 16;
    sprintf(buf, "L:%-4d R:%-4d      ", speed_left_enc, speed_right_enc);
    tft180_show_string(0, y_start, buf);
}

// ==================== 页面1: 图像运行页 ====================
static void UI_Draw_Page_Image(void) {
    // 布局：图像顶格 (Y=0)，高度70。底部留给数据。
    // 计算居中 X: (160 - 140)/2 = 10
    int16_t offset_x = (SCR_W - XM) / 2;
    int16_t offset_y = 0;

    // 1. 绘制二值化图像 (使用安全画点)
    // 放弃 show_gray_image，改用双重循环，最安全
    for(int y = 0; y < YM; y++) {
        for(int x = 0; x < XM; x++) {
            if(imgOSTU[y][x] == 255) { // 白点
                Safe_Draw_Point(offset_x + x, offset_y + y, RGB565_WHITE);
            }
            // 可以在这里加 else 画黑点清屏，或者依赖整体 clear
        }
    }

    // 2. 绘制辅助线 (中线红，左绿，右蓝)
    for(int i=0; i<YM; i++) {
        // 过滤无效数据 (比如255)
        if(mid_line[i] < XM)
            Safe_Draw_Point(offset_x + mid_line[i], offset_y + i, RGB565_RED);
    }
    for(int i=0; i<l_data_statics; i++) {
        if(points_l[i][0] < XM && points_l[i][1] < YM)
            Safe_Draw_Point(offset_x + points_l[i][0], offset_y + points_l[i][1], RGB565_GREEN);
    }
    for(int i=0; i<r_data_statics; i++) {
        if(points_r[i][0] < XM && points_r[i][1] < YM)
            Safe_Draw_Point(offset_x + points_r[i][0], offset_y + points_r[i][1], RGB565_BLUE);
    }

    // 3. 绘制底部数据
    Draw_Bottom_Dashboard();
}

// ==================== 页面2: PID调参页 ====================
static void UI_Draw_Page_Param(void) {
    tft180_show_string(30, 2, "== SETUP ==");
    char buff[32];

    // 1. 绘制参数列表 (上半屏)
    for(int i=0; i<PARAM_CNT; i++) {
        uint16_t y_pos = 22 + i * 16;

        // 绘制光标
        if(i == cursor) tft180_show_string(0, y_pos, ">");
        else            tft180_show_string(0, y_pos, " ");

        // 绘制参数 (带左对齐防重叠)
        if(params[i].is_int) {
            int16_t *val = (int16_t*)params[i].ptr_val;
            sprintf(buff, "%-5s: %-6d  ", params[i].name, *val);
        } else {
            float *val = (float*)params[i].ptr_val;
            sprintf(buff, "%-5s: %-6.2f  ", params[i].name, *val);
        }
        tft180_show_string(16, y_pos, buff);
    }

    // 2. 绘制底部数据 (复用同一个仪表盘)
    Draw_Bottom_Dashboard();

    // 右下角小提示
    tft180_show_string(110, 115, "Exit");
}

// ==================== 页面3: 其他信息 ====================
static void UI_Draw_Page_Other(void) {
    tft180_show_string(10, 10, "System Ready");
    tft180_show_string(10, 30, "Battery: OK");
    Draw_Bottom_Dashboard();
}

// ==================== 逻辑处理 ====================
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

    // === 全局退出逻辑 ===
    // 长按 DOWN 键 -> 强制返回图像主页
    if(key == KEY_DOWN_LONG_PRESS) {
        if(curr_page != PAGE_MAIN) {
            curr_page = PAGE_MAIN;
            cursor = 0;
            tft180_clear();
            return;
        }
    }

    switch(curr_page) {
        // --- Page 1: 图像 + 数据 ---
        case PAGE_MAIN:
            // 确认键 / 右键 -> 进入调参
            if(key == KEY_CENTER_CLICK || key == KEY_RIGHT_CLICK) {
                curr_page = PAGE_PARAM;
                cursor = 0;
                tft180_clear();
            }
            // 图像页需要高频刷新
            UI_Draw_Page_Image();
            break;

        // --- Page 2: 调参 + 数据 ---
        case PAGE_PARAM:
            // 上下移动光标
            if(key == KEY_UP_CLICK)   cursor = (cursor == 0) ? PARAM_CNT-1 : cursor-1;
            if(key == KEY_DOWN_CLICK) cursor = (cursor == PARAM_CNT-1) ? 0 : cursor+1;

            // 左右修改参数 (支持连按)
            if(key == KEY_RIGHT_CLICK || key == KEY_RIGHT_REPEAT) UI_Adjust_Param(1);
            if(key == KEY_LEFT_CLICK || key == KEY_LEFT_REPEAT)   UI_Adjust_Param(-1);

            UI_Draw_Page_Param();
            break;

        case PAGE_OTHER:
            UI_Draw_Page_Other();
            break;

        default:
            curr_page = PAGE_MAIN;
            break;
    }
}

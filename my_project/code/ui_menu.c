// ==========================================
// 文件名: ui_menu.c
// 描述: 高性能无断言菜单绘制及逻辑框架，适配 160x128 屏幕和140x90图像
// ==========================================
#include "ui_menu.h"


// ==================== 全局变量定义 ====================
uint8_t system_running = 0; // 发车标志。1=运行，0=停车(显示菜单)

// 演示用的变量 (这些通常在你的 main.c 或 control.c 里更新)
//float Yaw_plus = 0.0f;
int16_t error_val = 0;
int16_t speed_left_enc = 0;
int16_t speed_right_enc = 0;

float motor_kp = 1.00f;
float motor_ki = 0.50f;
float motor_kd = 0.20f;
float servo_kp = 2.50f;
float servo_kd = 1.80f;

int16_t expect_speed_str = 1500;
int16_t expect_speed_cur = 800;

// ==================== 菜单页面与结构定义 ====================
// 子菜单枚举定义
typedef enum {
    PAGE_MAIN = 0,  // 主页面(图像)
    PAGE_PID,       // PID 调节页
    PAGE_SPEED,     // 期望速度调节页
    PAGE_DEBUG,     // 备用1: 传感器 Debug
    PAGE_TEST,      // 备用2: 电机测试
    PAGE_MAX        // 页面上限，用于边界判断
} Page_Enum;

// 动态参数项的结构体，利用 void 指针强制转换来实现通用编辑
typedef struct {
    char *name;      // 参数显示名
    void *ptr_val;   // 指向该参数物理地址的指针
    float step;      // 每次按键增减的步长
    uint8_t is_int;  // 1: 该变量是 int16_t 类型; 0: 是 float 类型
} Param_Item_t;

// 页面内全局状态记录
static Page_Enum curr_page = PAGE_MAIN; // 当前页面
static uint8_t cursor_idx = 0;          // 光标指向第几项
static uint8_t edit_mode = 0;           // 0: 光标选择模式, 1: 参数修改模式

// 备份变量: 进入编辑模式时保存旧值，如果用户按 BACK 取消，就恢复旧值
static float old_val_float = 0;
static int16_t old_val_int = 0;

// --- 各页面的参数绑定表 ---
static Param_Item_t pid_params[] = {
    {"Mot KP", &motor_kp, 0.1f, 0},
    {"Mot KI", &motor_ki, 0.05f,0},
    {"Mot KD", &motor_kd, 0.1f, 0},
    {"Srv KP", &servo_kp, 0.1f, 0},
    {"Srv KD", &servo_kd, 0.1f, 0},
};
#define PID_PARAM_CNT (sizeof(pid_params)/sizeof(pid_params[0]))

static Param_Item_t speed_params[] = {
    {"Str Spd", &expect_speed_str, 50.0f, 1},
    {"Cur Spd", &expect_speed_cur, 50.0f, 1},
};
#define SPEED_PARAM_CNT (sizeof(speed_params)/sizeof(speed_params[0]))

// ==================== 屏幕绘图核心与底层防守 ====================

// 【绝对防御】安全画点函数，死守边界，防止由于越界引发硬件断言(Assert)直接死机
static void Safe_Draw_Point(int16_t x, int16_t y, uint16_t color) {
    // 只有坐标在 0~159 且 0~127 范围内才调用逐飞库画点
    if (x >= 0 && x < SCR_W && y >= 0 && y < SCR_H) {
        tft180_draw_point(x, y, color);
    }
}

// 通用底部仪表盘 (占据屏幕底部区域 Y=96 到 127)
// 在 128 高的屏幕下，使用 16 号字体 (高16)，两行正好占用 32 像素
static void Draw_Bottom_Dashboard(void) {
    char buf[32];
    int16_t y_start = 96; // 必须是 96，确保留给上半部的空间刚好为 96 像素(容纳 YM=90)

    // 第一行: Yaw_plus 和 巡线误差
    // %-5.1f: 负号代表左对齐，5代表总宽度，1代表小数位。左对齐可防数字跳动留下的残影
    sprintf(buf, "Y:%-5.1f E:%-4d  ", yaw_plus, error_val);
    tft180_show_string(0, y_start, buf);

    // 第二行: 左右轮速
    y_start += 16;
    sprintf(buf, "L:%-5d R:%-5d  ", speed_left_enc, speed_right_enc);
    tft180_show_string(0, y_start, buf);
}

// ==================== 各子菜单页面绘制函数 ====================

// 主页面: 绘制图像与中线
static void UI_Draw_Page_Main(void) {
    int16_t offset_x = (SCR_W - XM) / 2;
    int16_t offset_y = 0;

    // 【核心修复】绝不能用 for 循环画底层图像，直接用整块搬运！
    // imgOSTU 是你的二值化图（只有0和255），直接以灰度图形式传给屏幕，阈值设128。
    // 这行代码刷一张图只要几毫秒，瞬间释放 CPU！
    tft180_show_gray_image(offset_x, offset_y, (const uint8_t *)imgOSTU, XM, YM, XM, YM, 128);

    // 辅助线因为只有几十个点，继续用画点函数没问题
    for (int i = 0; i < YM; i++) {
        if (mid_line[i] < XM) {
            Safe_Draw_Point(offset_x + mid_line[i], offset_y + i, RGB565_RED);
        }
    }
    for (int i = 0; i < l_data_statics; i++) {
        if (points_l[i][0] < XM && points_l[i][1] < YM) {
            Safe_Draw_Point(offset_x + points_l[i][0], offset_y + points_l[i][1], RGB565_GREEN);
        }
    }
    for (int i = 0; i < r_data_statics; i++) {
        if (points_r[i][0] < XM && points_r[i][1] < YM) {
            Safe_Draw_Point(offset_x + points_r[i][0], offset_y + points_r[i][1], RGB565_BLUE);
        }
    }

    Draw_Bottom_Dashboard();
}

// 通用参数列表绘制引擎
// 传入参数表基址，数量，和标题
static void UI_Draw_Param_List(Param_Item_t *list, uint8_t count, const char* title) {
    char buf[32];
    tft180_show_string(30, 0, title); // 在顶部打印页面标题

    // 遍历所有参数项
    for (int i = 0; i < count; i++) {
        // 每项高度 16，从 y=16 开始往下排
        int16_t y_pos = 16 + i * 16;

        // --- 游标和编辑状态渲染 ---
        if (i == cursor_idx) {
            // 被选中项
            if (edit_mode) tft180_show_string(0, y_pos, "*"); // 编辑模式下用 '*' 提示
            else           tft180_show_string(0, y_pos, ">"); // 仅光标停留用 '>'
        } else {
            tft180_show_string(0, y_pos, " "); // 未选中留空
        }

        // --- 取出指针里的值，格式化打印 ---
        // 核心技术：用 void* 存放任何地址，根据 is_int 标志决定用什么类型的指针强转读取它！
        if (list[i].is_int) {
            int16_t *val_ptr = (int16_t*)list[i].ptr_val;
            sprintf(buf, "%-7s:%-6d ", list[i].name, *val_ptr);
        } else {
            float *val_ptr = (float*)list[i].ptr_val;
            sprintf(buf, "%-7s:%-6.2f ", list[i].name, *val_ptr);
        }
        tft180_show_string(12, y_pos, buf); // 在游标右侧显示文字
    }

    Draw_Bottom_Dashboard();
}

// 页面：PID 设置
static void UI_Draw_Page_PID(void) {
    UI_Draw_Param_List(pid_params, PID_PARAM_CNT, "== PID SET ==");
}

// 页面：速度 设置
static void UI_Draw_Page_Speed(void) {
    UI_Draw_Param_List(speed_params, SPEED_PARAM_CNT, "== SPEED SET ==");
}

// 页面：备用调试页 (占位示例)
static void UI_Draw_Page_Debug(void) {
    tft180_show_string(10, 20, "Sensor Debug");
    tft180_show_string(10, 50, "ADC_VAL: 4096 ");
    Draw_Bottom_Dashboard();
}

// ==================== 菜单逻辑运算引擎 ====================

// 参数调节实现逻辑 (加或者减)
static void Handle_Param_Adjust(Param_Item_t *list, int8_t dir) {
    Param_Item_t *p = &list[cursor_idx];

    // dir 是方向: 1是加, -1是减
    if (p->is_int) {
        int16_t *val_ptr = (int16_t*)p->ptr_val;
        *val_ptr += (int16_t)(dir * p->step);
    } else {
        float *val_ptr = (float*)p->ptr_val;
        *val_ptr += (dir * p->step);
    }
}

// 进入编辑模式时，将变量当前值备份
static void Backup_Current_Param(Param_Item_t *list) {
    Param_Item_t *p = &list[cursor_idx];
    if(p->is_int) old_val_int = *(int16_t*)p->ptr_val;
    else          old_val_float = *(float*)p->ptr_val;
}

// 取消编辑时，恢复备份的旧值
static void Restore_Old_Param(Param_Item_t *list) {
    Param_Item_t *p = &list[cursor_idx];
    if(p->is_int) *(int16_t*)p->ptr_val = old_val_int;
    else          *(float*)p->ptr_val = old_val_float;
}

// 处理带列表的页面的通用逻辑
// count为该页面的最大选项数量
static void Handle_List_Logic(Key_Event_t key, Param_Item_t *list, uint8_t count) {
    if (edit_mode == 0) {
        // --- 模式0：光标选择 ---
        if (key == KEY_UP_CLICK) {
            cursor_idx = (cursor_idx == 0) ? count - 1 : cursor_idx - 1;
        }
        else if (key == KEY_DOWN_CLICK) {
            cursor_idx = (cursor_idx == count - 1) ? 0 : cursor_idx + 1;
        }
        else if (key == KEY_ENTER_CLICK) {
            Backup_Current_Param(list); // 记下当前值
            edit_mode = 1;              // 进入编辑模式
        }
        else if (key == KEY_BACK_CLICK) {
            curr_page = PAGE_MAIN;      // 返回主菜单
            cursor_idx = 0;
            tft180_clear();             // 切换页面必须清屏防残影
        }
    }
    else {
        // --- 模式1：参数编辑修改 ---
        if (key == KEY_UP_CLICK || key == KEY_UP_REPEAT) {
            Handle_Param_Adjust(list, 1);  // 加
        }
        else if (key == KEY_DOWN_CLICK || key == KEY_DOWN_REPEAT) {
            Handle_Param_Adjust(list, -1); // 减
        }
        else if (key == KEY_ENTER_CLICK) {
            edit_mode = 0;  // 保存并退出编辑模式 (实际上因为直接修改内存，所以这里只要置0就行)
        }
        else if (key == KEY_BACK_CLICK) {
            Restore_Old_Param(list); // 后悔药，恢复旧值
            edit_mode = 0;
        }
    }
}

// ==================== 对外暴露 API ====================

// 初始化界面与按键
void UI_Menu_Init(void) {
    UI_Key_Init();
    // 强行设定为横屏模式 160x128
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_clear();
}

// 主机调度任务 (放在大 while 循环里)
void UI_Menu_Task(void) {
    // 扫描获取按键动作
    Key_Event_t key = UI_Key_Scan();

    // 【最高优先级拦截】长按 ENTER 触发发车！
    if (key == KEY_ENTER_LONG) {
        system_running = 1; // 置位后，控制权交予底层
        tft180_clear();
        tft180_show_string(40, 60, "RUNNING..."); // 留下一句霸气的提示，从此屏幕再也不刷
        return;
    }

    // 【算力极致榨取】如果已经发车了，绝对不允许屏幕再占用任何 SPI 通信时间和 CPU。直接退出！
    if (system_running) {
        return;
    }

    // 状态机路由分发
    switch (curr_page) {

        // --- 主页路由 ---
        case PAGE_MAIN:
            // 主菜单下，按 ENTER 切换到下一个菜单页
            if (key == KEY_ENTER_CLICK) {
                curr_page = PAGE_PID; // 默认跳向第一张配置页
                cursor_idx = 0;
                tft180_clear();
            }
            UI_Draw_Page_Main();
            break;

        // --- PID设置路由 ---
        case PAGE_PID:
            Handle_List_Logic(key, pid_params, PID_PARAM_CNT);
            // 这里加入页面切换彩蛋: 如果没在编辑参数，且按了特定的键(如BACK到顶)，可以在此做页面平切
            if (edit_mode == 0 && key == KEY_BACK_CLICK) {
                // Handle_List_Logic 已经把页跳回主页了，不需要写
            }
            // 当游标在最后一项，还按 DOWN，可以直接跳入下一个菜单页PAGE_SPEED (可根据喜好添加)
            if (edit_mode == 0 && cursor_idx == PID_PARAM_CNT - 1 && key == KEY_DOWN_CLICK) {
                curr_page = PAGE_SPEED;
                cursor_idx = 0;
                tft180_clear();
            }
            UI_Draw_Page_PID();
            break;

        // --- 速度设置路由 ---
        case PAGE_SPEED:
            Handle_List_Logic(key, speed_params, SPEED_PARAM_CNT);
            // 同理，实现向下翻页
            if (edit_mode == 0 && cursor_idx == SPEED_PARAM_CNT - 1 && key == KEY_DOWN_CLICK) {
                curr_page = PAGE_DEBUG;
                cursor_idx = 0;
                tft180_clear();
            }
            // 向上翻页回 PID
            if (edit_mode == 0 && cursor_idx == 0 && key == KEY_UP_CLICK) {
                curr_page = PAGE_PID;
                cursor_idx = PID_PARAM_CNT - 1;
                tft180_clear();
            }
            UI_Draw_Page_Speed();
            break;

        // --- 调试页路由 ---
        case PAGE_DEBUG:
            if (key == KEY_BACK_CLICK) {
                curr_page = PAGE_MAIN;
                tft180_clear();
            } else if (key == KEY_DOWN_CLICK) {
                curr_page = PAGE_TEST;
                tft180_clear();
            } else if (key == KEY_UP_CLICK) {
                curr_page = PAGE_SPEED;
                cursor_idx = SPEED_PARAM_CNT - 1;
                tft180_clear();
            }
            UI_Draw_Page_Debug();
            break;

        // --- 测试页路由 ---
        case PAGE_TEST:
            if (key == KEY_BACK_CLICK) {
                curr_page = PAGE_MAIN;
                tft180_clear();
            } else if (key == KEY_UP_CLICK) {
                curr_page = PAGE_DEBUG;
                tft180_clear();
            }
            tft180_show_string(30, 40, "Motor Test");
            tft180_show_string(10, 60, "Push UP to Run");
            Draw_Bottom_Dashboard();
            break;

        default:
            curr_page = PAGE_MAIN;
            break;
    }
}

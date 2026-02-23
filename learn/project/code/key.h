#ifndef __KEY_H__
#define __KEY_H__

#include "zf_common_headfile.h"

// 按键引脚定义 - 根据你的要求修改
#define KEY_UP (P11_0)    // 上 - 控制上
#define KEY_DOWN (P22_0)  // 下 - 控制下
#define KEY_RIGHT (P23_3) // 右 - 控制向右
#define KEY_SET (P23_4)   // 设置 - 保存和确定

// 菜单页面定义
#define MENU_MAIN_PAGE 0          // 主菜单页（发车/调参/图像显示）
#define MENU_PARAM_SELECT_PAGE 1  // 调参选择页
#define MENU_ANGLE_PID_PAGE 2     // 角度环PID页
#define MENU_GYRO_PID_PAGE 3      // 角速度环PID页
#define MENU_SPEED_PID_PAGE 4     // 速度环PID页
#define MENU_EXPECT_SPEED_PAGE 5  // 期望速度页
#define MENU_CAMERA_PAGE 6        // 摄像头页
#define MENU_MULTI_BREAK_SELECT_PAGE 7  // 多段断路选择页
#define MENU_BREAK_RECORD_PAGE 8        // 断路记录页
#define MENU_BREAK_SEQUENCE_PAGE 9      // 断路执行序列设置页
#define MENU_IMAGE_DISPLAY_PAGE 10      // 图像显示页
#define MENU_MAX_PAGE 10                // 最大页面索引

// 参数增量定义
#define PID_INCREMENT 0.1f        // PID参数增量
#define PID_INCREMENT_SMALL 0.01f // PID参数小增量
#define SPEED_INCREMENT1 1         // 速度参数增量
#define SPEED_INCREMENT 5        // 速度参数增量

// 主菜单选项定义
#define MAIN_OPTION_START 0        // 发车选项
#define MAIN_OPTION_PARAM 1        // 调参选项
#define MAIN_OPTION_MULTI_BREAK 2  // 多段断路选项
#define MAIN_OPTION_IMAGE 3        // 图像显示选项
#define MAIN_MAX_OPTION 3          // 最大选项索引

// 多段断路菜单选项定义
#define MULTI_BREAK_OPTION_BREAK1 0     // 断路1
#define MULTI_BREAK_OPTION_BREAK2 1     // 断路2
#define MULTI_BREAK_OPTION_BREAK3 2     // 断路3
#define MULTI_BREAK_OPTION_BREAK4 3     // 断路4
#define MULTI_BREAK_OPTION_SEQUENCE 4   // 执行序列设置
#define MULTI_BREAK_MAX_OPTION 4        // 最大选项索引

// 断路操作选项定义
#define BREAK_OP_RECORD 0      // 记录断路数据
#define BREAK_OP_VIEW 1        // 查看断路数据
#define BREAK_OP_CLEAR 2       // 清除断路数据
#define BREAK_OP_MANUAL 3      // 手动开始记录
#define BREAK_OP_PRE_RECORD 4  // 提前记录模式
#define BREAK_OP_BACK 5        // 返回
#define BREAK_OP_MAX_OPTION 5  // 最大选项索引

// 调参选择页选项定义
#define PARAM_OPTION_ANGLE 0  // 角度环
#define PARAM_OPTION_GYRO 1   // 角速度环
#define PARAM_OPTION_SPEED 2  // 速度环
#define PARAM_OPTION_EXPECT 3 // 期望速度
#define PARAM_OPTION_EXIT 4   // 退出
#define PARAM_MAX_OPTION 4    // 最大选项索引

// 各页面参数数量定义
#define ANGLE_PID_PARAM_COUNT 3    // 角度环PID参数数量(KP,KI,KD)
#define GYRO_PID_PARAM_COUNT 3     // 角速度环PID参数数量(KP,KI,KD)
#define SPEED_PID_PARAM_COUNT 3    // 速度环PID参数数量(KP,KI,KD)
#define EXPECT_SPEED_PARAM_COUNT 6 // 期望速度参数数量(正常,直角,弯道,环岛,断路,退出)

// 菜单状态变量声明
extern uint8 current_page;        // 当前菜单页面
extern uint8 param_select_index;  // 当前选择的参数索引
extern uint8 show_menu_flag;      // 菜单显示标志
extern uint8 system_running;      // 系统运行标志
extern uint8 edit_mode;           // 编辑模式：0=选择模式，1=编辑模式
extern uint8 image_display_mode;  // 图像显示模式标志
extern uint8 start_ready_flag;    // 启动准备标志：1=可以启动，0=不可启动
extern uint8 track_lost_flag;     // 赛道丢失标志：1=丢失赛道，0=正常
extern uint8 emergency_stop_flag; // 紧急停车标志：1=紧急停车，0=正常

// 多段断路菜单状态变量
extern uint8 multi_break_select_mode;  // 多段断路选择模式：0=选择断路，1=操作断路，2=序列设置
extern uint8 selected_break_id;        // 选中的断路ID (1-4)
extern uint8 sequence_edit_mode;       // 序列编辑模式：0=查看，1=编辑
extern uint8 sequence_edit_index;      // 序列编辑索引 (0-3)

// 函数声明
void my_key_init(void);           // 按键初始化
void key_scan(void);              // 按键扫描处理(无阻塞)
void menu_display(void);          // 菜单显示
void save_params_to_flash(void);  // 保存参数到Flash
uint8 simple_key_check(void);     // 简单按键检查(用于运行模式)
uint8 direct_key_check(void);     // 直接按键检查(用于调试)
void show_key_status_debug(void); // 显示按键状态调试信息
// 角度环PID参数
extern float angle_kp;
extern float angle_ki;
extern float angle_kd;

// 角速度环PID参数
extern float gyro_kp;
extern float gyro_ki;
extern float gyro_kd;

// 速度环PID参数
extern float speed_kp;
extern float speed_ki;
extern float speed_kd;

// 期望速度参数
extern uint16 normal_speed;   // 正常速度
extern uint16 straight_speed; // 直角速度
extern uint16 curve_speed;    // 弯道速度
extern uint16 circle_speed;   // 环岛速度
extern uint16 break_speed;    // 断路速度

// 标志位变量(预留8个)
extern uint8 flag_1;
extern uint8 flag_2;
extern uint8 flag_3;
extern uint8 flag_4;
extern uint8 flag_5;
extern uint8 flag_6;
extern uint8 flag_7;
extern uint8 flag_8;

// 摄像头页标志位(预留4个)
extern uint8 camera_flag_1;
extern uint8 camera_flag_2;
extern uint8 camera_flag_3;
extern uint8 camera_flag_4;

#endif /* __KEY_H__ */

#ifndef _HUANDAO_H_
#define _HUANDAO_H_

#include "zf_common_headfile.h"

/**
 * @brief 环岛状态机枚举定义
 * 定义环岛处理的各个状态，用于统一管理环岛流程
 */
typedef enum
{
    CIRCLE_IDLE = 0,      // 空闲状态，未检测到环岛
    CIRCLE_DETECTED = 1,  // 检测到环岛特征
    CIRCLE_ENTERING = 2,  // 准备进入/切入点确认
    CIRCLE_IN_CIRCLE = 3, // 在环岛中
    CIRCLE_EXITING = 4,   // 正在退出环岛
    CIRCLE_COMPLETE = 5   // 环岛完成，等待重置
} circle_state_t;

typedef struct L_HUAN
{
    uint8 left_area_true; // 图像左侧区域

    uint8 right_area_true; // 图像右侧区域

    uint8 top_area_true; // 图像上侧区域
    uint8 bottom_area_true;

    uint8 up_0_1_num1; // 0_1_num代表黑白跳变数量，1_0_num代表白黑跳变数量，up为图像顶部元素行

    uint8 up_1_0_num1;

    uint8 up_0_1_num2; // up为图像顶部元素行

    uint8 up_1_0_num2;

    uint8 down_0_1_num1; // down为图像底部的元素行

    uint8 down_1_0_num1;

    uint8 down_0_1_num2; // down为图像底部的元素行

    uint8 down_1_0_num2;

    uint8 left_0_1_num1; // left为图像左侧的元素列

    uint8 left_1_0_num1;

    uint8 left_0_1_num2; // left为图像左侧的元素列

    uint8 left_1_0_num2;

    uint8 right_0_1_num1; // right为图像右侧的元素列

    uint8 right_1_0_num1;

    uint8 right_0_1_num2; // right为图像右侧的元素列

    uint8 right_1_0_num2;
} L_HUAN;

typedef struct R_HUAN
{
    uint8 left_area_true; // 图像左侧区域

    uint8 right_area_true; // 图像右侧区域

    uint8 top_area_true; // 图像上侧区域

    uint8 bottom_area_true;

    uint8 up_0_1_num1; // 0_1_num代表黑白跳变数量，1_0_num代表白黑跳变数量，up为图像顶部元素行

    uint8 up_1_0_num1;

    uint8 up_0_1_num2; // up为图像顶部元素行

    uint8 up_1_0_num2;

    uint8 down_0_1_num1; // down为图像底部的元素行

    uint8 down_1_0_num1;

    uint8 down_0_1_num2; // down为图像底部的元素行

    uint8 down_1_0_num2;

    uint8 left_0_1_num1; // left为图像左侧的元素列

    uint8 left_1_0_num1;

    uint8 left_0_1_num2; // left为图像左侧的元素列

    uint8 left_1_0_num2;

    uint8 right_0_1_num1; // right为图像右侧的元素列

    uint8 right_1_0_num1;

    uint8 right_0_1_num2; // right为图像右侧的元素列

    uint8 right_1_0_num2;
} R_HUAN;

struct CIRCLE
{
    // 环岛状态机变量
    uint8 left_circle_state;  // 左环岛状态：0-5对应circle_state_t枚举
    uint8 right_circle_state; // 右环岛状态：0-5对应circle_state_t枚举

    // 保留现有标志位（向后兼容）
    uint8 Left_flag; // 环岛标志位

    uint8 Right_flag; // 右环岛标志位

    uint8 consider_circle_top; // 初步认为环岛情况

    uint8 consider_circle_down;

    uint8 consider_circle;

    uint8_t right_true; // 圆环识别右边标志位

    uint8 prepare_In; // 准备进环岛

    uint8 In_circle;

    uint8_t out_flag; // 出环标志位

    uint8 down_area_true;

    uint8 left_area_true; // 图像左侧区域

    uint8 right_area_true; // 图像右侧区域

    uint8 top_area_true; // 图像上侧区域

    uint8 begin_In_Circle; // 准备开始环岛

    uint8 up_0_1_num1; // 0_1_num代表黑白跳变数量，1_0_num代表白黑跳变数量，up为图像顶部元素行

    uint8 up_1_0_num1;

    uint8 up_0_1_num2; // up为图像顶部元素行

    uint8 up_1_0_num2;

    uint8 down_0_1_num1; // down为图像底部的元素行

    uint8 down_1_0_num1;

    uint8 down_0_1_num2; // down为图像底部的元素行

    uint8 down_1_0_num2;

    uint8 left_0_1_num1; // left为图像左侧的元素列

    uint8 left_1_0_num1;

    uint8 left_0_1_num2; // left为图像左侧的元素列

    uint8 left_1_0_num2;

    uint8 right_0_1_num1; // right为图像右侧的元素列

    uint8 right_1_0_num1;

    uint8 right_0_1_num2; // right为图像右侧的元素列

    uint8 right_1_0_num2;

    uint8 Out_circle; // 出环岛标志
    uint8 tracking;   // 角度跟踪激活标志
    float start_yaw;  // 记录起始角度
    float now_yaw;    // 累计旋转角度
};

extern struct CIRCLE Circle;

extern L_HUAN l_huan;
extern R_HUAN r_huan;

extern uint8 left_white;
extern uint8 right_white;

// 环岛处理函数 - 统一状态机架构
void circle_init(void);          // 环岛系统初始化
void circle_detection(void);     // 环岛识别检测
void circle_state_machine(void); // 环岛状态机处理
void left_circle_process(void);  // 左环岛处理
void right_circle_process(void); // 右环岛处理
void check_enter_circle(void);   // 环岛进入检查
void update_legacy_flags(void);  // 兼容标志位更新
void circle_process(void);       // 统一环岛处理入口
void Lianxian(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end);

#endif

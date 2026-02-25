#ifndef FAN_CTRL_H
#define FAN_CTRL_H
#include "zf_common_headfile.h"
  // IPS114屏幕库

/************************* 【必须改】硬件引脚宏定义 *************************/
// ===================== RZ7899驱动引脚（根据PCB焊接修改） =====================
#define RZ7899_PWM_CH        PWM0_CH0  // 备注：逐飞TC264 PWM通道定义（PWM0_CH0~PWM0_CH7）
#define RZ7899_PWM_PIN       P00_0     // 备注：TC264物理引脚（接RZ7899 EN端），根据PCB改
#define RZ7899_IN1_PIN       P00_1     // 备注：TC264物理引脚（接RZ7899 IN1），根据PCB改
#define RZ7899_IN2_PIN       P00_2     // 备注：TC264物理引脚（接RZ7899 IN2），根据PCB改

// ===================== IPS1.14寸屏幕引脚（根据屏幕焊接修改） =====================
#define IPS_SPI_PORT         SPI_0     // 备注：逐飞TC264 SPI端口（SPI_0/SPI_1），根据PCB改
#define IPS_CS_PIN           P01_0     // 备注：TC264物理引脚（接屏幕CS），根据PCB改
#define IPS_DC_PIN           P01_1     // 备注：TC264物理引脚（接屏幕DC），根据PCB改
#define IPS_RST_PIN          P01_2     // 备注：TC264物理引脚（接屏幕RST），根据PCB改

// ===================== 按键引脚（根据按键焊接修改） =====================
#define KEY_UP_PIN           P02_0     // 备注：TC264物理引脚（接转速+按键），根据PCB改
#define KEY_DOWN_PIN         P02_1     // 备注：TC264物理引脚（接转速-按键），根据PCB改
#define KEY_ENT_PIN          P02_2     // 备注：TC264物理引脚（接启停按键），根据PCB改

/************************* 【建议改】风扇参数宏定义（适配不同负压风扇） *************************/
#define MIN_SPEED            15U       // 备注：风扇最低启动转速（%），LQ1412-A需≥15
#define MAX_SPEED            80U       // 备注：风扇最高转速（%），避免烧电机
#define SPEED_STEP           5U        // 备注：转速调节步长（%）
#define PWM_FREQ             10000U    // 备注：PWM频率（Hz），LQ1412-A适配10kHz

/************************* 全局状态声明 *************************/
extern uint8 fan_speed;    // 当前转速（%）
extern uint8 fan_running;  // 运行状态（0:停 1:运行）

/************************* 函数声明 *************************/
void fan_hw_init(void);    // 硬件初始化
void fan_adjust_speed(int8 step);  // 调节转速
void fan_toggle_running(void);     // 启停切换
void fan_key_scan(void);           // 按键扫描
void fan_display_menu(void);       // 屏幕显示菜单

#endif

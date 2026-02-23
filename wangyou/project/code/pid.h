#ifndef _pid_h_
#define _pid_h_
#include "zf_common_headfile.h"

typedef struct
{
	float Kp; // ???????
	float Ki; // ??????
	float Kd; // ???????

	float Error;	  // ???????
	float Error_Last; // ??????
	float Error_Pre;  // ????????

	float Out_Put_Min; // ???
	float Out_Put_Max;

	float Output;
} Incremental_PID_TypeDef;

/**
 * @brief PID控制器状态结构体
 * @details 封装PID控制相关的全局变量，提高代码组织性和数据安全性
 */
typedef struct {
    float angular_rate_target;  // 角速度目标值
    float angular_rate_current; // 当前角速度（来自fusion模块）
    float angular_error;        // 角速度误差
    float speed_difference;     // 速度差值
    float target_left_speed;    // 左轮目标速度
    float target_right_speed;   // 右轮目标速度
    float output_left;          // 左轮PID输出
    float output_right;         // 右轮PID输出
} PID_Control_State_t;

void Incremental_PID_Init(Incremental_PID_TypeDef *PID, float Kp, float Ki, float Kd, float Out_Put_Max, float Out_Put_Min);
float Get_Incremental_PID_Value(Incremental_PID_TypeDef *PID, float Error);
void control_system_init(void); // ����ϵͳ��ʼ������
extern Incremental_PID_TypeDef pid_left, pid_right;
extern PID_Control_State_t pid_state; // PID控制状态实例

int PD_Camera(float expect_val, float error);

extern int target_speed;
// extern void Speed_ClosedLoop_Control(float left_target, float right_target);
extern float outer_loop_control(float error);
extern float middle_loop_control();
extern float sliding_average_filter(float input);
// void Speed_ClosedLoop_Control();
// void Speed_ClosedLoop_Control(float target_left_speed, float target_right_speed);
uint16 select_target_speed(void); // 根据路径状态选择目标速度
void Speed_ClosedLoop_Control();
void steering_control(float error);
extern float angular_rate_target; // ?????????
extern float angular_error;		  // ????????
extern float speed_difference;	  // ?????
extern float target_left_speed, target_right_speed;
extern float error_left;
extern float error_right;
extern int control;
extern float test_speed;

// ???PID???
extern float output_left;
extern float output_right;

#endif

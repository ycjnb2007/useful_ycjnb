#include "pid.h"
#include <math.h>
#include "deal_img.h" // 包含图像处理头文件以获取 IF, mid_line 等
// ---------------- 定义实际的调参全局变量 (带初始值) ----------------
int16_t speed_straight_l = 250;
int16_t speed_straight_s = 180;
int16_t speed_curve      = 120;

float turn_kp_base = 1.50f;
float turn_kp_var  = 0.05f;
float turn_kd      = 0.80f;

float gyro_kp      = 1.20f;
float gyro_kd      = 0.10f;

float speed_kp     = 18.0f;
float speed_ki     = 1.50f;

// 实例化控制器
System_Control_State_t ctrl_state;
Positional_PD_t  pid_turn_outer;
Positional_PD_t  pid_gyro_middle;
Incremental_PI_t pid_speed_L;
Incremental_PI_t pid_speed_R;

void PID_Init(void) {
    float max_pwm = 9500.0f;
    pid_speed_L.out_max = max_pwm; pid_speed_L.out_min = -max_pwm;
    pid_speed_R.out_max = max_pwm; pid_speed_R.out_min = -max_pwm;
}

float Low_Pass_Filter(float current_val, float last_val, float alpha) {
    return alpha * current_val + (1.0f - alpha) * last_val;
}

float Calc_Incremental_PI(Incremental_PI_t *pid, float target, float current) {
    float increment;
    pid->error = target - current;
    increment = pid->Kp * (pid->error - pid->last_error) + pid->Ki * pid->error;
    pid->output += increment;
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;
    pid->last_error = pid->error;
    return pid->output;
}

float Outer_Loop_Camera(void) {
    uint8_t aim_row = Deal_Bottom + 15;
    if(IF == straightlineL) aim_row = Deal_Bottom + 25;
    else if(IF == curve)    aim_row = Deal_Bottom + 10;
    if(aim_row > Deal_Top)  aim_row = Deal_Top;

    ctrl_state.camera_error = 70.0f - (float)mid_line[aim_row];
    pid_turn_outer.error = ctrl_state.camera_error;

    // 【关键优化】：将修改后的全局变量实时赋给底层！彻底告别宏定义写死的问题！
    pid_turn_outer.Kp = turn_kp_base + turn_kp_var * fabs(pid_turn_outer.error);
    pid_turn_outer.Kd = turn_kd;

    pid_turn_outer.output = pid_turn_outer.Kp * pid_turn_outer.error +
                            pid_turn_outer.Kd * (pid_turn_outer.error - pid_turn_outer.last_error);

    pid_turn_outer.last_error = pid_turn_outer.error;

    if(pid_turn_outer.output > 300.0f) pid_turn_outer.output = 300.0f;
    if(pid_turn_outer.output < -300.0f) pid_turn_outer.output = -300.0f;
    return pid_turn_outer.output;
}

void Control_Loop(void) {
    // 【关键优化】：每次进入循环，同步其他参数，保证 UI 调参后瞬间起效
    pid_gyro_middle.Kp = gyro_kp;
    pid_gyro_middle.Kd = gyro_kd;
    pid_speed_L.Kp = speed_kp; pid_speed_L.Ki = speed_ki;
    pid_speed_R.Kp = speed_kp; pid_speed_R.Ki = speed_ki;

    switch (IF) {
        case straightlineL: ctrl_state.base_speed = speed_straight_l; break;
        case straightlineS: ctrl_state.base_speed = speed_straight_s; break;
        case curve:         ctrl_state.base_speed = speed_curve;      break;
        default:            ctrl_state.base_speed = speed_straight_s; break;
    }

    ctrl_state.angular_rate_target = Outer_Loop_Camera();

    static float last_gyro = 0;
    // float raw_gyro = gyro_param.gyro_z; // 你需要在你的工程里确保 gyro_param 可见，或 extern 它
    float raw_gyro = 0; // 由于缺失文件，这里用0代位。请换回你 imu 读取的值

    ctrl_state.angular_rate_current = Low_Pass_Filter(raw_gyro, last_gyro, 0.6f);
    last_gyro = ctrl_state.angular_rate_current;

    pid_gyro_middle.error = ctrl_state.angular_rate_target - ctrl_state.angular_rate_current;
    pid_gyro_middle.output = pid_gyro_middle.Kp * pid_gyro_middle.error +
                             pid_gyro_middle.Kd * (pid_gyro_middle.error - pid_gyro_middle.last_error);
    pid_gyro_middle.last_error = pid_gyro_middle.error;
    ctrl_state.speed_diff = pid_gyro_middle.output;

    ctrl_state.target_left_speed  = ctrl_state.base_speed - ctrl_state.speed_diff;
    ctrl_state.target_right_speed = ctrl_state.base_speed + ctrl_state.speed_diff;

    // 假设 Actual_Speed 是外部拿到的，如果是 motor.c 里的记得 extern
    // ctrl_state.current_left_speed = Actual_Speed[0];
    // ctrl_state.current_right_speed = Actual_Speed[1];

    ctrl_state.output_left_pwm  = (int16_t)Calc_Incremental_PI(&pid_speed_L, ctrl_state.target_left_speed, ctrl_state.current_left_speed);
    ctrl_state.output_right_pwm = (int16_t)Calc_Incremental_PI(&pid_speed_R, ctrl_state.target_right_speed, ctrl_state.current_right_speed);

    // Motor_Control(ctrl_state.output_left_pwm, ctrl_state.output_right_pwm);
}

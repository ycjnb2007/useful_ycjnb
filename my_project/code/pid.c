#include "pid.h"
#include "filter.h"
#include "image_deal_best.h"
#include "imu660.h"
#include "motor.h"

int16_t speed_straight_l = 250;
int16_t speed_straight_s = 180;
int16_t speed_curve      = 120;

float turn_kp_base = 1.50f;
float turn_kp_var  = 0.05f;
float turn_kd      = 0.80f;

float gyro_kp      = 1.20f;
float gyro_kd      = 0.10f;
float blind_turn_kp = 4.5f;
float blind_turn_kd = 0.3f;
extern float Distance_Integral; // From motor.c

float speed_kp     = 18.0f;
float speed_ki     = 1.50f;

// ???????????
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

    // multi-row weighted average (0.1, 0.2, 0.4, 0.2, 0.1) anti-noise
    float weighted_mid = (float)mid_line[aim_row];
    if (aim_row >= 2 && aim_row <= YY - 2) {
        weighted_mid =  mid_line[aim_row - 2] * 0.1f +
                        mid_line[aim_row - 1] * 0.2f +
                        mid_line[aim_row]     * 0.4f +
                        mid_line[aim_row + 1] * 0.2f +
                        mid_line[aim_row + 2] * 0.1f;
    }
    ctrl_state.camera_error = 70.0f - weighted_mid;
    pid_turn_outer.error = ctrl_state.camera_error;

    // ???????????????????????????????????????????潩????????
    pid_turn_outer.Kp = turn_kp_base + turn_kp_var * fabs(pid_turn_outer.error);
    pid_turn_outer.Kd = turn_kd;

    pid_turn_outer.output = pid_turn_outer.Kp * pid_turn_outer.error +
                            pid_turn_outer.Kd * (pid_turn_outer.error - pid_turn_outer.last_error);

    pid_turn_outer.last_error = pid_turn_outer.error;

    if(pid_turn_outer.output > 300.0f) pid_turn_outer.output = 300.0f;
    if(pid_turn_outer.output < -300.0f) pid_turn_outer.output = -300.0f;
    return pid_turn_outer.output;
}


// float current_yaw = 0.0f; // removed, using yaw_plus from imu
uint8_t blind_turn_finished = 0;

void Control_Loop(void) {

    // ??????????????潩???????????????????????? UI ???潩??????潩
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

    float raw_gyro = gyro_param.gyro_z; // 获取Z轴角速度
    
    // 如果滤波器未初始化，调用filter库初始化(Alpha 0.6)
    if (!velocity_filter.initialized) {
        LPF_InitByAlpha(&velocity_filter, 0.6f);
    }
    
    // 使用用户 filter.h 的滤波器 updating
    ctrl_state.angular_rate_current = LPF_Update(&velocity_filter, raw_gyro);
    
    // 对角速度进行积分计算当前偏航角(假设控制周期10ms)
    // 这里的 0.010f 需按实际控制周期（如定时器中断源周期）调整
    // current_yaw += ... // removed, avoid dual integration
    
    // ---- 盲转逻辑介入 ----
    // blind turn straight exit now handled by blind_distance in isr.c // 新增直行计步器
    // ---- arch-compliant blind turn logic (with PD & odometer) ----
    static float Start_Dist = 0; // blind turn start distance
    
    if (is_blind_turning == 1) {
        float yaw_error = Yaw_Target - (yaw_plus - Yaw_Start);
        
        // [Case A] straight blind turn logic
        if (my_abs((int)Yaw_Target) < 1) {
            ctrl_state.angular_rate_target = 0;
            ctrl_state.base_speed = speed_straight_s; 
            
            // Exit based on absolute distance integral (e.g. 1500 pulses)
            if ((Distance_Integral - Start_Dist) > 1500.0f) {
                is_blind_turning = 0;
            }
        }
        // [Case B] curved blind turn logic (90 or -90)
        else {
            if (my_abs((int)yaw_error) < 5 || blind_turn_finished) {
                blind_turn_finished = 1;
                ctrl_state.angular_rate_target = 0; 
                is_blind_turning = 0; // angle reached, return to vision
            } else {
                // Positional PD flexible turning
                ctrl_state.angular_rate_target = blind_turn_kp * yaw_error - blind_turn_kd * gyro_param.gyro_z;
                if(ctrl_state.angular_rate_target > 250.0f) ctrl_state.angular_rate_target = 250.0f;
                if(ctrl_state.angular_rate_target < -250.0f) ctrl_state.angular_rate_target = -250.0f;
                ctrl_state.base_speed = speed_curve;
            }
        }
    } else {
        blind_turn_finished = 0;
        Yaw_Start = yaw_plus;             // refresh anchor
        Start_Dist = Distance_Integral;   // refresh distance anchor
    }

    pid_gyro_middle.error = ctrl_state.angular_rate_target - ctrl_state.angular_rate_current;
    pid_gyro_middle.output = pid_gyro_middle.Kp * pid_gyro_middle.error +
                             pid_gyro_middle.Kd * (pid_gyro_middle.error - pid_gyro_middle.last_error);
    pid_gyro_middle.last_error = pid_gyro_middle.error;
    ctrl_state.speed_diff = pid_gyro_middle.output;

    ctrl_state.target_left_speed  = ctrl_state.base_speed - ctrl_state.speed_diff;
    ctrl_state.target_right_speed = ctrl_state.base_speed + ctrl_state.speed_diff;

    // ???? Actual_Speed ??????????????? motor.c ????? extern
    ctrl_state.current_left_speed = Actual_Speed[0];
    ctrl_state.current_right_speed = Actual_Speed[1];

    ctrl_state.output_left_pwm  = (int16_t)Calc_Incremental_PI(&pid_speed_L, ctrl_state.target_left_speed, ctrl_state.current_left_speed);
    ctrl_state.output_right_pwm = (int16_t)Calc_Incremental_PI(&pid_speed_R, ctrl_state.target_right_speed, ctrl_state.current_right_speed);

    if (system_running == 1) {
        Motor_Control(ctrl_state.output_left_pwm, ctrl_state.output_right_pwm);
    } else {
        Motor_Control(0, 0);
    }
}

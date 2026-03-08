#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"

extern int16_t speed_straight_l;
extern int16_t speed_straight_s;
extern int16_t speed_curve;

extern float turn_kp_base;
extern float turn_kp_var;
extern float turn_kd;

extern float gyro_kp;
extern float gyro_kd;
extern float blind_turn_kp;
extern float blind_turn_kd;

extern float speed_kp;
extern float speed_ki;
extern float speed_kff;
extern float planned_speed;

typedef struct {
    float Kp;
    float Kd;
    float error;
    float last_error;
    float output;
} Positional_PD_t;

typedef struct {
    float Kp;
    float Ki;
    float error;
    float last_error;
    float output;
    float out_max;
    float out_min;
} Incremental_PI_t;

typedef struct {
    float camera_error;
    float angular_rate_target;
    float angular_rate_current;
    float speed_diff;
    int16_t base_speed;
    float target_left_speed;
    float target_right_speed;
    float current_left_speed;
    float current_right_speed;
    int16_t output_left_pwm;
    int16_t output_right_pwm;
} System_Control_State_t;

extern System_Control_State_t ctrl_state;

void PID_Init(void);
void Control_Loop(void);
float Low_Pass_Filter(float current_val, float last_val, float alpha);
float Calc_Incremental_PI(Incremental_PI_t *pid, float target, float current);

#endif

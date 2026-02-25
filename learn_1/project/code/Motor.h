#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR_DIR1			P10_2
#define MOTOR_DIR2			P09_0
#define MOTOR_PWM1			TCPWM_CH31_P10_3
#define MOTOR_PWM2			TCPWM_CH25_P09_1


extern int16 except_speed,except_speed_base;


extern float posi_KP,posi_KD,posi_KD_2;


extern int16 powerr;//右电机
extern int16 powerl;//右电机
extern int16 fan_duty;
extern int16 core_out;              //角速度环输出
extern float ti_lf, ti_lb,ti_rf,ti_rb;                       // 左前电机积分和微分系数
extern int16 speed_ring;
extern float k_ratio;//左右差速系数//0.3   
extern int16 speed_ring;
extern uint8 run_test_enable;





//////////////////直角///////////////////
extern float posi_KP_RIGHT;             //偏差环P
extern float posi_KD_RIGHT;             //偏差环D
//////////////////直角///////////////////


//////////////////圆环///////////////////
extern float posi_KP_RING;             //偏差环P
extern float posi_KD_RING;             //偏差环D
//////////////////圆环///////////////////


//////////////////直道///////////////////
extern float posi_KP_STRAIGHT;             //偏差环P
extern float posi_KD_STRAIGHT;             //偏差环D
//////////////////直道///////////////////

//////////////////正常///////////////////
extern float posi_KP_NORMAL;             //偏差环P
extern float posi_KD_NORMAL;             //偏差环D
//////////////////正常///////////////////








void motor_init( );
void set_speed(int powerl,int powerr); 
void motor_ctr( );
void pid_processing( );
void emergency_check(void);
void motor_ctr(void);
void fan_ctr(void);


void       gyro_pid();      //角速度环
void       speed_pid();     //速度环   
void       position_pid();  //转向环
void       motor_ctr();     //电机最终输出


#endif
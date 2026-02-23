#ifndef _pit_h_
#define _pit_h_

//*********************************************宏定义*****************************************************//
typedef struct PID
{
  
  float Kp;
  float Ki;
  float Kd;

  float LowPass;        //低通滤波
  float Out_P;
  float Out_I;
  float Out_D;

  float PrevError;      //上上次误差
  float LastError;      //上次误差
  float Error;          //本次误差        
  float LastData;
  float Special_error;
  


}PID;

/**
在宏定义中初始化变量要在末尾加上\，以保证下一行是当前宏的延续，叫做行继续符
并且\后面不能有任何字符，包括空格也不可以
**/
#define PID_CREATE(_kp, _ki, _kd, _lowpass) { \
    .Kp = _kp,       \
    .Ki = _ki,       \
    .Kd = _kd,       \
    .LowPass = _lowpass, \
    .Out_P = 0,      \
    .Out_I = 0,      \
    .Out_D = 0,      \
}
//*****************************************变量声明****************************************************//
extern PID Servo_pid;                                    //声明舵机PID
extern PID Servo_pid_open_road;
extern PID l_motor_pid;                                  //声明左轮PID                                        
extern PID r_motor_pid;                                  //声明右轮PID



//*****************************************函数声明****************************************************//

float PID_Position(PID *PID, float NowData, float Point);       //舵机位置式PD控制
void dynamic_pid_value_set(void);                               //动态PID
float PID_Increase(PID *PID, float NowData, float Point);       //增量式PID
void section_pit(void);                                         //分段式pit        
#endif
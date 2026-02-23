#include "zf_common_headfile.h"


//*****************************************变量定义**********************************************//

//初始化舵机pid值
PID Servo_pid = PID_CREATE(1.1, 0,5.9 , 0.8);//3参数：KP、KI、KD、滤波系数

PID Servo_pid_roundabout = PID_CREATE(1.0, 0, 6.2, 0.8);         //环岛专用PID        

//舵机动态pid基础参数
float Kp_base = 0.3;
float Kd_base = 15;



//电机pid设置

PID l_motor_pid = PID_CREATE(18, 2.6, 0, 1);//参数：KP、KI、KD、滤波系数
PID r_motor_pid = PID_CREATE(21, 2.8, 0, 1);


//PID l_motor_pid = PID_CREATE(22, 2, 0, 1);//参数：KP、KI、KD、滤波系数
//PID r_motor_pid = PID_CREATE(22, 2, 0, 1);

//PID l_motor_pid = PID_CREATE(13, 3, 0, 1);//参数：KP、KI、KD、滤波系数
//PID r_motor_pid = PID_CREATE(23, 3, 0, 1);

//*****************************************函数定义**********************************************//

/**
函数：舵机位置式PID（PD控制）

**/

float PID_Position(PID *PID, float NowData, float Point)
{  
  
    PID->LastError = PID->Error; 
    PID->Error = Point - NowData;         //获取本次误差 
    PID->Out_P = PID->Error;
   
    PID->Out_D = (PID->Error -  PID->LastError) * PID->LowPass + Servo_pid.Out_D * (1 - PID->LowPass);//Out_D = 本次误差 - 上次误差并且附带简单的低通滤波，当前未加入滤波
//    PID->Out_D = PID->Error - PID->LastError;
    
    return (PID->Kp * PID->Out_P + PID->Kd * PID->Out_D); //返回位置式(KD控制)PID公式计算出来的值
  
}

/**

分段式pit

**/
void section_pit(void)
{
    if(fabsf(Servo_pid.Error) >= 55 )
    {
      Servo_pid.Kp = 0.7;

    }else
    {
      Servo_pid.Kp = 0.5;

    }
      
}

/**

动态pid参数赋值

**/
void dynamic_pid_value_set(void)
{
  //一次Kp
//  Servo_pid.Kp = Kp_base + abs(MID_W - mid_point) * 0.045f;      //最大值为2.05
  
  
  //二次Kp
  //Kp基础值 + 误差值的平方
  Servo_pid.Kp = Kp_base + (MID_W - Mid_point) * (MID_W - Mid_point) * 0.00025f;  //最大值为2.05
  
  Servo_pid.Ki = 0;
  Servo_pid.Kd = Kd_base;
  
}

/**

增量式PID

**/
float PID_Increase(PID *PID, float NowData, float Point)
{
    PID->Error = Point - NowData;               //获取本次误差
    
    PID->Out_P = (PID->Error - PID->LastError); //Out_P = 本次误差 - 上次误差
    PID->Out_I = PID->Error;                    //本次误差
    
    PID->Out_D = (PID->Error - 2 * PID->LastError + PID->PrevError);//本次误差 - 2倍的上次误差 + 上上次误差
    
    PID->PrevError = 0.9f * PID->LastError + 0.1f * PID->PrevError;       //上上次误差 = 0.9倍的上次误差 + 0.1倍的上上次误差
    PID->LastError = 0.9f * PID->Error + 0.1f * PID->LastError;           //上次误差 = 0.9倍数的本次误差 + 0.1倍的上次误差        
    PID->LastData = NowData;    //上次实际值 = 本次实际值
    
    return (PID->Kp * PID->Out_P + PID->Ki * PID->Out_I + PID->Kd * PID->Out_D);

}

#include "zf_common_headfile.h"



/**
函数：舵机开环控制
说明：MID_W - mid，图像中值减去找到的白线中值
      若大于0，则表示左偏，舵机-temp用来右偏
      若小于0，则表示右偏，舵机+temp(舵机-(-temp))用来左偏

**/
void turn (float mid,float k)
{
  int32 temp;
  float_t error = MID_W - mid;      //误差值 = 画面中间 - 实际中线
  if(fabsf(error) < 5)              //如果误差绝对值小于5像素，则视为误差值为0
  {
    error = 0;
  }
  
  temp = (int32) (k * error);        //误差乘以比例系数        
  servo_pwm_control(SERVO_PWM_MID + temp);      //舵机中值加上赋值后的误差
                                                    //若误差值大于0，说明往左偏，SERVO_PWM_MID + |temp| 舵机往右转
                                                    //若误差值小于0，说明往右偏，SERVO_PWM_MID - |temp| 舵机往左转
}

/**
函数：舵机闭环控制
说明：
      
**/
void servo(float mid)
{

  float servo_value;
  int32 temp1;
  int32 out;
  servo_value = PID_Position(&Servo_pid, MID_W - mid, 0);       //舵机位置式PID计算出的输出值

  temp1 = (int32)Limit_float(-turn_left_limit,servo_value, turn_right_limit);//舵机中值减去舵机PID的输出值并且加入限幅，避免舵机堵死
  if(abs(temp1) < 5)
  {
    temp1 = 0;
  }
  
  out = SERVO_PWM_MID - temp1;

  servo_pwm_control(out);     //将限幅之后的PWM来控制舵机

}

void servo_open_road(float target,float actual)
{
    float servo_value;
    int32 temp1;
    int32 out;
    servo_value = PID_Position(&Servo_pid_open_road, target, actual);       //舵机位置式PID计算出的输出值

    temp1 = (int32)Limit_float(-turn_left_limit,servo_value, turn_right_limit);//舵机中值减去舵机PID的输出值并且加入限幅，避免舵机堵死

    
    out = SERVO_PWM_MID - temp1;

    servo_pwm_control(out);     //将限幅之后的PWM来控制舵机



}

/**
函数：电机闭环控制(增量式PID)
说明：
      
**/

void Motor_Control(int32 Speed_L, int32 Speed_R)
{
  L_motor.target_speed = Speed_L;
  R_motor.target_speed = Speed_R;
  
//  float current_speed_avg = (L_motor.encoder_speed + R_motor.encoder_speed) / 2.0;
  
  L_motor.duty = Limit_int32(-pid_limit, L_motor.duty + (int32)PID_Increase(&l_motor_pid, (float)L_motor.encoder_speed ,(float)L_motor.target_speed), pid_limit);
  R_motor.duty = Limit_int32(-pid_limit, R_motor.duty + (int32)PID_Increase(&r_motor_pid, (float)R_motor.encoder_speed , (float)R_motor.target_speed), pid_limit);
  
  
  L_motor_set_speed(L_motor.duty);
  R_motor_set_speed(R_motor.duty);


}

//void Motor_Control(int32 Speed_L, int32 Speed_R)
//{
//  L_motor.target_speed = Speed_L;
//  R_motor.target_speed = Speed_R;
//  
//  L_motor.duty = Limit_int32(-pid_limit, L_motor.duty + (int32)PID_Increase(&l_motor_pid, (float)L_motor.encoder_speed, (float)L_motor.target_speed), pid_limit);
//  R_motor.duty = Limit_int32(-pid_limit, R_motor.duty + (int32)PID_Increase(&r_motor_pid, (float)R_motor.encoder_speed, (float)R_motor.target_speed), pid_limit);
//  
//  L_motor_set_speed(L_motor.duty);
//  R_motor_set_speed(R_motor.duty);
//
//                
//}



/**
函数：电机差速控制
说明：
      
**/

void Final_motor_control(int32 speed, float k)
{
  
  Motor_Control(Limit_int32( min_speed, (int32)(speed + (k * (PID_Position(&Servo_pid, MID_W - Mid_point, 0)))), max_speed), 
                Limit_int32( min_speed, (int32)(speed - (k * (PID_Position(&Servo_pid, MID_W - Mid_point, 0)))), max_speed));

}
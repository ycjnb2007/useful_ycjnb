#ifndef _control_h
#define _control_h

#define turn_left_limit   60            //向左舵机限幅
#define turn_right_limit  60            //向右舵机限幅

#define turn_servo_roundabout_limit   50            //舵机在圆环内的限幅



#define pid_limit       6000            //后轮PWM限幅

#define min_speed       0
#define max_speed       180


void turn (float mid,float k);                                 //舵机开环控制
void servo(float mid);                                          //舵机闭环控制
void servo_control_roundabout(float mid);                       //舵机在六边形环岛处的控制
void servo_open_road(float target,float actual);                //舵机在断路区控制
void Motor_Control(int32 Speed_L, int32 Speed_R);               //电机闭环控制
void Final_motor_control(int32 speed, float k);    //电机差速控制
void servo_cut_off_control(float error);                        //舵机在断路区的控制
#endif
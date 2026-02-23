#include "Motor.h"


// 电机速度
#define  speed_lf   motor_value_left.receive_left_speed_data
#define  speed_lb   motor_value_left.receive_right_speed_data

#define  speed_rf   -motor_value_right.receive_left_speed_data
#define  speed_rb   -motor_value_right.receive_right_speed_data


int16 except_speed =1300;         //期望速度
int16 except_speed_base =1300;    //期望速度

float gyro_offset=0;            //期望角速度
float gyro_offset_last=0;       //上一个期望角速度

float posi_KP=7.5;             //偏差环P
float posi_KD=5.2;             //偏差环D
float posi_KD_2=0.00;          //偏差环D_2


int16 outer_out=0;             //偏差环输出

float gyro_KP=0.65;            //角速度环P
float gyro_KI=0.05;           //角速度环I
float gyro_KD=0.00;           //角速度环D
float Integral=0.05;             //积分量
int16 core_out=0;             //角速度环输出


// 无刷电机速度环变量定义
// 左前电机速度环
int16 ex_speed_lf = 0;
int16 er1_speed_lf = 0;
int16 er2_speed_lf = 0;
int16 er3_speed_lf = 0;

// 左后电机速度环
int16 ex_speed_lb = 0;
int16 er1_speed_lb = 0;
int16 er2_speed_lb = 0;
int16 er3_speed_lb = 0;

// 右前电机速度环
int16 ex_speed_rf = 0;
int16 er1_speed_rf = 0;
int16 er2_speed_rf = 0;
int16 er3_speed_rf = 0;

// 右后电机速度环
int16 ex_speed_rb = 0;
int16 er1_speed_rb = 0;
int16 er2_speed_rb = 0;
int16 er3_speed_rb = 0;

// 速度环PID参数
int16 kp_lf=7, kp_lb=7, kp_rf=7, kp_rb=7;  // 比例系数
float ti_lf=0.45, td_lf=0.2;                       // 左前电机积分和微分系数
float ti_lb=0.45, td_lb=0.2;                       // 左后电机积分和微分系数
float ti_rf=0.45, td_rf=0.2;                       // 右前电机积分和微分系数
float ti_rb=0.45, td_rb=0.2;                       // 右后电机积分和微分系数

int16 speed_out_lf=0;            //左前电机速度环输出
int16 speed_out_lb=0;            //左后电机速度环输出
int16 speed_out_rf=0;            //右前电机速度环输出
int16 speed_out_rb=0;            //右后电机速度环输出

int16 powerl_left_up=0;                 //最终左侧前电机输出
int16 powerl_left_down=0;               //最终左侧后电机输出
int16 powerl_right_up=0;                //最终右侧前电机输出
int16 powerl_right_down=0;              //最终右侧后电机输出

int16 test1=0,test2=0;

static float filtered_position = 0;

void position_pid(void) //位置环
{
      outer_out=(int16)(posi_KP*(mtv9_position)+posi_KD*(mtv9_position-mtv9_position_old)*func_abs((mtv9_position-mtv9_position_old))-posi_KD_2*imu660ra_gyro_z/10); //使用偏差计算一个目标角速度
}

uint8 cnt=2;            //陀螺仪角速度值增益
void gyro_pid(void)    //角速度环
{
      gyro_offset=outer_out-imu660ra_gyro_z/cnt;//   //期望角速度加上此时陀螺仪采集到的角速度作为内环的输入  ******极性对不对？
      
      // 计算PID输出（先不包含积分项）
      float pid_output = gyro_KP*gyro_offset+gyro_KD*(gyro_offset-gyro_offset_last);
      
      // 积分抗饱和：只有当输出未饱和时才累加积分项
      if(pid_output < 1400 && pid_output > -1400) 
      {
          Integral+=gyro_offset;         // 积分项累加
      }
      
      // 积分限幅，防止积分饱和
      if(Integral > 600) Integral = 600;
      if(Integral < -600) Integral = -600;
      
      // 最终输出包含积分项
      core_out = (int16)(pid_output + gyro_KI*Integral);
      gyro_offset_last=gyro_offset;
      core_out = (int16)func_limit(core_out,4200);
}

int16 power_turn_l = 0;
int16 power_turn_r = 0;
float k_ratio=0.30;//左右差速系数//0.3   
float front_rear_ratio = 1.00f; // 前后电机差速比系数// 前电机比后电机快10%


float base = 0.30;     // 最小窗口
float max = 0.80;      // 最大窗口
int16 step = 1000;

void speed_pid(void)   //速度环
{
  
    int16 power_turn=core_out/2;  
/////////////////////////////////////    
//     power_turn=0;
//     power_turn = (int16)((position_p*mtv9_position) - (position_d *imu660ra_gyro_z/10 ));  //imu963ra_gyro_y
//     power_turn = (int16)func_limit(power_turn,1000);
//////////////////////////////////////  
    
    power_turn = (int16)func_limit(power_turn,2100);
      
/////////////////////////////// 动态差速系数/////////////////////////////// 

    k_ratio = k_ratio + (speed-1100) /step+func_abs(mtv9_position/4200); // 速度每加100，k增加0.08
    k_ratio=func_limit_ab(k_ratio,base,max);
/////////////////////////////// 动态差速系数/////////////////////////////// 
    
    // 计算差速
    if(power_turn>0) 
    {
        power_turn_r=(int16)(power_turn);
        power_turn_l=(int16)(power_turn*(1+k_ratio));
    }
    else
    {
        power_turn_l=(int16)(power_turn);
        power_turn_r=(int16)(power_turn*(1+k_ratio));
    }

    // 左前电机速度环
    ex_speed_lf = (int16)(except_speed  - power_turn_l* front_rear_ratio);
    ex_speed_lf = (int16)func_limit_ab(ex_speed_lf,-1000,2800);
    er3_speed_lf = er2_speed_lf;
    er2_speed_lf = er1_speed_lf;
    er1_speed_lf = ex_speed_lf -speed_lf;
    speed_out_lf += kp_lf*(er1_speed_lf - er2_speed_lf) + ti_lf*er1_speed_lf + td_lf*(er1_speed_lf - 2*er2_speed_lf + er3_speed_lf);

    // 左后电机速度环
    ex_speed_lb = except_speed - power_turn_l;
    ex_speed_lb = (int16)func_limit_ab(ex_speed_lb,-1000,2800);
    er3_speed_lb = er2_speed_lb;
    er2_speed_lb = er1_speed_lb;
    er1_speed_lb = ex_speed_lb - speed_lb;
    speed_out_lb += kp_lb*(er1_speed_lb - er2_speed_lb) + ti_lb*er1_speed_lb + td_lb*(er1_speed_lb - 2*er2_speed_lb + er3_speed_lb);
    
    if((ex_speed_lf>0&&ex_speed_lb<0)||(ex_speed_lf<0&&ex_speed_lb>0))//防止由于前后轮差速系数导致前后轮一个正转一个反转
    {
      speed_out_lf=0;
      speed_out_lb=0;
    }
    
    // 右前电机速度环
    ex_speed_rf = (int16)(except_speed  + power_turn_r* front_rear_ratio);
    ex_speed_rf = (int16)func_limit_ab(ex_speed_rf,-1000,2800);
    er3_speed_rf = er2_speed_rf;
    er2_speed_rf = er1_speed_rf;
    er1_speed_rf = ex_speed_rf - speed_rf;
    speed_out_rf += kp_rf*(er1_speed_rf - er2_speed_rf) + ti_rf*er1_speed_rf + td_rf*(er1_speed_rf - 2*er2_speed_rf + er3_speed_rf);
    
    // 右后电机速度环
    ex_speed_rb = except_speed + power_turn_r;
    ex_speed_rb = (int16)func_limit_ab(ex_speed_rb,-1000,2800);
    er3_speed_rb = er2_speed_rb;
    er2_speed_rb = er1_speed_rb;
    er1_speed_rb = ex_speed_rb - speed_rb;
    speed_out_rb += kp_rb*(er1_speed_rb - er2_speed_rb) + ti_rb*er1_speed_rb + td_rb*(er1_speed_rb - 2*er2_speed_rb + er3_speed_rb);
    
    if((ex_speed_rf>0&&ex_speed_rb<0)||(ex_speed_rf<0&&ex_speed_rb>0))//防止由于前后轮差速系数导致前后轮一个正转一个反转
    {
      speed_out_rf=0;
      speed_out_rb=0;
    }
    // 限制输出范围
    speed_out_lf = (int16)func_limit(speed_out_lf,4500);
    speed_out_lb = (int16)func_limit(speed_out_lb,4500);
    speed_out_rf = (int16)func_limit(speed_out_rf,4500);
    speed_out_rb = (int16)func_limit(speed_out_rb,4500);
}

void motor_ctr(void)
{
    // 计算最终输出
    powerl_left_up = -speed_out_lf + test1;
    powerl_left_down = speed_out_lb + test1;
    powerl_right_up = speed_out_rf + test2;
    powerl_right_down = -speed_out_rb + test2;
    
    // 限制输出范围
    powerl_left_up = func_limit(powerl_left_up,4500);
    powerl_left_down = func_limit(powerl_left_down,4500);
    powerl_right_up = func_limit(powerl_right_up,4500);
    powerl_right_down = func_limit(powerl_right_down,4500);
    
    // 输出到电机
    small_driver_set_duty_left(powerl_left_up,powerl_left_down);
    small_driver_set_duty_right(powerl_right_up,powerl_right_down);
}


uint8 run_test_enable=0;
int32 run_test_distance=0;
int32 run_test_distance_limit=7000;
void run_test(void)
{
    if(run_test_enable)
    {
      run_test_distance+=speed/100;
      if(run_test_distance>run_test_distance_limit)
      {
        run_test_distance=0;
        finish_flag=2;
      }
    }
}


#define SPEED_BIG     -100   // 大环减速
#define SPEED_MEDIUM  -100   // 中环减速
#define SPEED_SMALL   -100   // 小环减速
int16 speed_ring=50;


//////////////////直角///////////////////
float posi_KP_RIGHT=9.1;             //偏差环P
float posi_KD_RIGHT=7.4;             //偏差环D
//////////////////直角///////////////////


//////////////////圆环///////////////////
float posi_KP_RING=8.5;             //偏差环P
float posi_KD_RING=6.8;             //偏差环D
//////////////////圆环///////////////////


//////////////////直道///////////////////
float posi_KP_STRAIGHT=8.2;             //偏差环P
float posi_KD_STRAIGHT=6.5;             //偏差环D
//////////////////直道///////////////////


//////////////////正常///////////////////
float posi_KP_NORMAL=8.8;             //偏差环P
float posi_KD_NORMAL=7.2;             //偏差环D
//////////////////正常///////////////////


void Speed_decision(void)               //速度决策
{
    // 根据不同的元素设置目标速度
    if(island_state!=ISLAND_NONE)         //小环
    {
        posi_KP  = posi_KP_RING;             //偏差环P
        posi_KD  = posi_KD_RING;             //偏差环D
        posi_KD_2 = 0.00;          //偏差环D_2
        except_speed = except_speed_base + speed_ring;         //期望速度
    }
//    else if(current_circle_type==RING_MEDIUM)   //中环
//    {
//        posi_KP  = 8.5;             //偏差环P
//        posi_KD  = 5.8;             //偏差环D
//        posi_KD_2 = 0.0;            //偏差环D_2
//        except_speed = except_speed_base - SPEED_MEDIUM;         //期望速度
//    }
//    else if(current_circle_type==RING_BIG)      //大环   
//    {
//        posi_KP  = 8.5;             //偏差环P
//        posi_KD  = 5.8;             //偏差环D
//        posi_KD_2 = 0.00;            //偏差环D_2
//        except_speed = except_speed_base - SPEED_BIG;   //期望速度
//    }
    else if(!(right_rightangle_flag==0&&left_rightangle_flag==0))  //直角
    {
        posi_KP  = posi_KP_RIGHT;             //偏差环P
        posi_KD  = posi_KD_RIGHT;             //偏差环D
        posi_KD_2 =0.00;            //偏差环D_2
        float raw_position = abs(mtv9_position / 4);//+func_abs((except_speed-1100)/2)
        raw_position = (int16)func_limit(raw_position,100);
        float alpha = 0.50f; // 滤波系数
        filtered_position = alpha * raw_position + (1 - alpha) * filtered_position;
        except_speed = except_speed_base - (int16)filtered_position; //期望速度
    }
    else if(straight_flag)               //直道
    {
        posi_KP  = posi_KP_STRAIGHT;             //偏差环P
        posi_KD  = posi_KD_STRAIGHT;             //偏差环D
        posi_KD_2 = 0.0;            //偏差环D_2
        except_speed = except_speed_base+300;     
    }
    else              //正常
    {
        posi_KP  = posi_KP_NORMAL;             //偏差环P
        posi_KD  = posi_KD_NORMAL;             //偏差环D
        posi_KD_2 = 0.0;            //偏差环D_2
        float raw_position = abs(mtv9_position / 4);//+func_abs((except_speed-1100)/2)
        raw_position = (int16)func_limit(raw_position,100);
        float alpha = 0.50f; // 滤波系数
        filtered_position = alpha * raw_position + (1 - alpha) * filtered_position;
        except_speed = except_speed_base - (int16)filtered_position; //期望速度
    }
    
}

int16 fan_duty=6000;
void fan_ctr(void)
{
      if(fan_enable)
      {
          small_driver_set_duty_fan(fan_duty, fan_duty);    // 负压风扇设置占空比
      }
}


void pid_processing(void)
{
    if(replay_mode)                                      //轨迹回放模式
    {
        outer_out = (trajectory_replay_control())*200;
        gyro_KP=0.50;
    }
    else                                                    //正常巡线
    {
        position_pid();                                     //将偏差环的输出输入角速度环
        gyro_KP=0.70;
    }
    Speed_decision();       //速度决策
    gyro_pid();             //角速度环
    speed_pid();            //速度环
    motor_ctr();            //电机输出
    run_test();             //测试
}



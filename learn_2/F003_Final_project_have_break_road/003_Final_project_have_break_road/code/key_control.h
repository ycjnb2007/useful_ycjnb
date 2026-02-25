#ifndef _Key_control_h
#define _key_control_h

extern uint8 exchange_show_page;                //屏幕显示切换
extern uint8 exchange_choose_option;            //充当选项选择



extern float differential_k;    //差速系数
extern float differential_k_normal;
extern float differential_k_roundabout;
extern int normol_speed;      //编码器速度
extern uint8 start_motor;
extern uint8 start_servo;

void car_start(void);
void exchange_show_page_control(void);






/*******************************断路UI信息*******************************/

/**********调试断路区域**********/
extern uint8 area_memery;                                   //断路区的采取区域选择（1-3）
extern uint8 write_flag;                                    //写入标志位
extern uint8 write_flag_finish1;
extern uint8 write_flag_finish2;
extern uint8 write_flag_finish3;
extern uint16 servo_angle_break_road;


//断路区的按键控制
void Key_control_break_road(void);
//断路区写入flash的按键控制
void Write_break_road_angle(void);
//断路写入前的舵机查看
void Break_road_servo_show(void);
/*******************************断路UI信息*******************************/




#endif
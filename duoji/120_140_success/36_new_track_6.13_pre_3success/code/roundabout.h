#ifndef _roundabout_h
#define _roundabout_h

#define GUSS_MID_NUM    2

typedef enum {
    NORMAL_TRACK,       // 正常巡线                 0
    ENTER_ROUNDABOUT,   // 进入环岛                 1
    IN_RoundABOUT,      // 环岛内                   2
    EXIT_ROUNDABOUT1,   // 退出环岛状态1            3
    EXIT_ROUNDABOUT2   //退出环岛状态2              4    
      
}LoopState;

extern LoopState roundabout_current_state;

extern uint8 roundabout_dir;    //环岛方向

//环岛处理
extern uint8 X_roundabout_start;
extern uint8 Y_roundabout_start;   
extern uint8 X_roundabout_end;   
extern uint8 Y_roundabout_end;  


extern uint8 Pre_turn_roundabout_flag;

uint16 Loop_tarck();

//锁存左右环岛识别
void Update_Roundabout_Dir(uint8 new_dir);
//环岛识别
uint8 Detect_roundabout();
//环内正常寻迹
uint16 In_roundabout();

//左侧入环处理
uint16 Left_enter_roundabout();
//左侧出环处理1
uint16 Left_exit_roundabout1();
////左侧出环处理2
uint16 Left_exit_roundabout2();

//右侧入环处理
uint16 Right_enter_roundabout();
//右侧出环处理1
uint16 Right_exit_roundabout1();
////右侧出环处理2
uint16 Right_exit_roundabout2();


void Pre_turn_flag_update(uint8 new_flag);
uint8 Pre_roundabout_turn();
uint8 Pre_left_roundabout_state();
uint8 Pre_right_roundabout_state();

#endif
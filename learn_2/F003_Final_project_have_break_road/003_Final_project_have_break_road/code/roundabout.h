#ifndef _roundabout_h
#define _roundabout_h

/*********************************圆环寻迹状态*********************************/
typedef enum {
    NORMAL_TRACK,       // 正常巡线                 0
    ENTER_ROUNDABOUT,   // 进入环岛                 1
    IN_RoundABOUT,      // 环岛内                   2
    EXIT_ROUNDABOUT1,   // 退出环岛状态1            3
    EXIT_ROUNDABOUT2   //退出环岛状态2              4    
      
}LoopState;
extern LoopState roundabout_current_state;      //圆环寻迹状态标志位
extern uint8 roundabout_dir;                    //环岛方向
/*********************************圆环寻迹状态*********************************/


/*********************************圆环寻迹起始点和终止点*********************************/
//环岛处理
extern uint8 X_roundabout_start;
extern uint8 Y_roundabout_start;   
extern uint8 X_roundabout_end;   
extern uint8 Y_roundabout_end;  
/*********************************圆环寻迹起始点和终止点*********************************/


/*********************************预入圆环的处理*********************************/
extern uint8 Pre_turn_roundabout_flag;                  //预入圆环方向          
void Pre_turn_flag_update(uint8 new_flag);              //预入圆环的状态锁存
uint8 Pre_roundabout_turn();                            //欲入环识别
uint8 Pre_left_roundabout_state();                      //左侧预入环处理
uint8 Pre_right_roundabout_state();                     //右侧预入环处理
/*********************************预入圆环的处理*********************************/


/*********************************圆环寻迹函数声明*********************************/
//圆环总处理
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
/*********************************圆环寻迹函数声明*********************************/


#endif
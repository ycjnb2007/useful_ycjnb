#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_
#include "zf_common_headfile.h"

#define  MAX_TRAJECTORY_POINTS          510       //每页最大存储点数
#define  MAX_BREAK_NUMBER               3         //最大断路数

extern uint8_t  current_replay_page;               //当前储存页
extern uint8_t  current_break_index;               //当前回放断路区编号
extern float    target_yaw;
extern uint16   record_indexreplay_index;      
extern uint8    stop_record_flag;
extern uint8    stop_replay_flag;                        //停止回放标志
extern uint16 record_index ;                         //记录索引

// 多断路区轨迹数组
extern float    yaw_record1[MAX_TRAJECTORY_POINTS];     //第1断路区
extern float    yaw_record2[MAX_TRAJECTORY_POINTS];     //第2断路区
extern float    yaw_record3[MAX_TRAJECTORY_POINTS];     //第3断路区
extern uint16_t yaw_record_count[MAX_BREAK_NUMBER];     //每个断路区的轨迹点数

extern int32 total_pulses;
extern uint8_t break_number;
extern uint8 currnt_recoder;                         //当前记录轨迹的个数



float angle_difference(float current, float target);
void trajectory_init(void);                    //轨迹初始化函数
void trajectory_processing(void);
int16 trajectory_replay_control(void);
#endif
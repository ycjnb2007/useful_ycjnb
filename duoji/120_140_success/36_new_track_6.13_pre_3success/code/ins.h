#ifndef _ins_h_
#define _ins_h_

#define ROAD_LOST_THRESHOLD     5



extern uint8 wuxiao_start_point_count;
extern bool last_frame_has_line;
extern bool current_has_line;


// 添加状态机相关定义
typedef enum 
{
    NAV_NORMAL,      // 正常巡线状态
    NAV_ENTERING,    // 正在进入断路区（防抖中）
    NAV_IN_CUTOFF,   // 已进入断路区
    NAV_EXITING      // 正在退出断路区（防抖中）
} NavigationState;

extern NavigationState nav_state;


#endif
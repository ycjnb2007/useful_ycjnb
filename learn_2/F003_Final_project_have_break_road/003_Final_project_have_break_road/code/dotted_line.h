#ifndef _dotted_line_h_
#define _dotted_line_h_


typedef enum
{
    link_up,
    link_left,
    link_right,
    link_up_roundabout_left,
    link_up_roundabout_right,
    
}Normal_track_mood;

extern Normal_track_mood normal_track_mood;

extern uint8 X_dotted_end;                    //虚线连接线的终点横坐标
extern uint8 Y_dotted_end;                    //虚线连接线的终点纵坐标

void dotted_link_state_solution(void);

#endif
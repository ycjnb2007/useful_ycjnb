#ifndef _progess_h_
#define _progess_h_


#define COLOR_NORMAL                    RGB565_RED
#define PRE_COLOR_ROUNDABOUT            RGB565_GREEN
#define COLOR_ROUNDABOUT                RGB565_BLUE
#define DOTTED_LINE_THRESHOLD           70


//#define ENABLE_BREAK     // 断路检测
//#define ENABLE_CROSS     //十字检测        
#define ENABLE_PRE_LOOP


typedef enum
{
    LINE_TRACKING,      //正常寻迹
    LOST_LINE,          //丢线(断路区)
    ROUNDABOUT,         //六边形环岛
    CROSS


}TrackingState;

extern TrackingState Trackstate;
extern bool break_road_initialized;


/******************************函数声明*******************************/     
//寻迹状态的确定
TrackingState determine_tracking_state();
//不同状态的解决方法
void line_tracking_state_machine();
//正常寻迹
void Line_track_solution();
//环岛寻迹
void roundabout_solution();
//断路寻迹
void open_road_solution();
//十字处理
void cross_solution();



#endif
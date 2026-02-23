#ifndef _IMAGE_PROCESSING_H_
#define _IMAGE_PROCESSING_H_

#include "zf_common_headfile.h"

#define IMAGE_CUT_LEFT      0        //二值化左裁剪大小
#define IMAGE_CUT_RIGHT     0        //二值化右裁剪大小
#define IMAGE_CUT_UP        5        //二值化上裁剪大小
#define IMAGE_CUT_DOWN      35        //二值化下裁剪大小


#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


#define IMAGE_H             (MT9V03X_H-IMAGE_CUT_UP-IMAGE_CUT_DOWN)
#define IMAGE_W             (MT9V03X_W-IMAGE_CUT_LEFT-IMAGE_CUT_RIGHT)
#define MIDLINE             (IMAGE_W/2)         //中线   
#define SEARCH_MIN               2              // 最小搜索列
#define SEARCH_MAX               (IMAGE_W-2)    // 最大搜索列
#define white                    255   // 白点值 
#define black                    0     // 黑点值
#define SEARCH_WINDOW            45    // 搜索窗口半径
#define MAX_BACKTRACK_DEPTH      35    // 最大回溯行数
#define MAX_INTERPOLATE_GAP      30    // 最大允许插值的行数间隔
#define JUMP                     2     //横向跳跃步长
#define cross_max_white_value    80    //十字最大白点阈值
#define BORDER_OFFSET            8     //补偿边界偏移值
#define ROW_DELTA_THRESHOLD      15    //行间差异阈值


// 边界状态机
 typedef enum 
{
    BORDER_VALID,              // 有效边界
    BORDER_NOT_FOUND,          // 未找到边界
    BORDER_FRAME_DIFF,         // 帧间差异过大  感觉没什么用
    BORDER_ROW_DIFF,           // 行间差异过大
    BORDER_INTERPOLATED,       // 插值边界
    BORDER_COMPENSATED,        // 补偿边界

} BorderStatus;
extern BorderStatus l_status[IMAGE_H], r_status[IMAGE_H];

//环岛状态机
typedef enum 
{
    ISLAND_NONE,               //无环岛
    ISLAND_LEFT,               //左环岛
    ISLAND_RIGHT,              //右环岛
    ISLAND_IN_PROGRESS,        // 环岛处理中
    ISLAND_LEAVE,              //准备出环
    ISLAND_POST_PROCESS        // 环后处理状态,避免出环后车身摇晃
} IslandState;
extern IslandState island_state;



// 断路状态机
typedef enum 
{
    BREAK_NONE,       // 未检测到断路
    BREAK_DETECTED,   // 检测到断路
    BREAK_RECOVERING  // 正在复现轨迹
} BreakState;

extern BreakState break_state;


// 环岛大小定义
typedef enum 
{ 
      RING_BIG,
      RING_MEDIUM,
      RING_SMALL ,
      RING_NONE
}  RingType;
extern RingType current_circle_type;



extern int16  l_border[IMAGE_H];        //左边界数组
extern int16  r_border[IMAGE_H];        //右边界数组
extern int16  border[IMAGE_H];          //中线数组

extern int16  mtv9_position, mtv9_position_old;      //偏差，上一次偏差
extern uint8  Thresholding_image[IMAGE_H][IMAGE_W];  //二值化图像数组
extern uint16 White_statist[IMAGE_H];      // 存储每行白点数的数组
extern uint16 max_white_value;             // 最大白点值
extern uint8  max_white_row;               // 白点最多的行号
extern int8   rightangle_row;              //找到的直角所在行    
extern uint8  right_rightangle_flag;       //右直角标志位
extern uint8  left_rightangle_flag;        //左直角标志位
extern int16  cross_row;                   //十字标志位
extern uint16  white_per_row[IMAGE_H];     //白点储存数组
extern uint8  start_angle_flag;            //开始角度积分标志位
extern uint8  finish_flag;                 // 终点标志
extern uint8  straight_flag;               //直角标志

extern uint16 tings;                       //中断计时变量1
extern uint16 ting;                        //中断计时变量2
extern uint16 ting_check_break;            //中断计时变量
extern uint16 ting_circle;                 //中断计时变量

extern uint16 max_white_value_upper;     // 上半部分最大白点值
extern uint16 max_white_value_lower;     // 下半部分最大白点值
extern uint8 max_row_upper;              // 上半部分最大白点行号
extern uint8 max_row_lower;              // 下半部分最大白点行号


extern  uint8 check_break_start;        //断路检测开始标志
extern  uint8 break_flag;               //断路标志
extern  uint8  break_angle_test;


extern  uint8  otsu_limit;              //大津阈值下限
extern  uint8  right_white_found_threshold;             //判断直角白点的阈值

extern  uint8 right_qianzhan_base;
extern uint8 break_rest_threshold;      //断路恢复阈值
extern uint8 break_rest_enable;      //断路恢复巡线使能


void   detect_right_angle(void);                                //直角检测
void   search_border(void);
void   image_Thresholding(uint8 value);                         //二值化
void   calc_corner(uint8 start_row,uint8 end_row);
void   right_angle_deal(uint8 start_h);
void   cross_check(void);
void   validate_border(int16 i) ;
void   circle_check(void);
void   straight_detection_check(void);
void   island_deal(void);
void   image_processing(void);
//void   check_break_condition(void) 
uint8   my_adapt_threshold(uint8 *image, uint16 col, uint16 row);//大津





// 白点值存储相关定义
#define WHITE_VALUE_PAGE 10      // 使用第10页存储白点值
#define WHITE_VALUE_FLAG 0xAA55  // 存储验证标志位

// 白点值存储结构体
typedef struct 
{
    uint16_t flag;                             // 存储验证标志位
    uint16_t normal_white_values[IMAGE_H];     // 存储直道正常时的白点值
} WhiteValueStorage;

// 摄像头参数存储相关定义
#define CAMERA_PARAMS_PAGE 11      // 使用第11页存储摄像头参数
#define CAMERA_PARAMS_FLAG 0xAA56  // 存储验证标志位

// 摄像头参数存储结构体
typedef struct 
{
    uint32_t flag;        // 存储验证标志位
    uint32_t exp_time;    // 曝光时间
    uint32_t fps;         // 帧率
    uint8_t otsu_limit;   // 大津法阈值下限
    uint8_t break_number; // 最大断路数

} CameraParamsStorage;


extern uint16 normal_white_per_row[IMAGE_H];  // 直道正常时的白点值数组

// 白点值存储相关函数声明
void   save_white_values_to_flash(void);                        //保存白点值到Flash
uint8  load_white_values_from_flash(void);                      //从Flash读取白点值
void   save_current_white_values(void);     
// 摄像头参数存储相关函数声明
void save_camera_params_to_flash(void);                        //保存摄像头参数到Flash
uint8 load_camera_params_from_flash(void);                     //从Flash读取参数


#endif

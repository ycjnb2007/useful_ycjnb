#ifndef _IMAGE_H_
#define _IMAGE_H_
#include "zf_common_headfile.h"

//============================ 宏定义区域 ============================//
// 图像处理相关宏定义
#define LCDH 70   // 压缩后图像高度
#define LCDW 120  // 压缩后图像宽度
#define WHITE 255 // 白色像素值
#define BLACK 0   // 黑色像素值

#define huandaoerr 20

//============================ 数据结构定义 ============================//
// 左边界结构体
typedef struct LEFT_EDGE
{
    int16 row;
    int16 col;
    uint8 flag;
} LEFT_EDGE;

// 右边界结构体
typedef struct RIGHT_EDGE
{
    int16 row;
    int16 col;
    uint8 flag;
} RIGHT_EDGE;

// 图像特征标志位结构体
typedef struct
{
    int16 Bend_Road;     /*0 :无               1 :右弯道     2 :左弯道*/
    int16 straight_long; /*长直道标志位*/
    int16 straight_xie;
    int16 Cross;
    int16 break_road;
    int16 stop;
} ImageFlagtypedef;

// 中线相关的结构体定义
struct MID
{
    uint8 row;
    uint8 col;
    uint8 flag;
};

//============================ 全局变量声明 ============================//
// 赛道特征标志
extern ImageFlagtypedef ImageFlag;

// 图像数据
extern uint8 image_use[LCDH][LCDW]; // 压缩后图像
extern uint8 image[LCDH][LCDW];
extern uint8 img_threshold;

// 边界数据
extern LEFT_EDGE Left_line[150];   // 左边界数组
extern RIGHT_EDGE Right_line[150]; // 右边界数组
extern uint8 L_edge_count, R_edge_count;
extern uint8 L_start_y, L_start_x, R_start_y, R_start_x;

// 中线数据
extern uint8 Mid_line_count;
extern struct MID Mid_line[150]; // 中线结构体
extern uint8 M_last_count;       // 上一帧中线点的个数
extern uint8 M_line_count;       // 中线点的个数

// 赛道数据
extern uint8 Road_width;  // 道路宽度

// 控制参数
extern float Err;

// 其他结构体
extern uint8 lose_line;                     // 丢线计数
extern uint16 RightLose, LeftLose, AllLose; // 边界丢失统计
extern uint16 last_valid_road_width;        // 最近有效赛道宽度
extern uint8 Road_Wide[LCDH];               // 每行赛道宽度记录
extern uint8 MidPri;                        // 上一次的中线位置

extern uint8 L_End_WHITELine;
extern uint8 R_End_WHITELine;
extern uint8 line_min;
extern uint8 line_max;
extern uint8 Left_line_count;
extern uint8 Right_line_count;

// Guide_lines相关变量 - 四行检测系统
// 上区域四行检测变量 (第1,6,11,16行)
extern uint8 top_line1_num; // 第1行赛道边界数量
extern uint8 top_line2_num; // 第6行赛道边界数量
extern uint8 top_line3_num; // 第11行赛道边界数量
extern uint8 top_line4_num; // 第16行赛道边界数量
// 中间区域四行检测变量 (第25,30,35,40行)
extern uint8 middle_line1_num; // 第25行赛道边界数量
extern uint8 middle_line2_num; // 第30行赛道边界数量
extern uint8 middle_line3_num; // 第35行赛道边界数量
extern uint8 middle_line4_num; // 第40行赛道边界数量
// 下区域四行检测变量 (第69,64,59,54行)
extern uint8 bottom_line1_num; // 第69行赛道边界数量
extern uint8 bottom_line2_num; // 第64行赛道边界数量
extern uint8 bottom_line3_num; // 第59行赛道边界数量
extern uint8 bottom_line4_num; // 第54行赛道边界数量

extern uint8 top_left_white;      // 顶部左侧区域白点数
extern uint8 top_middle_white;    // 顶部中间区域白点数
extern uint8 top_right_white;     // 顶部右侧区域白点数
extern uint8 bottom_left_white;   // 底部左侧区域白点数
extern uint8 bottom_middle_white; // 底部中间区域白点数
extern uint8 bottom_right_white;  // 底部右侧区域白点数

extern uint8 outsthreshold;
// 权重数组声明
extern const uint8 Weight[LCDH];       // 默认权重数组
extern const uint8 Weight_Angle[LCDH]; // 直角专用权重数组

//============================ 函数声明区域 ============================//
// 图像处理基础函数
void Get_userimage(void);
int My_Adapt_Threshold(uint8 *image, uint16 width, uint16 height);
void set_image_twovalues(uint8 value);
void Image_Process(void);

// 边界处理函数
void draw_line(void);
void Searchline_Improved(void); // 搜索车道边界

// 中线处理函数
float Err_Sum(void);

// 环岛检测函数
void Count_White(void);
void Guide_lines(void);

#endif

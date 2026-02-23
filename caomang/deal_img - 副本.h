#ifndef CODE_DEAL_IMG_H_
#define CODE_DEAL_IMG_H_
#include "zf_common_headfile.h"
/*
预定前瞻10-30cm，左右各10cm的视野
*/


#define XM MT9V03X_W       //处理图像 像素X
#define YM MT9V03X_H        //处理图像 像素Y
#define XX XM-1             //处理图像 下标X
#define YY YM-1             //处理图像 下标Y

//八邻域处理两行黑框
#define Deal_Left   2             //八邻域预处理 最左侧下标
#define Deal_Right  XX-2          //八邻域预处理 最右侧下标
#define Deal_Bottom 5             //八邻域预处理 最底端下标
#define Deal_Top    YY-8          //八邻域预处理 最顶端下标
//记录丢失Lost_line的边界位置
#define Lost_Left   Deal_Left - 2
#define Lost_Right  Deal_Right +2
#define Lost_Bottom Deal_Bottom -2
#define Lost_Top    Deal_Top + 2


#define IMG_W  MT9V03X_W      //原始图像 个数列
#define IMG_H  MT9V03X_H      //原始图像 个数行
#define _IMG_W IMG_W-1    //原始图像 下标列
#define _IMG_H IMG_H-1    //原始图像 下标行

#define COL IMG_W    //原始图像 个数列
#define ROW IMG_H    //原始图像 个数行
#define _COL_IMG_W //原始图像 下标列
#define _ROW _IMG_H //原始图像 下标行


#define IMG_SIZE (IMG_W * IMG_H)    //图像尺寸
#define MAP_SIZE (YM * XM)  // 地图尺寸

//图像各个点对应情况
#define Black 0  //黑色
#define White 255  //白色
#define Control_line 1 //红色
#define Left_line 2  //绿色
#define Right_line 3  //蓝色
#define Lost_line 4  //紫色
#define Judge_line 5  //青色
#define Make_up 6    //粉色
#define LandR 7      //橙色
#define USE_num YM*4   //定义找点的数组成员个数

enum ImgFlag
{
    straightlineS,   //短直道
    straightlineL,   //长直道
    curve,       //折角
    annulus_l,      //左环岛
    annulus_r,      //右环岛
    crossroads,     //十字路口
    startline,      //停车线
    disappear,      //断路
};//图像标志位

typedef enum Elements_Mode
{
    NO,
    YES,
}Elements_Mode;


typedef enum Elements_Lock
{
    Empty_,
    Annulus_,
    Disappear_,
    Stopline_,
}Elements_Lock;


typedef enum Annulus_Mode
{
    Differential,
    Tracing,
}Annulus_Mode;

extern volatile Elements_Mode elements_Mode;
extern volatile Elements_Lock elements_Lock[10];
extern volatile Annulus_Mode Annulus_mode;

extern int elements_index;

extern enum ImgFlag IF;
extern enum ImgFlag IF_L;

extern uint8 stop_num;
extern uint8 stopline_flag;
extern uint8 Annulus_flag;
extern int Annulus_Curvature[5];
extern int Annulus_num;
extern int Curvature;

extern uint8_t nowThreshold;
extern int minGrayscale;
extern int maxGrayscale;
extern uint8_t minThreshold;
extern int minGray;
extern int maxGray;
extern uint16 EXP_TIME;
extern int UD_OFFSET_DEF;
extern uint8_t maxThreshold;
extern uint8 img_threshold_group[3];//划分阈值区域   上中下
extern uint8 close_Threshold;
extern uint8 far_Threshold;
extern uint8 mid_Threshold;
extern uint8 imgGray[IMG_H][IMG_W];
extern uint8 img2wifi[IMG_H][IMG_W];
extern uint8 imgOSTU[YM][XM];
extern uint8 imgTran[IMG_H][IMG_W];
extern uint8 white_num_row[YM];
extern uint8 white_num_col[XM];
extern uint8 mid_line[YM];
extern uint8 start_center;
extern uint8 start_center_x;
extern uint8 start_center_y;
extern uint8 end_center_x;
extern uint8 end_center_y;
extern uint8 start_point_l;
extern uint8 start_point_r;
extern int points_l[(uint16)USE_num][2];//左线
extern int points_r[(uint16)USE_num][2];//右线
extern float dir_r[(uint16)USE_num];//用来存储右边生长方向
extern float dir_l[(uint16)USE_num];//用来存储左边生长方向
extern uint32 r_data_statics;//统计右边
extern uint32 l_data_statics;//统计右边
extern uint8 hightest;//最高点
extern uint16 leftmost[2];
extern uint16 rightmost[2];
extern uint16 topmost[2];
extern uint8 dealimg_finish_flag;
extern uint8 disappear_flag;
extern int deal_runing;
extern int dealimg_time;
extern int disappear_num;
extern int Disappear_angle_L[3];
extern int Disappear_angle_R[3];
extern float Start_angle;
extern int disappear_total;
extern float Disappear_Length;
extern float Disappear_Zero;
extern float Disappear_Yaw;
extern int Disappear_Dir_L[3][3];
extern int Disappear_Dir_R[3][3];
typedef enum RUN_Dir
{
    Left,
    Right,
} RUN_Dir;
extern volatile RUN_Dir run_dir;

extern uint8 l_lost_tip;
extern uint8 r_lost_tip;
extern uint8 t_lost_tip;
extern uint8 b_lost_tip;
extern uint8 b_lost_num;
extern uint8 t_lost_num;
extern uint8 l_lost_num;
extern uint8 r_lost_num;
extern uint8 Stop_line;
extern uint8 dis_Solidline;

extern uint16 b_center[3];
extern uint16 l_center[3];
extern uint16 r_center[3];
extern uint16 t_center[3];
extern float Out_Annulus_Length;
extern float Stopline_Position;
extern float Stop_Length;
extern float white_width[YM];
extern float k1[YM];
extern float k2[YM];
extern float start_time;
extern float end_time;
extern float deal_time;
extern uint16 EXP_TIME;
void standard(void);
uint8 getOSTUThreshold(void);
void Get_imgOSTU(void);
uint8 get_start_point(void);
uint8 search_l_r(uint8 start_l_x,uint8 start_l_y,uint8 start_r_x,uint8 start_r_y );
void Get_lost_tip(uint8 length);
void Get_start_center(void);
uint8 Get_top_straightline(void);
void Get_curve_line(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 n);
uint8 Left_curve_line(void);
uint8 Right_curve_line(void);
uint8 Get_Annulus();
uint8 Go_Annulus();
uint8 Deal_crossroads();
uint8 Get_stopline(void);

uint8 Dashedline_Makeup();

uint8 Disappear_detection();
uint8 Deal_disappear();


#endif // DEAL_IMG_H

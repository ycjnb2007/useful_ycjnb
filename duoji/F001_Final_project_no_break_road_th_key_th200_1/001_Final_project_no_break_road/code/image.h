#ifndef _image_h_
#define _image_h_


/*********************************修改裁剪图像的尺寸*********************************/
#define LEFT_CUT                         (20)    // 左侧裁剪列数
#define RIGHT_CUT                        (20)    // 右侧裁剪列数
#define BOTTOM_CUT_HEIGHT                (90)    // 底部保留的高度（行数）
#define MID_W                            (77)
// 计算裁剪后的尺寸
#define CUT_W                   (MT9V03X_W - LEFT_CUT - RIGHT_CUT)              // 裁剪后宽度
#define CUT_H                   (BOTTOM_CUT_HEIGHT)                             // 裁剪后高度
#define CUT_START_ROW (MT9V03X_H - CUT_HEIGHT)                                  // 自动计算起始行=50
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))  //划定范围
/*********************************修改裁剪图像的尺寸*********************************/

/*********************************可使用图像范围(可包含)*********************************/
#define LEFT_COL_LIMIT                  (2)                                             //左侧范围
#define RIGHT_COL_LIMIT                 (CUT_W - 3)                                     //右侧范围
#define DOWN_ROW_LIMIT                  (CUT_H - 2)                                     //下方范围
#define UP_ROW_LIMIT                     (2)                                            //上方范围
#define MAX_LINE                         (500)                                           //爬线找到的最大点数
#define FIND_START_POINT_MAX_LINE        (40)    //找起点最多找多少行        
/*********************************可使用图像范围*********************************/



/*********************************声明记录图像信息*********************************/
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];          //复制的另一份灰度图像
extern uint8 image_cut[CUT_H][CUT_W];          //切割后的灰度图像        
extern uint8 image[CUT_H][CUT_W];               //处理好的二值化图像[行][列]

extern uint8 L_start_point[2];            //声明左侧起始点
extern uint8 R_start_point[2];            //声明右侧起始点

extern uint8 L_line[MAX_LINE][2];       //左边线的坐标点 第一位为索引，从0开始，[0]为横坐标，[1]为纵坐标       
extern uint8 R_line[MAX_LINE][2];       //右边线的坐标点
extern uint16 L_Statics;                //统计左边找到点的个数，充当索引
extern uint16 R_Statics;                //统计右边找到点的个数
extern int8 l_dir[MAX_LINE];            //存放左侧边线每个点的生长方向,[]中是L_Statics，是边线的索引
extern int8 r_dir[MAX_LINE];            //存放右侧边线每个点的生长方向
extern uint8 X_meet;                    //记录左右两侧爬线相遇点的X坐标
extern uint8 Y_meet;                    //记录左右两侧爬线相遇点的Y坐标
extern float L_Thres_Record[MAX_LINE];  //将左侧边线的每个点的阈值进行记录，可实现图像迭代：
extern float R_Thres_Record[MAX_LINE];  //右侧边线历史阈值记录
extern uint8 x_start;                   //起点横坐标                
extern uint8 y_start;                   //起点纵坐标
extern uint8 th;
extern uint8 th1;
/*********************************记录图像信息*********************************/


/*********************************函数声明*********************************/
//裁剪图像
void crop_image(void);
//输入阈值二值化
void image_binaryzation(uint8_t threshold);
//大津法求阈值
uint8_t otsu_threshold();
//补黑框
void Draw_Black_Box();
//判断黑白跳变点
uint8 Trip_point(uint8 a, uint8 b);
//左右往中间扫线找起点
uint8 Get_Start_Point(uint8 start_row, uint8(*image)[CUT_W], uint8 *l_start_point, uint8 *r_start_point);
//迷宫爬线找边线
void Dir_Labyrinth_5(uint16 Break_Flag, uint8(*image)[CUT_W], uint8(*l_line)[2], uint8(*r_line)[2], int8* l_dir, int8* r_dir, uint16* l_stastic, uint16* r_stastic, uint8* x_meet, uint8* y_meet,
    uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 clip_value);


/*********************************函数声明*********************************/


#endif
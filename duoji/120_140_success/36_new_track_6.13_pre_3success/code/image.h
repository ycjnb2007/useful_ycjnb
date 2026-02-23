#ifndef _image_h_
#define _image_h_


#define MID_W     93

#define Thres_Filiter_Flag_1    0       //设置1，开启反向平滑滤波，设置0则关闭
#define Thres_Filiter_Flag_2    0       //设置1，开启反向平滑滤波，设置0则关闭

#define Thres_Num_Interfere 25          //中心点周围的个数
#define MAX_LINE 500

#define X_Border_Min    1
#define X_Border_Max    187

#define CUT_HEIGHT     90                       //裁剪后的高度
#define CUT_START_ROW (MT9V03X_H - CUT_HEIGHT) // 自动计算起始行=50

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAX_POINTS 400


extern uint8 image_copy[MT9V03X_H][MT9V03X_W];          //复制的另一份灰度图像
extern uint8 image[CUT_HEIGHT][MT9V03X_W];               //处理好的二值化图像[行][列]
extern uint8 image_cut[CUT_HEIGHT][MT9V03X_W];

extern uint8 L_start_point[2];            //声明左侧起始点
extern uint8 R_start_point[2];            //声明右侧起始点
extern uint8 x_start;
extern uint8 y_start;

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

extern uint8 L_Border[CUT_HEIGHT];       //存储左侧一维边线的数组
extern uint8 R_Border[CUT_HEIGHT];       //存储右侧一维边线的数组 


extern float Mid_point;


/*********************************************************函数声明*********************************************************/
void crop_image_from_bottom();                  //裁剪
uint8 otsu_threshold();                         //大津法求阈值
void Show_binaryzation(uint8 threshold);        //灰度图向二值化的处理
uint8 otsu_binaryze_optimized();                //大津法+二值化优化
void Draw_Black_Box(uint8 black_box_value);     //边线补黑框
uint8 Trip_point(uint8 a, uint8 b);             //黑白跳变点检测


//寻找左右侧起始点
uint8 Get_Start_Point(uint8 start_row, uint8(*image)[MT9V03X_W], uint8 *l_start_point, uint8 *r_start_point);   //寻找左右侧起始点

//求取赛道二维数组边线
void Dir_Labyrinth_5(uint16 Break_Flag, uint8(*image)[MT9V03X_W], uint8(*l_line)[2], uint8(*r_line)[2], int8* l_dir, int8* r_dir, uint16* l_stastic, uint16* r_stastic, uint8* x_meet, uint8* y_meet,
    uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 clip_value);

//由二维边线数组提取一维边线
void Get_Border(uint16 l_total, uint16 r_total, uint8 start, uint8 end, uint8 *l_border, uint8 *r_border, uint8(*l_line)[2], uint8(*r_line)[2]);
                   
//拟合中线的权重值
float find_mid_line_weight();



#endif
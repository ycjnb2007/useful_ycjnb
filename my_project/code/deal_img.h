#ifndef CODE_DEAL_IMG_H_
#define CODE_DEAL_IMG_H_
#include "zf_common_headfile.h"

// ==================== 图像尺寸定义 ====================
#define XM 140       // 处理图像宽度（像素）
#define YM 70      // 处理图像高度（像素）
#define XX XM-1            // 处理图像宽度下标
#define YY YM-1            // 处理图像高度下标

// ==================== 处理区域边界定义 ====================
#define Deal_Left   2             // 左边界起始列（避免边框干扰）
#define Deal_Right  (XX-2)          // 右边界结束列
#define Deal_Bottom 5             // 底部起始行（保留近处视野）
#define Deal_Top    (YY-8 )         // 顶部结束行（保留远处视野）这边要自己根据实际改

// ==================== 丢失线记录边界 ====================
#define Lost_Left   (Deal_Left - 2 )  // 左丢失线记录列
#define Lost_Right  (Deal_Right +2)   // 右丢失线记录列
#define Lost_Bottom (Deal_Bottom -2)  // 底丢失线记录行
#define Lost_Top    (Deal_Top + 2)    // 顶丢失线记录行

// ==================== 原始图像尺寸 ====================
#define IMG_W  MT9V03X_W           // 原始图像宽度
#define IMG_H  MT9V03X_H           // 原始图像高度
#define _IMG_W (IMG_W-1 )            // 原始图像宽度下标
#define _IMG_H (IMG_H-1 )            // 原始图像高度下标

// ==================== 图像像素颜色定义 ====================
#define Black 0        // 黑色：背景或未处理区域
#define White 255      // 白色：赛道或目标区域
#define Control_line 1 // 红色：控制中线
#define Left_line 2    // 绿色：左边界线
#define Right_line 3   // 蓝色：右边界线
#define Lost_line 4    // 紫色：丢失边界线
#define Judge_line 5   // 青色：判断线（用于十字路口）
#define Make_up 6      // 粉色：补全点（虚线补全）
#define LandR 7        // 橙色：左右连通点

// ==================== 数组大小定义 ====================
#define USE_num YM*4   // 八邻域探索最大点数
#define IMG_SIZE (IMG_W * IMG_H)    // 原始图像像素总数
#define MAP_SIZE (YM * XM)          // 处理图像像素总数

// ==================== 图像标志枚举 ====================
enum ImgFlag
{
    straightlineS,   // 短直道
    straightlineL,   // 长直道
    curve,           // 曲线/弯道
    crossroads,      // 十字路口
    disappear,       // 断路（开关元素）

};

// ==================== 元素模式枚举 ====================
typedef enum Elements_Mode
{
    NO,   // 不启用元素锁定
    YES,  // 启用元素锁定
}Elements_Mode;

// ==================== 元素锁定类型 ====================
typedef enum Elements_Lock
{
    Empty_,      // 空锁定
    Disappear_,  // 断路锁定（删除圆环和停车线，只保留断路）
}Elements_Lock;

// ==================== 运行方向枚举 ====================
typedef enum RUN_Dir
{
    Left,
    Right,
} RUN_Dir;

// ==================== 全局变量声明 ====================

// 图像处理标志
extern enum ImgFlag IF;    // 当前图像标志
extern enum ImgFlag IF_L;  // 上一帧图像标志
extern uint8 Stop_line ;
// 元素处理相关
extern volatile Elements_Mode elements_Mode;        // 元素模式开关
extern volatile Elements_Lock elements_Lock[10];    // 元素锁定数组（最多10个）
extern int elements_index;                          // 元素索引

// 断路（开关）处理相关
extern uint8 disappear_flag;                        // 断路检测标志
extern int disappear_num;                           // 当前断路编号
extern int disappear_total;                         // 断路总数量
extern float Disappear_Length;                      // 断路已行驶长度
extern float Disappear_Zero;                        // 断路起始角度
extern float Disappear_Yaw;                         // 断路当前偏航角
extern int Disappear_angle_L[3];                    // 左转断路角度表
extern int Disappear_angle_R[3];                    // 右转断路角度表
extern int Disappear_Dir_L[3][3];                   // 左转断路方向控制表
extern int Disappear_Dir_R[3][3];                   // 右转断路方向控制表
extern float Start_angle;                           // 比赛起始角度

// 运行方向
extern volatile RUN_Dir run_dir;                    // 当前运行方向

// 图像阈值处理
extern uint8_t nowThreshold;                        // 当前大津法阈值
extern int minGrayscale;                            // 最小灰度限制
extern int maxGrayscale;                            // 最大灰度限制
extern uint8_t minThreshold;                        // 最小阈值限制
extern uint8_t maxThreshold;                        // 最大阈值限制
extern int minGray;                                 // 图像最小灰度值
extern int maxGray;                                 // 图像最大灰度值
extern uint8 img_threshold_group[3];                // 分区阈值：近、中、远景
extern uint8 close_Threshold;                       // 近景阈值偏移
extern uint8 mid_Threshold;                         // 中景阈值偏移
extern uint8 far_Threshold;                         // 远景阈值偏移

// 图像存储数组
extern uint8 imgGray[IMG_H][IMG_W];                 // 原始灰度图像
extern uint8 imgOSTU[YM][XM];                       // 二值化处理图像
extern uint8 imgTran[YM][XM];                 // 临时过渡图像
extern uint8 white_num_row[YM];                     // 每行白点数量统计
extern uint8 white_num_col[XM];                     // 每列白点数量统计

// 中线相关
extern uint8 mid_line[YM];                          // 中线每行坐标
extern uint8 start_center_x;                        // 起点中心X坐标
extern uint8 start_center_y;                        // 起点中心Y坐标
extern uint8 end_center_x;                          // 终点中心X坐标
extern uint8 end_center_y;                          // 终点中心Y坐标
extern uint8 start_point_l;                         // 左起点坐标
extern uint8 start_point_r;                         // 右起点坐标

// 八邻域边界探索
extern int points_l[USE_num][2];                    // 左边界点坐标数组
extern int points_r[USE_num][2];                    // 右边界点坐标数组
extern uint32 l_data_statics;                       // 左边界点数量
extern uint32 r_data_statics;                       // 右边界点数量
extern uint8 hightest;                              // 巡线最高点行数

// 丢失边界统计
extern uint8 l_lost_tip;                            // 左边界丢失点数
extern uint8 r_lost_tip;                            // 右边界丢失点数
extern uint8 t_lost_tip;                            // 上边界丢失点数
extern uint8 b_lost_tip;                            // 下边界丢失点数
extern uint8 b_lost_num;                            // 下边界丢失区域数
extern uint8 t_lost_num;                            // 上边界丢失区域数
extern uint8 l_lost_num;                            // 左边界丢失区域数
extern uint8 r_lost_num;                            // 右边界丢失区域数
extern uint8 dis_Solidline;                         // 实线断开标志

// 丢失边界中心点
extern uint16 b_center[3];                          // 下边界中心点
extern uint16 l_center[3];                          // 左边界中心点
extern uint16 r_center[3];                          // 右边界中心点
extern uint16 t_center[3];                          // 上边界中心点

// 几何校正表（像素到实际距离转换）
extern float white_width[YM];                       // 每行标准赛道宽度（像素）
extern float Length_5cm[YM];                        // 每行5cm对应像素数
extern float k1[YM];                                // 宽度校正系数
extern float k2[YM];                                // 高度校正系数

// ==================== 函数声明 ====================
void standard(void);                                // 初始化标准化处理
void image_copy();                                  // 复制图像给imggray[][]
uint8 getOSTUThreshold(void);                       // 大津法阈值计算
void Get_imgOSTU(void);                             // 二值化图像获取
uint8 get_start_point(void);                        // 寻找起点
uint8 search_l_r(uint8 start_l_x,uint8 start_l_y,uint8 start_r_x,uint8 start_r_y); // 八邻域探索
void Get_lost_tip(uint8 length);                    // 获取丢失边界特征
void Get_start_center(void);                        // 获取起点中心
uint8 Get_top_straightline(void);                   // 绘制顶端直线
uint8 Left_curve_line(void);                        // 绘制左曲线
uint8 Right_curve_line(void);                       // 绘制右曲线
uint8 Deal_crossroads(void);                        // 十字路口处理
uint8 Dashedline_Makeup(void);                      // 虚线补全
uint8 Disappear_detection(void);                    // 断路检测
uint8 Deal_disappear(void);                         // 断路处理

#endif // DEAL_IMG_H

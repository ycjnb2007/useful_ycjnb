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
    crossroads,      // 十字路口（备用）
};

// ==================== 核心状态机 (21届疯狂电路组专用) ====================
// 这套状态机是为了代替以前无脑寻线，能够处理恶心的干扰件和路口盲转
typedef enum
{
    STATE_NORMAL,           // 正常八邻域巡线模式
    STATE_CHECK_NODE,       // 触发增宽，起动态框鉴别（真/假节点）
    STATE_FALSE_IGNORE,     // 假干扰强拉直线模式（无视电感/二极管等锯齿）
    STATE_SMOOTH_OFFSET,    // 真节点转弯第一段：动态平滑偏置中线，诱导姿态
    STATE_BLIND_TURN_YAW    // 真节点转弯第二段：陀螺仪 Yaw 角强行盲转
} RunState;

// ==================== 运行方向枚举 ====================
typedef enum RUN_Dir
{
    Left,
    Right,
    Straight,
} RUN_Dir;

// ==================== 全局变量声明 ====================

// 图像处理标志
extern enum ImgFlag IF;    // 当前图像标志
extern enum ImgFlag IF_L;  // 上一帧图像标志
extern RunState cur_state;
extern float Yaw_Start;
extern float Yaw_Target;
extern uint8 is_blind_turning; // 【新增】当前运行状态机

// 全局转弯方向数组 (由用户自行在策略主循环维护或查表)
// 比如 Path_Array 里面存了整条赛道经过节点的转向指令：0为左，1为右，2为直行

// 动态框触发及偏置相关
extern uint8 Y_trigger;                     // 触发动态检测框的起始行

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
void Get_imgOSTU(void);                             // 二值化图像获取及其侧边反光暴力消除
uint8 get_start_point(void);                        // 寻找起点
uint8 search_l_r(uint8 start_l_x,uint8 start_l_y,uint8 start_r_x,uint8 start_r_y); // 八邻域探索及 Trigger 2.0
uint8 Check_Node_Box(uint8 trigger_y);              // 【新增】依据动态框甄别真假节点
void Get_lost_tip(uint8 length);                    // 获取丢失边界特征
void Get_start_center(void);                        // 获取起点中心
uint8 Get_top_straightline(void);                   // 绘制顶端直线
uint8 Left_curve_line(void);                        // 绘制左曲线
uint8 Right_curve_line(void);                       // 绘制右曲线


#endif // DEAL_IMG_H

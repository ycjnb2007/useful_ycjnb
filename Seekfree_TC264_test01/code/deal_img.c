#include "deal_img.h"

// ==================== 全局变量定义 ====================

enum ImgFlag IF = straightlineS;        // 当前图像标志，初始为短直道
enum ImgFlag IF_L = straightlineS;      // 上一帧图像标志

volatile Elements_Mode elements_Mode = NO;               // 元素模式默认关闭
volatile Elements_Lock elements_Lock[10] = {Empty_};     // 元素锁定数组初始化
int elements_index = 0;                                  // 元素索引初始化
uint8 Stop_line = 0;
// 断路（开关）处理相关
uint8 disappear_flag = 0;                // 断路检测标志
int disappear_num = 0;                   // 当前断路编号
int disappear_total = 3;                 // 断路总数（疯狂电路通常3个）
float Disappear_Length = 0;              // 断路已行驶长度
float Disappear_Zero = 0;                // 断路起始角度
float Disappear_Yaw = 0;                 // 断路当前偏航角
int Disappear_angle_L[3];                // 左转断路角度表
int Disappear_angle_R[3];                // 右转断路角度表
float Start_angle = 0;                   // 比赛起始角度
volatile RUN_Dir run_dir = Right;        // 初始运行方向为右转
int Disappear_Dir_L[3][3] = {{0}};       // 左转断路方向控制表
int Disappear_Dir_R[3][3] = {{0}};       // 右转断路方向控制表
float Disappear_Position = 0;            // 断路起始位置
int Curvature = 10;                      // 当前曲率

// 图像处理状态
uint8 dealimg_finish_flag = 0;           // 图像处理完成标志
int deal_runing = 0;                     // 图像处理运行标志
int dealimg_time = 0;                    // 图像处理耗时

// 图像阈值处理
uint16_t histogram[256];                 // 灰度直方图
uint8_t previousThreshold = 0;           // 上一帧阈值
uint8_t nowThreshold = 0;                // 当前阈值
int minGrayscale = 40;                   // 最小灰度限制
int maxGrayscale = 160;                  // 最大灰度限制
int minGray = 0;                         // 图像最小灰度
int maxGray = 0;                         // 图像最大灰度
uint8_t minThreshold = 70;               // 最小阈值
uint8_t maxThreshold = 160;              // 最大阈值
uint8 img_threshold_group[3];            // 分区阈值：近、中、远景
uint8 close_Threshold = 0;               // 近景阈值偏移
uint8 far_Threshold = 0;                 // 远景阈值偏移
uint8 mid_Threshold = 0;                 // 中景阈值偏移
uint16 EXP_TIME = 255;                   // 曝光时间
int UD_OFFSET_DEF = 0;                   // 上下偏移默认值

// 图像存储数组
uint8 imgGray[IMG_H][IMG_W];             // 原始灰度图像
uint8 imgTran[YM][XM];             // 临时过渡图像
uint8 img2wifi[IMG_H][IMG_W];            // 发送到WIFI的图像
uint8 imgOSTU[YM][XM];                   // 二值化处理图像
uint8 white_num_row[YM];                 // 每行白点数量
uint8 white_num_col[XM];                 // 每列白点数量
uint8 white_row_max[2],white_row_min[2]; // 行白点最大最小值
uint8 white_col_max[2],white_col_min[2]; // 列白点最大最小值
uint8 mid_line[YM] = {XM/2};             // 中线数组，初始为中间
uint8 Make_up_num[YM] = {0};             // 补全点数统计

// 起点终点坐标
uint8 start_center_last = XM / 2;        // 上一帧起点中心
uint8 start_center = XM/2;               // 起点中心
uint8 start_point_l = 0;                 // 左起点
uint8 start_point_r = XX;                // 右起点
uint8 start_center_y = 0;                // 起点行坐标
uint8 start_center_x = XM/2;             // 起点列坐标

uint8 end_center = XM/2;                 // 终点中心
uint8 end_center_x = XM/2;               // 终点列坐标
uint8 end_center_y = Deal_Top;           // 终点行坐标

// 边界丢失统计
uint8 l_lost_tip = 0;                    // 左边界丢失点数
uint8 r_lost_tip = 0;                    // 右边界丢失点数
uint8 t_lost_tip = 0;                    // 上边界丢失点数
uint8 b_lost_tip = 0;                    // 下边界丢失点数

uint8 b_lost_num = 0;                    // 下边界丢失区域数
uint8 t_lost_num = 0;                    // 上边界丢失区域数
uint8 l_lost_num = 0;                    // 左边界丢失区域数
uint8 r_lost_num = 0;                    // 右边界丢失区域数

uint8 dis_Solidline = 0;                 // 实线断开标志
uint16 b_center[3] = {0};                // 下边界中心点
uint16 l_center[3] = {0};                // 左边界中心点
uint16 r_center[3] = {0};                // 右边界中心点
uint16 t_center[3] = {0};                // 上边界中心点

// 处理区域尺寸
int Deal_W = XX;                         // 处理宽度
int Deal_H = YY;                         // 处理高度

// 时间统计
float start_time = 0;                    // 处理开始时间
float end_time = 0;                      // 处理结束时间
float deal_time = 0;                     // 处理耗时

// 行驶长度
float Length = 0;                          // 行驶长度

// 虚线补全相关
int EXTERN_sum[16] = {0};                // 各种连通类型统计
int Goto_U = 1;                          // 向上连通标志 0001向上有白点 Goto_U + Goto_D = 3 (二进制 0011，上下都有白点)
int Goto_D = 2;                          // 向下连通标志 0010  下     Goto_U + Goto_L + Goto_R = 13 (二进制 1101，上左右都有白点)
int Goto_L = 4;                          // 向左连通标志 0100  左
int Goto_R = 8;                          // 向右连通标志 1000  右
int UD = 3;                              // 上下连通
int UL = 5;                              // 上左连通
int UR = 9;                              // 上右连通
int DL = 6;                              // 下左连通
int DR = 10;                             // 下右连通
int LR = 12;                             // 左右连通
int UDL = 7;                             // 上左下连通
int UDR = 11;                            // 上右下连通
int ULR = 13;                            // 上左右连通
int DLR = 14;                            // 下左右连通
int UDLR = 15;                           // 上下左右连通
int EXTERN_ROW[16][YM] = {{0}};          // 各种连通类型在各行统计
int EXTERN_L = 0;                        // 左边界连通数
int EXTERN_R = 0;                        // 右边界连通数

// 几何校正表（像素到实际距离转换）
float white_width[YM] = {17,17,17,17,17,16,16,16,16,16,      //表示在图像的第 i 行，赛道实际宽度对应多少个像素点。
                         16,16,16,15,15,15,15,14,14,14,
                         14,14,14,13,13,13,13,13,13,13,
                         12,12,12,12,12,11,11,11,11,10,
                         10,10,10,10,10,10,10,10,10, 9,
                          9, 9, 9, 9, 9, 8, 8, 8, 8, 8,
                          7, 7, 7, 7, 6, 6, 6, 6, 5, 5};

float Length_5cm[YM] = {30,29,28,28,27,27,26,26,25,25,    //表示在图像的第 i 行，垂直方向上 5cm 的实际距离对应多少个像素行高。
                         25,25,25,25,24,24,24,23,23,23,
                         22,22,22,22,21,21,21,20,20,20,
                         19,19,19,18,18,18,17,17,16,16,
                         15,15,15,14,14,14,13,13,12,12,
                         11,11,11,11,11,10,10,10,9, 9,
                          9, 8, 8, 8, 8, 7, 7, 7, 7, 7};

float k1[YM] = { 25/17, 25/17, 25/17, 25/17, 25/17, 25/16, 25/16, 25/16, 25/16, 25/16,
                 25/16, 25/16, 25/16, 25/15, 25/15, 25/15, 25/15, 25/14, 25/14, 25/14,
                 25/14, 25/14, 25/14, 25/13, 25/13, 25/13, 25/13, 25/13, 25/13, 25/13,
                 25/12, 25/12, 25/12, 25/12, 25/12, 25/11, 25/11, 25/11, 25/11, 25/10,
                 25/10, 25/10, 25/10, 25/10, 25/10, 25/10, 25/10, 25/10, 25/10,  25/9,
                  25/9,  25/9,  25/9,  25/9,  25/9,  25/8,  25/8,  25/8,  25/8,  25/8,
                  25/7,  25/7,  25/7,  25/7,  25/6,  25/6,  25/6,  25/6,  25/5,  25/5};

float k2[YM] = { 0.00,  0.28,  0.57,  0.85,  1.14,  1.45,  1.71,  1.99,  2.28,  2.56,
                 2.85,  3.13,  3.42,  3.70,  3.99,  4.27,  4.56,  4.84,  5.13,  5.41,
                 5.70,  5.98,  6.27,  6.55,  6.84,  7.12,  7.41,  7.69,  7.98,  8.26,
                 8.55,  8.83,  9.12,  9.40,  9.69,  9.97, 10.26, 10.54, 10.83, 11.11,
                11.28, 11.57, 11.85, 12.14, 12.43, 12.71, 13.00, 13.28, 13.57, 13.86,
                14.14, 14.43, 14.71, 15.00, 15.29, 15.57, 15.86, 16.14, 16.43, 16.72,
                17.00, 17.29, 17.57, 17.86, 18.15, 18.43, 18.72, 19.01, 19.29, 20.00};

// ==================== 八邻域变量区域 ====================
int points_l[USE_num][2] = { {0} };      // 左边界点坐标
int points_r[USE_num][2] = { {0} };      // 右边界点坐标
float dir_r[USE_num] = { 0 };            // 右边生长方向
float dir_l[USE_num] = { 0 };            // 左边生长方向
uint32 l_data_statics = 0;               // 左边界点统计
uint32 r_data_statics = 0;               // 右边界点统计
uint8 hightest = 0;                      // 巡线最高点
uint16 leftmost[2] = {0};                // 最左点记录
uint16 rightmost[2] = {0};               // 最右点记录
uint16 topmost[2] = {0};                 // 最上点记录

// 左边八邻域变量
int center_point_l[2] = { 0 };           // 八邻域中心坐标
uint8 search_filds_l[8][2] = { {0} };    // 八邻域坐标
uint8 index_l = 0;                       // 索引下标
uint8 temp_l[8][2] = { {0} };            // 临时存储
int8_t seeds_l[8][2] = { {0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1, 0},{1, -1} }; // 左生长种子

// 右边八邻域变量
int center_point_r[2] = { 0 };           // 八邻域中心坐标
uint8 search_filds_r[8][2] = { {0} };    // 八邻域坐标
uint8 index_r = 0;                       // 索引下标
uint8 temp_r[8][2] = { {0} };            // 临时存储
int8_t seeds_r[8][2] = { {0, -1},{1,-1},{1,0}, {1,1},{0,1},{-1,1}, {-1,0},{-1, -1} }; // 右生长种子

// ==================== 函数实现 ====================

/******************************************************************************
* 函数名称     : standard
* 描述        : 图像处理初始化标准化
* 进入参数     : void
* 返回参数     : void
******************************************************************************/
void standard(void)
{
    Deal_W = XX;
    Deal_H = YY;
    disappear_flag = 0;
    disappear_num = 0;
    disappear_total = 3;//待定
    elements_index = 0;
}
/******************************************************************************
* 函数名称     : image_copy
* 描述        : 图像复制
* 进入参数     : void
* 返回参数     : void
******************************************************************************/
void image_copy()
{
    memset(imgGray, 0, sizeof(imgGray));
    for(int i=0;i<MT9V03X_H;i++)
    {
        for(int j=0;j<MT9V03X_W;j++)
        {
            imgGray[i][j]= mt9v03x_image[i][j];


        }

    }

}

/******************************************************************************
* 函数名称     : my_abs
* 描述        : 取绝对值函数
* 进入参数     : value
* 返回参数     : 绝对值
******************************************************************************/
int my_abs(int value)
{
    if(value >= 0) return value;
    else return -value;
}

/******************************************************************************
* 函数名称     : getGrayscaleHistogram
* 描述        : 灰度直方图统计
* 进入参数     : void
* 返回参数     : void
******************************************************************************/
void getGrayscaleHistogram(void)
{
    memset(histogram, 0, sizeof(histogram));
    minGray = 0;
    maxGray = 255;
    // 起始位置：行=IMG_H-YM（120-90=30），列= (IMG_W-XM)/2（(188-140)/2=24，居中取140列）
    uint8_t *ptr = &imgGray[IMG_H-YM][(IMG_W-XM)/2];
    // 结束位置：行=IMG_H-1（119），列= (IMG_W-XM)/2 + XM - 1（24+140-1=163），再+1到下一个地址
    uint8_t *ptrEnd = &imgGray[IMG_H-1][(IMG_W-XM)/2 + XM - 1] + 1;
    while (ptr != ptrEnd)
    {
        minGray = *ptr < minGray ? *ptr:minGray;
        maxGray = *ptr > maxGray ? *ptr:maxGray;
        ++histogram[*ptr++];
    }
}

/******************************************************************************
* 函数名称     : getOSTUThreshold
* 描述        : 大津法阈值计算
* 进入参数     : void
* 返回参数     : nowThreshold 大津法阈值
******************************************************************************/
uint8 getOSTUThreshold(void)
{
    getGrayscaleHistogram();
    uint32_t sum = 0, valueSum = 0;
    uint64_t sigma = 0, maxSigma = 0;
    float w1 = 0, w2 = 0;
    int32_t u1 = 0, u2 = 0;
    uint8 min = 0, max = 255;

    min = minGray;
    max = maxGray;
    if(max < minThreshold)     return minThreshold;
    if(min > maxThreshold)     return maxThreshold;

    min = min < minGrayscale ? minGrayscale : min;
    max = max > maxGrayscale ? maxGrayscale : max;

    uint32_t lowSum[256] = {0};
    uint32_t lowValueSum[256] = {0};
    for (uint16_t i = min; i <= max; ++i)
    {
        sum += histogram[i];
        valueSum += histogram[i] * i;
        lowSum[i] = sum;
        lowValueSum[i] = valueSum;
    }
    for (uint16_t i = min; i < max + 1; ++i)
    {
        w1 = (float)lowSum[i] / sum;
        w2 = 1 - w1;
        u1 = (int32_t)(lowValueSum[i] / w1);
        u2 = (int32_t)((valueSum - lowValueSum[i]) / w2);
        sigma = (uint64_t)(w1 * w2 * (u1 - u2) * (u1 - u2));
        if (sigma >= maxSigma)
        {
            maxSigma = sigma;
            nowThreshold = i;
        }
        else
        {
            break;
        }
    }
    nowThreshold = nowThreshold < minThreshold ? minThreshold : nowThreshold;
    nowThreshold = nowThreshold > maxThreshold ? maxThreshold : nowThreshold;
    previousThreshold = nowThreshold;
    return nowThreshold;
}

/******************************************************************************
* 函数名称     : Get_imgOSTU
* 描述         : 大津法二值化处理图像
* 进入参数     : void
* 返回参数     : void
******************************************************************************/
void Get_imgOSTU(void)
{
    memset(imgOSTU, Black, sizeof(imgOSTU));
    memset(white_num_row, 0, sizeof(white_num_row));
    memset(white_num_col, 0, sizeof(white_num_col));

    img_threshold_group[0] = nowThreshold + close_Threshold; // 近景
    img_threshold_group[1] = nowThreshold - mid_Threshold;   // 中景
    img_threshold_group[2] = nowThreshold - far_Threshold;   // 远景

    int k = 0;
    int Loop[9][2] = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};

    for(int i = 0; i <= YY; i++)  // 腐蚀去噪
    {
        if(i == YY/3)
        {
            k = 1;
        }
        else if(i == YY*2/3)
        {
            k = 2;
        }

        for(int j = 0; j <= XX; j++)
        {
            if(imgGray[IMG_H-1-i][(IMG_W-XM)/2+j] >= img_threshold_group[k])
            {
                int White_num = 0;
                int Black_num = 0;
                for(int m = 0; m <= 8; m++)
                {
                    int x = j + Loop[m][0];
                    int y = i + Loop[m][1];
                    if(x < 0 || x > XX || y < 0 || y > YY)
                    {
                        White_num++;
                    }
                    else if(imgOSTU[y][x] == White)
                    {
                        White_num = 8;
                        break;
                    }
                    else if(imgGray[IMG_H-1-y][(IMG_W-XM)/2+x] >= nowThreshold)
                    {
                        White_num++;
                    }
                    else
                    {
                        Black_num++;
                    }
                    if(Black_num >= 2)
                    {
                        break;
                    }
                }
                if(White_num >= 8)
                {
                    imgOSTU[i][j] = White;
                    white_num_row[i]++;
                    white_num_col[j]++;
                    for(int m = 0; m <= 8; m++)
                    {
                        int x = j + Loop[m][0];
                        int y = i + Loop[m][1];
                        if(x < 0 || x > XX || y < 0 || y > YY)
                        {
                            continue;
                        }
                        else if(imgOSTU[y][x] != White)
                        {
                            imgOSTU[y][x] = White;
                            white_num_row[y]++;
                            white_num_col[x]++;
                        }
                    }
                }
            }
        }
    }
}

/******************************************************************************
* 函数名称     : Dashedline_Makeup
* 描述         : 虚线区衔接弥补
* 返回参数     : 0
******************************************************************************/
uint8 Dashedline_Makeup(void)
{
    // 最长最短白行列统计
    white_row_max[0] = 0;
    white_row_min[0] = YY;
    white_col_max[0] = 0;
    white_col_min[0] = XX;

    for(uint8 i = Deal_Bottom; i <= Deal_Top; i++)
    {
        if(white_num_row[i] > white_row_max[0])
        {
            white_row_max[0] = white_num_row[i];//[0]赋的是该行最大的白点数，[1]赋的是第几行
            white_row_max[1] = i;
        }
        else if(white_num_row[i] < white_row_min[0])
        {
            white_row_min[0] = white_num_row[i];
            white_row_min[1] = i;
        }
    }

    for(uint8 i = Deal_Left; i <= Deal_Right; i++)
    {
        if(white_num_col[i] > white_col_max[0])
        {
            white_col_max[0] = white_num_col[i];
            white_col_max[1] = i;
        }
        else if(white_num_col[i] < white_col_min[0])
        {
            white_col_min[0] = white_num_col[i];
            white_col_min[1] = i;
        }
    }

    if(white_col_max[0] >= YY && white_row_max[0] < white_width[0])  //应该是处理赛道旁边白色反光带的，直接画黑色边框，把边界外的区域涂黑
    {
//绘制黑边框
        for(uint8 x = 0; x <= XX; x++ )
        {
            if(imgOSTU[Deal_Bottom-1][x] == White || imgOSTU[Deal_Bottom-2][x] == White)
            {
                imgOSTU[Deal_Bottom][x] = White;
            }
            imgOSTU[Deal_Bottom-1][x] = Black;
            imgOSTU[Deal_Bottom-2][x] = Black;

            if(imgOSTU[Deal_Top+1][x] == White || imgOSTU[Deal_Top+2][x] == White)
            {
                imgOSTU[Deal_Top][x] = White;
            }
            imgOSTU[Deal_Top+1][x] = Black;
            imgOSTU[Deal_Top+2][x] = Black;
        }

        for(uint8 y = 0; y <= Deal_Top; y++ )
        {
            if(imgOSTU[y][Deal_Left-1] == White || imgOSTU[y][Deal_Left-2] == White)
            {
                imgOSTU[y][Deal_Left] = White;
            }
            imgOSTU[y][Deal_Left-1] = Black;
            imgOSTU[y][Deal_Left-2] = Black;

            if(imgOSTU[y][Deal_Right+1] == White || imgOSTU[y][Deal_Right+2] == White)
            {
                imgOSTU[y][Deal_Right] = White;
            }
            imgOSTU[y][Deal_Right+1] = Black;
            imgOSTU[y][Deal_Right+2] = Black;
        }
        return 0;
    }

    memset(imgTran, Black, sizeof(imgTran));  // 逻辑是：如果右边有个黑点，距离我不到“两倍赛道宽”，我就认为它是断掉的赛道碎片，我要标记它。
                                              // 如果超过这个距离，那可能是隔壁赛道的干扰，我就不管了

    // imgTran断点统计，全部填充为0（Black），用来存储每个位置的连通性标记，就是找虚线
    for(uint8 i = 0; i <= YY; i++)
    {
        for(uint8 j = 0; j <= XX; j++)
        {
            if(imgOSTU[i][j] == White)
            {
                int w_start = j-2*white_width[i] < 0 ? 0 : j-2*white_width[i];  //以当前白点为中心，向左右各搜索2倍赛道宽度的范围，这个是向左
                int w_end = j+2*white_width[i] > XX ? XX : j+2*white_width[i];  //如果i=0, white_width[0]=17，则左右各搜索34像素，这个是右

                if(j != XX)
                {
                    for(int w = j+1; w <= w_end; w++)
                    {
                        if(imgOSTU[i][w] == Black)
                        {
                            imgTran[i][w] += Goto_R;    //这个点是白点往右走两倍赛道长度之内发现的黑点，上面的其他位置的同理
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                if(j != 0)
                {
                    for(int w = j-1; w >= w_start; w--)
                    {
                        if(imgOSTU[i][w] == Black)
                        {
                            imgTran[i][w] += Goto_L;
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                int h_start = i-Length_5cm[i] < 0 ? 0 : i-Length_5cm[i];
                int h_end = i+Length_5cm[i] > YY ? YY : i+Length_5cm[i];

                if(i != YY)
                {
                    for(int h = i+1; h <= h_end; h++)
                    {
                        if(imgOSTU[h][j] == Black)
                        {
                            imgTran[h][j] += Goto_U;
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                if(i != 0)
                {
                    for(int h = i-1; h >= h_start; h--)
                    {
                        if(imgOSTU[h][j] == Black)
                        {
                            imgTran[h][j] += Goto_D;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    // 这边就是给虚线补线的
    memset(EXTERN_sum, 0, sizeof(EXTERN_sum));
    memset(EXTERN_ROW, 0, sizeof(EXTERN_ROW));

    for(uint8 i = 0; i <= YY; i++)
    {
        for(uint8 j = 0; j <= XX; j++)
        {
            if(imgOSTU[i][j] == Black && imgTran[i][j] >= 1)
            {
                if(imgTran[i][j] == ULR || imgTran[i][j] == DLR || imgTran[i][j] == UDLR)
                {
                    imgOSTU[i][j] = LandR;// 左右连通点//// 变成橙色（算作白点）
                }
                else
                {
                    imgOSTU[i][j] = Make_up;// 普通补全点// 变成粉色（算作白点）
                }
                int Combination = (int)imgTran[i][j];
                EXTERN_sum[Combination]++;
                EXTERN_ROW[Combination][i]++;
            }
        }
    }

    // 左右边界白点迁移
    EXTERN_L = 0;  // EXTERN_L/R 用来统计我们在边缘“捞”回了多少点（调试用）
    EXTERN_R = 0;
    uint8 l_white = 0, r_white = 0;  //// 临时变量，存左边/右边边缘的白点总数
    uint8 l_end = Deal_Left, r_end = Deal_Right;
    // l_end 和 r_end 是“打捞边界”。
    // Deal_Left = 2。默认只捞第2列。
    // 如果发现第1列、第0列也有白点，我们会把这个 l_end 往左扩，扩到 0

    for(uint8 k = 0; k <= (int)white_width[Deal_Top]; k++) // 左边界位移计算
    {
        l_white += white_num_col[0+k];
        if(l_white)
        {
            l_end = 0+k;
            if(l_white >= 25)
            {
                break;
            }
        }
    }

    for(uint8 k = 0; k <= (int)white_width[Deal_Top]; k++) // 右边界位移计算
    {
        r_white += white_num_col[XX-k];
        if(r_white)
        {
            r_end = XX-k;
            if(r_white >= 25)
            {
                break;
            }
        }
    }

    // 左边界白点迁移
    for(uint8 y = Deal_Bottom; y <= YY; y++)
    {
        for(uint8 k = Deal_Left; k <= l_end; k++)
        {
            if(imgOSTU[y][k] == White)
            {
                imgOSTU[y][Deal_Left] = White;
                EXTERN_L++;
                white_num_row[y]++;
                white_num_col[Deal_Left]++;
                break;
            }
            else if(imgOSTU[y][k] == Black)
            {
                break;
            }
        }
    }

    // 右边界白点迁移
    for(uint8 y = Deal_Bottom; y <= YY; y++)
    {
        for(uint8 k = Deal_Right; k >= r_end; k--)
        {
            if(imgOSTU[y][k] == White)
            {
                imgOSTU[y][Deal_Right] = White;
                EXTERN_R++;
                white_num_row[y]++;
                white_num_col[Deal_Right]++;
                break;
            }
            else if(imgOSTU[y][k] == Black)
            {
                break;
            }
        }
    }

    // 上边界白点迁移
    {
        uint8 t_end = YY;
        uint8 White_Top[XX] = {0};

        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            for(uint8 y = Deal_Top; y <= t_end; y++)
            {
                if(imgOSTU[y][x] == White)
                {
                    White_Top[x] = 1;
                    break;
                }
                else if(imgOSTU[y][x] == Black)
                {
                    White_Top[x] = 0;
                    break;
                }
            }
        }

        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            if(imgOSTU[Deal_Top][x] == White && White_Top[x] == 1)
            {
                White_Top[x] = 0;
                for(int w = x+1; w <= Deal_Right; w++)
                {
                    if(White_Top[w] == 1)
                    {
                        White_Top[w] = 0;
                    }
                    else
                    {
                        break;
                    }
                }
                for(int w = x-1; w >= Deal_Left; w--)
                {
                    if(White_Top[w] == 1)
                    {
                        White_Top[w] = 0;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }

        for(uint8 x = Deal_Left; x <= Deal_Right; x++)
        {
            if(White_Top[x] == 1)
            {
                imgOSTU[Deal_Top-0][x] = White;
                imgOSTU[Deal_Top-1][x] = White;
                imgOSTU[Deal_Top-2][x] = White;
                white_num_row[Deal_Top-0]++;
                white_num_row[Deal_Top-1]++;
                white_num_row[Deal_Top-2]++;
                white_num_col[x]++;
            }
        }
    }

    // 绘制四周黑色边框
    for(uint8 x = 0; x <= XX; x++)
    {
        if(imgOSTU[Deal_Bottom-1][x] == White || imgOSTU[Deal_Bottom-2][x] == White)
        {
            imgOSTU[Deal_Bottom][x] = White;
        }
        imgOSTU[Deal_Bottom-1][x] = Black;
        imgOSTU[Deal_Bottom-2][x] = Black;

        if(imgOSTU[Deal_Top+1][x] == White || imgOSTU[Deal_Top+2][x] == White)
        {
            imgOSTU[Deal_Top][x] = White;
        }
        imgOSTU[Deal_Top+1][x] = Black;
        imgOSTU[Deal_Top+2][x] = Black;
    }

    for(uint8 y = 0; y <= Deal_Top; y++)
    {
        if(imgOSTU[y][Deal_Left-1] == White || imgOSTU[y][Deal_Left-2] == White)
        {
            imgOSTU[y][Deal_Left] = White;
        }
        imgOSTU[y][Deal_Left-1] = Black;
        imgOSTU[y][Deal_Left-2] = Black;

        if(imgOSTU[y][Deal_Right+1] == White || imgOSTU[y][Deal_Right+2] == White)
        {
            imgOSTU[y][Deal_Right] = White;
        }
        imgOSTU[y][Deal_Right+1] = Black;
        imgOSTU[y][Deal_Right+2] = Black;
    }

    return 0;
}

/******************************************************************************
* 函数名称     : get_start_point
* 描述         : 寻找左右边界起点
* 进入参数     : void
* 返回参数     : 1找到左右边界；0未找到左右边界
******************************************************************************/
uint8 get_start_point(void)
{
    b_lost_tip = 0;
    start_center_y = 0;
    uint8 l_found = 0, r_found = 0;
    uint32 l_point = 0, r_point = 0;

    for(uint8 i = Deal_Bottom; i <= Deal_Bottom + 30; i++)
    {
        if(white_num_row[i] >= 5 && white_num_row[i+1] >= 5)
        {
            start_center_y = i;

            // 从左向右找黑白跳变点
            for(uint8 j = 0; j < XX-1; j++)
            {
                if((imgOSTU[start_center_y][j+1] == Make_up || imgOSTU[start_center_y][j+1] == White) &&
                   imgOSTU[start_center_y][j] == Black)
                {
                    l_point = j;
                    l_found = 1;
                    break;
                }
            }

            // 从右向左找黑白跳变点
            for(uint8 j = XX; j > 0+1; j--)
            {
                if((imgOSTU[start_center_y][j-1] == Make_up || imgOSTU[start_center_y][j-1] == White) &&
                   imgOSTU[start_center_y][j] == Black)
                {
                    r_point = j;
                    r_found = 1;
                    break;
                }
            }

            if(r_found && l_found)
            {
                break;
            }
        }
    }

    if(l_found && r_found)
    {
        // 记录b_lost_tip
        for(uint8 i = l_point; i <= r_point; i++)
        {
            if(imgOSTU[start_center_y][i] == White)
            {
                b_lost_tip++;
                imgOSTU[Lost_Bottom][i] = Lost_line;
            }
            else if(imgOSTU[start_center_y + 1][i] == White)
            {
                b_lost_tip++;
                imgOSTU[Lost_Bottom][i] = Lost_line;
            }
            else if(imgOSTU[Deal_Bottom][i] == White)
            {
                b_lost_tip++;
                imgOSTU[Lost_Bottom][i] = Lost_line;
            }
        }

        // 起点位置记录
        start_center_x = (l_point + r_point) / 2;
        start_point_l = l_point;
        start_point_r = r_point;
        return 1;
    }
    else
    {
        return 0;
    }
}

/******************************************************************************
* 函数名称     : search_l_r
* 描述         : 八邻域探索边界线
* 进入参数     : start_l_x,start_l_y 左八邻域开始坐标
*                start_r_x,start_r_y 右八邻域开始坐标
* 返回参数     : void
******************************************************************************/
uint8 search_l_r(uint8 start_l_x, uint8 start_l_y, uint8 start_r_x, uint8 start_r_y)
{
    hightest = 0;
    leftmost[0] = Deal_Right;
    rightmost[0] = Deal_Left;
    topmost[1] = 0;

    l_lost_tip = 0;
    r_lost_tip = 0;
    t_lost_tip = 0;

    l_data_statics = 0;
    r_data_statics = 0;

    center_point_l[0] = start_l_x;
    center_point_l[1] = start_l_y;
    center_point_r[0] = start_r_x;
    center_point_r[1] = start_r_y;

    points_l[l_data_statics][0] = center_point_l[0];
    points_l[l_data_statics][1] = center_point_l[1];
    points_r[r_data_statics][0] = center_point_r[0];
    points_r[r_data_statics][1] = center_point_r[1];

    uint32 break_flag = USE_num;

    while(break_flag--)
    {
        // 左边8邻域初始化
        for(uint8 i = 0; i < 8; i++)
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
        }

        // 右边8邻域初始化
        for(uint8 i = 0; i < 8; i++)
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
        }

        // 左边生长判断
        l_data_statics++;
        index_l = 0;
        for(uint8 i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;
            temp_l[i][1] = 0;
        }

        for(uint8 i = 0; i < 8; i++)
        {
            if((imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Black ||
                imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Lost_line) &&
               (imgOSTU[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] != Black &&
                imgOSTU[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] != Lost_line))
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = i;
            }

            if(index_l)
            {
                center_point_l[0] = temp_l[0][0];
                center_point_l[1] = temp_l[0][1];
                for(uint8 j = 0; j < index_l; j++)
                {
                    if(center_point_l[1] < temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];
                        center_point_l[1] = temp_l[j][1];
                        dir_l[l_data_statics - 1] = i;
                    }
                }
            }

            // 丢失点记录
            if(center_point_l[0] == Deal_Left - 1 &&
               imgOSTU[center_point_l[1]][Deal_Left] == White &&
               imgOSTU[center_point_l[1]][Deal_Left-2] != Lost_line)
            {
                l_lost_tip++;
                imgOSTU[center_point_l[1]][Lost_Left] = Lost_line;
            }
            else if(center_point_l[0] == Deal_Right + 1 &&
                    imgOSTU[center_point_l[1]][Deal_Right] == White &&
                    imgOSTU[center_point_l[1]][Lost_Right] != Lost_line)
            {
                r_lost_tip++;
                imgOSTU[center_point_l[1]][Lost_Right] = Lost_line;
            }
            else if(center_point_l[1] == Deal_Top + 1 &&
                    imgOSTU[Deal_Top][center_point_l[0]] == White &&
                    imgOSTU[Deal_Top + 2][center_point_l[0]] != Lost_line)
            {
                t_lost_tip++;
                imgOSTU[Lost_Top][center_point_l[0]] = Lost_line;
            }

            // 更新最高点
            if(center_point_l[1] > hightest)
            {
                hightest = center_point_l[1];
                topmost[0] = center_point_l[0];
                topmost[1] = center_point_l[1];
            }

            // 更新最左点
            if(center_point_l[0] < leftmost[0])
            {
                leftmost[0] = center_point_l[0];
                leftmost[1] = center_point_l[1];
            }
        }

        // 右边生长判断
        r_data_statics++;
        index_r = 0;
        for(uint8 i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;
            temp_r[i][1] = 0;
        }

        for(uint8 i = 0; i < 8; i++)
        {
            if((imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Black ||
                imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Lost_line) &&
               (imgOSTU[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] != Black &&
                imgOSTU[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] != Lost_line))
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;
                dir_r[r_data_statics - 1] = i;
            }

            if(index_r)
            {
                center_point_r[0] = temp_r[0][0];
                center_point_r[1] = temp_r[0][1];
                for(uint8 j = 0; j < index_r; j++)
                {
                    if(center_point_r[1] < temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];
                        center_point_r[1] = temp_r[j][1];
                        dir_r[r_data_statics - 1] = i;
                    }
                }
            }

            // 丢失点记录
            if(center_point_r[0] == Deal_Right + 1 &&
               imgOSTU[center_point_r[1]][Deal_Right] == White &&
               imgOSTU[center_point_r[1]][Lost_Right] != Lost_line)
            {
                r_lost_tip++;
                imgOSTU[center_point_r[1]][Lost_Right] = Lost_line;
            }
            else if(center_point_r[0] == Deal_Left - 1 &&
                    imgOSTU[center_point_r[1]][Deal_Left] == White &&
                    imgOSTU[center_point_r[1]][Lost_Left] != Lost_line)
            {
                l_lost_tip++;
                imgOSTU[center_point_r[1]][Lost_Left] = Lost_line;
            }
            else if(center_point_r[1] == Deal_Top + 1 &&
                    imgOSTU[Deal_Top][center_point_r[0]] == White &&
                    imgOSTU[Lost_Top][center_point_r[0]] != Lost_line)
            {
                t_lost_tip++;
                imgOSTU[Lost_Top][center_point_r[0]] = Lost_line;
            }

            // 更新最高点
            if(center_point_r[1] > hightest)
            {
                hightest = center_point_r[1];
                topmost[0] = center_point_r[0];
                topmost[1] = center_point_r[1];
            }

            // 更新最右点
            if(center_point_r[0] > rightmost[0])
            {
                rightmost[0] = center_point_r[0];
                rightmost[1] = center_point_r[1];
            }
        }

        // 清除顶部丢失线
        if(hightest == Deal_Top - 1)
        {
            for(uint8 i = Deal_Left; i <= Deal_Right; i++)
            {
                imgOSTU[Lost_Top][i] = Black;
                imgOSTU[Lost_Top-1][i] = Black;
            }
        }

        // 储存边界数组
        points_l[l_data_statics][0] = center_point_l[0];
        points_l[l_data_statics][1] = center_point_l[1];
        points_r[r_data_statics][0] = center_point_r[0];
        points_r[r_data_statics][1] = center_point_r[1];

        // 退出条件判断
        if((points_r[r_data_statics][0] == points_r[r_data_statics-1][0] &&
            points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] &&
            points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] &&
            points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) ||
           (points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] &&
            points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0] &&
            points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] &&
            points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            break;
        }

        if(my_abs(points_r[r_data_statics][0] - points_l[l_data_statics][0]) <= 1 &&
           my_abs(points_r[r_data_statics][1] - points_l[l_data_statics][1]) <= 1)
        {
            break;
        }

        if((l_data_statics > YY/2 && my_abs(start_l_x - points_l[l_data_statics][0]) <= 1 &&
            my_abs(start_l_y - points_l[l_data_statics][1]) <= 1) ||
           (r_data_statics > YY/2 && my_abs(start_r_x - points_r[r_data_statics][0]) <= 1 &&
            my_abs(start_r_y - points_r[r_data_statics][1]) <= 1))
        {
            break;
        }
    }

    // 绘制左右边线
    for(int i = l_data_statics; i > 0; i--)
    {
        imgOSTU[points_l[i][1]][points_l[i][0]] = Left_line;
    }
    for(int i = r_data_statics; i > 0; i--)
    {
        imgOSTU[points_r[i][1]][points_r[i][0]] = Right_line;
    }

    return 0;
}

/******************************************************************************
* 函数名称     : Get_lost_tip
* 描述         : 获取四方位边缘特征
* 进入参数     : length 正常边界丢失的判断长度
******************************************************************************/
void Get_lost_tip(uint8 length)
{
    dis_Solidline = 0;

    b_lost_num = 0;
    t_lost_num = 0;
    l_lost_num = 0;
    r_lost_num = 0;

    memset(b_center, 0, sizeof(b_center));
    memset(l_center, 0, sizeof(l_center));
    memset(r_center, 0, sizeof(r_center));
    memset(t_center, 0, sizeof(t_center));

    /*********** 获取底部边缘特征 ***********/
    if(b_lost_tip)
    {
        for(uint8 i = Deal_Left; i <= Deal_Right; i++)
        {
            if(imgOSTU[Lost_Bottom][i] == Lost_line)
            {
                if(i <= Deal_Left + length || i >= Deal_Right - length ||
                   (i + length <= XX && imgOSTU[Lost_Bottom][i+length] == Lost_line))
                {
                    for(uint8 k = i; k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Bottom][k] == Black)
                        {
                            b_center[b_lost_num] = ((uint32)k-1-(uint32)i)/2+(uint32)i;
                            b_lost_num++;
                            i = k;
                            break;
                        }
                    }
                }
                else
                {
                    for(uint8 k = i; k <= i + length; k++)
                    {
                        if(imgOSTU[Lost_Bottom][k] == Lost_line)
                        {
                            imgOSTU[Lost_Bottom][k] = Black;
                            b_lost_tip--;
                        }
                        else if(imgOSTU[Lost_Bottom][k] == Black)
                        {
                            break;
                        }
                    }
                }
            }
            if(b_lost_num == 2)
            {
                break;
            }
        }
    }

    /*********** 获取左部边缘特征 ***********/
    if(l_lost_tip)
    {
        for(uint8 i = Deal_Bottom; i <= Deal_Top; i++)
        {
            if(imgOSTU[i][Lost_Left] == Lost_line)
            {
                if(i <= Deal_Bottom + length)
                {
                    for(uint8 j = Deal_Left; j <= Deal_Left + length; j++)
                    {
                        if(imgOSTU[Lost_Bottom][j] == Lost_line)
                        {
                            imgOSTU[Lost_Bottom][Lost_Left] = Lost_line;
                            for(uint8 k = i; k <= Deal_Top/2; k++)
                            {
                                if(imgOSTU[k][Lost_Left] == Lost_line)
                                {
                                    imgOSTU[k][Lost_Left] = Black;
                                    l_lost_tip--;
                                }
                                else if(imgOSTU[k][Lost_Left] == Black)
                                {
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    uint8 k = i;
                    for(; i <= Lost_Top; i++)
                    {
                        if(imgOSTU[i][Lost_Left] == Black)
                        {
                            l_center[l_lost_num] = ((uint32)i-1-(uint32)k)/2+(uint32)k;
                            l_lost_num++;
                            break;
                        }
                    }
                }
            }
            if(l_lost_num == 2)
            {
                break;
            }
        }
    }

    /*********** 获取右部边缘特征 ***********/
    if(r_lost_tip)
    {
        for(uint8 i = Deal_Bottom; i <= Deal_Top; i++)
        {
            if(imgOSTU[i][Lost_Right] == Lost_line)
            {
                if(i <= Deal_Bottom + length)
                {
                    for(uint8 j = Deal_Right; j >= Deal_Right - length; j--)
                    {
                        if(imgOSTU[Lost_Bottom][j] == Lost_line)
                        {
                            imgOSTU[Lost_Bottom][Lost_Right] = Lost_line;
                            for(uint8 k = i; k <= Deal_Top/2; k++)
                            {
                                if(imgOSTU[k][Lost_Right] == Lost_line)
                                {
                                    imgOSTU[k][Lost_Right] = Black;
                                    r_lost_tip--;
                                }
                                else if(imgOSTU[k][Lost_Left] == Black)
                                {
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    uint8 k = i;
                    for(; i <= Lost_Top; i++)
                    {
                        if(imgOSTU[i][Deal_Right + 2] == Black)
                        {
                            r_center[r_lost_num] = ((uint32)i-1-(uint32)k)/2+(uint32)k;
                            r_lost_num++;
                            break;
                        }
                    }
                }
            }
            if(r_lost_num == 2)
            {
                break;
            }
        }
    }

    /*********** 获取顶部边缘特征 ***********/
    if(t_lost_tip)
    {
        uint16 center[5] = {0};
        uint16 num = 0;

        for(uint8 i = Deal_Left; i <= Deal_Right; i++)
        {
            if(imgOSTU[Lost_Top][i] == Lost_line)
            {
                if(i <= Deal_Left + length && l_lost_num)
                {
                    for(uint8 j = Deal_Top; j >= Deal_Top - length; j--)
                    {
                        if(imgOSTU[j][Lost_Left] == Lost_line)
                        {
                            imgOSTU[Lost_Top][Lost_Left] = Lost_line;
                            for(uint8 k = j; k >= YM/2; k--)
                            {
                                if(imgOSTU[k][Lost_Left] == Lost_line)
                                {
                                    l_lost_tip--;
                                    imgOSTU[k][Lost_Left] = Black;
                                }
                                else if(imgOSTU[k][Lost_Left] == Black)
                                {
                                    l_lost_num--;
                                    break;
                                }
                            }
                            for(uint8 k = i; k <= XX; k++)
                            {
                                if(imgOSTU[Lost_Top][k] == Black)
                                {
                                    center[num] = ((uint32)k-1-(uint32)i)/2+(uint32)i;
                                    num++;
                                    i = k;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                else if(i+length >= XX || (i+length <= XX && imgOSTU[Lost_Top][i+length] == Lost_line))
                {
                    for(uint8 k = i; k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Top][k] == Black)
                        {
                            center[num] = ((uint32)k-1-(uint32)i)/2+(uint32)i;
                            num++;
                            i = k;
                            if(i >= Deal_Right - length && r_lost_num)
                            {
                                for(uint8 j = Deal_Top; j >= Deal_Top - length; j--)
                                {
                                    if(imgOSTU[j][Lost_Right] == Lost_line)
                                    {
                                        imgOSTU[Lost_Top][Lost_Right] = Lost_line;
                                        for(uint8 k = j; k >= YM/2; k--)
                                        {
                                            if(imgOSTU[k][Lost_Right] == Lost_line)
                                            {
                                                r_lost_tip--;
                                                imgOSTU[k][Lost_Right] = Black;
                                            }
                                            else if(imgOSTU[k][Lost_Right] == Black)
                                            {
                                                r_lost_num--;
                                                break;
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
                else
                {
                    for(uint8 k = i; k <= i + length; k++)
                    {
                        if(imgOSTU[Deal_Top + 2][k] == Lost_line)
                        {
                            imgOSTU[Deal_Top + 2][k] = Black;
                            t_lost_tip--;
                        }
                        else if(imgOSTU[Deal_Top + 2][k] == Black)
                        {
                            break;
                        }
                    }
                }
            }
        }

        if(num > 2)
        {
            uint8 Get_center[2] = {0};
            for(uint8 k = 0; k <= Deal_W/2-1; k++)
            {
                for(uint8 i = 0; i <= 4; i++)
                {
                    if(center[i] == Deal_W/2+k)
                    {
                        Get_center[t_lost_num] = Deal_W/2+k;
                        t_lost_num++;
                    }
                    if(center[i] == Deal_W/2-k)
                    {
                        Get_center[t_lost_num] = Deal_W/2-k;
                        t_lost_num++;
                    }
                }
                if(t_lost_num >= 2)
                {
                    t_center[0] = Get_center[0] < Get_center[1] ? Get_center[0] : Get_center[1];
                    t_center[1] = Get_center[0] > Get_center[1] ? Get_center[0] : Get_center[1];
                    break;
                }
            }
        }
        else
        {
            t_lost_num = num;
            t_center[0] = center[0];
            t_center[1] = center[1];
        }
    }
    else
    {
        if(l_lost_tip + r_lost_tip == 0 && hightest >= Deal_Top)
        {
            dis_Solidline = 1;
            int left_lost = 0, left_end = Deal_Top;
            int right_lost = 0, right_end = Deal_Top;

            // 检测左边界实线断开
            for(int y = Deal_Top; y >= Deal_Bottom; y--)
            {
                if(imgOSTU[y][Deal_Left] == Left_line || imgOSTU[y][Deal_Left] == Right_line)
                {
                    for(int k = y; k >= Deal_Bottom; k--)
                    {
                        if(imgOSTU[k][Deal_Left] != Black)
                        {
                            left_lost++;
                        }
                        else
                        {
                            if(k < Deal_H/2)
                            {
                                left_lost = 0;
                            }
                            else
                            {
                                left_end = (y+k)/2;
                            }
                            break;
                        }
                    }
                    break;
                }
            }

            // 检测右边界实线断开
            for(int y = Deal_Top; y >= Deal_Bottom; y--)
            {
                if(imgOSTU[y][Deal_Right] == Left_line || imgOSTU[y][Deal_Right] == Right_line)
                {
                    for(int k = y; k >= Deal_Bottom; k--)
                    {
                        if(imgOSTU[k][Deal_Right] != Black)
                        {
                            right_lost++;
                        }
                        else
                        {
                            if(k < Deal_H/2)
                            {
                                right_lost = 0;
                            }
                            else
                            {
                                right_end = (y+k)/2;
                            }
                            break;
                        }
                    }
                    break;
                }
            }

            // 补录边界
            if(left_lost && !right_lost)
            {
                imgOSTU[left_end-1][Lost_Left] = Lost_line;
                imgOSTU[left_end][Lost_Left] = Lost_line;
                imgOSTU[left_end+1][Lost_Left] = Lost_line;
                l_center[0] = left_end;
                l_lost_num = 1;
            }
            else if(right_lost && !left_lost)
            {
                imgOSTU[right_end-1][Lost_Right] = Lost_line;
                imgOSTU[right_end][Lost_Right] = Lost_line;
                imgOSTU[right_end+1][Lost_Right] = Lost_line;
                r_center[0] = right_end;
                r_lost_num = 1;
            }
            else if(hightest >= Deal_Top)
            {
                int x_mid = topmost[0];
                int x_l = topmost[0];
                int x_r = topmost[0];
                int y_mid = topmost[1];

                for(uint8 y = Deal_Top; y >= Deal_Bottom; y--)
                {
                    if(imgOSTU[y][x_mid] == White)
                    {
                        y_mid = y;
                        break;
                    }
                }
                for(uint8 x = x_mid; x >= Deal_Left; x--)
                {
                    if(imgOSTU[y_mid][x] == White)
                    {
                        imgOSTU[Lost_Top][x] = Lost_line;
                    }
                    else
                    {
                        x_l = x;
                        break;
                    }
                }

                for(uint8 x = x_mid; x <= Deal_Right; x++)
                {
                    if(imgOSTU[y_mid][x] == White)
                    {
                        imgOSTU[Lost_Top][x] = Lost_line;
                    }
                    else
                    {
                        x_r = x;
                        break;
                    }
                }

                t_lost_num = 1;
                t_center[0] = (x_l + x_r) / 2;
            }
        }
    }
}

/******************************************************************************
* 函数名称     : Get_start_center
* 描述         : 获取中线起点
******************************************************************************/
void Get_start_center(void)
{
    if(b_lost_tip >= white_width[Deal_Bottom] + 5 && b_lost_num == 1)
    {
        uint32 max = 0, min = Deal_Top, mid = Deal_Top/2;
        uint8 white_heigh[XM] = {0};

        for(uint8 i = start_point_l; i <= start_point_r; i++)
        {
            if(imgOSTU[Lost_Bottom][i] == Lost_line)
            {
                for(uint8 j = Deal_Bottom; j <= Deal_Top + 1; j++)
                {
                    if(imgOSTU[j][i] == Black || imgOSTU[j][i] == Left_line || imgOSTU[j][i] == Right_line)
                    {
                        white_heigh[i] = j;
                        max = max > j ? max : j;
                        min = min < j ? min : j;
                        break;
                    }
                }
            }
        }

        if(b_lost_tip >= 2*white_width[Deal_Bottom])
        {
            mid = (max - min)*2/3 + min;
        }
        else
        {
            mid = (max + min)/2;
        }

        for(uint8 i = start_point_l; i <= start_point_r; i++)
        {
            if(imgOSTU[Lost_Bottom][i] == Lost_line && white_heigh[i] < mid)
            {
                b_lost_tip--;
                imgOSTU[Lost_Bottom][i] = Black;
            }
        }
    }
    else if(b_lost_num == 2)
    {
        uint8 l_point = 0, r_point = XM;
        float l_num = 0, r_num = 0;
        float l_sum = 0, r_sum = 0;
        float l_average = 0, r_average = 0;

        for(uint8 i = start_point_l; i <= start_point_r; i++)
        {
            if(imgOSTU[Lost_Bottom][i] == Lost_line)
            {
                l_point = i;
                for(uint8 j = l_point; j <= Deal_Right; j++)
                {
                    if(imgOSTU[Lost_Bottom][j] == Lost_line)
                    {
                        for(uint8 k = Deal_Bottom + 1; k <= Deal_Top + 1; k++)
                        {
                            if(imgOSTU[k][j] == Left_line || imgOSTU[k][j] == Right_line)
                            {
                                l_num++;
                                l_sum = l_sum + k;
                                break;
                            }
                        }
                    }
                    else if(imgOSTU[Lost_Bottom][j] == Black)
                    {
                        break;
                    }
                }
                break;
            }
        }

        for(uint8 i = start_point_r; i >= start_point_l; i--)
        {
            if(imgOSTU[Lost_Bottom][i] == Lost_line)
            {
                r_point = i;
                for(uint8 j = i; j >= Deal_Left; j--)
                {
                    if(imgOSTU[Lost_Bottom][j] == Lost_line)
                    {
                        for(uint8 k = Deal_Bottom; k <= Deal_Top + 1; k++)
                        {
                            if(imgOSTU[k][j] == Black || imgOSTU[k][j] == Left_line || imgOSTU[k][j] == Right_line)
                            {
                                r_num++;
                                r_sum = r_sum + k;
                                break;
                            }
                        }
                    }
                    else if(imgOSTU[Lost_Bottom][j] == Black)
                    {
                        break;
                    }
                }
                break;
            }
        }

        l_average = l_sum/l_num;
        r_average = r_sum/r_num;

        if(l_average >= r_average)
        {
            for(uint8 i = r_point; i >= l_point; i--)
            {
                if(imgOSTU[Lost_Bottom][i] == Lost_line)
                {
                    imgOSTU[Lost_Bottom][i] = Black;
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            for(uint8 i = l_point; i <= r_point; i++)
            {
                if(imgOSTU[Lost_Bottom][i] == Lost_line)
                {
                    imgOSTU[Lost_Bottom][i] = Black;
                }
                else
                {
                    break;
                }
            }
        }
        b_lost_num = 1;
    }

    uint32 l_point = 0, r_point = 0;
    for(uint8 i = start_point_l; i < start_point_r; i++)
    {
        if(imgOSTU[Lost_Bottom][i] == Lost_line)
        {
            l_point = i;
            break;
        }
    }

    for(uint8 i = start_point_r; i > start_point_l; i--)
    {
        if(imgOSTU[Lost_Bottom][i] == Lost_line)
        {
            r_point = i;
            break;
        }
    }

    start_center_x = (l_point + r_point) / 2;
}

/******************************************************************************
* 函数名称     : Get_top_straightline
* 描述         : 绘制顶端直线
* 返回参数     : 终点行坐标
******************************************************************************/
uint8 Get_top_straightline(void)
{
    uint8 Black_num = 0;

    if(t_lost_num == 1)
    {
        if((t_lost_tip >= white_width[Deal_Top] + 5 && t_lost_num == 1) ||
           (t_lost_tip >= 3*white_width[Deal_Top]))
        {
            uint32 max = 0, min = Deal_Top, mid = Deal_Top/2;
            uint8 white_low[XM] = {0};

            for(uint8 i = Deal_Left; i <= Deal_Right; i++)
            {
                if(imgOSTU[Lost_Top][i] == Lost_line)
                {
                    for(uint8 j = Deal_Top; j >= 0; j--)
                    {
                        if(imgOSTU[j][i] != White)
                        {
                            white_low[i] = j;
                            max = max > j ? max : j;
                            min = min < j ? min : j;
                            break;
                        }
                    }
                }
            }

            mid = (max + min)/2;

            for(uint8 i = Deal_Left; i <= Deal_Right; i++)
            {
                if(imgOSTU[Lost_Top][i] == Lost_line && white_low[i] > mid)
                {
                    t_lost_tip--;
                    imgOSTU[Deal_Top + 2][i] = Black;
                }
            }

            uint32 l_point = 0, r_point = 0;
            end_center_y = Deal_Top;

            for(uint8 i = Deal_Left; i <= Deal_Right; i++)
            {
                if(imgOSTU[Lost_Top][i] == Lost_line)
                {
                    l_point = i;
                    break;
                }
            }

            for(uint8 i = Deal_Right; i >= Deal_Left; i--)
            {
                if(imgOSTU[Lost_Top][i] == Lost_line)
                {
                    r_point = i;
                    break;
                }
            }

            end_center_x = (l_point + r_point) / 2;
        }
        else
        {
            end_center_y = Deal_Top;
            end_center_x = t_center[0];
        }

        if(end_center_y <= start_center_y)
        {
            return 0;
        }

        float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
        for(uint8 i = start_center_y; i <= end_center_y; ++i)
        {
            mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
            if(imgOSTU[i][(uint8)mid_line[i]] != White)
            {
                Black_num++;
            }
            imgOSTU[i][(uint8)mid_line[i]] = Control_line;
        }
    }
    else if(t_lost_num == 2)
    {
        end_center_y = Deal_Top;
        uint32 t_center_l = t_center[0], t_center_r = t_center[1];

        uint8 left_line[YM] = {XM/2};
        uint8 right_line[YM] = {XM/2};
        uint8 Black_num_l = 0, Black_num_r = 0;

        float kl = ((float)t_center_l - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
        for(uint8 i = start_center_y; i <= end_center_y; ++i)
        {
            left_line[i] = (uint8)(start_center_x + kl * (i - start_center_y));
            if(imgOSTU[i][left_line[i]] != White)
            {
                Black_num_l++;
            }
            else
            {
                imgOSTU[i][left_line[i]] = Judge_line;
            }
        }

        float kr = (float)((float)t_center_r - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
        for(uint8 i = start_center_y; i <= end_center_y; ++i)
        {
            right_line[i] = (uint8)(start_center_x + kr * (i - start_center_y));
            if(imgOSTU[i][right_line[i]] != White)
            {
                Black_num_r++;
            }
            else
            {
                imgOSTU[i][right_line[i]] = Judge_line;
            }
        }

        if(Black_num_l > Black_num_r)
        {
            end_center_x = t_center_r;
            Black_num = Black_num_r;
            for(uint8 i = start_center_y; i <= end_center_y; ++i)
            {
                mid_line[i] = right_line[i];
                imgOSTU[i][mid_line[i]] = Control_line;
            }
        }
        else
        {
            end_center_x = t_center_l;
            Black_num = Black_num_l;
            for(uint8 i = start_center_y; i <= end_center_y; ++i)
            {
                mid_line[i] = left_line[i];
                imgOSTU[i][mid_line[i]] = Control_line;
            }
        }
    }

    if(start_center_y == Deal_Bottom && Black_num < 3)
    {
        IF = straightlineL;
    }
    else if(start_center_y == Deal_Bottom)
    {
        IF = straightlineS;
    }

    return end_center_y;
}

/******************************************************************************
* 函数名称     : Left_curve_line
* 描述         : 绘制左边界曲线
* 返回参数     : 终点行坐标
******************************************************************************/
uint8 Left_curve_line(void)
{
    end_center_x = Deal_Left;
    end_center_y = l_center[l_lost_num - 1];
    uint8 Black_num = 0;

    if(end_center_y <= start_center_y)
    {
        return 0;
    }

    float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
    for(uint8 i = start_center_y; i <= end_center_y; ++i)
    {
        mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
        if(imgOSTU[i][(uint8)mid_line[i]] != White)
        {
            Black_num++;
        }
        imgOSTU[i][(uint8)mid_line[i]] = Control_line;
    }

    IF = curve;
    return end_center_y;
}

/******************************************************************************
* 函数名称     : Right_curve_line
* 描述         : 绘制右边界曲线
* 返回参数     : 终点行坐标
******************************************************************************/
uint8 Right_curve_line(void)
{
    end_center_x = Deal_Right;
    end_center_y = r_center[r_lost_num - 1];
    uint8 Black_num = 0;

    if(end_center_y <= start_center_y)
    {
        return 0;
    }

    float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
    for(uint8 i = start_center_y; i <= end_center_y; ++i)
    {
        mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
        if(imgOSTU[i][(uint8)mid_line[i]] != White)
        {
            Black_num++;
        }
        imgOSTU[i][(uint8)mid_line[i]] = Control_line;
    }

    IF = curve;
    return end_center_y;
}

/******************************************************************************
* 函数名称     : Deal_crossroads
* 描述         : 十字路口识别和处理
* 返回参数     : 终点行坐标
******************************************************************************/
uint8 Deal_crossroads()
{
    if(elements_Mode == YES && elements_Lock[elements_index] != Disappear_)
    {
        return 0;
    }

    if((b_lost_num + t_lost_num + l_lost_num + r_lost_num) <= 3)
    {
        return 0;
    }

    if((b_lost_num + t_lost_num + l_lost_num + r_lost_num) == 3 && l_lost_num + r_lost_num == 0)
    {
        return 0;
    }

    if(b_lost_num == 0 || t_lost_num == 0)
    {
        return 0;
    }

    if(b_lost_tip > Deal_W/2)
    {
        return 0;
    }

    if((b_lost_num + t_lost_num + l_lost_num + r_lost_num) == 4)
    {
        uint8 point[4][2];
        uint8 j = 0;

        if(b_lost_num)
            for(int i = 0; i <= b_lost_num-1 && j <= 3; i++)
            {
                point[j][0] = b_center[i];
                point[j][1] = Deal_Bottom;
                j++;
            }

        if(r_lost_num)
            for(int i = 0; i <= r_lost_num-1 && j <= 3; i++)
            {
                point[j][0] = Deal_Right;
                point[j][1] = r_center[i];
                j++;
            }

        if(t_lost_num)
            for(int i = t_lost_num-1; i >= 0 && j <= 3; i--)
            {
                point[j][0] = t_center[i];
                point[j][1] = Deal_Top;
                j++;
            }

        if(l_lost_num)
            for(int i = l_lost_num-1; i >= 0 && j <= 3; i--)
            {
                point[j][0] = Deal_Left;
                point[j][1] = l_center[i];
                j++;
            }

        if(j < 3)
        {
            return 0;
        }

        if(my_abs(point[1][0] - point[2][0]) < Deal_W*2/5 ||
           my_abs(point[2][0] - point[3][0]) < Deal_W*2/5 ||
           my_abs(point[1][0] - point[3][0]) < Deal_W*2/5)
        {
            return 0;
        }

        uint8 Black_num = 0;
        float k = ((float)point[2][0] - (float)point[0][0]) / ((float)point[2][1] - (float)point[0][1]);
        for(uint8 i = point[0][1]; i <= point[2][1]; ++i)
        {
            if(imgOSTU[i][(uint8)(point[2][0] + k * (i - point[0][1]))] != White)
            {
                Black_num++;
            }
        }

        if(Black_num < Deal_H/2)
        {
            start_center_x = point[0][0];
            start_center_y = point[0][1];
            end_center_x = point[2][0];
            end_center_y = point[2][1];
            for(uint8 i = start_center_y; i <= end_center_y; ++i)
            {
                mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
                imgOSTU[i][(uint8)mid_line[i]] = Control_line;
            }
            IF = crossroads;
            return end_center_y;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}


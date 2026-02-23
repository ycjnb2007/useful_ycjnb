#include "zf_common_headfile.h"

uint8 image_copy[MT9V03X_H][MT9V03X_W];         //复制原来捕获到的图像
uint8 image_cut[CUT_H][CUT_W];         //裁剪后的图像
uint8 image[CUT_H][CUT_W];          //[行][列] 裁剪后二值化的图像




/**
函数：裁剪图像的宽和高

说明：将原来的188*120图像进行裁剪，裁剪的参数在.h文件中
      
**/
//void crop_image(void) 
//{
//    // 计算裁剪起始位置
//    const int start_row = MT9V03X_H - BOTTOM_CUT_HEIGHT; // 起始行（高度方向）
//    const int start_col = LEFT_CUT;                      // 起始列（宽度方向）
//
//    // 循环填充裁剪后的图像
//    for (int i = 0; i < CUT_HEIGHT; i++) 
//    {
//        for (int j = 0; j < CUT_WIDTH; j++) 
//        {
//            // 将原始图像对应位置的值复制到裁剪图像中
//            image_cut[i][j] = image_copy[start_row + i][start_col + j];
//        }
//    }
//
//}

// 高度优化的裁剪函数
void crop_image() 
{
  // 计算裁剪起始点 - 对称裁剪宽度
    const int start_y = MT9V03X_H - BOTTOM_CUT_HEIGHT;  // 从底部开始裁剪
    const int start_x = LEFT_CUT;                       // 从左侧裁剪边界开始
    
    // 边界检查（确保不会越界）
    zf_assert(start_y >= 0);
    zf_assert(start_x >= 0 );
    zf_assert((start_x + CUT_W) <= MT9V03X_W );
    
    uint8_t *src_ptr = &image_copy[start_y][start_x];  // 正确起始位置
    uint8_t *dst_ptr = &image_cut[0][0];
    const size_t bytes_per_line = CUT_W * sizeof(uint8_t);
    
    for (int i = 0; i < CUT_H; i++) 
    {
        memcpy(dst_ptr, src_ptr, bytes_per_line);
        src_ptr += MT9V03X_W;  // 移动到原始图像下一行
        dst_ptr += CUT_W;      // 移动到目标图像下一行
    }
}


/**
函数：黑白/白黑跳变点检测

返回：如果找到，则返回1，如果未找到返回0
      
**/
//优化后
inline uint8 Trip_point(uint8 a, uint8 b)
{
    // 位运算融合条件判断：异或检测互补性，且 a 必须为极值（0或255）
    return ((a ^ b) == 0xFF) & ((a == 0x00) | (a == 0xFF));
}



/**
函数简介： 二值化图像

说明：     调用前请使用        

示例:      Show_binaryzation(220);         

**/
//优化
//输入阈值二值化
void image_binaryzation(uint8_t threshold) 
{
    // 获取指向图像数据的指针
    uint8_t *in_ptr = &image_cut[0][0];
    uint8_t *out_ptr = &image[0][0];
    const uint8_t *const end_ptr = in_ptr + CUT_H * CUT_W;

    // 循环展开处理（每次处理4个像素）
    while ((end_ptr - in_ptr) >= 4) 
    {
        *out_ptr++ = (*in_ptr++ > threshold) ? 255 : 0;
        *out_ptr++ = (*in_ptr++ > threshold) ? 255 : 0;
        *out_ptr++ = (*in_ptr++ > threshold) ? 255 : 0;
        *out_ptr++ = (*in_ptr++ > threshold) ? 255 : 0;
    }

    // 处理剩余像素（0-3个）
    while (in_ptr < end_ptr) 
    {
        *out_ptr++ = (*in_ptr++ > threshold) ? 255 : 0;
    }
}

/**
函数简介：   大津法求阈值


**/
// 大津法计算最佳阈值
uint8 th = 0;
uint8_t otsu_threshold() 
{
    uint32_t hist[256] = {0};       // 灰度直方图
    uint32_t total_pixels = CUT_H * CUT_W;
    uint32_t sum_total = 0;         // 图像总灰度值

    // 1. 计算直方图和总灰度值
    for (int i = 0; i < CUT_H; i++) 
    {
        for (int j = 0; j < CUT_W; j++) 
        {
            uint8_t gray = image_cut[i][j];
            hist[gray]++;
            sum_total += gray;
        }
    }

    uint32_t sum_back = 0;        // 背景灰度总和
    uint32_t w_back = 0;          // 背景像素数
    float max_variance = 0.0;     // 最大类间方差
    uint8_t best_threshold = 0;   // 最佳阈值

    // 2. 遍历所有可能的阈值
    for (uint16_t t = 0; t < 256; t++) 
    {
        w_back += hist[t];        // 更新背景像素数
        if (w_back == 0) continue;

        uint32_t w_fore = total_pixels - w_back; // 前景像素数
        if (w_fore == 0) break;

        sum_back += t * hist[t];  // 更新背景灰度总和

        // 计算前景和背景的平均灰度
        float mean_back = (float)sum_back / w_back;
        float mean_fore = (float)(sum_total - sum_back) / w_fore;

        // 计算类间方差
        float variance = (float)w_back * w_fore;
        variance *= (mean_back - mean_fore) * (mean_back - mean_fore);

        // 3. 更新最佳阈值
        if (variance > max_variance) 
        {
            max_variance = variance;
            best_threshold = t;
        }
    }

    return best_threshold;
}




/**
函数简介：   补黑框
**/
void Draw_Black_Box()
{
  uint8 i = 0,j = 0;
    for(i = 2; i < CUT_H; i++)
    {
        image[i][1] = 0;
        image[i][CUT_W - 2] = 0;   
        
    }
    for(j = 1; j < CUT_W - 1; j++)
    {
        image[1][j] = 0;
    }
}




/**
* 函数简介：      爬线算法找起点
* 特殊说明：      无
* 形  参：        uint8 start_row              找起点的图像行Y坐标
*                uint8(*image)[MT9V03X_W]        要处理的图像
*                uint8 *l_start_point          存储左侧起始点的数组（全局变量）
*                uint8 *r_start_point          存储右侧起始点的数组（全局变量）
*
* 示例：          Get_Start_Point(CUT_H - 2, image, L_start_point, R_start_point);
* 返回值：        两边都找到返回1，否则返回0.
*/
uint8 L_start_point[2] = { 0 }; //左侧起始点
uint8 R_start_point[2] = { 0 }; //右侧起始点
uint8 x_start = 0;
uint8 y_start = 0;
uint8 Get_Start_Point(uint8 start_row, uint8(*image)[CUT_W], uint8 *l_start_point, uint8 *r_start_point)
{
    uint8 i = 0, j = 0;
    uint8 L_Is_Found = 0, R_Is_Found = 0;
    // 使用新的列范围定义
    uint8 L_Start_X = LEFT_COL_LIMIT;        // 左侧起始X坐标
    uint8 R_Start_X = RIGHT_COL_LIMIT;       // 右侧起始X坐标

    // 指定的行没找到起点时，向上走一行继续找，最多找FIND_START_POINT_MAX_LINE行（在.h中修改）
    for(j = 0; j < FIND_START_POINT_MAX_LINE; j += 2)
    {
        // 检查行范围是否有效
        if(start_row < UP_ROW_LIMIT) 
        {
            break;  // 超过最小行限制，退出循环
        }
        
        l_start_point[1] = start_row;  // y坐标
        r_start_point[1] = start_row;  // y坐标

        // 左侧向右侧找黑白跳变点 (扫描范围: LEFT_COL_LIMIT ~ RIGHT_COL_LIMIT-1)
        for (i = L_Start_X; i <= RIGHT_COL_LIMIT - 1; i++) 
        {
            if (Trip_point(image[start_row][i + 1], image[start_row][i])) 
            {
                l_start_point[0] = i;   // 记录X坐标
                L_Is_Found = 1;         // 设置找到标志
                break;
            }
        }

        // 右侧向左找黑白跳变点 (扫描范围: RIGHT_COL_LIMIT ~ LEFT_COL_LIMIT+1)
        for (i = R_Start_X; i >= LEFT_COL_LIMIT + 1; i--) 
        {
            if (Trip_point(image[start_row][i - 1], image[start_row][i])) 
            {
                r_start_point[0] = i;   // 记录X坐标
                R_Is_Found = 1;         // 设置找到标志
                break;
            }
        }

        if(L_Is_Found && R_Is_Found) 
        {
            return 1;  // 两边都找到
        }
        else 
        {
            start_row -= 2;  // 向上移动一行
        }
    }
    return 0;  // 未找到或超出行范围
}



/**函数：迷宫算法爬线**/
//**************方向计算****************
//记录方向时，要将每个坐标变为单一数字，便于简化后续算法。例如八邻域的方向（逆时针    顺时针）
//3  2  1               1  2  3
//4     0               0     4
//5  6  7               7  6  5

//{-1,-1},{0,-1},{+1,-1},
//{-1, 0},       {+1, 0},
//{-1,+1},{0,+1},{+1,+1},
//迷宫缺点是无法像八邻域一样逐步记录方向，但八邻域无非就是以上八个点，且与中心点差值固定，不会随迷宫算法的朝向和移动方向而改变，因此先将每个横坐标乘 3
//{-3,-1},{0,-1},{+3,-1},
//{-3, 0},       {+3, 0},
//{-3,+1},{0,+1},{+3,+1},
//再将横坐标减去纵坐标
// -2, 1, 4
// -3,    3
// -4,-1, 2
//由此可以得到一个八邻方向坐标。只需在每次移动后，在方向数组里记录对应数字，就可确定生长方向
//此算法无任何原理，只是为了得到八个不一样的值可以用来判定方向，横坐标可以乘大于 2 的任意值，2以内会出现重复
//************************************
//******************自适应方向迷宫参数************************

const int8 L_Face_Dir[4][2] = { {0,-1},{1,0},{0,1},{-1,0} };  //左侧迷宫面向
//  0
//3   1
//  2

const int8 L_Face_Dir_L[4][2] = { {-1,-1},{1,-1},{1,1},{-1,1} };  //左侧面向的左前方
//0   1
//
//3   2

const int8 R_Face_Dir[4][2] = { {0,-1},{1,0},{0,1},{-1,0} };  //右侧迷宫面向
//  0
//3   1
//  2

const int8 R_Face_Dir_R[4][2] = { {1,-1},{1,1},{-1,1},{-1,-1} };  //右侧面向的右前方
//3   0
//
//2   1

const int8 Square_0[25][2] = {              //一个5 * 5的矩阵，用来求中心点周围的局部阈值（局部阈值）
{-2,-2},{-1,-2},{0,-2},{+1,-2},{+2,-2},
{-2,-1},{-1,-1},{0,-1},{+1,-1},{+2,-1},
{-2,-0},{-1, 0},{0, 0},{+1, 0},{+2,-0},
{-2,+1},{-1,+1},{0,+1},{+1,+1},{+2,+1},
{-2,+2},{-1,+2},{0,+2},{+1,+2},{+2,+2}
};

//迷宫单侧停止爬线标志位
uint8 L_Stop_Flag = 0;
uint8 R_Stop_Flag = 0;

uint8 L_line[MAX_LINE][2] = {0};
uint8 R_line[MAX_LINE][2] = {0};
uint16 L_Statics = 0;
uint16 R_Statics = 0;
int8 l_dir[MAX_LINE] = {0};
int8 r_dir[MAX_LINE] = {0};
uint8 X_meet = 0;
uint8 Y_meet = 0;
float L_Thres_Record[MAX_LINE] = {0};
float R_Thres_Record[MAX_LINE] = {0};
//**********************************************************
/******
* 函数功能：      求取赛道二维数组边线
* 特殊说明：      基于上交代码的自适应迷宫优化后的自适应八向迷宫
* 形  参：        uint16 Break_Flag         最大循环次数，防止卡死程序，一般为3~4倍图像宽度
*                uint8(*image)[CUT_W]       提取边线的图像
*                uint8(*l_line)[2]          存放左侧边线的二维数组
*                uint8(*r_line)[2]          存放右侧边线的二维数组
*                int8 *l_dir                存放左侧边线每个点的生长方向
*                int8 *r_dir                存放右侧边线每个点的生长方向
*                uint16 *l_stastic          记录左侧边线点的个数
*                uint16 *r_stastic          记录右侧边线点的个数
*                uint8 *x_meet              记录左右两侧爬线相遇点的X坐标
*                uint8 *y_meet              记录左右两侧爬线相遇点的Y坐标
*                uint8 l_start_x            左侧爬线起始点的X坐标
*                uint8 l_start_y            左侧爬线起始点的Y坐标
*                uint8 r_start_x            右侧爬线起始点的X坐标
*                uint8 r_start_y            右侧爬线起始点的Y坐标
*                uint8 clip_value           计算每个阈值时相加的经验值，一般为-5 ~ 5，避免强行分割，可直接设为0
*
* 示例：      
          Dir_Labyrinth_5(MAX_LINE,     image,
                          L_line,       R_line,
                          l_dir,        r_dir,
                          &L_Statics,   &R_Statics,
                          &X_meet,      &Y_meet,
                          L_start_point[0],L_start_point[1],
                          R_start_point[0],R_start_point[1],0)
* 返回值：        无
*/
void Dir_Labyrinth_5(uint16 Break_Flag, uint8(*image)[CUT_W], uint8(*l_line)[2], uint8(*r_line)[2], int8* l_dir, int8* r_dir, uint16* l_stastic, uint16* r_stastic, uint8* x_meet, uint8* y_meet,
    uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 clip_value)
{
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    // 左边变量
    uint8  L_Center_Point[2] = { 0 };     // 存放每次找到的XY坐标
    uint16 L_Data_Statics = 0;          // 统计左边找到的边线点的个数

    uint8  L_Front_Value = 0;           // 左侧 面向的前方点的灰度值
    uint8  L_Front_L_Value = 0;         // 左侧 面向的左前方点的灰度值

    uint8  L_Dir = 0;                   // 此参数用于转向
    uint8  L_Turn_Num = 0;              // 记录转向次数

    // 右边变量
    uint8  R_Center_Point[2] = { 0 };     // 存放每次找到的XY坐标
    uint16 R_Data_Statics = 0;          // 统计右边找到的边线点的个数

    uint8  R_Front_Value = 0;           // 右侧 面向的前方点的灰度值
    uint8  R_Front_R_Value = 0;         // 右侧 面向的左前方点的灰度值

    uint8  R_Dir = 0;                   // 此参数用于转向
    uint8  R_Turn_Num = 0;              // 记录转向次数

    // 第一次更新坐标点
    L_Center_Point[0] = l_start_x + 1;  // x
    L_Center_Point[1] = l_start_y;      // y
    R_Center_Point[0] = r_start_x - 1;  // x
    R_Center_Point[1] = r_start_y;      // y

    // 开启方向迷宫循环
    while (Break_Flag--)
    {
        // 左边
        if (L_Stop_Flag == 0)
        {
            l_line[L_Data_Statics][0] = L_Center_Point[0];
            l_line[L_Data_Statics][1] = L_Center_Point[1];
            L_Data_Statics++;

        L_Judge_Again:
            if (L_Stop_Flag == 0)
            {
                L_Front_Value = image[L_Center_Point[1] + L_Face_Dir[L_Dir][1]][L_Center_Point[0] + L_Face_Dir[L_Dir][0]];
                L_Front_L_Value = image[L_Center_Point[1] + L_Face_Dir_L[L_Dir][1]][L_Center_Point[0] + L_Face_Dir_L[L_Dir][0]];
                
                if (L_Front_Value == 0)  // 使用固定阈值0判断黑色像素
                {
                    L_Dir = (L_Dir + 1) % 4;
                    L_Turn_Num++;
                    if (L_Turn_Num == 4)
                    {
                        L_Stop_Flag = 1;
                    }
                    goto L_Judge_Again;
                }
                else if (L_Front_L_Value == 0)  // 使用固定阈值0
                {
                    L_Center_Point[0] += L_Face_Dir[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir[L_Dir][1];
                    
                    // 修改点1：使用新边界宏
                    L_Center_Point[0] = CLAMP(L_Center_Point[0], LEFT_COL_LIMIT, RIGHT_COL_LIMIT);
                    L_Center_Point[1] = CLAMP(L_Center_Point[1], UP_ROW_LIMIT, DOWN_ROW_LIMIT);
                    
                    if (L_Data_Statics > 0) {
                        l_dir[L_Data_Statics - 1] = (L_Face_Dir[L_Dir][0] * 3) - L_Face_Dir[L_Dir][1];
                    }
                    L_Turn_Num = 0;
                }
                else
                {
                    L_Center_Point[0] += L_Face_Dir_L[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir_L[L_Dir][1];
                    
                    // 修改点2：使用新边界宏
                    L_Center_Point[0] = CLAMP(L_Center_Point[0], LEFT_COL_LIMIT, RIGHT_COL_LIMIT);
                    L_Center_Point[1] = CLAMP(L_Center_Point[1], UP_ROW_LIMIT, DOWN_ROW_LIMIT);
                    
                    if (L_Data_Statics > 0) {
                        l_dir[L_Data_Statics - 1] = (L_Face_Dir_L[L_Dir][0] * 3) - L_Face_Dir_L[L_Dir][1];
                    }
                    L_Dir = (L_Dir + 3) % 4;
                    L_Turn_Num = 0;
                }
                
                // 死区检测(保留)
                if (L_Data_Statics >= 5)
                {
                    if (l_line[L_Data_Statics - 1][0] == l_line[L_Data_Statics - 5][0] &&
                        l_line[L_Data_Statics - 1][1] == l_line[L_Data_Statics - 5][1])
                    {
                        L_Stop_Flag = 1;
                    }
                }
            }
        }

        // 右边
        if (R_Stop_Flag == 0)
        {
            r_line[R_Data_Statics][0] = R_Center_Point[0];
            r_line[R_Data_Statics][1] = R_Center_Point[1];
            R_Data_Statics++;

        R_Judgme_Again:
            if (R_Stop_Flag == 0)
            {
                R_Front_Value = image[R_Center_Point[1] + R_Face_Dir[R_Dir][1]][R_Center_Point[0] + R_Face_Dir[R_Dir][0]];
                R_Front_R_Value = image[R_Center_Point[1] + R_Face_Dir_R[R_Dir][1]][R_Center_Point[0] + R_Face_Dir_R[R_Dir][0]];
                
                if (R_Front_Value == 0)  // 使用固定阈值0
                {
                    R_Dir = (R_Dir + 3) % 4;
                    R_Turn_Num++;
                    if (R_Turn_Num == 4)
                    {
                        R_Stop_Flag = 1;
                    }
                    goto R_Judgme_Again;
                }
                else if (R_Front_R_Value == 0)  // 使用固定阈值0
                {
                    R_Center_Point[0] += R_Face_Dir[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir[R_Dir][1];
                    
                    // 修改点3：使用新边界宏
                    R_Center_Point[0] = CLAMP(R_Center_Point[0], LEFT_COL_LIMIT, RIGHT_COL_LIMIT);
                    R_Center_Point[1] = CLAMP(R_Center_Point[1], UP_ROW_LIMIT, DOWN_ROW_LIMIT);
                    
                    if (R_Data_Statics > 0) {
                        r_dir[R_Data_Statics - 1] = R_Face_Dir[R_Dir][0] * 3 - R_Face_Dir[R_Dir][1];
                    }
                    R_Turn_Num = 0;
                }
                else
                {
                    R_Center_Point[0] += R_Face_Dir_R[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir_R[R_Dir][1];
                    
                    // 修改点4：使用新边界宏
                    R_Center_Point[0] = CLAMP(R_Center_Point[0], LEFT_COL_LIMIT, RIGHT_COL_LIMIT);
                    R_Center_Point[1] = CLAMP(R_Center_Point[1], UP_ROW_LIMIT, DOWN_ROW_LIMIT);
                    
                    if (R_Data_Statics > 0) {
                        r_dir[R_Data_Statics - 1] = R_Face_Dir_R[R_Dir][0] * 3 - R_Face_Dir_R[R_Dir][1];
                    }
                    R_Dir = (R_Dir + 1) % 4;
                    R_Turn_Num = 0;
                }
                
                // 死区检测(保留)
                if (R_Data_Statics >= 5)
                {
                    if (r_line[R_Data_Statics - 1][0] == r_line[R_Data_Statics - 5][0] &&
                        r_line[R_Data_Statics - 1][1] == r_line[R_Data_Statics - 5][1])
                    {
                        R_Stop_Flag = 1;
                    }
                }
            }
        }

        // 相遇点检测(完全保留)
        if (L_Stop_Flag == 0 && R_Stop_Flag == 0)
        {
            if ((abs(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 1) &&
                (abs(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 1))
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;
                break;
            }
        }
        else
        {
            if ((abs(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 3) &&
                (abs(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 3))
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;
                break;
            }
        }
    }
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    *l_stastic = L_Data_Statics;
    *r_stastic = R_Data_Statics;
}





#include "zf_common_headfile.h"


uint8 image_copy[MT9V03X_H][MT9V03X_W];         //复制原来捕获到的图像
uint8 image_cut[CUT_HEIGHT][MT9V03X_W];         //裁剪后的图像
uint8 image[CUT_HEIGHT][MT9V03X_W];          //[行][列] 裁剪后二值化的图像


uint8 x_start = 0;
uint8 y_start = 0;


//裁剪图像

void crop_image_from_bottom() 
{
    for (int row = 0; row < CUT_HEIGHT; row++) 
    {
        memcpy(
            image_cut[row],
            image_copy[CUT_START_ROW + row],
            sizeof(uint8) * MT9V03X_W
        );
    }
}




/**
函数：黑白/白黑跳变点检测

说明：如果左右两个点为255和0，则说明找到了跳变点
      
**/
//uint8 Trip_point(uint8 a, uint8 b)
//{
//  if(a == 255 && b == 0)
//  {
//    return 1;
//  }else if(a == 0 && b == 255)
//  {
//    return 1;
//  }else
//  {
//    return 0;
//  }
//  
//}

////优化1
inline uint8 Trip_point(uint8 a, uint8 b)
{
    // 位运算融合条件判断：异或检测互补性，且 a 必须为极值（0或255）
    return ((a ^ b) == 0xFF) & ((a == 0x00) | (a == 0xFF));
}



/**
函数简介： 二值化图像

说明：     调用前请使用          memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);    //将原始图像拷贝到 image_copy  这个数组中  

示例:      Show_binaryzation(220);         

**/
//void Show_binaryzation(uint8 threshold)
//{
//    for(uint8 i = 0; i < CUT_HEIGHT; i++)
//    {
//      for(uint8 j = 0; j < MT9V03X_W; j++)
//      {
//          if(image_cut[i][j] < threshold)
//          {
//            image[i][j] = 0;
//          }
//          else
//          {
//            image[i][j] = 255;
//          }
//      } 
//    }
//}

////优化1
//// 预先生成二值化查找表（LUT），大幅减少循环内计算
//void Show_binaryzation(uint8 threshold) {
//    // 创建查找表：0~255每个灰度值对应的二值化结果
//    uint8 lut[256];
//    for (int val = 0; val < 256; val++) {
//        lut[val] = (val < threshold) ? 0 : 255;
//    }
//
//    // 获取连续内存指针（按行优先存储）
//    uint8 *src = (uint8 *)image_cut; // 源图像起始地址
//    uint8 *dst = (uint8 *)image;      // 目标图像起始地址
//    const int total_pixels = CUT_HEIGHT * MT9V03X_W; // 总像素数
//
//    // 单层循环连续访问内存（避免二维索引计算）
//    for (int i = 0; i < total_pixels; i++) {
//        dst[i] = lut[src[i]]; // 直接查表赋值
//    }
//}

////优化2
//输入阈值二值化
void Show_binaryzation(uint8 threshold)
{
    // 生成查找表（LUT），避免每个像素重复条件判断
    uint8 lut[256];
    for (int v = 0; v < 256; v++) 
    {
        lut[v] = (v < threshold) ? 0 : 255;
    }

    // 将二维数组视为一维连续内存处理
    uint8 *src = &image_cut[0][0];
    uint8 *dst = &image[0][0];
    uint32 total_pixels = (uint32)CUT_HEIGHT * MT9V03X_W;

    // 单循环连续访问内存，提升缓存效率
    for (uint32 k = 0; k < total_pixels; k++) 
    {
        dst[k] = lut[src[k]];
    }
}

/**
函数简介：   大津法求阈值


**/
uint8 otsu_threshold(void) 
{
    int histogram[256] = {0};

    // 统计灰度直方图
    for (int y = 0; y < CUT_HEIGHT; y++) 
    {
        for (int x = 0; x < MT9V03X_W; x++) 
        {
            uint8 gray = image_cut[y][x];
            histogram[gray]++;
        }
    }

    int n_pixels = CUT_HEIGHT * MT9V03X_W;
    long long sum_total = 0;

    // 计算总灰度值
    for (int i = 0; i < 256; i++) 
    {
        sum_total += (long long)i * histogram[i];
    }

    long long sumB = 0;      // 背景像素数累积
    long long sumF = 0;      // 背景总灰度值累积
    float max_variance = -1.0f;
    int best_threshold = 0;

    // 遍历所有可能的阈值
    for (int t = 0; t < 256; t++) 
    {
        sumB += histogram[t];
        sumF += (long long)t * histogram[t];

        // 跳过无效分割（全背景或全前景）
        if (sumB == 0 || sumB == n_pixels) 
        {
            continue;
        }

        // 计算类间方差分子和分母
        long long numerator = sumF * n_pixels - sumB * sum_total;
        long long denominator = sumB * (n_pixels - sumB);
        float variance = (float)(numerator * numerator) / (float)denominator;

        // 更新最大方差和最佳阈值
        if (variance > max_variance) 
        {
            max_variance = variance;
            best_threshold = t;
        }
    }

    // 处理所有像素灰度相同的情况
    if (max_variance == -1.0f) 
    {
        best_threshold = (int)(sum_total / n_pixels);
    }

    return (uint8)best_threshold;
}



/**
函数简介：大津法+二值化结合

**/

// 综合大津法求阈值与二值化的高效实现
uint8 otsu_binaryze_optimized(void)
{
    /* 阶段1：直方图统计与阈值计算 */
    int histogram[256] = {0};
    uint32 total_pixels = CUT_HEIGHT * MT9V03X_W;
    uint8* src = (uint8*)image_cut;  // 一维化输入

    // 单次连续内存访问统计直方图
    for (uint32 k = 0; k < total_pixels; k++) {
        histogram[src[k]]++;
    }

    /* 大津法核心计算 */
    long long sum_total = 0;
    for (int i = 0; i < 256; i++) {
        sum_total += (long long)i * histogram[i];
    }

    int best_threshold = 0;
    float max_variance = -1.0f;
    long long sumB = 0, sumF = 0;

    for (int t = 0; t < 256; t++) {
        sumB += histogram[t];
        sumF += (long long)t * histogram[t];
        
        if (sumB == 0 || sumB == total_pixels) continue;

        // 整数优化计算（避免浮点除法）
        long long numerator = sumF * total_pixels - sumB * sum_total;
        long long denominator = sumB * (total_pixels - sumB);
        long long variance_sq = numerator * numerator / denominator;

        if (variance_sq > max_variance) {
            max_variance = (float)variance_sq;
            best_threshold = t;
        }
    }

    /* 处理全同色特殊情况 */
    if (max_variance < 0) {
        best_threshold = (int)(sum_total / total_pixels);
    }

    /* 阶段2：LUT二值化 */
    uint8 lut[256];
    uint8 thres = (uint8)best_threshold;
    for (int v = 0; v < 256; v++) {
        lut[v] = (v < thres) ? 0 : 255;
    }

    // 连续内存块操作
    uint8* dst = (uint8*)image;
    for (uint32 k = 0; k < total_pixels; k++) {
        dst[k] = lut[src[k]];
    }

    return thres;
}



/**
函数简介：   补黑框
**/
void Draw_Black_Box(uint8 black_box_value)
{
    uint8 i = 0,j = 0;
    for(i = 1; i < CUT_HEIGHT; i++)
    {
        image[i][MT9V03X_W - 2] = black_box_value;
        image[i][1] = black_box_value;
    }
    for(j = 1; j < MT9V03X_W - 2; j++)
    {
        image[1][j] = black_box_value;
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
* 示例：          Get_Start_Point(CUT_HEIGHT - 2, image, L_start_point, R_start_point);
* 返回值：        两边都找到返回1，否则返回0.
*/
uint8 L_start_point[2] = { 0 }; //左侧起始点
uint8 R_start_point[2] = { 0 }; //右侧起始点

uint8 Get_Start_Point(uint8 start_row, uint8(*image)[MT9V03X_W], uint8 *l_start_point, uint8 *r_start_point)
{
    uint8 i = 0, j = 0;
    uint8 L_Is_Found = 0, R_Is_Found = 0;                                  //找到起点时挂出对应标志位
    uint8 L_Start_X = 2;                                                   //左侧起始X坐标,从该点开始往右找白线
    uint8 R_Start_X = MT9V03X_W - 3;                                      //右侧起始X坐标,从该点开始往左找白线 
//    uint8 Start_Row_0 = start_row;                                        //初始行
    
    //指定的行没找到起点时，向上走一行继续找，最多找/行
    for(j = 0; j < 40; j ++)
    {
        l_start_point[1] = start_row;//y
        r_start_point[1] = start_row;//y
 
        //左侧向右侧找黑白跳变点
       for (i = L_Start_X; i < MT9V03X_W - 1 ; i++)                 
       {
            //寻找边缘
            if (Trip_point(image[start_row][i + 1], image[start_row][i]))//差比和为真
            {
                    l_start_point[0] = i;                              //找到后记录X坐标
                    L_Is_Found = 1;                                     //挂出找见标志位
                    break;
             }
        }  
       
        //右侧向左找黑白跳变点
        for (i = R_Start_X; i > 0; i--)                      //向右找起始点
        {
            if (Trip_point(image[start_row][i - 1], image[start_row][i]))//差比和为真
            {
               
                r_start_point[0] = i;
                R_Is_Found = 1;
                break;
                
            }
        }
       
        //两次都找到标志位
        if(L_Is_Found && R_Is_Found)
        {
            return 1;
        }
        else
        {
            start_row --;      //当此行有一侧没找到，就向上移动一行重新找
        }
    }
    return 0;

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
*                uint8(*image)[MT9V03X_W]     提取边线的图像
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
void Dir_Labyrinth_5(uint16 Break_Flag, uint8(*image)[MT9V03X_W], uint8(*l_line)[2], uint8(*r_line)[2], int8* l_dir, int8* r_dir, uint16* l_stastic, uint16* r_stastic, uint8* x_meet, uint8* y_meet,
    uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 clip_value)
{
    uint8 j = 0;

    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    //左边变量
    uint8  L_Center_Point[2] = { 0 };     //存放每次找到的XY坐标
    uint16 L_Data_Statics = 0;          //统计左边找到的边线点的个数

    uint8  L_Front_Value = 0;           //左侧 面向的前方点的灰度值
    uint8  L_Front_L_Value = 0;         //左侧 面向的左前方点的灰度值

    uint8  L_Dir = 0;                   //此参数用于转向
    uint8  L_Turn_Num = 0;              //记录转向次数，若中心点前后左右都是黑色像素，就会在一个点转向四次，记录到四次时退出循环防止卡死程序
    uint16 L_Pixel_Value_Sum = 0;       //中心点与周围24个点的像素值和
    float L_Thres = 0;                 //局部阈值,即L_Pixel_Value_Sum / 25

    //右边变量
    uint8  R_Center_Point[2] = { 0 };     //存放每次找到的XY坐标
    uint16 R_Data_Statics = 0;          //统计右边找到的边线点的个数

    uint8  R_Front_Value = 0;           //右侧 面向的前方点的灰度值
    uint8  R_Front_R_Value = 0;         //右侧 面向的左前方点的灰度值

    uint8  R_Dir = 0;                   //此参数用于转向
    uint8  R_Turn_Num = 0;              //记录转向次数，若中心点前后左右都是黑色像素，就会在一个点转向四次，记录到四次时退出循环防止卡死程序
    uint16 R_Pixel_Value_Sum = 0;       //中心点与周围24个点的像素值和
    float R_Thres = 0;                 //局部阈值
    
//    float L_Thres_Record[MAX_LINE] = {0};
//    float R_Thres_Record[MAX_LINE] = {0};
    
    //第一次更新坐标点  将找到的起点值传进来，左侧横坐标减一和右侧横坐标加一是为了要在白点处开始扫线
    L_Center_Point[0] = l_start_x + 1;//x
    L_Center_Point[1] = l_start_y;//y
    R_Center_Point[0] = r_start_x - 1;//x
    R_Center_Point[1] = r_start_y;//y

    //开启方向迷宫循环
    while (Break_Flag--)
    {
        //左边
       //判定出死区后，挂出停止标志位，单侧爬线停止。
        if (L_Stop_Flag == 0)
        {
            l_line[L_Data_Statics][0] = L_Center_Point[0];  //找到的中心点X坐标计入左边线数组
            l_line[L_Data_Statics][1] = L_Center_Point[1];  //找到的中心点Y坐标计入左边线数组

            if (L_Data_Statics != 0)
            {
                switch (l_dir[L_Data_Statics - 1])  //下面这一坨可以根据上一个点的生长方向大幅优化爬线时间
                {
                    //从第二个点开始，第一个点的阈值要25个点全部加一遍
                    //当横向或纵向生长时，将原先的25次加法运算简化为十次计算
                    //当斜向生长时，将原先的25次加法计算简化为十八次运算
                    //可以画出图来，跟着代码走几遍，就可以很快速的理解
                case 1:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 3][L_Center_Point[0] + 2] - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1]
                        - image[L_Center_Point[1] + 3][L_Center_Point[0] + 0] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1]
                        - image[L_Center_Point[1] + 3][L_Center_Point[0] - 2]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2];
                    break;
                }
                case -2:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 0][L_Center_Point[0] + 3]
                        - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 2][L_Center_Point[0] + 3]
                        - image[L_Center_Point[1] + 3][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 3][L_Center_Point[0] + 2]
                        - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 0]
                        - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                        + image[L_Center_Point[1] + 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] - 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2];
                    break;
                }
                case -3:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 2][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3]
                        - image[L_Center_Point[1] + 0][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3]
                        - image[L_Center_Point[1] + 2][L_Center_Point[0] + 3]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                        + image[L_Center_Point[1] - 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2];
                    break;
                }
                case -4:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0]
                        - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 2]
                        - image[L_Center_Point[1] - 3][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 2][L_Center_Point[0] + 3]
                        - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 0][L_Center_Point[0] + 3]
                        - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                        + image[L_Center_Point[1] + 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] - 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2];
                    break;
                }
                case -1:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 3][L_Center_Point[0] - 2] - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1]
                        - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1]
                        - image[L_Center_Point[1] - 3][L_Center_Point[0] + 2]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2];
                    break;
                }
                case 2:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 0][L_Center_Point[0] - 3]
                        - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 2][L_Center_Point[0] - 3]
                        - image[L_Center_Point[1] - 3][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 3][L_Center_Point[0] - 2]
                        - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0]
                        - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                        + image[L_Center_Point[1] - 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2];
                    break;
                }
                case 3:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 2][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3]
                        - image[L_Center_Point[1] - 0][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3]
                        - image[L_Center_Point[1] - 2][L_Center_Point[0] - 3]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                        + image[L_Center_Point[1] + 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2];
                    break;
                }
                case 4:
                {
                    L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 0]
                        - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 2]
                        - image[L_Center_Point[1] + 3][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 2][L_Center_Point[0] - 3]
                        - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 0][L_Center_Point[0] - 3]
                        - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3]
                        + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                        + image[L_Center_Point[1] - 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                        + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2];
                    break;
                }
                }
            }
            else
            {
                for (j = 0; j < 25; j++)    //第一个阈值将25个点全部加一遍，后续点的阈值根据生长方向计算
                {
                    L_Pixel_Value_Sum += image[L_Center_Point[1] + Square_0[j][1]][L_Center_Point[0] + Square_0[j][0]];
//                  int x = L_Center_Point[0] + Square_0[j][0];
//                  int y = L_Center_Point[1] + Square_0[j][1];
//                  if (x < 0 || x >= MT9V03X_W || y < 0 || y >= CUT_HEIGHT) continue;
//                  L_Pixel_Value_Sum += image[y][x];
                }
            }

            //备注：Thres_Interfere：为手动干预阈值和，可以在二十五个值的和上进行修改。但最终未使用。
            //    Thres_Num_Interfere：点的个数，这里取25即可
            //    clip_value：经验值参数，上交代码中引用此参数用于微调阈值，防止强行分割，取值为0 ~5即可，
            //    但最终本人未使用

//            L_Thres = (L_Pixel_Value_Sum + Thres_Interfere) / Thres_Num_Interfere;   //阈值为25个点灰度值的平均值
            L_Thres = L_Pixel_Value_Sum  / Thres_Num_Interfere;   //阈值为25个点灰度值的平均值
            
            L_Thres -= clip_value;              //将得到的灰度阈值减去一个经验值，用来优化判定
              
            //平滑滤波
            if (Thres_Filiter_Flag_1 == 1 || Thres_Filiter_Flag_2 == 1)
            {
                if (L_Data_Statics > 3)
                {
                    //求三次阈值给予不同的权重
                    L_Thres = L_Thres * 1.3f - L_Thres_Record[L_Data_Statics - 1] * 0.2f - L_Thres_Record[L_Data_Statics - 2] * 0.1f;
                }
            }
            //将左侧边线的每个点的阈值进行记录，可实现图像迭代：
            L_Thres_Record[L_Data_Statics] = L_Thres;
            L_Data_Statics++;                   //每找到一个点统计个数+1

        L_Judge_Again:    //L_Judge_Again 与 goto 配合使用
            if (L_Stop_Flag == 0)
            {
                L_Front_Value = image[L_Center_Point[1] + L_Face_Dir[L_Dir][1]][L_Center_Point[0] + L_Face_Dir[L_Dir][0]];          //记录面向的前方点的灰度值
                L_Front_L_Value = image[L_Center_Point[1] + L_Face_Dir_L[L_Dir][1]][L_Center_Point[0] + L_Face_Dir_L[L_Dir][0]];    //记录面向的左前方点的灰度值
                if ((float)L_Front_Value < L_Thres)     //面向的前方点是黑色
                {
                    L_Dir = (L_Dir + 1) % 4;    //需右转一次
                    L_Turn_Num++;
                    if (L_Turn_Num == 4)        //死区处理
                    {
                        L_Stop_Flag = 1;       //当前后左右都是黑色时，进入死区，停止左侧爬线
                    }
                    goto L_Judge_Again;
                }
                else if ((float)L_Front_L_Value < L_Thres)   //左前方点是黑色，前方点是白色
                {
                    L_Center_Point[0] += L_Face_Dir[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir[L_Dir][1];      //向前走一步
                    
                    L_Center_Point[0] = CLAMP(L_Center_Point[0], 2, MT9V03X_W - 3);     //加入限幅
                    L_Center_Point[1] = CLAMP(L_Center_Point[1], 2, CUT_HEIGHT- 1);
                    
                    //减1是由于点数往前走了，记录的是上一个点的生长方向
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir[L_Dir][0] * 3) - L_Face_Dir[L_Dir][1];
                    L_Turn_Num = 0;
                }
                else        //左前方和前方都是白色点
                {
                    
                    L_Center_Point[0] += L_Face_Dir_L[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir_L[L_Dir][1];        //向左前方走一步
                    
                    L_Center_Point[0] = CLAMP(L_Center_Point[0], 2, MT9V03X_W - 3);     //加入限幅
                    L_Center_Point[1] = CLAMP(L_Center_Point[1], 2, CUT_HEIGHT- 1);
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir_L[L_Dir][0] * 3) - L_Face_Dir_L[L_Dir][1];
                    L_Dir = (L_Dir + 3) % 4;        //左转一次
                    L_Turn_Num = 0;
                }
                if (L_Data_Statics >= 5)     //O环处理，即转了一圈后回到原处，也是一种死区，当立即停止爬线
                {
                    if (l_line[L_Data_Statics - 1][0] == l_line[L_Data_Statics - 5][0] &&
                        l_line[L_Data_Statics - 1][1] == l_line[L_Data_Statics - 5][1])
                    {
                        L_Stop_Flag = 1;
                    }
                }
            }
        }

        //右侧与左侧同理，代码也类似，理解左侧后右侧就很简单
        if (R_Stop_Flag == 0)
        {
            r_line[R_Data_Statics][0] = R_Center_Point[0];
            r_line[R_Data_Statics][1] = R_Center_Point[1];

            if (R_Data_Statics != 0)
            {
                switch (r_dir[R_Data_Statics - 1])
                {
                case 1:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 3][R_Center_Point[0] + 2] - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1]
                        - image[R_Center_Point[1] + 3][R_Center_Point[0] + 0] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1]
                        - image[R_Center_Point[1] + 3][R_Center_Point[0] - 2]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2];
                    break;
                }
                case -2:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 0][R_Center_Point[0] + 3]
                        - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 2][R_Center_Point[0] + 3]
                        - image[R_Center_Point[1] + 3][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 3][R_Center_Point[0] + 2]
                        - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 0]
                        - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                        + image[R_Center_Point[1] + 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] - 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2];
                    break;
                }
                case -3:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 2][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3]
                        - image[R_Center_Point[1] + 0][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3]
                        - image[R_Center_Point[1] + 2][R_Center_Point[0] + 3]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                        + image[R_Center_Point[1] - 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2];
                    break;
                }
                case -4:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0]
                        - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 2]
                        - image[R_Center_Point[1] - 3][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 2][R_Center_Point[0] + 3]
                        - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 0][R_Center_Point[0] + 3]
                        - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                        + image[R_Center_Point[1] + 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] - 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2];
                    break;
                }
                case -1:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 3][R_Center_Point[0] - 2] - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1]
                        - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1]
                        - image[R_Center_Point[1] - 3][R_Center_Point[0] + 2]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2];
                    break;
                }
                case 2:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 0][R_Center_Point[0] - 3]
                        - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 2][R_Center_Point[0] - 3]
                        - image[R_Center_Point[1] - 3][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 3][R_Center_Point[0] - 2]
                        - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0]
                        - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                        + image[R_Center_Point[1] - 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2];
                    break;
                }
                case 3:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 2][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3]
                        - image[R_Center_Point[1] - 0][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3]
                        - image[R_Center_Point[1] - 2][R_Center_Point[0] - 3]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                        + image[R_Center_Point[1] + 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2];
                    break;
                }
                case 4:
                {
                    R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 0]
                        - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 2]
                        - image[R_Center_Point[1] + 3][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 2][R_Center_Point[0] - 3]
                        - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 0][R_Center_Point[0] - 3]
                        - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3]
                        + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                        + image[R_Center_Point[1] - 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                        + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2];
                    break;
                }
                }
            }
            else
            {
                for (j = 0; j < 25; j++)
                {
                    R_Pixel_Value_Sum += image[R_Center_Point[1] + Square_0[j][1]][R_Center_Point[0] + Square_0[j][0]];
                }
            }

            R_Thres = (R_Pixel_Value_Sum) / Thres_Num_Interfere;
            R_Thres -= clip_value;

            if (Thres_Filiter_Flag_1 == 1 || Thres_Filiter_Flag_2 == 1)
            {
                if (R_Data_Statics > 3)
                {
                    R_Thres = R_Thres * 1.3f - R_Thres_Record[R_Data_Statics - 1] * 0.2f - R_Thres_Record[R_Data_Statics - 2] * 0.1f;
                }
            }

            R_Thres_Record[R_Data_Statics] = R_Thres;

            R_Data_Statics++;

        R_Judgme_Again:
            if (R_Stop_Flag == 0)
            {
                R_Front_Value = image[R_Center_Point[1] + R_Face_Dir[R_Dir][1]][R_Center_Point[0] + R_Face_Dir[R_Dir][0]];
                R_Front_R_Value = image[R_Center_Point[1] + R_Face_Dir_R[R_Dir][1]][R_Center_Point[0] + R_Face_Dir_R[R_Dir][0]];
                if ((float)R_Front_Value < R_Thres)
                {
                    R_Dir = (R_Dir + 3) % 4;
                    R_Turn_Num++;
                    if (R_Turn_Num == 4)
                    {
                        R_Stop_Flag = 1;
                    }
                    goto R_Judgme_Again;
                }
                else if ((float)R_Front_R_Value < R_Thres)
                {
                    R_Center_Point[0] += R_Face_Dir[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir[R_Dir][1];
                    
                    R_Center_Point[0] = CLAMP(R_Center_Point[0], 2, MT9V03X_W - 3);     //加入限幅
                    R_Center_Point[1] = CLAMP(R_Center_Point[1], 2, CUT_HEIGHT - 1);
                    r_dir[R_Data_Statics - 1] = R_Face_Dir[R_Dir][0] * 3 - R_Face_Dir[R_Dir][1];
                    R_Turn_Num = 0;
                }
                else
                {
                    R_Center_Point[0] += R_Face_Dir_R[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir_R[R_Dir][1];
                    
                    R_Center_Point[0] = CLAMP(R_Center_Point[0], 2, MT9V03X_W - 3);     //加入限幅
                    R_Center_Point[1] = CLAMP(R_Center_Point[1], 2, CUT_HEIGHT - 1);
                    r_dir[R_Data_Statics - 1] = R_Face_Dir_R[R_Dir][0] * 3 - R_Face_Dir_R[R_Dir][1];
                    R_Dir = (R_Dir + 1) % 4;
                    R_Turn_Num = 0;
                }
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

        if (L_Stop_Flag == 0 && R_Stop_Flag == 0)
        {
            if ((abs(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 1)
                && (abs(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 1))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
        //有一侧存在死区时，对相遇点的判定放宽松一些，防止实际相遇但没有判定出，导致爬线紊乱的情况
        else
        {
            if ((abs(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 3)
                && (abs(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 3))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
    }
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    *l_stastic = L_Data_Statics;    //记录左侧边线点个数
    *r_stastic = R_Data_Statics;    //记录右侧边线点个数
}



/**
* 函数功能：      由二维边线数组提取一维边线
* 特殊说明：      无
* 形  参：        uint16 l_total       //左侧二维边线点的个数
*                 uint16 r_total      //右侧二维边线点的个数
*                 uint8 start         //起始行（图像底部）
*                 uint8 end           //截止行（图像顶部）
*                 uint8 *l_border     //存储左侧一维边线的数组
*                 uint8 *r_border     //存储右侧一维边线的数组
*                 uint8(*l_line)[2]   //存储左侧二维边线的数组
*                 uint8(*r_line)[2]   //存储右侧二维边线的数组
*
* 示例：          Get_Border(L_Statics, R_Statics, CUT_HEIGHT - 2, 1, L_Border, R_Border, L_line, R_line);
* 返回值：        无
*/
uint8 L_Border[CUT_HEIGHT] = {0};
uint8 R_Border[CUT_HEIGHT] = {0};

//void Get_Border(uint16 l_total, uint16 r_total, uint8 start, uint8 end, uint8 *l_border, uint8 *r_border, uint8(*l_line)[2], uint8(*r_line)[2])
//{
//    uint8 i = 0;
//    uint16 j = 0;
//    uint8 h = 0;
//    for (i = 0; i < CUT_HEIGHT; i++)
//    {
//        l_border[i] = X_Border_Min;
//        r_border[i] = X_Border_Max;     //右边线初始化放到最右边，左边线放到最左边，这样闭合区域外的中线就会在中间，不会干扰得到的数据
//    }
//    h = start;
//    //右边
//    
//    for (j = 0; j < r_total; j++)
//    {
//
//        if (r_line[j][1] == h)
//        {
//            r_border[h] = r_line[j][0];
//        }
//        else
//        {
//            continue;//每行只取一个点，没到下一行就不记录
//        }
//        h--;
//        if (h <= end)
//        {
//            break;//到最后一行退出
//        }
//      
//    }
//    h = start;
//    for (j = 0; j < l_total; j++)
//    {
//
//        if (l_line[j][1] == h)
//        {
//            l_border[h] = l_line[j][0];
//        }
//        else
//        {
//            continue;//每行只取一个点，没到下一行就不记录
//        }
//        h--;
//        if (h <= end)
//        {
//            break;//到最后一行退出
//        }
//      
//    }
//}

void Get_Border(uint16 l_total, uint16 r_total, uint8 start, uint8 end, uint8 *l_border, uint8 *r_border, uint8(*l_line)[2], uint8(*r_line)[2])
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < CUT_HEIGHT; i++)
    {
        l_border[i] = X_Border_Min;
        r_border[i] = X_Border_Max;     //右边线初始化放到最右边，左边线放到最左边，这样闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = start;
    //右边
    
    for (j = 0; j < r_total; j++)
    {

        if (r_line[j][1] == h)
        {
            r_border[h] = r_line[j][0];
        }
        else
        {
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h <= end)
        {
            break;//到最后一行退出
        }
      
    }
    h = start;
    for (j = 0; j < l_total; j++)
    {

        if (l_line[j][1] == h)
        {
            l_border[h] = l_line[j][0];
        }
        else
        {
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h <= end)
        {
            break;//到最后一行退出
        }
      
    }
}

      
/**
函数：权重值数组

说明：用于求整幅画面的中点值，权重数越靠上，转弯越早

**/
uint8 mid_weight_list[70] = 
{
    // 0~9 行：低权重
    6,6,6,6,6,6,6,6,6,6,   // [0-9]
    
    // 10~19 行：逐渐过渡到中等权重
    6,6,6,6,6,6,6,6,6,6,   // [10-19]
    
    // 20~39 行（索引19~38）：权重递增（6→16）
    7,8,9,10,11,12,13,14,15,16,
    16,16,16,16,16,16,16,16,16,16,  // [20-39]
    
    // 40~49 行（索引39~48）：峰值权重（20）
    20,20,20,20,20,20,20,20,20,20,  // [40-49]
    
    // 50~59 行（索引49~58）：权重递减（16→6）
    16,15,14,13,12,11,10,9,8,7,   
    6,6,6,6,6,6,6,6,6,6,   // [60-69]
    
};







/**
函数：根据权重求中线值

说明：

**/
float Mid_point = MID_W;
float find_mid_line_weight(void)
{
  float Mid_point_value = MID_W;
  float weight_midline_sum  = MID_W;
  float weight_sum = 1;
  
  
  for(uint8 i = 0; i < num_points; i++ )
  {
    weight_midline_sum += (line_points[i].x) * (mid_weight_list[CUT_HEIGHT - 2 - i]);
    weight_sum += mid_weight_list[CUT_HEIGHT - 2 - i];
    
  }
  
  Mid_point_value = (weight_midline_sum/ weight_sum);
  
  return Mid_point_value;
  
}



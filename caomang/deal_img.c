#include "deal_img.h"
enum ImgFlag IF = straightlineS;
enum ImgFlag IF_L = straightlineS;

volatile Elements_Mode elements_Mode = NO ;
volatile Elements_Lock elements_Lock[10] = {Empty_};
volatile Annulus_Mode Annulus_mode = Differential;
int elements_index = 0;

uint8 stopline_flag = 0;
uint8 stop_num = 0;
uint8 Annulus_flag = 0;
float Annulus_Zero = 0;
float Annulus_Yaw = 0;
int Annulus_Curvature[5] = {10,10,10,10,10};
int Annulus_num = 0;
uint8 dealimg_finish_flag = 0;
int deal_runing = 0;
int dealimg_time= 0;
uint8 disappear_flag = 0;
int disappear_num = 0;

int Disappear_angle_L[3];
int Disappear_angle_R[3];
float Start_angle = 0;

int disappear_total = 3;
float Disappear_Zero = 0;
float Disappear_Yaw = 0;
float Disappear_Length = 0;
volatile RUN_Dir run_dir  = Right ;
int Disappear_Dir_L[3][3] = {{0}};
int Disappear_Dir_R[3][3] = {{0}};
float Disappear_Position = 0;
int Curvature = 10;

uint16_t histogram[256];
uint8_t previousThreshold = 0;
uint8_t nowThreshold = 0;
int minGrayscale = 40;
int maxGrayscale = 160;
int minGray = 0;
int maxGray = 0;
uint8_t minThreshold = 70;
uint8_t maxThreshold = 160;
uint8 img_threshold_group[3];//划分阈值区域   上中下
uint8 close_Threshold = 0;
uint8 far_Threshold = 0;
uint8 mid_Threshold = 0;
uint16 EXP_TIME = 255;
int UD_OFFSET_DEF = 0;
uint8 imgGray[IMG_H][IMG_W];
uint8 imgTran[IMG_H][IMG_W];
uint8 img2wifi[IMG_H][IMG_W];
uint8 imgOSTU[YM][XM];
uint8 white_num_row[YM];
uint8 white_num_col[XM];
uint8 white_row_max[2],white_row_min[2];
uint8 white_col_max[2],white_col_min[2];
uint8 mid_line[YM] = {XM/2};
uint8 Make_up_num[YM] = {0};

uint8 start_center_last=XM / 2;
uint8 start_center=XM/2;
uint8 start_point_l = 0;
uint8 start_point_r = XX;
uint8 start_center_y=0;
uint8 start_center_x=XM/2;

uint8 end_center=XM/2;
uint8 end_center_x=XM/2;
uint8 end_center_y=Deal_Top;

uint8 l_lost_tip = 0;
uint8 r_lost_tip = 0;
uint8 t_lost_tip = 0;
uint8 b_lost_tip = 0;

uint8 b_lost_num = 0;
uint8 t_lost_num = 0;
uint8 l_lost_num = 0;
uint8 r_lost_num = 0;

uint8 Stop_line = 0;
uint8 dis_Solidline = 0;
uint16 b_center[3] = {0};
uint16 l_center[3] = {0};
uint16 r_center[3] = {0};
uint16 t_center[3] = {0};

int Deal_W = XX;
int Deal_H = YY;

float start_time = 0;
float end_time = 0;
float deal_time = 0;

extern float Length;
float Out_Annulus_Length = 0;
float Stopline_Position = 0;
float Stopline_Judge = 300;
float Stop_Length = 0;

int EXTERN_sum[16] = {0};
int Goto_U = 1;
int Goto_D = 2;
int Goto_L = 4;
int Goto_R = 8;
int UD = 3;
int UL = 5;
int UR = 9;
int DL = 6;
int DR = 10;
int LR = 12;
int UDL = 7;
int UDR = 11;
int ULR = 13;
int DLR = 14;
int UDLR = 15;
int EXTERN_ROW[16][YM] = {{0}};
int EXTERN_L = 0;
int EXTERN_R = 0;

float white_width[YM] = {17,17,17,17,17,16,16,16,16,16,
                         16,16,16,15,15,15,15,14,14,14,
                         14,14,14,13,13,13,13,13,13,13,
                         12,12,12,12,12,11,11,11,11,10,
                         10,10,10,10,10,10,10,10,10, 9,
                          9, 9, 9, 9, 9, 8, 8, 8, 8, 8,
                          7, 7, 7, 7, 6, 6, 6, 6, 5, 5};

float Length_5cm[YM] = {30,29,28,28,27,27,26,26,25,25,
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


/******************************************************************************
* 函数名称     : standard
* 描述        : 一系列初始的标准化处理
* 进入参数     : voidֵ
* 返回参数     : void
******************************************************************************/
void standard(void) //图像一系列初始化
{
    Deal_W = XX;
    Deal_H = YY;
    stopline_flag = 0;
    stop_num = 0;
    Annulus_flag = 0;
    Annulus_num = 0;
    Annulus_Zero = 0;
    Annulus_Yaw = 0;
    disappear_flag = 0;
    disappear_num = 0;
    disappear_total = 3;
    elements_index = 0;
}
/******************************************************************************
* 函数名称     : my_abs
* 描述        : 取绝对值函数
* 进入参数     : value
* 返回参数     : 绝对值
******************************************************************************/
int my_abs(int value)
{
if(value>=0) return value;
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
//    memset(histogram, 0, sizeof(histogram));
    minGray = 255;
    maxGray = 0;
    uint8 *ptr = &imgGray[IMG_H-YM][0];
    uint8 *ptrEnd = &imgGray[IMG_H-1][_IMG_W] + 1;
    while (ptr != ptrEnd)
    {
        minGray = *ptr < minGray ? *ptr:minGray;
        maxGray = *ptr > maxGray ? *ptr:maxGray;
        ++histogram[*ptr++];
    }
}

/******************************************************************************
* 函数名称     : getOSTUThreshold
* 描述        : 大津法
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

//    for (min = 0; histogram[min] == 0 && min < 255; ++min)
//        ;
//    for (max = 255; histogram[max] == 0 && max > 0; --max)
//        ;

    min = minGray;
    max = maxGray;
    if(max < minThreshold)     return minThreshold;
    if(min > maxThreshold)     return maxThreshold;

    min = min < minGrayscale ? minGrayscale : min;    //控制全局高曝光或低曝光
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
    nowThreshold = nowThreshold < minThreshold ? minThreshold : nowThreshold;   //控制部分阈值不均匀
    nowThreshold = nowThreshold > maxThreshold ? maxThreshold : nowThreshold;
    previousThreshold = nowThreshold;
    return nowThreshold;
}
/******************************************************************************
* 函数名称     : Get_imgOSTU
* 描述         :得到大津法二值化处理图像，图像转置
* 进入参数     : void
* 返回参数     : void
******************************************************************************/
void Get_imgOSTU(void)
{
    memset(imgOSTU, Black, sizeof(imgOSTU));//初始化黑白图全为黑色
    memset(white_num_row, 0, sizeof(white_num_row));//初始化每行白点数
    memset(white_num_col, 0, sizeof(white_num_col));//初始化每列白点数
    img_threshold_group[0]=nowThreshold + close_Threshold; //近景       //近景。 少加，能看清最近
    img_threshold_group[1]=nowThreshold - mid_Threshold;   //中景
    img_threshold_group[2]=nowThreshold - far_Threshold;   //远景       ///远景。，多降，能清晰识别更远
    int k = 0;
    int Loop[9][2] = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};
    for(int i = 0;i <= YY;i++)  //腐蚀去噪
    {
        if(i == YY/3)
        {
            k = 1;
        }
        else if(i == YY*2/3)
        {
            k = 2;
        }
        for(int j = 0;j <= XX;j++)
        {
          if(imgGray[_IMG_H-i][j] >= img_threshold_group[k])
            {
                int White_num = 0;
                int Black_num = 0;
                for(int k = 0;k <= 8;k++)
                {
                    int x = j + Loop[k][0];
                    int y = i + Loop[k][1];
                    if(x < 0 || x > XX || y < 0 || y > YY)
                    {
                        White_num++;
                    }
                    else if(imgOSTU[y][x] == White)
                    {
                        White_num = 8;
                        break;
                    }
                    else if(imgGray[_IMG_H-y][x] >= nowThreshold)
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
                    white_num_row[i]++;  //计算每行白点数
                    white_num_col[j]++;  //计算每列白点数
                    for(int k = 0;k <= 8;k++)
                    {
                        int x = j + Loop[k][0];
                        int y = i + Loop[k][1];
                        if(x < 0 || x > XX || y < 0 || y > YY)
                        {
                        }
                        else if(imgOSTU[y][x] != White)
                        {
                            imgOSTU[y][x] = White;
                            white_num_row[y]++;  //计算每行白点数
                            white_num_col[x]++;  //计算每列白点数
                        }
                    }
                }
            }
        }
    }
}
/******************************************************************************
* 函数名称     : Dashedline_Makeup
* 描述         :虚线区衔接弥补
******************************************************************************/
uint8 Dashedline_Makeup(void)
{
/*******************************最长最短白行列统计****************************************/
    white_row_max[0] = 0;
    white_row_min[0] = XX;
    white_col_max[0] = 0;
    white_col_min[0] = XX;
    for(uint8 i = Deal_Bottom; i <= Deal_Top; i++)
    {
        if(white_num_row[i] > white_row_max[0])
        {
            white_row_max[0] = white_num_row[i]; //行白点最大
            white_row_max[1] = i;                //最大白点行
        }
        else if(white_num_row[i] < white_row_min[0])
        {
            white_row_min[0] = white_num_row[i]; //行白点最小
            white_row_min[1] = i;                //最小白点行
        }
    }
    for(uint8 i = Deal_Left;i <= Deal_Right;i++)
    {
        if(white_num_col[i] > white_col_max[0])
        {
            white_col_max[0] = white_num_col[i]; //列白点最大
            white_col_max[1] = i;                //白点最大列
        }
        else if(white_num_col[i] < white_col_min[0])
        {
            white_col_min[0] = white_num_col[i]; //列白点最小
            white_col_min[1] = i;                //白点最小列
        }
    }
    if(white_col_max[0] >= YY && white_row_max[0] < white_width[0])
    {
    //绘制四周黑色边框
    for(uint8 x = 0;x<=XX;x++ )
    {
        if(imgOSTU[Deal_Bottom-1][x] == White || imgOSTU[Deal_Bottom-2][x] == White )
        {
            imgOSTU[Deal_Bottom][x] = White;
        }
        imgOSTU[Deal_Bottom-1][x] = Black;
        imgOSTU[Deal_Bottom-2][x] = Black;

        if(imgOSTU[Deal_Top+1][x] == White || imgOSTU[Deal_Top+2][x] == White )
        {
            imgOSTU[Deal_Top][x] = White;
        }
        imgOSTU[Deal_Top+1][x] = Black;
        imgOSTU[Deal_Top+2][x] = Black;
    }
    for(uint8 y = 0;y<=Deal_Top;y++ )
    {
        if(imgOSTU[y][Deal_Left-1] == White || imgOSTU[y][Deal_Left-2] == White )
        {
            imgOSTU[y][Deal_Left] = White;
        }
        imgOSTU[y][Deal_Left-1] = Black;
        imgOSTU[y][Deal_Left-2] = Black;

        if(imgOSTU[y][Deal_Right+1] == White || imgOSTU[y][Deal_Right+2] == White )
        {
            imgOSTU[y][Deal_Right] = White;
        }
        imgOSTU[y][Deal_Right+1] = Black;
        imgOSTU[y][Deal_Right+2] = Black;
    }
        return 0;
    }
/******************************************************************************/
    memset(imgTran, Black, sizeof(imgTran));//初始化黑白图全为黑色
/**************************imgTran断点统计**************************************/
    for(uint8 i = 0;i <= YY;i++)
    {
        for(uint8 j = 0;j <= XX;j++)
        {
            if(imgOSTU[i][j] == White)
            {
                int w_start = j-2*white_width[i] < 0 ? 0 :j-2*white_width[i];
                int w_end = j+2*white_width[i] > XX ? XX :j+2*white_width[i];
                if(j!=XX)
                {
                    for(int w = j+1;w <= w_end;w++)
                    {
                        if(imgOSTU[i][w] == Black)
                        {
                                imgTran[i][w] += Goto_R;

                        }
                        else
                        {
                            break;
                        }

//                        if(imgOSTU[i][w] == Black)
//                        {
//                            if(imgTran[i][w] == Goto_T)
//                            {
//                                imgTran[i][w] = RT;
//                            }
//                            else
//                            {
//                                imgTran[i][w] = Goto_R;
//                            }
//                        }
//                        else
//                        {
//                            break;
//                        }

                    }
                }
                if(j!=0)
                {
                    for(int w = j-1;w >= w_start;w--)
                    {
                        if(imgOSTU[i][w] == Black)
                        {
                                imgTran[i][w] += Goto_L;

                        }
                        else
                        {
                            break;
                        }
//                        if(imgOSTU[i][w] == Black)
//                        {
//                            if(imgTran[i][w] == Goto_R)
//                            {
//                                imgTran[i][w] = LR;
//                            }
//                            else if(imgTran[i][w] == RT)
//                            {
//                                for(int y = i;y >= 0;y--)
//                                {
//                                    if(imgOSTU[y][w] == Black)
//                                    {
//                                        imgTran[y][w] = LRT;
//                                    }
//                                    else
//                                    {
//                                        break;
//                                    }
//                                }
//                            }
//                            else
//                            {
//                                imgTran[i][w] = Goto_L;
//                            }
//                        }
//                        else
//                        {
//                            break;
//                        }

                    }
                }
                int h_start = i-Length_5cm[i] < 0 ? 0 : i-Length_5cm[i];
                int h_end = i+Length_5cm[i] > YY ? YY : i+Length_5cm[i];
                if(i!=YY)
                {
                    for(int h = i+1;h <= h_end;h++)
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
                if(i!=0)
                {int count = Length_5cm[i];
                    for(int h = i-1;h >= h_start;h--,count--)
                    {
                        if(imgOSTU[h][j] == Black)
                        {
                            imgTran[h][j] += Goto_D;
                        }
                        else
                        {
                            break;
                        }
//                        if(imgOSTU[h][j] == Black)
//                        {
//                            if(imgTran[h][j] == LR || imgTran[h][j] == LRT)
//                            {
//                                for(int y = h;y <= i-1;y++)
//                                {
//                                    imgTran[y][j]=LRD;
//                                }
//                                for(int y = h-1;y >= 0;y--)
//                                {
//                                    if(imgTran[y][j] == LR || imgTran[y][j] == LRT)
//                                    {
//                                        imgTran[y][j]=LRD;
//                                    }
//                                    else
//                                    {
//                                        break;
//                                    }
//                                }
//                                break;
//                            }
//                            else
//                            {
//                                imgTran[h][j]=Goto_D;
//                            }
//                        }
//                        else
//                        {
//                            break;
//                        }
                    }
                }

            }
        }
    }
/**************************断路 Make_up**************************************/
    memset(EXTERN_sum, 0, sizeof(EXTERN_sum));
    memset(EXTERN_ROW, 0, sizeof(EXTERN_ROW));

    for(uint8 i = 0;i <= YY;i++)
    {
        for(uint8 j = 0;j <= XX;j++)
       {
            if(imgOSTU[i][j] == Black && imgTran[i][j] >= 1)//(Length_5cm[i]+white_width[i])/4)
            {
                if(imgTran[i][j] == ULR || imgTran[i][j] == DLR || imgTran[i][j] == UDLR)
                {
                imgOSTU[i][j] = LandR;
                }
                else
                {
                imgOSTU[i][j] = Make_up;
                }
                int Combination = (int)imgTran[i][j];
                EXTERN_sum[Combination]++;
                EXTERN_ROW[Combination][i]++;

            }
        }
    }
/**************************左右边界白点迁移**************************************/
    EXTERN_L = 0;
    EXTERN_R = 0;
    uint8 l_white = 0,r_white = 0;
    uint8 l_end = Deal_Left,r_end = Deal_Right;
    for(uint8 k = 0;k <= (int)white_width[Deal_Top];k++) //左边界位移大小计算
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
    for(uint8 k = 0;k <= (int)white_width[Deal_Top];k++)  //右边界位移大小计算
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
    
    for(uint8 y = Deal_Bottom;y<= YY;y++)  //左边界白点迁移
    {
        for(uint8 k = Deal_Left;k <= l_end;k++)
        {
            if(imgOSTU[y][k] == White)
            {
                imgOSTU[y][Deal_Left] = White;
                EXTERN_L++;
                white_num_row[y]++;  //计算每行白点数
                white_num_col[Deal_Left]++;
                break;
            }
            else if(imgOSTU[y][k] == Black)
            {
                break;
            }
        }
    }
    for(uint8 y = Deal_Bottom;y<= YY;y++)   //右边界白点迁移
    {
        for(uint8 k = Deal_Right;k >= r_end;k--)
        {
            if(imgOSTU[y][k] == White)
            {
                imgOSTU[y][Deal_Right] = White;
                EXTERN_R++;
                white_num_row[y]++;  //计算每行白点数
                white_num_col[Deal_Right]++;
                break;
            }
            else if(imgOSTU[y][k] == Black)
            {
                break;
            }
        }
    }
/**************************上边界白点迁移**************************************/
    if(!Annulus_flag)
    {
    uint8 t_end = YY,b_end = 0;
    uint8 White_Top[XX] = {0};
    for(uint8 x = Deal_Left;x<= Deal_Right;x++) //统计上边界迁移白点数
    {
        for(uint8 y = Deal_Top;y <= t_end;y++)
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
    for(uint8 x = Deal_Left;x<= Deal_Right;x++)  //删除重复区块
    {
       if(imgOSTU[Deal_Top][x] == White && White_Top[x] == 1)
       {
           White_Top[x] = 0;
           for(int w = x+1;w <= Deal_Right;w++)
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
           for(int w = x-1;w >= Deal_Left;w--)
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
    for(uint8 x = Deal_Left;x<= Deal_Right;x++)    //迁移上边界白点
    {
        if(White_Top[x] == 1)
        {
            imgOSTU[Deal_Top-0][x] = White;
            imgOSTU[Deal_Top-1][x] = White;
            imgOSTU[Deal_Top-2][x] = White;
            white_num_row[Deal_Top-0]++;  //计算每行白点数
            white_num_row[Deal_Top-1]++;  //计算每行白点数
            white_num_row[Deal_Top-2]++;  //计算每行白点数
            white_num_col[x]++;
        }
    }
    }

    //绘制四周黑色边框
    for(uint8 x = 0;x<=XX;x++ )
    {
        if(imgOSTU[Deal_Bottom-1][x] == White || imgOSTU[Deal_Bottom-2][x] == White )
        {
            imgOSTU[Deal_Bottom][x] = White;
        }
        imgOSTU[Deal_Bottom-1][x] = Black;
        imgOSTU[Deal_Bottom-2][x] = Black;

        if(imgOSTU[Deal_Top+1][x] == White || imgOSTU[Deal_Top+2][x] == White )
        {
            imgOSTU[Deal_Top][x] = White;
        }
        imgOSTU[Deal_Top+1][x] = Black;
        imgOSTU[Deal_Top+2][x] = Black;
    }
    for(uint8 y = 0;y<=Deal_Top;y++ )
    {
        if(imgOSTU[y][Deal_Left-1] == White || imgOSTU[y][Deal_Left-2] == White )
        {
            imgOSTU[y][Deal_Left] = White;
        }
        imgOSTU[y][Deal_Left-1] = Black;
        imgOSTU[y][Deal_Left-2] = Black;

        if(imgOSTU[y][Deal_Right+1] == White || imgOSTU[y][Deal_Right+2] == White )
        {
            imgOSTU[y][Deal_Right] = White;
        }
        imgOSTU[y][Deal_Right+1] = Black;
        imgOSTU[y][Deal_Right+2] = Black;
    }
}
/******************************************************************************
* 函数名称     :uint8 get_start_point(void)
* 描述         :寻找左右边界起点
* 进入参数     : void
* 返回参数     ：start_center_x找到左右边界；0未找到左右边界
*使用示例     ：get_start_point();
******************************************************************************/
uint8 get_start_point(void)
{
    b_lost_tip =0;  //底端边界点初始化
    start_center_y = 0;  //起始行数初始化
    uint8 l_found = 0,r_found = 0;//清零
    uint32 l_point = 0,r_point = 0;
    for(uint8 i = Deal_Bottom;i <= Deal_Bottom + 30;i++)  //从下往上寻找可寻边界行数
    {
        if(white_num_row[i] >= 5 && white_num_row[i+1] >= 5) //白点数大于5，开始寻找边界
        {
            start_center_y = i;
            //从左向右，先找黑白跳变点
            for (uint8 i = 0; i < XX-1; i++)
            {
                if ((imgOSTU[start_center_y][i+1] == Make_up || imgOSTU[start_center_y][i+1] == White)&& imgOSTU[start_center_y][i] == Black)
                {//找到左边起点
                    l_point = i;
                    l_found = 1;
                    break;
                }
            }
            //从右向左，先找黑白跳变点
            for (uint8 i = XX; i > 0+1; i--)
            {
                if ((imgOSTU[start_center_y][i-1] == Make_up || imgOSTU[start_center_y][i-1] == White)&& imgOSTU[start_center_y][i] == Black)
                {//找到右边起点
                    r_point = i;
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
    //如果左右边界之间存在黑色区块,进行处理判断，将小区域块删除
//    if(r_point - l_point - 1 > white_num_row[start_center_y])
//    {
//        uint8 white_num_L = 0,white_num_R = 0;
//        //获取左右区域10*10范围白色数目
//        for(uint8 i = l_point;i < l_point + 10 && i <= Deal_Right;i++)
//        {
//            for(uint8 j = start_center_y;j < start_center_y + 10 ;j++)
//            {
//                if (imgOSTU[j][i] == White)
//                {
//                   white_num_L++;
//                }
//            }
//        }
//        for(uint8 i = r_point;i > r_point - 10 && i >= Deal_Left;i--)
//        {
//            for(uint8 j = start_center_y;j < start_center_y + 10 ;j++)
//            {
//                if (imgOSTU[j][i] == White)
//                {
//                   white_num_R++;
//                }
//            }
//        }
//        //数目较少位置删除处理
//        if(white_num_L > 50 && white_num_R < 10)
//        {
//            r_found = 0;
//            for (uint8 i = l_point; i < r_point; i++)
//            {
//                if (imgOSTU[start_center_y][i] == White && imgOSTU[start_center_y][i+1] == Black)
//                {
//                    r_point = i+1;
//                    r_found = 1;
//                    break;
//                }
//            }
//        }
//        else if(white_num_R > 50 && white_num_L < 10)
//        {
//            l_found = 0;
//            for (uint8 i = r_point; i > l_point; i--)
//            {
//                if (imgOSTU[start_center_y][i-1] == Black && imgOSTU[start_center_y][i] == White)
//                {
//                    l_point = i-1;
//                    l_found = 1;
//                    break;
//                }
//            }
//        }
//    }
        if(l_found && r_found)//找到左右起点
        {
             //记录b_lost_tip
            for (uint8 i = l_point; i <= r_point; i++)
            {
                if (imgOSTU[start_center_y][i] == White)
                {
                    b_lost_tip++;
                    imgOSTU[Lost_Bottom][i] = Lost_line;
                }
                else if (imgOSTU[start_center_y + 1][i] == White )
                {
                    b_lost_tip++;
                    imgOSTU[Lost_Bottom][i] = Lost_line;
                }
                else if (imgOSTU[Deal_Bottom][i] == White )
                {
                    b_lost_tip++;
                    imgOSTU[Lost_Bottom][i] = Lost_line;
                }
            }
            //起点位置记录
            start_center_x = (l_point+r_point)/2;
            start_point_l = l_point;
            start_point_r = r_point;
            return 1;
        }
        else                     //未找到起点
        {
            return 0;
        }
}


/*********************************八邻域变量区域*********************************/
int points_l[(uint16)USE_num][2] = { {  0 } };//左线
int points_r[(uint16)USE_num][2] = { {  0 } };//右线
float dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
float dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint32 l_data_statics = 0;//统计左边
uint32 r_data_statics = 0;//统计右边
uint8 hightest = 0;//最高点
uint16 leftmost[2] ={0};//最左点记录
uint16 rightmost[2] ={0};//最右点记录
uint16 topmost[2] ={0};//最顶点记录

//左边变量
int center_point_l[2] = {  0 };//八邻域中心坐标点
uint8 search_filds_l[8][2] = { {  0 } }; //八邻域坐标
uint8 index_l = 0;//索引下标
uint8 temp_l[8][2] = { {  0 } };
//定义八个邻域左生长种子
int8_t seeds_l[8][2] = { {0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1, 0},{1, -1}, };
//{-1,+1},{0,+1},{+1,+1},   3  4  5
//{-1, 0},       {+1, 0},   2     6
//{-1,-1},{0,-1},{+1,-1},   1  0  7
//这个是逆时针，先入左黑区域，寻找出界点

//右边变量
int center_point_r[2] = { 0 };//八邻域中心坐标点
uint8 search_filds_r[8][2] = { {  0 } };//八邻域坐标
uint8 index_r = 0;//索引下标
uint8 temp_r[8][2] = { {  0 } };
//定义八个邻域右生长种子
int8_t seeds_r[8][2] = { {0, -1},{1,-1},{1,0}, {1,1},{0,1},{-1,1}, {-1,0},{-1, -1}, };
//{-1,+1},{0,+1},{+1,+1},   5  4  3
//{-1, 0},       {+1, 0},   6     2
//{-1,-1},{0,-1},{+1,-1},   7  0  1
//这个是顺时针，先入右黑区域，寻找出界点
/*********************************八邻域变量区域*********************************/

/******************************************************************************
* 函数名称     :uint8 search_l_r(uint8 start_l_x,uint8 start_l_y,uint8 start_r_x,uint8 start_r_y )
* 描述         :八邻域探索边界线，记录相关数据
* 进入参数     : start_l_x,start_l_y,左八邻域开始坐标
*               start_r_x,start_r_y,左八邻域开始坐标
*使用示例      :earch_l_r(start_point_l,start_center_y,start_point_r,start_center_y);
* 备注        :边界记录为黑色点部分
******************************************************************************/
uint8 search_l_r(uint8 start_l_x,uint8 start_l_y,uint8 start_r_x,uint8 start_r_y )
{
  //初始化巡线最高处
    hightest = 0;
    leftmost[0] =Deal_Right;
    rightmost[0] =Deal_Left;
    topmost[1] =0;
  //左右丢失点初始化
    l_lost_tip = 0;
    r_lost_tip = 0;
    t_lost_tip = 0;
  //初始化左右索引
    l_data_statics = 0;
    r_data_statics = 0;
  //更新中心点
    center_point_l[0] = start_l_x;//x
    center_point_l[1] = start_l_y;//y
    center_point_r[0] = start_r_x;//x
    center_point_r[1] = start_r_y;//y
    points_l[l_data_statics][0] = center_point_l[0];//x
    points_l[l_data_statics][1] = center_point_l[1];//y
    points_r[r_data_statics][0] = center_point_r[0];//x
    points_r[r_data_statics][1] = center_point_r[1];//y
    uint32 break_flag = USE_num; //总循环数
  while (break_flag--)
    {
        //左边8F初始化
        for (uint8 i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到边线数组内
        //右边8F初始化
        for (uint8 i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到边线数组内
        //左边生长判断
        l_data_statics++;//索引加一
        index_l = 0;//先清零，后使用
        for (uint8 i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }
        for (uint8 i = 0; i < 8; i++)
        {
            if ((imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Black || imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Lost_line)
                && (imgOSTU[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] != Black && imgOSTU[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] != Lost_line))   //生长种子由黑变白位置点
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = i;//记录生长方向，数组索引比边界少一
            }
            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (uint8 j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] < temp_l[j][1])//中心点向高处生长
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                        dir_l[l_data_statics - 1] = i;//记录生长方向
                    }
                }
            }
            if(center_point_l[0] == Deal_Left -1
                    && imgOSTU[center_point_l[1]][Deal_Left] == White
                    && imgOSTU[center_point_l[1]][Deal_Left-2] != Lost_line)
            {
                l_lost_tip++;
                imgOSTU[center_point_l[1]][Lost_Left] = Lost_line;
            }//l_lost_tip记录
            else if(center_point_l[0] == Deal_Right + 1
                    && imgOSTU[center_point_l[1]][Deal_Right] == White
                    && imgOSTU[center_point_l[1]][Lost_Right] != Lost_line)
            {
                r_lost_tip++;
                imgOSTU[center_point_l[1]][Lost_Right] = Lost_line;
            }
            else if(center_point_l[1] == Deal_Top + 1
                    && imgOSTU[Deal_Top][center_point_l[0]] == White
                    && imgOSTU[Deal_Top + 2][center_point_l[0]] != Lost_line)
            {
                t_lost_tip++;
                imgOSTU[Lost_Top][center_point_l[0]] = Lost_line;
            }//t_lost_tip记录
            if(center_point_l[1] > hightest)
            {
                hightest = center_point_l[1];
                topmost[0] =center_point_l[0];
                topmost[1] =center_point_l[1];
            }//更新记录最高点
            if(center_point_l[1] > hightest)
            {
                hightest = center_point_l[1];
                topmost[0] = center_point_l[0];
                topmost[1] = center_point_l[1];
            }//更新记录最高点
            if(center_point_l[0] < leftmost[0])
            {
              leftmost[0] = center_point_l[0];
              leftmost[1] = center_point_l[1];
            }//更新记录最左点
        }
        //右边判断生长判断
        r_data_statics++;//索引加一
        index_r = 0;//先清零，后使用
        for (uint8 i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }
        for (uint8 i = 0; i < 8; i++)
        {
            if ((imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Black || imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Lost_line)
                && (imgOSTU[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] != Black && imgOSTU[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] != Lost_line))
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = i;//记录生长方向
            }
            if (index_r)
            {
                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (uint8 j = 0; j < index_r; j++)  //中心点向高处生长
                {
                    if (center_point_r[1] < temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                        dir_l[l_data_statics - 1] = i;//记录生长方向
                    }
                }
            }

            if(center_point_r[0] == Deal_Right + 1
                    && imgOSTU[center_point_r[1]][Deal_Right] == White
                    && imgOSTU[center_point_r[1]][Lost_Right] != Lost_line)
            {
                r_lost_tip++;
                imgOSTU[center_point_r[1]][Lost_Right] = Lost_line;
            }//r_lost_tip记录
            else if(center_point_r[0] == Deal_Left - 1
                    && imgOSTU[center_point_r[1]][Deal_Left] == White
                    && imgOSTU[center_point_r[1]][Lost_Left] != Lost_line)
            {
                l_lost_tip++;
                imgOSTU[center_point_r[1]][Lost_Left] = Lost_line;
            }//r_lost_tip记录
            else if(center_point_r[1] == Deal_Top + 1
                    && imgOSTU[Deal_Top][center_point_r[0]] == White
                    && imgOSTU[Lost_Top][center_point_r[0]] != Lost_line)
            {
                t_lost_tip++;
                imgOSTU[Lost_Top][center_point_r[0]] = Lost_line;
            }//t_lost_tip记录
            if(center_point_r[1] > hightest)
            {
                hightest = center_point_r[1];
                topmost[0] =center_point_r[0];
                topmost[1] =center_point_r[1];
            }//更新记录最高点
            if(center_point_r[0] > rightmost[0])
            {
              rightmost[0] = center_point_r[0];
              rightmost[1] = center_point_r[1];
            }//更新记录最右点
        }
        if(hightest == Deal_Top -1)
        {
            for(uint8 i = Deal_Left;i <= Deal_Right;i++)
            {
                imgOSTU[Lost_Top][i] = Black;
                imgOSTU[Lost_Top-1][i] = Black;
            }
        }
        //储存边界数组
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
         //三次进入同一个点，退出
            break;
        }

        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics][0]) <= 1
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics][1]) <= 1 //左右相遇，相遇距离可调
            )
        {
            //左右相遇退出
            break;
        }
        if ((l_data_statics > YY/2 && my_abs(start_l_x - points_l[l_data_statics][0]) <= 1 && my_abs(start_l_y - points_l[l_data_statics][1]) <= 1) ||
            (r_data_statics > YY/2 && my_abs(start_r_x - points_r[r_data_statics][0]) <= 1 && my_abs(start_r_y - points_r[r_data_statics][1]) <= 1))
        {
            //回到起点退出
            break;
        }
}
  //imgOSTU图像绘制左右边线
  for(int i = l_data_statics;i>0;i--)
  {
      imgOSTU[points_l[i][1]][points_l[i][0]] = Left_line;
  }
  for(int i = r_data_statics;i>0;i--)
  {
      imgOSTU[points_r[i][1]][points_r[i][0]] = Right_line;
  }
}

/******************************************************************************
* 函数名称     : void Get_lost_tip(uint8 length)
* 描述        : 获取四方位边缘特征
* 进入参数     : void
* 返回参数     : uint8 length 是正常边界丢失的判断长度
* 使用实例     ：Get_lost_tip(5);
* 备注        :lost点没有一定长度,即小于length时，返回判断
*              底端与左右相连时，仅保留底端
*              顶端与左右相连时，仅保留顶端
******************************************************************************/
void Get_lost_tip(uint8 length)
{
    dis_Solidline = 0;
    //边缘特征的区域数量初始化
    b_lost_num = 0;
    t_lost_num = 0;
    l_lost_num = 0;
    r_lost_num = 0;
    //边缘特征的中心点初始化
    memset(b_center, 0, sizeof(b_center));
    memset(l_center, 0, sizeof(l_center));
    memset(r_center, 0, sizeof(r_center));
    memset(t_center, 0, sizeof(t_center));
    /***********获取底部边缘特征***********/
    if(b_lost_tip)
    {
        for(uint8  i = Deal_Left; i <= Deal_Right; i++)   //从左到右检测紫色丢失线
        {
            if(imgOSTU[Lost_Bottom][i] == Lost_line)      //检测到紫色丢失线
            {
                if(i <= Deal_Left + length || i >= Deal_Right - length  //在左右临界处检测到紫色丢失线
                   || (i + length <= XX && imgOSTU[Lost_Bottom][i+length] == Lost_line))  //紫色丢失线有一定长度
                {
                    for(uint8 k = i; k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Bottom][k] == Black)  //判断黑色为紫色丢失线尽头
                        {
                        b_center[b_lost_num] = ((uint32)k-1-(uint32)i)/2+(uint32)i;
                        b_lost_num ++;  //底部区域数量记录
                        i = k;
                        break;
                        }
                    }
                }
                else  //零碎的杂点，删除处理
                {
                    for(uint8  k = i; k <= i + length; k++)
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
//    qout<<"b_lost_num"<<b_lost_num;
    /***********获取左部边缘特征***********/
    if(l_lost_tip)
    {
       for(uint8  i = Deal_Bottom; i <= Deal_Top; i++)  //从下到上检测紫色丢失线
       {
           if(imgOSTU[i][Lost_Left] == Lost_line)  //检测到紫色丢失线
           {
               if(i <= Deal_Bottom + length)  //左侧边缘块过低，考虑底部连续，仅保留底部
               {
                   for(uint8 j = Deal_Left; j <= Deal_Left + length; j++)
                   {
                     if(imgOSTU[Lost_Bottom][j] == Lost_line)
                     {
                        imgOSTU[Lost_Bottom][Lost_Left] = Lost_line;
                        for(; i <= Deal_Top/2; i++)
                        {
                           if(imgOSTU[i][Lost_Left] == Lost_line)
                           {
                               imgOSTU[i][Lost_Left] = Black;
                               l_lost_tip--;
                           }
                           else if(imgOSTU[i][Lost_Left] == Black)
                           {
                               break;
                           }
                        }
                        break;
                     }
                   }
               }
               else  //正常区域，进行记录
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
//    qout<<"l_lost_num"<<l_lost_num;
    /***********获取右部边缘特征***********/
    if(r_lost_tip)
    {
       for(uint8  i = Deal_Bottom; i <= Deal_Top; i++)  //从下到上检测紫色丢失线
       {
           if(imgOSTU[i][Lost_Right] == Lost_line)  //检测到紫色丢失线
           {
               if(i <= Deal_Bottom + length)  //右侧边缘块过低，考虑底部连续，仅保留底部
               {
                   for(uint8 j = Deal_Right; j >= Deal_Right - length; j--)
                   {
                     if(imgOSTU[Lost_Bottom][j] == Lost_line)
                     {
                        imgOSTU[Lost_Bottom][Lost_Right] = Lost_line;
                        for(; i <= Deal_Top/2; i++)
                        {
                           if(imgOSTU[i][Lost_Right] == Lost_line)
                           {
                               imgOSTU[i][Lost_Right] = Black;
                               r_lost_tip--;
                           }
                           else if(imgOSTU[i][Lost_Left] == Black)
                           {
                               break;
                           }
                        }
                     }
                     break;
                   }
               }
               else  //正常记录区域
               {
                   uint8 k = i;
                  for(;  i<= Lost_Top; i++)
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
//    qout<<"r_lost_num"<<r_lost_num;
    /***********获取顶部边缘特征***********/
    if(t_lost_tip)
    {
        uint16 center[5] = {0};
        uint16 num = 0;
        for(uint8  i = Deal_Left; i <= Deal_Right; i++)
        {
            if(imgOSTU[Lost_Top][i] == Lost_line)
            {
                if(i <= Deal_Left + length && l_lost_num)  //区域偏左，且存在左区域，考虑临界区域，保留顶端
                {
                    for(uint8 j = Deal_Top; j >= Deal_Top - length; j--)
                    {
                      if(imgOSTU[j][Lost_Left] == Lost_line)
                      {
                         imgOSTU[Lost_Top][Lost_Left] = Lost_line;
                         for(uint8 k = j; k >= YM/2;k--)
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
                         for(uint8  k = i; k <= XX; k++)
                         {
                             if(imgOSTU[Lost_Top][k] == Black)
                             {
                                 center[num] = ((uint32)k-1-(uint32)i)/2+(uint32)i;
                                 num ++;
                                 i = k;
                                 break;
                             }
                         }
                         break;
                      }
                    }
                }
                else if(i+length >= XX || (i+length <= XX && imgOSTU[Lost_Top][i+length] == Lost_line))  //紫色区域有一定长度
                {
                    for(uint8  k = i; k <= XX; k++)
                    {
                        if(imgOSTU[Lost_Top][k] == Black)
                        {
                            center[num] = ((uint32)k-1-(uint32)i)/2+(uint32)i;
                            num ++;
                            i = k;
                            if(i >= Deal_Right - length && r_lost_num)  //区域偏右，且存在右区域，考虑临界区域，保留顶端
                            {
                                for(uint8 j = Deal_Top; j >= Deal_Top - length; j--)
                                {
                                  if(imgOSTU[j][Lost_Right] == Lost_line)
                                  {
                                     imgOSTU[Lost_Top][Lost_Right] = Lost_line;
                                     for(uint8 k = j; k >= YM/2;k--)
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
                else //零碎杂点，清除
                {
                    for(uint8  k = i; k <= i + length; k++)
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
            for(uint8 k = 0;k <= Deal_W/2-1;k++)
            {
                for(uint8 i = 0;i<=4;i++)
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
//           ((l_lost_tip && l_center[l_lost_num-1] <= hightest/2) ||
//            (r_lost_tip && r_center[r_lost_num-1] <= hightest/2) ||
//            (!l_lost_tip && !r_lost_tip)))
        {
           dis_Solidline = 1;
           int left_lost = 0,left_end = Deal_Top;
           int right_lost = 0,right_end = Deal_Top;
           for(int y = Deal_Top;y >= Deal_Bottom;y--)
           {

               if(imgOSTU[y][Deal_Left] == Left_line || imgOSTU[y][Deal_Left] == Right_line)
               {
                   for(int k = y;k >= Deal_Bottom;k--)
                   {
                       if(imgOSTU[k][Deal_Left] != Black)
                       {
                           left_lost ++;
                       }
                       else
                       {
                         if(k < Deal_H/2)
                         {
                           left_lost = 0;
                           break;
                         }
                         else
                         {
                           left_end = (y+k)/2;
                           break;
                         }
                       }
                   }
                   break;
               }
           }
           for(int y = Deal_Top;y >= Deal_Bottom;y--)
           {

               if(imgOSTU[y][Deal_Right] == Left_line || imgOSTU[y][Deal_Right] == Right_line)
               {
                   for(int k = y;k >= Deal_Bottom;k--)
                   {
                       if(imgOSTU[k][Deal_Right] != Black)
                       {
                           right_lost ++;
                       }
                       else
                       {
                         
                         if(k < Deal_H/2)
                         {
                           right_lost = 0;
                           break;
                         }
                         else
                         {
                           right_end = (y+k)/2;
                           break;
                         }
                       }
                   }
                   break;
               }
           }
           if(left_lost && !right_lost) //补录左端
           {
               imgOSTU[left_end-1][Lost_Left] = Lost_line;
               imgOSTU[left_end][Lost_Left] = Lost_line;
               imgOSTU[left_end+1][Lost_Left] = Lost_line;
               l_center[0] = left_end;
               l_lost_num = 1;
           }
           else if(right_lost && !left_lost) //补录右端
           {
               imgOSTU[right_end-1][Lost_Right] = Lost_line;
               imgOSTU[right_end][Lost_Right] = Lost_line;
               imgOSTU[right_end+1][Lost_Right] = Lost_line;
               r_center[0] = right_end;
               r_lost_num = 1;
           }
           else if(hightest >= Deal_Top)//补录顶端
           {
           int x_mid = topmost[0];
           int x_l = topmost[0];
           int x_r = topmost[0];
           int y_mid = topmost[1];
           for(uint8 y = Deal_Top;y >= Deal_Bottom;y--)
           {
               if(imgOSTU[y][x_mid] == White)
               {
                   y_mid = y;
                   break;
               }
           }
           for(uint8 x = x_mid;x >= Deal_Left;x--)
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
           for(uint8 x = x_mid;x <= Deal_Right;x++)
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
           t_center[0] = (x_l+x_r)/2;
           }
        }
    }
//    qout<<"t_lost_num"<<t_lost_num;
}
/******************************************************************************
* 函数名称     :void Get_start_center(void)
* 描述        :获取中线起点
* 进入参数     : void
* 返回参数     : void
* 使用实例     :Get_start_center();
* 备注        :单区域寻找起点，删除边界较低部分(统计最高处与最低处，获取中间值)
*             双区域寻找起点，删除边界较低部分(统计两处平均值)
******************************************************************************/
void Get_start_center(void)
{
    //底部仅有一块特征区域，且较宽（一般为停车线或者十字处），则进行删除处理
    if(b_lost_tip >= white_width[Deal_Bottom] + 5 && b_lost_num == 1)
    {
        uint32 max = 0,min = Deal_Top,mid = Deal_Top/2;
        uint8 white_heigh[XM] = {0};
        //统计白列高度，并找出最高和最低白列
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
        //计算分割白列高度
        if(b_lost_tip >= 2*white_width[Deal_Bottom])
        {
            mid = (max - min)*2/3 + min;
        }
        else
        {
            mid = (max + min)/2;
        }
//        qout<<"b_mid"<<mid;
        //删除低于分割白列的点
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
        uint8 l_point = 0,r_point = XM;
        float l_num = 0,r_num = 0;
        float l_sum = 0,r_sum = 0;
        float l_average = 0,r_average = 0;
        //找到左右特征区域的起点，并计算平均白列高度
        for (uint8 i = start_point_l; i <= start_point_r; i++)
        {
            if (imgOSTU[Lost_Bottom][i] == Lost_line)
            {
                l_point = i;
                for (uint8 j = l_point; j <= Deal_Right; j++)
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
        for (uint8 i = start_point_r; i >= start_point_l; i--)
        {
            if (imgOSTU[Lost_Bottom][i] == Lost_line)
            {
                r_point = i;
                for (uint8 j = i; j >= Deal_Left; j--)
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
        //计算平均白列高度
        l_average = l_sum/l_num;
        r_average = r_sum/r_num;
//        qout<<"l_average"<<l_average;
//        qout<<"r_average"<<r_average;
        //比较判断，仅仅保留白列高的一块特征区域
        if(l_average >= r_average)
        {
            for (uint8 i = r_point; i >= l_point; i--)
            {
                if (imgOSTU[Lost_Bottom][i] == Lost_line)
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
            for (uint8 i = l_point; i <= r_point; i++)
            {
                if (imgOSTU[Lost_Bottom][i] == Lost_line)
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
//    qout<<"b_lost_num"<<b_lost_num;
    //重新获取左右起点以及中心起点
    uint32 l_point = 0,r_point = 0;
    for (uint8 i = start_point_l; i < start_point_r; i++)
    {
        if (imgOSTU[Lost_Bottom][i] == Lost_line)
        {
            l_point = i;
            break;
        }
    }
    for (uint8 i = start_point_r; i > start_point_l; i--)
    {
        if (imgOSTU[Lost_Bottom][i] == Lost_line)
        {
            r_point = i;
            break;
        }
    }
    start_center_x = (l_point+r_point)/2;
//    qout<<"start_center_x"<<start_center_x;
}
/******************************************************************************
* 函数名称     :void Get_top_straightline(void)
* 描述        : 绘制顶端直线
* 进入参数     : void
* 返回参数     : void
* 使用实例     :Get_top_straightline();
* 备注        :先删除较短边界处
*             单区域绘制，直接直线相连
*             双区域绘制，判断那一段为直线
******************************************************************************/
uint8 Get_top_straightline(void)
{
    uint8 Black_num = 0;
    /**********************单顶端区域处理**********************/
    if(t_lost_num == 1)
    {
        //底部仅有一块特征区域，且较宽（一般为停车线或者十字处，或者直角弯），则进行删除处理
        if((t_lost_tip >= white_width[Deal_Top] + 5 && t_lost_num == 1)||
            (t_lost_tip >= 3*white_width[Deal_Top]))
        {
            //统计顶端白列长度，进行分割
            uint32 max = 0,min = Deal_Top,mid = Deal_Top/2;
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
    //        qout<<"t_mid"<<mid;
            for(uint8 i = Deal_Left; i <= Deal_Right; i++)
            {
                if(imgOSTU[Lost_Top][i] == Lost_line && white_low[i] > mid)
                {
                    t_lost_tip--;
                    imgOSTU[Deal_Top + 2][i] = Black;
                }
            }
            //获取终点
            uint32 l_point = 0,r_point = 0;
            end_center_y = Deal_Top;
            for (uint8 i = Deal_Left; i <= Deal_Right; i++)
            {
                if (imgOSTU[Lost_Top][i] == Lost_line)
                {//找到左边起点
                    l_point = i;
                    break;
                }
            }
            //从右向左，先找黑白跳变点
            for (uint8 i = Deal_Right; i >= Deal_Left; i--)
            {
                if (imgOSTU[Lost_Top][i] == Lost_line)
                {//找到右边起点
                    r_point = i;//x
                    break;
                }
            }
            end_center_x = (l_point+r_point)/2;
        }
        else
        {
            end_center_y = Deal_Top;
            end_center_x = t_center[0];
        }
        //通过顶点起点拟合直线
        if(end_center_y <= start_center_y)
        {
          return 0;
        }
        float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
        for (uint8 i = start_center_y; i <= end_center_y; ++i)
        {
            mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
            if(imgOSTU[i][(uint8)mid_line[i]] != White)
            {
                Black_num++;
            }
            imgOSTU[i][(uint8)mid_line[i]] = Control_line;
        }
    }
    /**********************双顶端区域进行直线判断**********************/
    else if(t_lost_num == 2)
    {
        end_center_y = Deal_Top;
        uint32 t_center_l = t_center[0],t_center_r = t_center[1]; //左右顶点初始化
//        qout<<"t_center_l"<<t_center_l;
//        qout<<"t_center_r"<<t_center_r;
        //判断左右连线之间黑色点数目
        uint8 left_line[YM] = {XM/2};
        uint8 right_line[YM] = {XM/2};
        uint8 Black_num_l = 0,Black_num_r = 0;
        float kl = ((float)t_center_l - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
        for (uint8 i = start_center_y; i <= end_center_y; ++i)
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
        for (uint8 i = start_center_y; i <= end_center_y; ++i)
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
        //        qout<<"Black_num_l"<<Black_num_l;
        //        qout<<"Black_num_r"<<Black_num_r;
        //通过比较黑色点数，判断引用哪条中线
        if(Black_num_l > Black_num_r)
        {
           end_center_x  = t_center_r;
           Black_num = Black_num_r;
           for (uint8 i = start_center_y; i <= end_center_y; ++i)
           {
               mid_line[i] = right_line[i];
               imgOSTU[i][mid_line[i]] = Control_line;
           }
        }
        else
        {
            end_center_x  = t_center_l;
            Black_num = Black_num_l;
            for (uint8 i = start_center_y; i <= end_center_y; ++i)
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
* 函数名称     :uint8 Left_curve_line(void)
* 描述        : 绘制左边界曲线,曲率有两点间黑色点绝对
* 进入参数     : void
* 返回参数     : void
* 使用实例     :Left_curve_line();
******************************************************************************/
uint8 Left_curve_line(void)
{
    end_center_x = Deal_Left;
    end_center_y = l_center[l_lost_num -1];
    uint8 Black_num = 0;
    if(end_center_y <= start_center_y)
    {
      return 0;
    }

    float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
    for (uint8 i = start_center_y; i <= end_center_y; ++i)
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
* 函数名称     :void Right_curve_line(void)
* 描述        : 绘制右边界曲线,曲率有两点间黑色点绝对
* 进入参数     : void
* 返回参数     : void
* 使用实例     :Right_curve_line();
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
    for (uint8 i = start_center_y; i <= end_center_y; ++i)
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
* 函数名称     :uint8 Get_stopline(void)
* 描述        :判断停车线函数
* 进入参数     : void
* 返回参数     : 1：识别到为停车线  0：识别不是停车线
* 使用实例     :Get_stopline();
******************************************************************************/
uint8 Get_stopline(void)
{
    if(elements_Mode == YES && elements_Lock[elements_index] != Stopline_)
    {
      return 0;
    }
    if(IF != straightlineL)  //仅有直道才判断停车线，否则退出
    {
        return 0;
    }
    int Yaw_error = my_abs((int)imu_data.yaw-Start_angle) - (int)my_abs((int)imu_data.yaw-Start_angle)/360 * 360;
    if(Yaw_error > 30 && Yaw_error < 360-30) //复原到发车角度
    {
        return 0;
    }
    if(b_lost_num + t_lost_num != 2)
    {
        return 0;
    }
    if((b_lost_num+t_lost_num+l_lost_num+r_lost_num) >= 4) //防止十字误判，退出
    {
        return 0;
    }
    if(l_lost_num && r_lost_num) //防止十字误判，退出
    {
        return 0;
    }
    if((l_lost_num && EXTERN_L == 0) || (r_lost_num && EXTERN_R == 0) )
    {
      return 0;
    }
    if(stopline_flag == 0 && Length < Stopline_Judge)  //发车起步前100cm不判断停车线
    {
        return 0;
    }
    if(stopline_flag != 0 && Length - Stopline_Position < Stopline_Judge)   //判断停车线后100cm不判断停车线
    {
        return 0;
    }
    uint8 get_flag = 0; //定义全局判断标志
    uint8 boom_b = 0,boom_t = 0;  //定义白色宽度boom增长的位置，boom_t为增长顶端，boom_b为增长底端
    for(uint8 i = Deal_Top - 5 ; i >= Deal_Bottom + 10;i--) //搜索增长顶端boom_t
    {
        if(white_num_row[i] >= white_num_row[i+5]*2 && white_num_row[i] < Deal_W*4/5) //boom条件
        {
            boom_t = i;
            get_flag = 1; //判断标志增加
        }
        if(white_num_row[i] == 0) //boom条件
        {
            return 0;
        }
    }
    if(get_flag != 1)
    {
        return 0;
    }
    for(uint8 i = Deal_Bottom + 5 ; i < boom_t - 3;i++) //搜索增长顶端boom_b
    {
        if(white_num_row[i] >= white_num_row[i-5]*2 && white_num_row[i] < Deal_W*2/3)//boom条件
        {
            boom_b = i;
            get_flag = 2; //判断标志增加
        }
        if(white_num_row[i] == 0)
        {
            return 0;
        }
    }
    if(get_flag != 2)
    {
        return 0;
    }
    if(boom_t - boom_b < Length_5cm[boom_t]/4 || boom_t - boom_b > Length_5cm[boom_b]*2) //boom增长间距控制在一定范围内
    {
        return 0;
    }
//    qout<<boom_b<<boom_t;
    /**********************正入停车线**********************/
    if(start_center_x > Deal_Left + 15 && start_center_x < Deal_Right - 15 &&
       end_center_x > Deal_Left + 15 && end_center_x < Deal_Right - 15)
    {
        uint8 L_T = YY,L_B = 0,R_T = YY,R_B = 0;
        uint8 x = 0;
        for (uint8 i = start_center_y; i <= end_center_y; ++i)  //中心左偏移寻找左边界顶点
        {
            x = mid_line[i] - b_lost_tip;
            if(imgOSTU[i][x] == White)
            {
                L_B = i;
                break;
            }
            imgOSTU[i][x] = Judge_line;
        }
        for (uint8 i = start_center_y; i <= end_center_y; ++i)  //中心右偏移寻找右边界顶点
        {
            x = mid_line[i] + b_lost_tip;
            if(imgOSTU[i][x] == White)
            {
                R_B = i;
                break;
            }
            imgOSTU[i][x] = Judge_line;
        }
        for (uint8 i = end_center_y; i >= start_center_y; --i)  //中心左偏移寻找左边界底端
        {
            x = mid_line[i] - b_lost_tip;
            if(imgOSTU[i][x] == White)
            {
                L_T = i;
                break;
            }
            imgOSTU[i][x] = Judge_line;
        }
        for (uint8 i = end_center_y; i >= start_center_y; --i)  //中心右偏移寻找右边界底端
        {
            x = mid_line[i] + b_lost_tip;
            if(imgOSTU[i][x] == White)
            {
                R_T = i;
                break;
            }
            imgOSTU[i][x] = Judge_line;
        }
//      qout<<L_T<<L_B<<R_T<<R_B;
      if(L_T - L_B >= 8 && L_T - L_B <= 30 && R_T - R_B >= 8 && R_T - R_B <= 30)  //限制停车线宽度
      {
          get_flag++;
      }
      if(my_abs((L_T - L_B)-(R_T - R_B)) <= 5) //限制限制两顶端、两底端平行
      {
          get_flag++;
      }
    }
    /**********************偏右停车线**********************/
    else if(0 && start_center_x > Deal_Right - 15 || end_center_x > Deal_Right - 15)
    {
      uint8 L_T1 = YY,L_B1 = 0,L_T2 = YY,L_B2 = 0;
      uint8 x = 0;
      for (uint8 i = start_center_y; i <= end_center_y; ++i) //中线左偏移寻找左边界底端1
      {
          x = mid_line[i] - b_lost_tip;
          if(imgOSTU[i][x] == White)
          {
              L_B1 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      for (uint8 i = end_center_y; i >= start_center_y; --i) //中线左偏移寻找左边界顶点1
      {
          x = mid_line[i] - b_lost_tip;
          if(imgOSTU[i][x] == White)
          {
              L_T1 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      for (uint8 i = start_center_y; i <= end_center_y; ++i) //中线左偏移寻找左边界底端2
      {
          x = mid_line[i] - b_lost_tip*3/2;
          if(imgOSTU[i][x] == White)
          {
              L_B2 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      for (uint8 i = end_center_y; i >= start_center_y; --i) //中线左偏移寻找左边界顶点2
      {
          x = mid_line[i] - b_lost_tip*3/2;
          if(imgOSTU[i][x] == White)
          {
              L_T2 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      if(L_T1 - L_B1 >= 5 && L_T1 - L_B1 <= 35 && L_T2 - L_B2 >= 5 && L_T2 - L_B2 <= 35) //限制停车线宽度
      {
          get_flag++;
      }
      if(my_abs((L_T1 - L_B1)-(L_T2 - L_B2)) <= 5) //限制限制两顶端、两底端平行
      {
          get_flag++;
      }
    }
    /**********************偏左停车线**********************/
    else if(0 && start_center_x < Deal_Left + 15 || end_center_x < Deal_Left + 15)
    {
      uint8 R_T1 = YY,R_B1 = 0,R_T2 = YY,R_B2 = 0;
      uint8 x = 0;
      for (uint8 i = start_center_y; i <= end_center_y; ++i)  //中线右偏移寻找右边界底端1
      {
          x = mid_line[i] + b_lost_tip;
          if(imgOSTU[i][x] == White)
          {
              R_B1 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      for (uint8 i = end_center_y; i >= start_center_y; --i) //中线右偏移寻找右边界顶点1
      {
          x = mid_line[i] + b_lost_tip;
          if(imgOSTU[i][x] == White)
          {
              R_T1 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      for (uint8 i = start_center_y; i <= end_center_y; ++i) //中线右偏移寻找右边界底端2
      {
          x = mid_line[i] + b_lost_tip*3/2;
          if(imgOSTU[i][x] == White)
          {
              R_B2 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      for (uint8 i = end_center_y; i >= start_center_y; --i) //中线右偏移寻找右边界顶点2
      {
          x = mid_line[i] + b_lost_tip*3/2;
          if(imgOSTU[i][x] == White)
          {
              R_T2 = i;
              break;
          }
          imgOSTU[i][x] = Judge_line;
      }
      if(R_T1 - R_B1 >= 5 && R_T1 - R_B1 <= 35 && R_T2 - R_B2 >= 5 && R_T2 - R_B2 <= 35) //限制停车线宽度
      {
          get_flag++;
      }
      if(my_abs((R_T1 - R_B1)-(R_T2 - R_B2)) <= 5) //限制限制两顶端、两底端平行
      {
          get_flag++;
      }
    }
//    qout<<get_flag;
    if(get_flag == 4) //满足所有条件,判断为停车线
    {
//        SpeedLoop.OutPut_L = 0; 
//        SpeedLoop.OutPut_R = 0; 
        disappear_num = 0;//断路区清空
        Annulus_num = 0;//环岛数目清空
        memset(histogram, 0, sizeof(histogram));//全局灰度清空
        Stopline_Position = Length;  //记录本次停车线位置
        Start_angle = imu_data.yaw;  //记录停车角度
        elements_index = 0;
        return 1;
    }
    else
    {
       return 0;
    }
}
/******************************************************************************
* 函数名称     : uint8 Get_Annulus()
* 描述        : 判断环岛函数
* 进入参数     : void
* 返回参数     : end_center_y
* 使用实例     : Get_Annulus();
******************************************************************************/
uint8 Get_Annulus()
{
//      show_string(18, 1, "Annulus1",RGB565_BLUE ,RGB565_WHITE);
  if(Length < Out_Annulus_Length + 300)
  {
    return 0;
  }
    /*************统计连续5帧下k聚点数目****************/
    if(elements_Mode == YES && elements_Lock[elements_index] != Annulus_)
    {
      return 0;
    }
    static int DLR_appear[10] = {0};
    DLR_appear[9] = DLR_appear[8];
    DLR_appear[8] = DLR_appear[7];
    DLR_appear[7] = DLR_appear[6];
    DLR_appear[6] = DLR_appear[5];
    DLR_appear[5] = DLR_appear[4];
    DLR_appear[4] = DLR_appear[3];
    DLR_appear[3] = DLR_appear[2];
    DLR_appear[2] = DLR_appear[1];
    DLR_appear[1] = DLR_appear[0];
//    qout<<DLR_appear[0] + DLR_appear[1] + DLR_appear[2] + DLR_appear[3] + DLR_appear[4] + DLR_appear[5] + DLR_appear[6] + DLR_appear[7] + DLR_appear[8] + DLR_appear[9];
    if(EXTERN_sum[DLR] > 20)
    {
        DLR_appear[0] = 1;
        if(DLR_appear[0] + DLR_appear[1] + DLR_appear[2] + DLR_appear[3] + DLR_appear[4] + DLR_appear[5] + DLR_appear[6] + DLR_appear[7] + DLR_appear[8] + DLR_appear[9] >= 3)
        {
            //有连续几帧下k聚点，判断环岛
        }
        else
        {
            return 0;
        }
    }
    else
    {
        DLR_appear[0] = 0;
        return 0;
    }

    if( (b_lost_num+t_lost_num+l_lost_num+r_lost_num) > 4 || (b_lost_num+t_lost_num+l_lost_num+r_lost_num) < 3)
    {//限制仅有四个特征区域
        return 0;
    }
    if(l_lost_num && r_lost_num)
    {//限制不是十字
        return 0;
    }
    if(EXTERN_sum[ULR] < 20 || EXTERN_sum[DLR] < 20)
    {//有上下聚点，限制k字特征
        return 0;
    }
    //限制白点宽度,防止十字误判
    uint8 L_end = Deal_Left,R_end = Deal_Right;
    for(int x = Deal_Left;x <= Deal_Right;x++)
    {
        if(white_num_col[x])
        {
            L_end = x;
        }
    }
    for(int x = Deal_Right;x >= Deal_Left;x--)
    {
        if(white_num_col[x])
        {
            R_end = x;
        }
    }
    if(R_end - L_end > Deal_W*4/5)
    {
        return 0;
    }
    //四个特征区域搜索
    int BL_x,BR_x,BL_y,BR_y;
    int TL_x,TR_x,TL_y,TR_y;
    if((b_lost_num+t_lost_num+l_lost_num+r_lost_num) == 4)
    {/********************************实线环岛环岛判断********************************/
        if(b_lost_num == 2 && t_lost_num == 2 )
        {
            BL_y = start_center_y;
            BL_x = b_center[0];
            BR_y = start_center_y;
            BR_x = b_center[1];
            TL_y = Deal_Top;
            TL_x = t_center[0];
            TR_y = Deal_Top;
            TR_x = t_center[1];
        }
        else if(l_lost_num)
        {
            if(t_center[t_lost_num-1] > Deal_W*0.7 || b_center[b_lost_num-1] > Deal_W*0.7)
            {
                return 0;
            }
            //左斜入，顺时针统计特征位置
            uint8 point[4][2] = {{0,0},{0,0},{0,0},{0,0}}; // 初始化为0
            uint8 j = 0;
            if(b_lost_num)
            for(int i = b_lost_num-1; i >= 0 && j<=3;i--)
            {
                point[j][0] = b_center[i];
                point[j][1] = start_center_y;
                j++;
            }
            if(l_lost_num)
            for(int i = 0; i <= l_lost_num-1 && j<=3;i++)
            {
                point[j][0] = Deal_Left;
                point[j][1] = l_center[i];
                j++;
            }
            if(t_lost_num)
            for(int i = 0; i <= t_lost_num-1 && j<=3 ;i++)
            {
                point[j][0] = t_center[i];
                point[j][1] = Deal_Top;
                j++;
            }
            if(j != 4)
            {
                return 0;
            }
            BL_y = point[1][1];
            BL_x = point[1][0];
            BR_y = point[0][1];
            BR_x = point[0][0];

            TL_y = point[2][1];
            TL_x = point[2][0];
            TR_y = point[3][1];
            TR_x = point[3][0];
        }
        else if(r_lost_num)
        {
            if(t_center[t_lost_num-1] < Deal_W*0.3 || b_center[b_lost_num-1] < Deal_W*0.3)
            {
                return 0;
            }
            //右斜入，逆时针统计特征位置
            uint8 point[4][2] = {{0,0},{0,0},{0,0},{0,0}}; // 初始化为0
            uint8 j = 0;
            if(b_lost_num)
            for(int i = 0; i <= b_lost_num-1 && j <= 3;i++)
            {
                point[j][0] = b_center[i];
                point[j][1] = start_center_y;
                j++;
            }
            if(r_lost_num)
            for(int i = 0; i <= r_lost_num-1 && j <= 3;i++)
            {
                point[j][0] = Deal_Right;
                point[j][1] = r_center[i];
                j++;
            }
            if(t_lost_num)
            for(int i = t_lost_num-1; i >= 0 && j <= 3;i--)
            {
                point[j][0] = t_center[i];
                point[j][1] = Deal_Top;
                j++;
            }
            if(j != 4)
            {
                return 0;
            }

            BL_y = point[0][1];
            BL_x = point[0][0];
            BR_y = point[1][1];
            BR_x = point[1][0];

            TL_y = point[3][1];
            TL_x = point[3][0];
            TR_y = point[2][1];
            TR_x = point[2][0];

        }
    }
    else if((b_lost_num+t_lost_num+l_lost_num+r_lost_num) == 3) //虚线入环强制找开口
    {
        if(l_lost_num || r_lost_num)//限制无倾斜
        {
            return 0;
        }
        //寻找上下看聚点位置
        int DLR_x,DLR_y,ULR_x,ULR_y;
        int B_Found = 0,T_Found = 0;
        for(int y = Deal_Bottom;y <= Deal_Top;y++)
        {
            if(EXTERN_ROW[DLR][y] && EXTERN_ROW[DLR][y+1])
            {
                DLR_y = y+1;
                for(int x1 = Deal_Left;x1 <= Deal_Right;x1++ )
                {
                    if(imgTran[DLR_y][x1] == DLR)
                    {
                        for(int x2 = x1;x2 <= Deal_Right;x2++ )
                            if(imgTran[DLR_y][x2] != DLR)
                            {
                                DLR_x = (int)(x1+x2)/2;
                                B_Found = 1;
                                break;
                            }
                        break;
                    }
                }
                break;
            }
        }
        for(int y = Deal_Top;y >= Deal_Bottom;y--)
        {
            if(EXTERN_ROW[ULR][y] && EXTERN_ROW[ULR][y-1])
            {
                ULR_y = y-1;
                for(int x1 = Deal_Left;x1 <= Deal_Right;x1++ )
                {
                    if(imgTran[ULR_y][x1] == ULR)
                    {
                        for(int x2 = x1;x2 <= Deal_Right;x2++ )
                            if(imgTran[ULR_y][x2] != ULR)
                            {
                                ULR_x = (int)(x1+x2)/2;
                                T_Found = 1;
                                break;
                            }
                        break;
                    }
                }
                break;
            }
        }
        if(B_Found+T_Found != 2 || my_abs(ULR_x - DLR_x) > Deal_W/3 || ULR_y - DLR_y < 20) //保证找到上下k，且位置合理
        {
            return 0;
        }
        //寻找四特征点
        if(b_lost_num == 2) //存在正底部
        {
            BL_y = start_center_y;
            BR_y = start_center_y;
            BL_x = b_center[0];
            BR_x = b_center[1];
        }
        else //虚底部
        {
            int start_L = Deal_Left,start_R = Deal_Right; //限制左右搜索起点
            for(int x = Deal_Left;x <= DLR_x;x++)
            {
                if(white_num_col[x])
                {
                    start_L = x;
                    break;
                }
            }
            for(int x = Deal_Right;x >= DLR_x;x--)
            {
                if(white_num_col[x])
                {
                    start_R = x;
                    break;
                }
            }
            //下寻下起点
            int found_flag = 0;
            for(int y = start_center_y;y <= DLR_y &&  !found_flag;y++)
            {
                for(int x1 = start_L;x1 <= DLR_x &&  !found_flag;x1++)
                {
                   if(imgOSTU[y][x1] == White)
                   {
                       for(int x2 = start_L;x2 <= DLR_x &&  !found_flag;x2++)
                       {
                           if(imgOSTU[y+1][x2] == White)
                           {
                               for(int x3 = x2;x3 <= DLR_x && !found_flag;x3++)
                               {
                                   if(imgOSTU[y+1][x3] == White)
                                   {
                                       BL_x = x3;
                                       BL_y = y+1;
                                       found_flag = 1;
                                       break;
                                   }
                               }
                           }
                       }
                   }
                }
            }
            if(found_flag != 1)
            {
                return 0;
            }
            else
            {
              found_flag = 0;
            }
            for(int y = start_center_y;y <= DLR_y && !found_flag;y++)
            {
                for(int x1 = start_R;x1 >= ULR_x && !found_flag;x1--)
                {
                   if(imgOSTU[y][x1] == White)
                   {
                       for(int x2 = start_R;x2 >= DLR_x && !found_flag;x2--)
                       {
                           if(imgOSTU[y+1][x2] == White)
                           {
                               for(int x3 = x2;x3 >= DLR_x && !found_flag;x3--)
                               {
                                   if(imgOSTU[y+1][x3] == White)
                                   {
                                       BR_x = x3;
                                       BR_y = y+1;
                                       found_flag=2;
                                       break;
                                   }
                               }
                           }
                       }
                   }
                }
            }
            if(found_flag != 1)
            {
                return 0;
            }
        }
        if(t_lost_num == 2) //存在正顶部
        {
            TL_y = Deal_Top;
            TR_y = Deal_Top;
            TL_x = t_center[0];
            TR_x = t_center[1];
        }
        else //虚顶部
        {
            int start_L = Deal_Left,start_R = Deal_Right;
            for(int x = Deal_Left;x <= DLR_x;x++)
            {
                if(white_num_col[x])
                {
                    start_L = x;
                    break;
                }
            }
            for(int x = Deal_Right;x >= DLR_x;x--)
            {
                if(white_num_col[x])
                {
                    start_R = x;
                    break;
                }
            }

            int found_flag = 0;
            for(int y = Deal_Top;y >= ULR_y && !found_flag;y--)
            {
                for(int x1 = start_L;x1 <= ULR_x && !found_flag;x1++)
                {
                   if(imgOSTU[y][x1] == White)
                   {
                       for(int x2 = start_L;x2 <= ULR_x && !found_flag;x2++)
                       {
                           if(imgOSTU[y-1][x2] == White)
                           {
                               for(int x3 = x2;x3 <= ULR_x && !found_flag;x3++)
                               {
                                   if(imgOSTU[y-1][x3] == White)
                                   {
                                       TL_x = x3;
                                       TL_y = y-1;
                                       found_flag ++;
                                       break;
                                   }
                               }
                           }
                       }
                   }
                }
            }
            if(found_flag != 1)
            {
                return 0;
            }
            else
            {
              found_flag = 0;
            }
            for(int y = Deal_Top;y >= ULR_y && !found_flag;y--)
            {
                for(int x1 = start_R;x1 >= ULR_x && !found_flag;x1--)
                {
                   if(imgOSTU[y][x1] == White)
                   {
                       for(int x2 = start_R;x2 >= ULR_x && !found_flag;x2--)
                       {
                           if(imgOSTU[y-1][x2] == White)
                           {
                               for(int x3 = x2;x3 >= ULR_x && !found_flag;x3--)
                               {
                                   if(imgOSTU[y-1][x3] == White)
                                   {
                                       TR_x = x3;
                                       TR_y = y-1;
                                       found_flag++;
                                       break;
                                   }
                               }
                           }
                       }
                   }
                }

            }
            if(found_flag != 1)
            {
                return 0;
            }

        }
    }
    else
    {
        return 0;
    }
    if(TR_x - TL_x > Deal_W*4/5 || BR_x - BL_x > Deal_W*4/5 ||
       TR_x <= TL_x || BR_x <= BL_x ||
       my_abs(TL_x - BL_x) > Deal_W*3/5 || my_abs(TR_x - BR_x) > Deal_W*3/5)
    {//四特征x起点位置限制
        return 0;
    }
    if(TR_y < Deal_H/2 || TL_y < Deal_H/2 || BR_y > Deal_H/2 || BL_y > Deal_H/2 ||
       TR_y < BR_y+5 || TL_y < BL_y+5 || TR_y > Deal_Top || TL_y > Deal_Top || BL_y < Deal_Bottom  || BR_y < Deal_Bottom)
    {//四特征y起点位置限制
        return 0;
    }
    //连线进行k字开口朝向判断
    uint8 Left_Feature = 0;
    uint8 Right_Feature = 0;
    float White_num_l = 0,White_num_r = 0;
    float Black_num_l = 0,Black_num_r = 0; 

    float kl = ((float)TL_x - (float)BL_x) / ((float)TL_y - (float)BL_y);
    float kr = ((float)TR_x - (float)BR_x) / ((float)TR_y - (float)BR_y);
    if((kl > 1 || kl < -1) && (kr > 1 || kl < -1))//过于倾斜不予判断
    {
        return 0;
    }

    //    /***********左K特征判断************/
    for (uint8 i = BL_y; i <= TL_y; ++i)
    {
        int xl = (int)(BL_x + kl * (i - BL_y));
        if (xl > Deal_Right) 
        {
          xl = Deal_Right;  // 钳制到有效范围
        } 
        else if (xl < Deal_Left) 
        {
          xl = Deal_Left;
        }
        if(imgOSTU[i][xl] == White)
        {
            White_num_l+= (float)Length_5cm[0]/(float)Length_5cm[i];
        }
        else
        {
            imgOSTU[i][xl] = Judge_line;
            Black_num_l+= (float)Length_5cm[0]/(float)Length_5cm[i];
        }
    }
    if(Black_num_l > White_num_l *2)
    {
        Left_Feature = 1;
    }
    else if(Black_num_l < White_num_l *1.1)
    {
        Left_Feature = 0;
    }
    else
    {
        Left_Feature = 3;
    }
    /***********右K特征判断************/
    for (uint8 i = BR_y; i <= TR_y; ++i)
    {
        int xr = (int)(BR_x + kr * (i - BR_y));
        if (xr > Deal_Right) 
        {
          xr = Deal_Right;  // 钳制到有效范围
        } 
        else if (xr < Deal_Left) 
        {
          xr = Deal_Left;
        }
        if(imgOSTU[i][xr] == White)
        {
            White_num_r += (float)Length_5cm[0]/(float)Length_5cm[i];
        }
        else
        {
            imgOSTU[i][xr] = Judge_line;
            Black_num_r+= (float)Length_5cm[0]/(float)Length_5cm[i];
        }
    }
    if(Black_num_r > White_num_r *2)
    {
        Right_Feature = 1;
    }
    else if(Black_num_r < White_num_r *1.1)
    {
        Right_Feature = 0;
    }
    else
    {
        Right_Feature = 3;
    }

        //根据K字情况，判断左右环岛
        if(Left_Feature && !Right_Feature && !r_lost_num)
        {
            //左环岛
            int start_R = BR_x > TR_x ? BR_x : TR_x;
            int white_num = 0;
            for(int x = start_R; x <=  Deal_Right;x++)
            {
                if(imgOSTU[Lost_Top][x] != Lost_line && imgOSTU[Lost_Bottom][x] != Lost_line)
                {
                    start_R = x + 1;
                    for(int k = start_R; k <=  Deal_Right;k++)
                    {
                        white_num += white_num_col[k];
                    }
                    break;
                }
            }
            if(white_num > 3)
            {
                return 0;
            }
            
            
            start_center_x = b_center[1];
            end_center_x = t_center[0];
            end_center_y = Deal_Top;
            Get_curve_line(start_center_x, start_center_y, end_center_x, end_center_y, (int)10);
            IF = annulus_l;
            Annulus_Zero = imu_data.yaw;
            Annulus_num++;
            elements_index++;
            return end_center_y;
        }
        else if(Right_Feature && !Left_Feature && !l_lost_num)
        {
            //右环岛
            int start_L = BL_x > TL_x ? BL_x : TL_x;
            int white_num = 0;
            for(int x = start_L; x >=  Deal_Left;x--)
            {
                if(imgOSTU[Lost_Top][x] != Lost_line && imgOSTU[Lost_Bottom][x] != Lost_line)
                {
                    start_L = x - 1;
                    for(int k = start_L; k >=  Deal_Left;k--)
                    {
                        white_num += white_num_col[k];
                    }
                    break;
                }
            }
            if(white_num > 3)
            {
                return 0;
            }
            
            
            start_center_x = b_center[0];
            end_center_x = t_center[1];
            end_center_y = Deal_Top;
            Get_curve_line(end_center_x, end_center_y,start_center_x, start_center_y,(int)10);
            IF = annulus_r;
            Annulus_Zero = imu_data.yaw;
            Annulus_num++;
            elements_index++;
            return end_center_y;
        }
        else
        {
            return 0;
        }
}
/******************************************************************************
* 函数名称     :uint8 Into_Annulus_ing()
* 描述        : 正在驶入环岛的巡线操作
* 进入参数     : void
* 返回参数     : end_center_y
* 使用实例     :Get_Annulus();
******************************************************************************/
uint8 Into_Annulus_ing()
{
//      show_string(18, 1, "Annulus5",RGB565_BLUE ,RGB565_WHITE);
  uint8 point[4][2] = {{0,0},{0,0},{0,0},{0,0}};
  int count = 0;
    if(IF == annulus_l)
    {
        //左环岛逆时针记录特征点
        uint8 j = 0;
        if(b_lost_num)
        for(int i = b_lost_num-1; i <= b_lost_num-1 && j<=3;i++)
        {
            point[j][0] = b_center[i];
            point[j][1] = start_center_y;
//            qout<<j<<i<<point[j][0]<<point[j][1];
            j++;
        }
        if(t_lost_num)
        for(int i = t_lost_num-1; i >= 0  && j<=3 ;i--)
        {
            point[j][0] = t_center[i];
            point[j][1] = Deal_Top;
//            qout<<j<<i<<point[j][0]<<point[j][1];
            j++;
        }
        if(l_lost_num)
        for(int i = l_lost_num-1; i >= 0 && j<=3;i--)
        {
            point[j][0] = Deal_Left;
            point[j][1] = l_center[i];
//            qout<<j<<i<<point[j][0]<<point[j][1];
            j++;
        }
        count = j-1;
        int White_num = 0;
        for(uint8 x = Deal_Right;x >= point[0][0];x--)
        {
            White_num += white_num_col[x];
            if(White_num >= Deal_H*3)
            {
                point[0][0] = x;
                break;
            }
        }
    }
    else if(IF == annulus_r)
    {
        //右环岛逆时针记录特征点
        uint8 j = 0;
        if(b_lost_num)
        for(int i = 0; i >= 0 && j <= 3;i--)
        {
            point[j][0] = b_center[i];
            point[j][1] = start_center_y;
//            qout<<j<<i<<point[j][0]<<point[j][1];
            j++;
        }
        if(t_lost_num)
        for(int i = 0; i <= t_lost_num-1 && j <= 3;i++)
        {
            point[j][0] = t_center[i];
            point[j][1] = Deal_Top;
//            qout<<j<<i<<point[j][0]<<point[j][1];
            j++;
        }
        if(r_lost_num)
        for(int i = r_lost_num-1; i >= 0 && j <= 3;i--)
        {
            point[j][0] = Deal_Right;
            point[j][1] = r_center[i];
//            qout<<j<<i<<point[j][0]<<point[j][1];
            j++;
        }
        count = j-1;
        int White_num = 0;
        for(uint8 x = Deal_Left;x <= point[0][0];x++)
        {
            White_num += white_num_col[x];
            if(White_num >= Deal_H*3)
            {
                point[0][0] = x;
                break;
            }
        }
    }
    //起始点，终点赋值
    if(count != 0)
    {
      start_center_x = point[0][0];
    }
    else
    {
      return 0;
    }
    if(count >= 2)
    {
    end_center_x = point[2][0];
    end_center_y = point[2][1];
    }
    else
    {
      return 0;
    }

//    qout<<start_center_x<<start_center_y<<end_center_x<<end_center_y;
    //记录中线
    if(end_center_y <= start_center_y)
    {
      return 0;
    }
    float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
//    qout<<"k"<<k;
    for (uint8 i = start_center_y; i <= end_center_y; ++i)
    {
        mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
        if(mid_line[i] < Deal_Left)
        {
          mid_line[i] = Deal_Left;
        }
        else if(mid_line[i] > Deal_Right)
        {
          mid_line[i] = Deal_Right;
        }
        imgOSTU[i][(uint8)mid_line[i]] = Control_line;
    }
    return end_center_y;
}
/******************************************************************************
* 函数名称     :uint8 Into_Annulus_judge()
* 描述        :判断成功驶入环岛，开始环岛巡线
* 进入参数     : void
* 返回参数     : 1：判断进入环岛  0：判断未进入环岛
* 使用实例     :Into_Annulus_judge();
******************************************************************************/
uint8 Into_Annulus_judge()
{
//   show_string(18, 1, "Annulus2",RGB565_BLUE ,RGB565_WHITE);
   if(b_lost_num != 1)
   {
       return 0;
   }
    static int DLR_disappear[2] = {0};
    DLR_disappear[1] = DLR_disappear[0];
    if(EXTERN_sum[DLR] == 0)
    {
        DLR_disappear[0] = 1;
    }
    else
    {
        DLR_disappear[0] = 0;
    }

    if(DLR_disappear[0] + DLR_disappear[1])
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/******************************************************************************
* 函数名称     :uint8 Runing_Annulus()
* 描述        :成功驶入环岛，开始环岛巡线
* 进入参数     : void
* 返回参数     : end_center_y
* 使用实例     :Runing_Annulus();
******************************************************************************/
uint8 Runing_Annulus()
{
//  Annulus_num
  int Curvature = 10;
  
  if(Annulus_num > 5)
  {
    Curvature = Annulus_Curvature[4];
  }
  else
  {
    Curvature = Annulus_Curvature[Annulus_num - 1];
  }

    if(IF == annulus_l)
    {
        //左环岛搜左线
        if(b_lost_num)
        {
            start_center_x = b_center[0];
        }
        else
        {
            return  0;
        }

        if(l_lost_num)
        {
          end_center_y = l_center[l_lost_num-1];
          end_center_x = Deal_Left;
//          qout<<end_center_y<<end_center_x;
        }
        else if(imgOSTU[Lost_Top][Deal_Left] == Lost_line)
        {
            end_center_y = Deal_Top;
            end_center_x = Deal_Left;
        }
        else if(t_lost_num)
        {
            end_center_y = Deal_Top;
            end_center_x = t_center[0];
//            qout<<end_center_y<<end_center_x;
        }
        else
        {
            return 0;
        }
        if(start_center_x <= end_center_x)
        {
          return 0;
        }
        Get_curve_line(start_center_x,start_center_y,end_center_x, end_center_y,Curvature);
    }
    else if(IF == annulus_r)
    {
        //右环岛搜右线
        if(b_lost_num)
        {
            start_center_x = b_center[b_lost_num-1];
        }
        else
        {
            return 0;
        }

        if(r_lost_num)
        {
            end_center_y = r_center[r_lost_num-1];
            end_center_x = Deal_Right;
        }
        else if(imgOSTU[Lost_Top][Deal_Right] == Lost_line)
        {
            end_center_y = Deal_Top;
            end_center_x = Deal_Right;
        }
        else if(t_lost_num)
        {
            end_center_y = Deal_Top;
            end_center_x = t_center[t_lost_num-1];
        }
        else
        {
            return 0;
        }
        if(end_center_x <= start_center_x)
        {
          return 0;
        }
        Get_curve_line(end_center_x, end_center_y,start_center_x, start_center_y,Curvature);
    }
    return end_center_y;
}
/******************************************************************************
* 函数名称     :uint8 Leave_Annulus_judge()
* 描述        :即将驶出环岛判断
* 进入参数     : void
* 返回参数     : 1：为判断成功 0：判断失败
* 使用实例     :Leave_Annulus_judge();
******************************************************************************/
uint8 Leave_Annulus_judge()
{
//    show_string(18, 1, "Annulus3",RGB565_BLUE ,RGB565_WHITE);
    Annulus_Yaw = imu_data.yaw - Annulus_Zero;
    if(!(Annulus_Yaw <= -250) && !(Annulus_Yaw >= 250))
    {
      return 0;
    }
    static int UDLR_sum = 0;
    UDLR_sum += EXTERN_sum[UDLR];
    if(b_lost_num+t_lost_num+l_lost_num+r_lost_num >= 3 && UDLR_sum >= 400)
    {
        UDLR_sum = 0;
        Out_Annulus_Length = Length;
        return 1;
    }
    else
    {
        if(IF == annulus_r)
            {
                //右环岛，判断左边缘特征出现，即左直道出现
                if((b_lost_num+t_lost_num+l_lost_num+r_lost_num) < 3)
                {
                    return 0;
                }
                //右寻线特征依然存在，且顶端特征位置正常
                if(b_lost_num && l_lost_num && l_center[l_lost_num-1] < Deal_Top - 10 && l_center[l_lost_num-1] > Deal_Bottom + 3)
                {
                    int White_num = 0;
                    int x1 = Deal_Left,x2 = b_center[0];
                    int y1 = start_center_y,y2 = l_center[l_lost_num-1];
                    for(int x = x1;x<=x2;x++)
                    {
                        for(int y= y1;y<=y2;y++)
                        {
                            if(imgOSTU[y][x] == White)
                            {
                                White_num ++;
                            }
                        }
                    }
                    if(White_num < (x2-x1)*(y2-y1))
                    {
                    Out_Annulus_Length = Length;
                    return 1;
                    }
                    else
                    {
                        return 0;
                    }
                }
                else
                {
                    return 0;
                }
            }
            if(IF == annulus_l)
            {
                //左环岛，判断右边缘特征出现，即右直道出现
                if((b_lost_num+t_lost_num+l_lost_num+r_lost_num) < 3|| b_lost_num == 0)
                {
                    return 0;
                }
                if(b_lost_num && r_lost_num && r_center[r_lost_num-1] < Deal_Top - 10 && r_center[r_lost_num-1] > Deal_Bottom + 3)
                {
                    int White_num = 0;
                    int x1 = b_center[b_lost_num-1],x2 = Deal_Right;
                    int y1 = start_center_y , y2 = r_center[r_lost_num-1];
                    for(int x = x1;x<=x2;x++)
                    {
                        for(int y= y1;y<=y2;y++)
                        {
                            if(imgOSTU[y][x] == White)
                            {
                                White_num ++;
                            }
                        }
                    }
                    if(White_num < (x2-x1)*(y2-y1))
                    {
                    Out_Annulus_Length = Length;
                    return 1;
                    }
                    else
                    {
                        return 0;
                    }
                }
                else
                {
                    return 0;
                }
            }
    }
}
/******************************************************************************
* 函数名称     :uint8 Leave_Annulus_ing()
* 描述        :即将驶出环岛，进行驶出环岛的寻线操作
* 进入参数     : void
* 返回参数     : end_center_y
* 使用实例     :Leave_Annulus_ing();
******************************************************************************/
uint8 Leave_Annulus_ing()
{
//      show_string(18, 1, "Annulus7",RGB565_BLUE ,RGB565_WHITE);

    //出环时，寻直道走
    if(IF == annulus_r)
    {
        uint8 start_found = 0;
        if(l_lost_num)
        {
            start_center_y = Deal_Bottom;
            start_center_x = Deal_Left;
            start_found = 1;
        }
        else if(b_lost_num)
        {
            start_center_y = start_center_y;
            start_center_x = b_center[0];
            start_found = 1;
        }
        uint8 end_found = 0;
        if(t_lost_num)
        {
            end_center_y = Deal_Top;
            end_center_x = t_center[0];
            end_found = 1;
        }
        else if(r_lost_num)
        {
            end_center_y = r_center[r_lost_num-1];
            end_center_x = Deal_Right;
            end_found = 1;
        }
//        qout<<start_center_x<<start_center_y<<end_center_x<<end_center_y;
//        qout<<start_found<<end_found;
        if(start_found == 0 || end_found == 0)
        {
            return 0;
        }
        if(end_center_y <= start_center_y)
        {
          return 0;
        }
       float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
       for (uint8 i = start_center_y; i <= end_center_y; ++i)
       {
           mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
           if(mid_line[i] < Deal_Left)
           {
             mid_line[i] = Deal_Left;
           }
           else if(mid_line[i] > Deal_Right)
           {
             mid_line[i] = Deal_Right;
           }
           imgOSTU[i][(uint8)mid_line[i]] = Control_line;
       }
       return end_center_y;
    }
    if(IF == annulus_l)
    {
        uint8 start_found = 0;
        if(r_lost_num)
        {
            start_center_y = Deal_Bottom;
            start_center_x = Deal_Right;
            start_found = 1;
        }
        else if(b_lost_num)
        {
            start_center_y = start_center_y;
            start_center_x = b_center[b_lost_num-1];
            start_found = 1;
        }
        uint8 end_found = 0;
        if(t_lost_num)
        {
            end_center_y = Deal_Top;
            end_center_x = t_center[t_lost_num-1];
            end_found = 1;
        }
        else if(l_lost_num)
        {
            end_center_y = l_center[l_lost_num-1];
            end_center_x = Deal_Left;
            end_found = 1;
        }
//        qout<<start_center_x<<start_center_y<<end_center_x<<end_center_y;
//        qout<<start_found<<end_found;
        if(start_found == 0 || end_found == 0)
        {
            return 0;
        }
        if(end_center_y <= start_center_y)
        {
          return 0;
        }
       float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
       for (uint8 i = start_center_y; i <= end_center_y; ++i)
       {
           mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
           if(mid_line[i] < Deal_Left)
           {
             mid_line[i] = Deal_Left;
           }
           else if(mid_line[i] > Deal_Right)
           {
             mid_line[i] = Deal_Right;
           }
           imgOSTU[i][(uint8)mid_line[i]] = Control_line;
       }
       return end_center_y;
    }
}
/******************************************************************************
* 函数名称     :uint8 Out_Annulus()
* 描述        :完全驶出环岛的判断，防止环岛重新识别
* 进入参数     : void
* 返回参数     : 1：驶出 0：未驶出
* 使用实例     :Out_Annulus();
******************************************************************************/
uint8 Out_Annulus()
{
    //起点终点倾斜则退出
    if(Length < Out_Annulus_Length + 50)
    {
      return 0;
    }
    if(t_center[0] < Deal_Left + 10 || t_center[0] > Deal_Right - 10 ||
       b_center[0] < Deal_Left + 10 || b_center[0] > Deal_Right - 10)
    {
        return 0;
    }
    //驶出为直道
//    if(b_lost_num == 1 && t_lost_num == 1 && !l_lost_num && !r_lost_num)
//    {
//        return 1;
//    }
    if(Length > Out_Annulus_Length + 60)
    {
    //未判断驶出，则依靠行驶距离判断
        return 1;
    }
    else
    {
        return 0;
    }
}
/******************************************************************************
* 函数名称     :uint8 Go_Annulus()
* 描述        :环岛处理的总调用函数
* 进入参数     : void
* 返回参数     :status：当前所处环岛的状态
* 使用实例     :Go_Annulus();
******************************************************************************/
uint8 Go_Annulus()
{
    uint8 status = Annulus_flag;
    //环岛状态判断
    if(status == 0)
    {
        Stop_line = Get_Annulus(); //初步环岛识别
        if(Stop_line)
        {
            status = 1;
        }
        return status;
    }
    else if(status == 1)
    {
        status = status + Into_Annulus_judge(); //正式驶入环岛判断
    }
    else if(status == 2)
    {
        status = status + Leave_Annulus_judge(); //即将驶离环岛判断
    }
    else if(status == 3)
    {
        if(Out_Annulus()) //正式离开环岛判断2
        {
            status = 0;
            return status;
        }
    }

    if(status == 1) //驶入环岛
    {
        Stop_line = Into_Annulus_ing();
        return status;
    }
    else if(status == 2) //正式环岛运行
    {
        Stop_line = Runing_Annulus();
        return status;
    }
    else if(status == 3)  //驶离环岛
    {
        Stop_line = Leave_Annulus_ing();
        return status;
    }

}
/******************************************************************************
* 函数名称     :uint8 Deal_crossroads()
* 描述        :十字路口的识别和处理
* 进入参数     : void
* 返回参数     :end_center_y
* 使用实例     :Go_Annulus();
******************************************************************************/
uint8 Deal_crossroads()
{
    if(elements_Mode == YES && elements_Lock[elements_index] != Disappear_)
    {
      return 0;
    }
    //无四边缘，返回
    if( (b_lost_num+t_lost_num+l_lost_num+r_lost_num) <= 3)
    {
        return 0;
    }
    if( (b_lost_num+t_lost_num+l_lost_num+r_lost_num) == 3 && l_lost_num+r_lost_num == 0)
    {
        return 0;
    }
    //没有底端区域，返回
    if(b_lost_num == 0 || t_lost_num == 0)
    {
        return 0;
    }
    if(b_lost_tip > Deal_W/2) //横向广底端
    {
        return 0;
    }
    if((b_lost_num+t_lost_num+l_lost_num+r_lost_num) == 4)
    {
      uint8 point[4][2];
    uint8 j = 0;
    if(b_lost_num)
    for(int i = 0; i <= b_lost_num-1  && j<=3;i++)
    {
        point[j][0] = b_center[i];
        point[j][1] = Deal_Bottom;
//        qout<<j<<i<<point[j][0]<<point[j][1];
        j++;
    }
    if(r_lost_num)
    for(int i = 0; i <= r_lost_num-1  && j<=3;i++)
    {
        point[j][0] = Deal_Right;
        point[j][1] = r_center[i];
//        qout<<j<<i<<point[j][0]<<point[j][1];
        j++;
    }
    if(t_lost_num)
    for(int i = t_lost_num-1; i >= 0  && j<=3;i--)
    {
        point[j][0] = t_center[i];
        point[j][1] = Deal_Top;
//        qout<<j<<i<<point[j][0]<<point[j][1];
        j++;
    }
    if(l_lost_num)
    for(int i = l_lost_num-1; i >= 0  && j<=3;i--)
    {
        point[j][0] = Deal_Left;
        point[j][1] = l_center[i];
//        qout<<j<<i<<point[j][0]<<point[j][1];
        j++;
    }
    if(j < 3)
    {
        return 0;
    }
    if(my_abs(point[1][0]-point[2][0]) < Deal_W*2/5 ||
       my_abs(point[2][0]-point[3][0]) < Deal_W*2/5 ||
       my_abs(point[1][0]-point[3][0]) < Deal_W*2/5 )
    {
        return 0;
    }
    //中线直线判断
    uint8 Black_num = 0;
    float k = ((float)point[2][0] - (float)point[0][0]) / ((float)point[2][1] - (float)point[0][1]);
    for (uint8 i = point[0][1]; i <= point[2][1]; ++i)
    {
        if(imgOSTU[i][(uint8)(point[2][0] + k * (i - point[0][1]))] != White)
        {
            Black_num++;
        }
    }
//    qout<<Black_num;
    if(Black_num < Deal_H/2)
    {
        start_center_x = point[0][0];
        start_center_y = point[0][1];
        end_center_x = point[2][0];
        end_center_y = point[2][1];
        for (uint8 i = start_center_y; i <= end_center_y; ++i)
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
    float White_num[12] = {0};
    float Black_num[12] = {0};
    //顺时针四边缘统计
    int point[6][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
    int j = 0;
    int count_piont = 0;
    float k_count[12] = {0};
    if(r_lost_num)
    for(int i = 0; i <= r_lost_num-1  && j<=5;i++)
    {
        point[j][0] = Deal_Right;
        point[j][1] = r_center[i];
        j++;
    }
    if(t_lost_num)
    for(int i = t_lost_num-1; i >= 0  && j<=5;i--)
    {
        point[j][0] = t_center[i];
        point[j][1] = Deal_Top;
        j++;
    }
    if(l_lost_num)
    for(int i = l_lost_num-1; i >= 0  && j<=5;i--)
    {
        point[j][0] = Deal_Left;
        point[j][1] = l_center[i];
        j++;
    }
    count_piont = j-1;
    if(count_piont > 5)
    {
        return 0;
    }
    for(int i = 0;i <= b_lost_num-1 && i < 2;i++)
    {
        for(int j = 0;j <= count_piont && j < 6;j++)
        {
            int start_x = b_center[i];
            int start_y = start_center_y;
            int end_x = point[j][0];
            int end_y = point[j][1];
            int index = 6*i+j;
            if (end_y <= start_y)
            {
                 Black_num[index] = 0;
                 White_num[index] = 0;
                 continue;  // 跳过无效点对
            }
            float k = ((float)end_x - (float)start_x) / ((float)end_y - (float)start_y);
            k_count[index] = k;
            uint8 y_start = (start_y < end_y) ? start_y : end_y;
            uint8 y_end = (start_y < end_y) ? end_y : start_y;
            for (uint8 y = start_y; y <= end_y; ++y)
            {
                int x = (float)(start_x + k * (y - start_y));
                if (x > Deal_Right) 
                {
                  x = Deal_Right;  // 钳制到有效范围
                } 
                else if (x < Deal_Left) 
                {
                  x = Deal_Left;
                }
                
                if(imgOSTU[y][x] == White)
                {
                    White_num[index] += Length_5cm[0]/Length_5cm[y];
                }
                else
                {
                    Black_num[index] += Length_5cm[0]/Length_5cm[y];
                    imgOSTU[y][x] = Judge_line;
                }
                
            }
        }
    }
    int Straight_index = 0;
    float Straight_scale = 0;
    for(int i = 0;i <= b_lost_num-1 && i < 2;i++)
    {
        for(int j = 0;j <= count_piont && j < 6;j++)
        {
            int index =  6*i+j;
            float scale = 0;

            if(White_num[index] >  15 && White_num[index] > Black_num[index])
            {
                scale = (float)White_num[index]/((float)White_num[index]+(float)Black_num[index]);
            }
            else
            {
                scale = 0;
            }

            if(scale > Straight_scale)
            {
                Straight_scale = scale;
                Straight_index = index;
            }
            else if(scale >=  Straight_scale*0.7 && scale != 0)
            {
                int index_y = point[(int)index%6][1];
                int Straight_y = point[(int)Straight_index%6][1];

                if(index_y >= Straight_y - 10)
                {
                    if(nowDeviation*k_count[index] > 0 && nowDeviation*k_count[Straight_index] < 0)  //取误差斜率同号
                    {
                        Straight_scale = scale;
                        Straight_index = index;
                    }
                    else if(nowDeviation*k_count[index] < 0 && nowDeviation*k_count[Straight_index] > 0)  //取误差斜率同号
                    {
                        Straight_scale = Straight_scale;
                        Straight_index = Straight_index;
                    }
                    else
                    {
                        float index_k = nowDeviation-k_count[index];
                        float Straight_k = nowDeviation-k_count[Straight_index];
                        index_k = index_k > -index_k ? index_k : -index_k;
                        Straight_k = Straight_k > -Straight_k ?Straight_k:-Straight_k;
                        if(index_k < Straight_k)
                        {
                            Straight_scale = scale;
                            Straight_index = index;
                        }
                    }
                }

            }
        }
    }

    start_center_x = b_center[(int)Straight_index/6];
    end_center_x = point[(int)Straight_index%6][0];
    end_center_y = point[(int)Straight_index%6][1];
    if(end_center_y <= start_center_y)
    {
      return 0;
    }
    float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
    for (uint8 i = start_center_y; i <= end_center_y; ++i)
    {
        mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
        if (mid_line[i] > Deal_Right) 
        {
          mid_line[i] = Deal_Right;  // 钳制到有效范围
        } 
        else if (mid_line[i] < Deal_Left) 
        {
          mid_line[i] = Deal_Left;
        }
        imgOSTU[i][(uint8)mid_line[i]] = Control_line;
    }
    IF = crossroads;
    return end_center_y;

    return 0;
}


/******************************************************************************
* 函数名称     :uint8 Disappear_detection()
* 描述         : 断路检测函数
******************************************************************************/
uint8 Disappear_detection() //断路检测函数
{
    static int Dis_appear[5] = {0}; //断路特征缓冲数组
    static int offsets_angle[5] = {0}; //倾斜角度补偿组
    
    Dis_appear[4] = Dis_appear[3];
    Dis_appear[3] = Dis_appear[2];
    Dis_appear[2] = Dis_appear[1];
    Dis_appear[1] = Dis_appear[0];
    if(
      disappear_num < disappear_total&& //少于断路数目
      (Stop_line == 0 || dis_Solidline) &&  //没有获取边缘巡线特征
      start_center_y <= Deal_Bottom+3 &&
      b_lost_num <= 1 &&  //单个或无底部特征
      b_lost_tip <= white_width[start_center_y]*3/2 &&   //底部宽度正常
      EXTERN_sum[LR] <= 50 && EXTERN_sum[UD] <= 50 &&
      Dis_appear[1] >= 1 &&//上一帧满足断路特征
      imgOSTU[Lost_Bottom][Lost_Right] != Lost_line && imgOSTU[Lost_Bottom][Lost_Left] != Lost_line
      )
    {
      int white_num = 0;
      for(uint8 i = Deal_Bottom+5;i <= Deal_H/2;i++)
      {
        white_num += white_num_row[i]; //统计底部白点
      }
      if(white_num <= 3)
      {
        Dis_appear[0] = 2;
      }
      else
      {
        Dis_appear[0] = 1;
      }
    }
    else if(
      (disappear_num < disappear_total)&& //少于断路数目
      (Stop_line == 0 || dis_Solidline)&&  //没有获取边缘巡线特征
      start_center_y <= Deal_Bottom + 3 &&  //白底起点不为0
      b_lost_num == 1 &&  //单个底部特征
      b_lost_tip <= white_width[start_center_y]*3/2 &&   //底部宽度正常
      EXTERN_sum[LR] <= 50 && EXTERN_sum[UD] <= 50 &&
      imgOSTU[Lost_Bottom][Lost_Right] != Lost_line && imgOSTU[Lost_Bottom][Lost_Left] != Lost_line
      )
    {
      if(nowDeviation == Deviation_limit || nowDeviation == -Deviation_limit)
      {
        if(EXTERN_L || EXTERN_R || dis_Solidline)
        {
           Dis_appear[0] = 1;
        }
        else
        {
            Dis_appear[0] = 0;
        }
      }
      else
      {
         Dis_appear[0] = 1;
      }
        if(Dis_appear[1] == 1 ) //连续断路特征
        {
        for(uint8 i = 0;i <= hightest;i++)
        {
            if(white_num_row[i] > 2*white_width[i])  //判断赛道不过宽
            {
                Dis_appear[0] = 0;
                break;
            }
            else if(white_num_row[i] == 0)  //限制断路尽头高度
            {
                if(i >= 50)
                {
                    Dis_appear[0] = 0;
                }
                else if(i < Deal_Bottom+15)
                {
                  int white_num = 0;
                  for(int y=i;y<=Deal_Top;y++)
                  {
                    white_num += white_num_row[y];
                    if(white_num>=0)
                    {
                      Dis_appear[0] = 0;
                      break;
                    }
                  }
                    if(white_num==0)
                    {
                    Dis_appear[0] = 2;
                    }
                }
                else
                {
                    Dis_appear[0] = 1;
                }
                break;
            }
        }
        }
    }
    else
    {
        Dis_appear[0] = 0;
    }
       if(Disappear_angle_L[disappear_num] != 0 && runState != TEST && run_dir == Left && 
         (imu_data.yaw - Start_angle > Disappear_angle_L[disappear_num] + 45 || imu_data.yaw - Start_angle < Disappear_angle_L[disappear_num] - 45))
      {
        return 0;
      }
      if(Disappear_angle_R[disappear_num] != 0 && runState != TEST && run_dir == Right && 
         (imu_data.yaw - Start_angle > Disappear_angle_R[disappear_num] + 45 || imu_data.yaw - Start_angle < Disappear_angle_R[disappear_num] - 45))
      {
        return 0;
      }
    if(Dis_appear[0] == 1)        //存在长白道，进行巡线
        {
        get_start_point();
        end_center_y = 0;
        uint8 l_found = 0,r_found = 0;//清零
        int32 l_point = 0,r_point = 0;
        end_center_y = 0;
        for(int i = Deal_Top; i >= Deal_Bottom;i--)
        {
            if(white_num_row[i] >= 3 && white_num_row[i+1] <= 3)
            {
              end_center_y = i;
              for(int j = i;j >= Deal_Bottom;j--)
              {
                if(white_num_row[i] > 2*white_width[i])
                {
                   end_center_y = 0;
                   break;
                }
              }
                break;
            }
        }
        if(end_center_y >= 10)
        {
            for (uint8 i = 0; i < XX-1; i++)
            {
                if (imgOSTU[end_center_y][i+1] == White && imgOSTU[end_center_y][i] != White)
                {//找到左边起点
                    l_point = i;
                    l_found = 1;
                    break;
                }
            }
            //从右向左，先找黑白跳变点
            for (uint8 i = XX; i > 0+1; i--)
            {
                if (imgOSTU[end_center_y][i-1] == White && imgOSTU[end_center_y][i] != White)
                {//找到右边起点
                    r_point = i;
                    r_found = 1;
                    break;
                }
            }
        }
        if(l_found && r_found)//找到左右起点
            {
            end_center_x = (int)(l_point+r_point)/2;
            float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
            for (uint8 i = start_center_y; i <= end_center_y; ++i)
            {
                mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
                imgOSTU[i][(uint8)mid_line[i]] = Control_line;
            }
            }
        Stop_line = end_center_y;
        }
    offsets_angle[4] = offsets_angle[3];
    offsets_angle[3] = offsets_angle[2];
    offsets_angle[2] = offsets_angle[1];
    offsets_angle[1] = offsets_angle[0];
    offsets_angle[0] = kbmid[0];
    if(Dis_appear[1] != 1 && Dis_appear[2] != 1 && Dis_appear[3] != 1 && Dis_appear[4] != 1)
    {
      return 0;
    }
    if(Dis_appear[0] == 2 && Dis_appear[1]+Dis_appear[2]+Dis_appear[3]+Dis_appear[4] >= 4)  //三帧断路特征
    {
      if(Disappear_angle_L[disappear_num] != 0 && runState != TEST && run_dir == Left && 
         (imu_data.yaw - Start_angle > Disappear_angle_L[disappear_num] + 45 || imu_data.yaw - Start_angle < Disappear_angle_L[disappear_num] - 45))
      {
        return 0;
      }
      if(Disappear_angle_R[disappear_num] != 0 && runState != TEST && run_dir == Right && 
         (imu_data.yaw - Start_angle > Disappear_angle_R[disappear_num] + 45 || imu_data.yaw - Start_angle < Disappear_angle_R[disappear_num] - 45))
      {
        return 0;
      }
        IF = disappear;
        elements_index++;
        disappear_num++;
//        Disappear_Zero = imu_data.yaw + (offsets_angle[4]*4+offsets_angle[3]*3+offsets_angle[2]*2+offsets_angle[1]*1)/2;
//        Disappear_Zero = 0;
        Disappear_Zero =  imu_data.yaw;

        Disappear_Position = Length;
        return 1;
    }
    else
    {
        return 0;
    }
}

/******************************************************************************
* 函数名称     :uint8 Deal_disappear()
* 描述         : 断路处理函数
******************************************************************************/
uint8 Deal_disappear() //断路处理函数
{
    int status = disappear_flag;
    if(status == 0)
    {   Disappear_Zero = Start_angle;
        status = Disappear_detection(); //初步识别断路，存在白点状态
    }
    if(status || Write_Dir)
    {
        Disappear_Yaw = imu_data.yaw - Disappear_Zero; //断路拐角
        Disappear_Length = Length - Disappear_Position;  //断路路程
    }
    if(status == 1) //纯黑色区域，真正断路地带,纯偏航角度1
    {
        if(run_dir == Left)  //获取断路偏航角度
        {
            nowDeviation =  (float)Disappear_Dir_L[disappear_num-1][0] - Disappear_Yaw;
        }
        else if(run_dir == Right)
        {
            nowDeviation =  (float)Disappear_Dir_R[disappear_num-1][0] - Disappear_Yaw;
        }

        //超过行驶路程判断进入下一个状态
        if(run_dir == Left && Disappear_Length >= Disappear_Dir_L[disappear_num-1][2])
        {
            status = 2;
        }
        else if(run_dir == Right && Disappear_Length >= Disappear_Dir_R[disappear_num-1][2])
        {
            status = 2;
        }

    }
    if(status == 2) //即将走出纯黑区，偏正驶入角度
    {

        if(run_dir == Left)
        {
            nowDeviation =  (float)Disappear_Dir_L[disappear_num-1][1] - Disappear_Yaw;
        }
        else if(run_dir == Right)
        {
            nowDeviation =  (float)Disappear_Dir_R[disappear_num-1][1] - Disappear_Yaw;
        }

        //判断找到白线，进入巡线状态
        if(nowDeviation > -80 || nowDeviation < 80)
        {
            int32 Black_row = 0;
            uint8 Black_end = YY;

            for(uint8 i = YY;i >= 30;i--)
            {
                if(white_num_row[i] >= 5)
                {
                    for(uint8 k = i;k >= 30;k--)
                    {
                        if(white_num_row[k] >= 3)
                        {
                            Black_row++;
                        }
                        else
                        {
                            Black_end = k+1;
                            break;
                        }
                    }
                    if(Black_row >=5 && Black_end)
                    {
                        status = 3;
                    }
                }
            }
        }

    }
    if(status == 3)
    {
        uint8 l_found = 0,r_found = 0;//清零
        int32 l_point = 0,r_point = 0;
        end_center_y = 0;
        for(uint8 i = Deal_Top; i >= Deal_Bottom+20;i--)
        {
            if(white_num_row[i] >= 3)
            {
                end_center_y = i;
                break;
            }
        }
        if(end_center_y)
        {
            for (uint8 i = 0; i < XX-1; i++)
                    {
                        if (imgOSTU[Deal_Top][i+1] == White && imgOSTU[Deal_Top][i] != White)
                        {//找到左边起点
                            l_point = i;
                            l_found = 1;
                            break;
                        }
                    }
                    //从右向左，先找黑白跳变点
                    for (uint8 i = XX; i > 0+1; i--)
                    {
                        if (imgOSTU[Deal_Top][i-1] == White && imgOSTU[Deal_Top][i] != White)
                        {//找到右边起点
                            r_point = i;
                            r_found = 1;
                            break;
                        }
                    }
        }
        if(l_found && r_found)//找到左右起点
        {
            end_center_x = (int)(l_point+r_point)/2;
            start_center_x = (Deal_Left + Deal_Right)/2;
            start_center_y = Deal_Bottom;
            float k = ((float)end_center_x - (float)start_center_x) / ((float)end_center_y - (float)start_center_y);
            for (uint8 i = start_center_y; i <= end_center_y; ++i)
            {
                mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
                imgOSTU[i][(uint8)mid_line[i]] = Control_line;
            }
        }
        Stop_line = end_center_y;


        for(uint8 i = Deal_Bottom + 30;i >= Deal_Bottom + 20;i--)
        {
            if(white_num_row[i] >= 5)
            {
                status = 0;
            }
        }
    }
    if(status)
    {
      IF = disappear;
    }
    return status;
}

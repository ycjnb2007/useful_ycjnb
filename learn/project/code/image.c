#include "image.h"
#include <math.h>
//============================ 全局变量定义 ============================//
// 图像数据存储
uint8 image_use[LCDH][LCDW]; // 压缩后图像
uint8 image[LCDH][LCDW];     // 二值化图像
uint8 img_threshold = 0; // 二值化阈值
uint8 M_line_count = 0; // 中线点的个数
uint8 M_last_count = 0;           // 上一帧中线点的个数
uint8 Road_width = 10;            // 道路宽度，根据实际情况调整
float Err = 0;                    // 摄像头误差
ImageFlagtypedef ImageFlag = {0}; // 初始化为0
uint8 Road_Wide[LCDH];            // 赛道宽度数组
uint8 L_End_WHITELine = 0; // 左边搜索到的白色线位置
uint8 R_End_WHITELine = 0; // 右边搜索到的白色线位置
uint8 line_min = 5;        // 图像处理左边界
uint8 line_max = 95;       // 图像处理右边界
uint8 outsthreshold; // 二值化阈值

const uint8 Weight_Cricle[LCDH] =
    {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,           // 图像近端10-19行权重（最低权重）
        1, 1, 1, 1, 1, 1, 1, 1, 2, 3,           // 图像近端10-19行权重（最低权重）
        10, 11, 12, 13, 14, 15, 16, 17, 18, 19, // 图像中部30-39行权重（逐渐增加到高权重）
        20, 20, 19, 18, 17, 16, 15, 14, 13, 12, // 图像中远端40-49行权重（高权重区域平滑下降）
        10, 11, 12, 13, 14, 15, 16, 17, 18, 19, // 图像中部30-39行权重（逐渐增加到高权重）
        20, 20, 19, 18, 17, 16, 15, 14, 13, 12, // 图像中远端40-49行权重（高权重区域平滑下降）
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,           // 图像最近端00-09行权重（最低权重）
}; // 环岛权重数组：适中强度钟形分布，峰值20，平衡控制力度与稳定性

// 直角专用权重数组
const uint8 Weight_Angle[LCDH] =
    {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像近端10-19行权重（最低权重）
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像最近端00-09行权重（最低权重）
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像近端10-19行权重（最低权重）
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像近端20-29行权重（最低权重）
        2, 2, 3, 3, 3, 4, 4, 4, 4, 4, // 图像中部30-39行权重（适中权重，逐渐增加）
        4, 4, 4, 4, 4, 5, 5, 5, 6, 6, // 图像最远端50-59行权重（最低权重）//这套十字怪怪的，提前好多转
        6, 7, 7, 8, 8, 8, 8, 8, 8, 8, // 图像远端40-49行权重（适中权重，逐渐减小
}; // 直角权重数组：平滑钟形分布，强调中部区域，提高转向响应速度

const uint8 Weight[LCDH] =
    {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像近端10-19行权重（最低权重）
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像最近端00-09行权重（最低权重）
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像近端10-19行权重（最低权重）
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像近端20-29行权重（最低权重）
        2, 2, 3, 3, 3, 4, 4, 4, 4, 4, // 图像中部30-39行权重（适中权重，逐渐增加）
        4, 4, 4, 4, 4, 5, 5, 5, 6, 6, // 图像最远端50-59行权重（最低权重）//这套十字怪怪的，提前好多转
        6, 7, 7, 8, 8, 8, 8, 8, 8, 8, // 图像远端40-49行权重（适中权重，逐渐减小
                                      // 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, // 图像最近端00-09行权重（最低权重）
}; // 权重数组
//注意我的边界记录翻转了坐标系
//这个数组其实完全没必要算这么多次，留个20行就完全够了，但考虑到时间不够了更改后转向环会有大幅的改变就别动了
//不要学这个，一个数组就够了

/**
 * @brief 图像采集和裁剪函数
 * @details 从MT9V03X摄像头获取图像并裁剪到指定尺寸
 * @note 裁剪区域: 从原图(188x120)裁剪到(120x70)
 *       起始位置: 行偏移10, 列偏移34，保证是拍前面的部分保证前瞻够长，70行是为了尽量保留近处细节
 *       优化: 使用指针操作提高内存访问效率
 * @author 尘烟
 */
void Get_userimage(void)
{
    uint8 *src_ptr = &mt9v03x_image[13][34]; // 源图像起始指针
    uint8 *dst_ptr = &image_use[0][0];       // 目标图像起始指针

    for (int row = 0; row < LCDH; row++)
    {
        for (int col = 0; col < LCDW; col++) // 逐像素拷贝，保证数据完整性
        {
            *dst_ptr++ = *src_ptr++;
        }
        src_ptr += (MT9V03X_W - LCDW); // 跳到下一行的起始位置
    }
}


/**
 * @brief 大津法
 * @param image 输入图像数据指针
 * @param width 图像宽度
 * @param height 图像高度
 * @return 计算得到的最优阈值(40-255)
 * @note 注意最低阈值的改变。因为微缩赛道特殊性，阈值可能出现过低和过大频繁跳动的情况，所以给最低限幅，省赛时改为了50，国赛限幅改为了80
 * @author 来源安财曙光队
 */
int My_Adapt_Threshold(uint8 *image, uint16 width, uint16 height)
{
#define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height / 4;
    int threshold = 0;
    uint8 *data = image; // 指向像素数据的指针

    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum = 0;
    for (i = 0; i < height; i += 2) // 统计灰度级中每个像素在整幅图像中的个数
    {
        for (j = 0; j < width; j += 2)
        {
            pixelCount[(int)data[i * width + j]]++; // 将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];   // 灰度值总和
        }
    }

    for (i = 0; i < GrayScale; i++) // 计算每个像素值的点在整幅图像中的比例
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

    for (j = 0; j < GrayScale; j++) // 遍历灰度级[0,255]
    {
        w0 += pixelPro[j];        // 背景部分每个灰度值的像素点所占比例之和
        u0tmp += j * pixelPro[j]; // 背景部分每个灰度值的点的比例*灰度值
        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;
        u0 = u0tmp / w0;                                                // 背景平均灰度
        u1 = u1tmp / w1;                                                // 前景平均灰度
        u = u0tmp + u1tmp;                                              // 全局平均灰度
        deltaTmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u); // 类间方差计算

        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp; // 最大类间方差法
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }

    if (threshold > 255) threshold = 255;
    if (threshold < 40) threshold = 40; // 最小限制
    return threshold;
}

/**
 * @brief 优化迭代阈值算法
 * @details 基于加权迭代法计算二值化阈值，偏向高亮区域
 * @param image 源图像数据指针
 * @param total_pixels 图像总像素数
 * @param min_max_dell 是否删除极端灰度值
 * @return 计算得到的阈值
 * @note 算法较慢，比大津法慢，而且我没有去进行优化，实际效果差不了多少，最终没采用
 * @author 来源广技师佬
 */
uint8 Iterative_Threshold_Opt(uint8 *image, uint16 total_pixels, uint8 min_max_dell)
{
    uint16 pixel_Count[256] = {0};
    uint8 predict_Threshold = 0, threshold = 0;
    static uint8 s_threshold = 0;
    float alpha = 0.8; // 前景权重系数（0.5~0.8）

    // 1. 直方图统计（不修改total_pixels）
    uint16 actual_valid = 0;
    for (uint16 i = 0; i < total_pixels; i++)
    {
        uint8 pixel = image[i];
        if (pixel != 0 && pixel != 255)
        {
            pixel_Count[pixel]++;
            actual_valid++;
        }
    }

    // 2. 清除极端灰度（不依赖valid_Count）
    if (min_max_dell)
    {
        for (int i = 0; i < 5; i++)
            pixel_Count[i] = 0;
        for (int i = 251; i < 256; i++)
            pixel_Count[i] = 0;
        // 移除孤岛检测逻辑
    }

    // 3. 计算有效灰度范围
    uint8 minGray = 0, maxGray = 255;
    while (minGray < 255 && pixel_Count[minGray] == 0)
        minGray++;
    while (maxGray > 0 && pixel_Count[maxGray] == 0)
        maxGray--;

    // 4. 计算总灰度和
    uint32 total_sum = 0;
    for (int i = minGray; i <= maxGray; i++)
    {
        total_sum += pixel_Count[i] * i;
    }

    // 5. 动态初始阈值
    predict_Threshold = (s_threshold >= minGray && s_threshold <= maxGray)
                            ? s_threshold
                            : (minGray + maxGray) / 2;

    // 6. 加权迭代（偏向高亮区域）
    for (uint8 t = 0; t < 20; t++)
    {
        uint32 sum0 = 0, sum1 = 0;
        uint16 w0 = 0, w1 = 0;

        for (int i = minGray; i <= maxGray; i++)
        {
            if (i < predict_Threshold)
            {
                w0 += pixel_Count[i];
                sum0 += pixel_Count[i] * i;
            }
            else
            {
                w1 += pixel_Count[i];
                sum1 += pixel_Count[i] * i;
            }
        }

        float u0 = (w0 > 0) ? (sum0 / (float)w0) : 0;
        float u1 = (w1 > 0) ? (sum1 / (float)w1) * alpha : 0; // 加权前景
        predict_Threshold = (uint8)((u0 + u1) / (1 + alpha)); // 新预测阈值

        if (abs(predict_Threshold - threshold) <= 1)
            break; // 比较阈值，合适就返回
        else if (abs(predict_Threshold - s_threshold) >= 10)
            threshold = s_threshold;
        else if (predict_Threshold < 75 || s_threshold < 75)
            threshold = 75; // 32
        else
            threshold = predict_Threshold; // 记录阈值用于下一次比较
    }

    s_threshold = predict_Threshold;
    return threshold;
} // 可以和大津法一起优化下

/**
 * @brief 图像二值化处理函数
 * @param value 二值化阈值
 * @details 将灰度图像转换为黑白二值图像
 * @note 优化特性:
 *       - 使用指针操作替代二维数组访问
 *       - 采用4路循环展开提高处理速度
 *       - 像素值 < threshold -> BLACK(0)
 *       - 像素值 >= threshold -> WHITE(255)
 * @author 尘烟
 */
void set_image_twovalues(uint8 value)
{
    // 优化：使用指针操作和循环展开
    uint8 *src_ptr = &image_use[0][0];
    uint8 *dst_ptr = &image[0][0];
    const int total_pixels = LCDH * LCDW;

    // 4路循环展开优化
    int remaining = total_pixels;
    while (remaining >= 4)
    {
        // 处理4个像素
        *dst_ptr++ = (*src_ptr++ < value) ? BLACK : WHITE;
        *dst_ptr++ = (*src_ptr++ < value) ? BLACK : WHITE;
        *dst_ptr++ = (*src_ptr++ < value) ? BLACK : WHITE;
        *dst_ptr++ = (*src_ptr++ < value) ? BLACK : WHITE;
        remaining -= 4;
    }

    // 处理剩余像素
    while (remaining--)
    {
        *dst_ptr++ = (*src_ptr++ < value) ? BLACK : WHITE;
    }
}

/*--------------------------------找边界-------------------------------*/
struct LEFT_EDGE Left_line[150];
struct RIGHT_EDGE Right_line[150];

uint8 L_edge_count = 0, R_edge_count = 0; // 左右边点的个数

uint8 L_start_y = 0;
uint8 L_start_x = 0;

uint8 R_start_y = 0;
uint8 R_start_x = 0;

uint8 Left_line_count = 0;
uint8 Right_line_count = 0;

struct MID Mid_line[150]; // 中线结构体数组
uint8 Mid_line_count = 0; // 中线点的数量
/************************************扫线******************************************/
// 全局变量声明
uint16 AllLose = 0;          // 完全丢线计数
uint16 LeftLose = 0;         // 左边丢线计数
uint16 RightLose = 0;        // 右边丢线计数
uint8 lose_line = 0;         // 连续丢线行数
uint8 MidPri = LCDW / 2;     // 上一次的中线位置
uint8 Road_Wide[LCDH] = {0}; // 每行赛道宽度记录

/// 在文件开头添加全局变量
uint16 last_valid_road_width = 20; // 默认赛道宽度，初始值设为20

/**
 * @brief 改进的边界搜索和中线计算函数
 * @details 从图像底部向上逐行搜索左右边界，计算中线
 * @note 基于黑白白跳变检测边界，支持虚线处理和赛道宽度自适应
 * @author 尘烟、郑轻工lmy(根据汩汩机之前发的改)
 */

void Searchline_Improved(void)
{
    int16 row = LCDH - 1; // 从最底行开始
    int16 col = 0;
    int16 leftStart = angle_ki;       // 左边界搜索起始列
    int16 rightEnd = LCDW - angle_ki; // 右边界搜索结束列
    uint16 current_road_width = 0;    // 当前行的赛道宽度

    // 重置计数器和标志位
    AllLose = 0;
    LeftLose = 0;
    RightLose = 0;
    Left_line_count = 0;
    Right_line_count = 0;
    Mid_line_count = 0;
    lose_line = 0;

    // 从底部向上扫描
    while (row > 1)
    {
        uint8 left_found = 0;
        uint8 right_found = 0;

        // 从左到右搜索左边界（黑白白跳变）
        for (col = leftStart; col < rightEnd - 2; col++)
        {
            if (image[row][col] == BLACK &&
                image[row][col + 1] == WHITE &&
                image[row][col + 2] == WHITE)
            {
                // 将找到的边界点存入结构体数组
                if (Left_line_count < 150)
                {
                    Left_line[Left_line_count].row = row;
                    Left_line[Left_line_count].col = col + 1; // 存储白色的第一个点

                    Left_line_count++;
                    left_found = 1;
                }
                break; // 找到后跳出循环
            }
        }

        // 从右到左搜索右边界（黑白白跳变）
        for (col = rightEnd; col > leftStart + 2; col--)
        {
            if (image[row][col] == BLACK &&
                image[row][col - 1] == WHITE &&
                image[row][col - 2] == WHITE)
            {
                // 将找到的边界点存入结构体数组
                if (Right_line_count < 150)
                {
                    Right_line[Right_line_count].row = row;
                    Right_line[Right_line_count].col = col - 1; // 存储白色的第一个点

                    Right_line_count++;
                    right_found = 1;
                }
                break; // 找到后跳出循环
            }
        }

        // 计算中间边界
        uint8 mid_col = LCDW / 2; // 默认中线位置

        // 获取当前行的左右边界列坐标
        int16 left_col = (left_found) ? Left_line[Left_line_count - 1].col : leftStart;
        int16 right_col = (right_found) ? Right_line[Right_line_count - 1].col : rightEnd;

        // 计算并更新赛道宽度
        if (left_found && right_found)
        {
            // 如果两边都找到了边界，计算实际宽度
            current_road_width = right_col - left_col;

            // 更新全局赛道宽度，条件：宽度在合理范围内
            if (current_road_width >= 5 && current_road_width <= 20)
            {
                // 平滑过渡 - 新宽度占75%，旧宽度占25%
                last_valid_road_width = (current_road_width * 3 + last_valid_road_width) >> 2;

                // 记录该行的赛道宽度
                Road_Wide[row] = current_road_width;
            }

            // 双边界都找到，取中点
            mid_col = (left_col + right_col) >> 1;
            if (image[row][mid_col] == WHITE)
                lose_line = 0; // 中线在白色区域，重置丢线计数
        }
        else if (left_found && !right_found)
        {
            // 只有左边界，使用最近有效的赛道宽度推算中线
            RightLose++;
            mid_col = left_col + (last_valid_road_width >> 1);

            // 记录该行使用的估计赛道宽度
            Road_Wide[row] = last_valid_road_width;
        }
        else if (!left_found && right_found)
        {
            // 只有右边界，使用最近有效的赛道宽度推算中线
            LeftLose++;
            mid_col = right_col - (last_valid_road_width >> 1);

            // 记录该行使用的估计赛道宽度
            Road_Wide[row] = last_valid_road_width;
        }
        else // 双边界都丢失
        {
            AllLose++;
            if (lose_line < 5) // 连续丢线少于5行时才补中线
            {
                if (row == LCDH - 1)
                {
                    mid_col = MidPri; // 使用上一帧的中线
                }
                else if (Mid_line_count > 0)
                {
                    mid_col = Mid_line[Mid_line_count - 1].col; // 使用上一行的中线
                }

                if (image[row][mid_col] == BLACK)
                {
                    lose_line++; // 中线在黑色区域，丢线计数增加
                }
            }
            // 记录该行使用的估计赛道宽度
            Road_Wide[row] = last_valid_road_width;
        }
        // 限制中线位置在有效范围内
        if (mid_col > rightEnd)
            mid_col = rightEnd;
        if (mid_col < leftStart)
            mid_col = leftStart;

        // 记录中线点
        if (Mid_line_count < 150)
        {
            Mid_line[Mid_line_count].row = row;
            Mid_line[Mid_line_count].col = mid_col;
            Mid_line_count++;
        }

        // 为下一行更新搜索范围
        if (left_found)
            leftStart = left_col - 20; // 以当前行左边界为中心向左扩展15像素
        if (right_found)
            rightEnd = right_col + 20; // 以当前行右边界为中心向右扩展15像素

        // 确保搜索范围不超出图像
        if (leftStart < 5)
            leftStart = 5;
        if (rightEnd > LCDW - 5)
            rightEnd = LCDW - 5;

        // 记录最底行的中线位置，用于下一帧
        if (row == LCDH - 1)
        {
            MidPri = mid_col;
        }

        row--; // 向上移动一行
    }
}

/**
 * @brief 边界线和中线绘制函数
 * @details 在LCD屏幕上绘制检测到的左右边界线和中线
 * @note 左边界-蓝色，右边界-绿色，中线-红色，单像素宽度
 * @author 尘烟
 */
void draw_line(void)
{
    for (int i = 0; i < Left_line_count; i++)
    {
        uint8 col = Left_line[i].col; // 只绘制单个像素点
        uint8 row = Left_line[i].row;
        if (col < LCDW && row < LCDH) // 确保不越界
        {
            tft180_draw_point(col, row, RGB565_BLUE);
        }
    }

    for (int i = 0; i < Right_line_count; i++)
    {
        uint8 col = Right_line[i].col; // 只绘制单个像素点
        uint8 row = Right_line[i].row;
        if (col < LCDW && row < LCDH) // 确保不越界
        {
            tft180_draw_point(col, row, RGB565_GREEN);
        }
    }

    if (Mid_line_count > 0)
    {
        for (int i = 0; i < Mid_line_count; i++)
        {
            uint8 col = (uint8)(Mid_line[i].col); // 只绘制单个点，使中线宽度为1
            uint8 row = (uint8)(Mid_line[i].row);
            tft180_draw_point(col, row, RGB565_RED); // 只绘制主点
        }
    }
}

/**
 * @brief 加权误差计算函数
 * @details 基于中线点计算车辆偏离赛道中心的加权误差
 * @return 计算得到的偏差误差值
 * @note 根据赛道类型自动选择权重数组，支持环岛特殊处理
 * @author 尘烟
 */
float Err_Sum(void)
{
    int i;
    float err = 0;
    float weight_count = 0;
    const uint8 *current_weight; // 当前使用的权重数组指针

    if (judge.Left_Flag || judge.Right_Flag) // 根据直角检测结果选择权重数组
    {
        current_weight = Weight_Angle; // 直角时使用专用权重
    }
    else
    {
        current_weight = Weight; // 正常情况使用默认权重
    }

    for (i = 0; i < Mid_line_count && i <= 70; i++) // 使用中线点计算误差，最多使用前70个点
    {
        err += (LCDW / 2 - Mid_line[i].col) * current_weight[i]; // 计算中线点到图像中心的偏差
        weight_count += current_weight[i];
    }

    if (weight_count > 0)
    {
        err = err / weight_count;

        if (err < huandaoerr && Circle.In_circle && Circle.now_yaw > 60)
        {
            err = huandaoerr;
        }
        else if (err > -huandaoerr && Circle.In_circle && Circle.now_yaw < -60)
        {
            err = -huandaoerr;
        }
    }

    return err;
}

// 上区域四行检测变量 (第1,6,11,16行)
uint8 top_line1_num = 0; // 第1行赛道边界数量
uint8 top_line2_num = 0; // 第6行赛道边界数量
uint8 top_line3_num = 0; // 第11行赛道边界数量
uint8 top_line4_num = 0; // 第16行赛道边界数量
// 中间区域四行检测变量 (第25,30,35,40行)
uint8 middle_line1_num = 0; // 第25行赛道边界数量
uint8 middle_line2_num = 0; // 第30行赛道边界数量
uint8 middle_line3_num = 0; // 第35行赛道边界数量
uint8 middle_line4_num = 0; // 第40行赛道边界数量
// 下区域四行检测变量 (第69,64,59,54行)
uint8 bottom_line1_num = 0; // 第69行赛道边界数量
uint8 bottom_line2_num = 0; // 第64行赛道边界数量
uint8 bottom_line3_num = 0; // 第59行赛道边界数量
uint8 bottom_line4_num = 0; // 第54行赛道边界数量
/**
 * @brief 统计图像中特定行的赛道边界数量 - 四行检测系统
 * @details 统计图像上、中、下三个区域各4行中黑白边界的数量
 * @note 用于判断赛道类型和辅助环岛识别，提高检测鲁棒性
 * @author 尘烟
 */
void Guide_lines(void)
{
    // 行号配置常量 - 按用户要求设置
    const uint8 TOP_ROWS[4] = {3, 9, 16, 25};      // 上区域：每5行间隔
    const uint8 MIDDLE_ROWS[4] = {29, 32, 35, 38}; // 中间区域：每5行间隔
    const uint8 BOTTOM_ROWS[4] = {69, 60, 56, 45}; // 下区域：倒序排列

    // 变量指针数组，便于循环处理
    uint8 *top_vars[4] = {&top_line1_num, &top_line2_num, &top_line3_num, &top_line4_num};
    uint8 *middle_vars[4] = {&middle_line1_num, &middle_line2_num, &middle_line3_num, &middle_line4_num};
    uint8 *bottom_vars[4] = {&bottom_line1_num, &bottom_line2_num, &bottom_line3_num, &bottom_line4_num};

    // 重置所有12个计数器
    for (int i = 0; i < 4; i++)
    {
        *top_vars[i] = 0;
        *middle_vars[i] = 0;
        *bottom_vars[i] = 0;
    }

    // 上区域四行检测 - 从右到左扫描
    for (int i = 0; i < 4; i++)
    {
        uint8 row = TOP_ROWS[i];
        for (int j = LCDW - 6; j >= 5; j--)
        {
            if (image[row][j] == BLACK && image[row][j - 1] == WHITE && image[row][j - 2] == WHITE)
            {
                (*top_vars[i])++;
            }
        }
    }

    // 中间区域四行检测 - 从左到右扫描
    for (int i = 0; i < 4; i++)
    {
        uint8 row = MIDDLE_ROWS[i];
        for (int j = 5; j <= LCDW - 6; j++)
        {
            if (image[row][j] == BLACK && image[row][j + 1] == WHITE && image[row][j + 2] == WHITE)
            {
                (*middle_vars[i])++;
            }
        }
    }

    // 下区域四行检测 - 从右到左扫描
    for (int i = 0; i < 4; i++)
    {
        uint8 row = BOTTOM_ROWS[i];
        for (int j = LCDW - 6; j >= 5; j--)
        {
            if (image[row][j] == BLACK && image[row][j - 1] == WHITE && image[row][j - 2] == WHITE)
            {
                (*bottom_vars[i])++;
            }
        }
    }

    // 设置Circle标志位 - 使用||逻辑判断每个区域的4行中是否有跳变
    Circle.consider_circle_top = (top_line1_num >= 2 || top_line2_num >= 2 || top_line3_num >= 2 || top_line4_num >= 2) ? 1 : 0;
    Circle.consider_circle = (middle_line1_num == 1 || middle_line2_num == 1 || middle_line3_num == 1 || middle_line4_num == 1) ? 1 : 0;
    Circle.consider_circle_down = (bottom_line1_num >= 2 || bottom_line2_num >= 2 || bottom_line3_num >= 2 || bottom_line4_num >= 2) ? 1 : 0;
}

// void Guide_lines(void)
// {
//     // 行号配置常量 - 按用户要求设置
//     const uint8 TOP_ROWS[4] = {3, 9, 16};      // 上区域：每5行间隔
//     const uint8 MIDDLE_ROWS[4] = { 32, 35, 38}; // 中间区域：每5行间隔
//     const uint8 BOTTOM_ROWS[4] = {69, 56, 45}; // 下区域：倒序排列

//     // 变量指针数组，便于循环处理
//     uint8 *top_vars[3] = {&top_line1_num, &top_line2_num, &top_line3_num};
//     uint8 *middle_vars[3] = {&middle_line1_num, &middle_line2_num, &middle_line3_num};
//     uint8 *bottom_vars[3] = {&bottom_line1_num, &bottom_line2_num, &bottom_line3_num};

//     // 重置所有12个计数器
//     for (int i = 0; i < 3; i++)
//     {
//         *top_vars[i] = 0;
//         *middle_vars[i] = 0;
//         *bottom_vars[i] = 0;
//     }

//     // 上区域四行检测 - 从右到左扫描
//     for (int i = 0; i < 3; i++)
//     {
//         uint8 row = TOP_ROWS[i];
//         for (int j = LCDW - 6; j >= 5; j--)
//         {
//             if (image[row][j] == BLACK && image[row][j - 1] == WHITE && image[row][j - 2] == WHITE)
//             {
//                 (*top_vars[i])++;
//             }
//         }
//     }

//     // 中间区域四行检测 - 从左到右扫描
//     for (int i = 0; i < 3; i++)
//     {
//         uint8 row = MIDDLE_ROWS[i];
//         for (int j = 5; j <= LCDW - 6; j++)
//         {
//             if (image[row][j] == BLACK && image[row][j + 1] == WHITE && image[row][j + 2] == WHITE)
//             {
//                 (*middle_vars[i])++;
//             }
//         }
//     }

//     // 下区域四行检测 - 从右到左扫描
//     for (int i = 0; i < 3; i++)
//     {
//         uint8 row = BOTTOM_ROWS[i];
//         for (int j = LCDW - 6; j >= 5; j--)
//         {
//             if (image[row][j] == BLACK && image[row][j - 1] == WHITE && image[row][j - 2] == WHITE)
//             {
//                 (*bottom_vars[i])++;
//             }
//         }
//     }

//     // 设置Circle标志位 - 使用||逻辑判断每个区域的4行中是否有跳变
//     Circle.consider_circle_top = (top_line1_num >= 2 || top_line2_num >= 2 || top_line3_num >= 2) ? 1 : 0;
//     Circle.consider_circle = (middle_line1_num == 1 || middle_line2_num == 1 || middle_line3_num == 1 ) ? 1 : 0;
//     Circle.consider_circle_down = (bottom_line1_num >= 2 || bottom_line2_num >= 2 || bottom_line3_num >= 2) ? 1 : 0;
// }

// 添加到现有函数之后
// 添加到全局变量区域

// 区域白点统计变量
uint8 top_left_white = 0;      // 顶部左侧区域白点数
uint8 top_middle_white = 0;    // 顶部中间区域白点数
uint8 top_right_white = 0;     // 顶部右侧区域白点数
uint8 bottom_left_white = 0;   // 底部左侧区域白点数
uint8 bottom_middle_white = 0; // 底部中间区域白点数
uint8 bottom_right_white = 0;  // 底部右侧区域白点数
/**
 * @brief 区域白点统计函数
 * @details 统计图像顶部和底部左右区域的白色像素数量
 * @note 用于检测赛道边界和异常情况，辅助安全保护机制
 * @author 尘烟
 */
void Count_White(void)
{
    const uint8 TOP_ROWS[4] = {1, 14, 17, 26};     // 顶部4行检测点
    const uint8 BOTTOM_ROWS[4] = {68, 63, 55, 45}; // 底部4行检测点

    top_left_white = 0;
    top_right_white = 0;
    bottom_left_white = 0;
    bottom_right_white = 0; // 重置计数器

    for (int i = 0; i < 4; i++)
    { // 顶部左区4行检测
        for (int j = 10; j < 55; j++)
        {
            if (image[TOP_ROWS[i]][j] == WHITE)
            {
                top_left_white++;

            } // 任意一行检测到即计入
        }
    }

    for (int i = 0; i < 4; i++)
    { // 顶部右区4行检测
        for (int j = LCDW - 10; j > LCDW - 55; j--)
        {
            if (image[TOP_ROWS[i]][j] == WHITE)
            {
                top_right_white++;
            }
        }
    }

    for (int i = 0; i < 4; i++)
    { // 底部左区4行检测
        for (int j = 10; j < 55; j++)
        {
            if (image[BOTTOM_ROWS[i]][j] == WHITE)
            {
                bottom_left_white++;
            }
        }
    }

    for (int i = 0; i < 4; i++)
    { // 底部右区4行检测
        for (int j = LCDW - 10; j > LCDW - 55; j--)
        {
            if (image[BOTTOM_ROWS[i]][j] == WHITE)
            {
                bottom_right_white++;
            }
        }
    }
    for (int i = 0; i < 4; i++)
    { // 底部右区4行检测
        for (int j = LCDW/2 + 10; j > LCDW/2 - 10; j--)
        {
            if (image[BOTTOM_ROWS[i]][j] == WHITE)
            {
                bottom_middle_white++;
            }
        }
    }
}
/**
 * @brief 安全保护函数
 * @details 检测图像四个区域白点数量，异常时停止系统运行
 * @note 当所有区域白点数量超过阈值时触发保护机制
 * @author 尘烟
 */
void baohu()
{
    if (top_right_white > 50 && top_left_white > 50 && bottom_right_white > 50 && bottom_left_white > 50)
    {
        system_running = 0;
    }
}

/**
 * @brief 图像处理主函数
 * @details 完整的图像处理流程，包含采集、二值化、边界检测、元素识别
 */
void Image_Process(void)
{
    if (mt9v03x_finish_flag)
    {
        timer_clear(TC_TIME2_CH0);
        timer_start(TC_TIME2_CH0);
        Get_userimage();
        outsthreshold = My_Adapt_Threshold(image_use[0], 120, 70);
        set_image_twovalues(outsthreshold);

        Guide_lines(); // 环岛初步判断
        Count_White();

        Searchline_Improved(); // 搜索车道边界

        circle_process(); // 环岛处理

        judge_angle(); // 元素识别

        baohu();
        break_road_check();

        timer_stop(TC_TIME2_CH0);
        mt9v03x_finish_flag = 0;
    }

    Err = Err_Sum();
}

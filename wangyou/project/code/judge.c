#include "judge.h"
#include "image.h"

struct JUDGE judge;
struct ANGLE height[LCDW];       // 记录从下往上搜索的线截止高度
struct ANGLE right_height[LCDW]; // 新增右侧高度记录

struct ANGLE left[LCDW];
struct ANGLE right[LCDW]; // 新增右侧对应数组

struct ANGLE BLACK_col[LCDW];
struct ANGLE WHITE_col[LCDW]; // 新增右侧白列记录

/**
 * @brief 赛道元素识别主函数
 * @details 通过元素行列识别直角、十字、直道等赛道元素
 * @note 1. 清除上一帧标志位
 *       2. 扫描四个区域的黑白跳变特征
 *       3. 使用位运算组合判断条件
 *       4. 根据条件组合识别元素类型并处理
 * @author 尘烟
 */
void judge_angle(void)
{
    // 先清除上一次的标志位
    clear_flags();

    // 扫描并检测区域特征
    judge_rowline();

    // 使用位运算组合判断条件
    uint8 condition = (judge.left_area_true << 3) |
                      (judge.top_area_true << 2) |
                      (judge.bottom_area_true << 1) |
                      (judge.right_area_true);

    // 根据不同的条件组合判断直角类型
    switch (condition)
    {
    case 0b1010:             // 左区域和下区域有跳变点，上无，右无
        judge.Left_Flag = 1; // 左直角
        Handle_Left_Angle(); // 处理左直角
        // if (!Circle.In_circle && judge.Left_Flag)
        // {
        //     Handle_Left_Angle();
        // }
        break;

    case 0b0011: // 右区域和下区域有跳变点，上无，左无

        judge.Right_Flag = 1; // 右直角
        Handle_Right_Angle(); // 处理右直角
        // if (!Circle.In_circle && judge.Right_Flag)
        // {
        //     Handle_Right_Angle(); // 处理右直角
        // }
        break;

    case 0b0110:                     // 上下区域有跳变点，左右无
        ImageFlag.straight_long = 1; // 直道

        break;
    case 0b1111:             // 上下左右都有跳变点 - 十字路口
        ImageFlag.Cross = 1; // 十字标志位
       
        break;
        
    case 0b0000:                 // 上下右区域有跳变点，左无
        ImageFlag.break_road = 1;//断路初步判断
        break;

        // default: // 处理所有其他情况
        //     judge.Left_Flag = 0;
        //     judge.Right_Flag = 0;
        //     ImageFlag.Cross = 0;
        //     ImageFlag.straight_long = 1;
        //     ImageFlag.Bend_Road = 0;
        //     ImageFlag.break_road = 0;
        // break;
    }
}

/**
 * @brief 直角专用连线函数
 * @details 使用Bresenham算法为直角处理生成中线点序列
 * @param x_start 起始点X坐标
 * @param y_start 起始点Y坐标
 * @param x_end 终点X坐标
 * @param y_end 终点Y坐标
 * @note - 直道状态下自动退出，避免破坏正确中线
 *       - 边界检查防止越界
 *       - 基于Bresenham算法保证连续性
 * @author 尘烟
 */
void Angle_Lianxian(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end)
{
    if (ImageFlag.straight_long == 1 || (!judge.Left_Flag && !judge.Right_Flag))
    {
        return; // 直接退出拉线循环，避免破坏已有的正确中线数据
    }

    x_start = (x_start < LCDW) ? x_start : LCDW - 1; // 边界检查
    y_start = (y_start < LCDH) ? y_start : LCDH - 1;
    x_end = (x_end < LCDW) ? x_end : LCDW - 1;
    y_end = (y_end < LCDH) ? y_end : LCDH - 1;

    int16 dx = abs(x_end - x_start);
    int16 dy = abs(y_end - y_start);
    int16 sx = (x_start < x_end) ? 1 : -1;
    int16 sy = (y_start < y_end) ? 1 : -1;
    int16 err = dx - dy;
    int16 current_err;

    Mid_line_count = 0; // 重置中线数组索引

    while (Mid_line_count < 130) // 防止数组溢出
    {
        Mid_line[Mid_line_count].col = x_start; // 将当前点加入数组
        Mid_line[Mid_line_count].row = y_start;
        Mid_line_count++;

        if (x_start == x_end && y_start == y_end) // 到达终点则退出循环
        {
            break;
        }

        current_err = 2 * err;

        if (current_err > -dy) // 更新误差和坐标
        {
            err -= dy;
            x_start += sx;
        }
        if (current_err < dx)
        {
            err += dx;
            y_start += sy;
        }
    }
}
#define angle_start 1
#define angle_end 0

/**
 * @brief 左直角处理函数
 * @details 检测左直角特征并生成对应的中线轨迹
 * @note 算法流程：
 *       1. 使用现有中线点作为起点
 *       2. 在左侧区域搜索黑白白跳变作为终点
 *       3. 调用连线函数生成轨迹
 *       4. 双边检测时取消左直角标志
 * @author 尘烟、郑轻工lmy
 */
void Handle_Left_Angle()
{
    if (ImageFlag.straight_long == 1 || (!judge.Left_Flag && !judge.Right_Flag)) // 直线状态下不执行拉线操作
    {
        return; // 直接退出拉线循环，避免破坏已有的正确中线数据
    }

    uint8 x_end = 0, y_end = 0;   // 重置终点坐标
    uint8 x_start = LCDW / 2 - 5; // 先设定默认值
    uint8 y_start = LCDH - 1;     // 最底行
    uint8 found = 0;              // 拐点找到标志
    uint8 vivid_row = 0;

    // 如果已经计算了中线点，直接使用最底部的中线点作为起点
    if (Mid_line_count > 0)
    {
        // 寻找最底行的中线点
        for (int i = 0; i < Mid_line_count; i++)
        {
            if (Mid_line[i].row == LCDH - angle_start) // 应该可以决定直角转的时机
            {
                x_start = Mid_line[i].col;
                y_start = Mid_line[i].row;
                break;
            }
        }

        // 如果没找到最底行，使用靠近底部的那个点
        if (x_start == LCDW / 2 - 1)
        {
            // 从下往上找最接近底部的中线点
            for (int row = LCDH - 2; row > LCDH - 20; row--)
            {
                for (int i = 3; i < Mid_line_count; i++)
                {
                    if (Mid_line[i].row == row)
                    {
                        x_start = Mid_line[i].col - 2;
                        // 调整y_start以匹配找到的中线点行
                        y_start = row;
                        break;
                    }
                }
                if (x_start != LCDW / 2 - 1) // 如果已找到，退出循环
                    break;
            }
        }
    }
    // 搜索第20行之后的合适跳变点作为终点
    for (int i = 0; i < LCDH - 1; i++) // 从第20行开始向下搜索到中间行
    {
        for (int j = 15; j < 25; j++) // 只在左半边搜索
        {
            // 查找黑白跳变点作为终点坐标
            if (image[i][j] == BLACK && image[i + 1][j] == WHITE && image[i + 2][j] == WHITE)
            {
                x_end = angle_end; // 将终点设在白色区域开始处
                y_end = i - 1;     // 记录当前行
                found = 1;
                break;
            }
        }
        if (found)
            break; // 找到后退出循环
    }

    // 如果没找到有效终点，使用默认值

    // 连接起点和终点
    Angle_Lianxian(x_start, y_start, x_end, y_end);
    // for (int i = 0; i < 15; i++)
    // {
    //     for (int j = LCDW - 10; j > 10; j--)
    //     {
    //         if (image[i][j] == BLACK && image[i][j - 1] == WHITE && image[i][j - 2] == WHITE)
    //         {
    //             vivid_row++;
    //             break;
    //         }
    //     }

    //     if (vivid_row > 10)
    //     {
    //         judge.Left_Flag = 0;
    //         break;
    //     }
    // }
    if ((judge.right_area_true == 1 && judge.left_area_true == 1) || (judge.right_area_true == 0 && judge.left_area_true == 0))
    {
        judge.Left_Flag = 0;
    }
}

/**
 * @brief 右直角处理函数
 * @details 检测右直角特征并生成对应的中线轨迹
 * @note 算法流程：
 *       1. 使用现有中线点作为起点
 *       2. 在右侧区域搜索黑白白跳变作为终点
 *       3. 调用连线函数生成轨迹
 *       4. 双边检测时取消右直角标志
 * @author 尘烟、郑轻工lmy
 */
void Handle_Right_Angle()
{
    if (ImageFlag.straight_long == 1 || (!judge.Left_Flag && !judge.Right_Flag)) // 直线状态下不执行拉线操作
    {
        return; // 直接退出拉线循环，避免破坏已有的正确中线数据
    }

    uint8 x_end = LCDW - 1, y_end = 0; // 重置终点坐标
    uint8 x_start = LCDW / 2 + 2;      // 先设定默认值
    uint8 y_start = LCDH - 1;          // 最底行
    uint8 found = 0;                   // 拐点找到标志
    uint8 vivid_row = 0;

    // 如果已经计算了中线点，直接使用最底部的中线点作为起点
    if (Mid_line_count > 0)
    {
        // 寻找最底行的中线点
        for (int i = 0; i < Mid_line_count; i++)
        {
            if (Mid_line[i].row == LCDH - angle_start) // 应该可以决定直角转的时机
            {
                x_start = Mid_line[i].col;
                y_start = Mid_line[i].row; // 记录行坐标
                break;
            }
        }

        // 如果没找到最底行，使用靠近底部的那个点
        if (x_start == LCDW / 2)
        {
            // 从下往上找最接近底部的中线点
            for (int row = LCDH - 2; row > LCDH - 20; row--)
            {
                for (int i = 3; i < Mid_line_count; i++)
                {
                    if (Mid_line[i].row == row)
                    {
                        x_start = Mid_line[i].col + 2;
                        // 调整y_start以匹配找到的中线点行
                        y_start = row;
                        break;
                    }
                }
                if (x_start != LCDW / 2) // 如果已找到，退出循环
                    break;
            }
        }
    }
    for (int i = 0; i < LCDH - 1; i++) // 从第20行开始向下搜索到中间行
    {
        for (int j = LCDW - 16; j >= LCDW - 26; j--) // 只在右半边搜索
        {
            // 查找白黑跳变点作为终点坐标
            if (image[i][j] == BLACK && image[i + 1][j] == WHITE && image[i + 2][j] == WHITE)
            {
                x_end = LCDW - angle_end; // 将终点设在白色区域处
                y_end = i - 1;            // 记录当前行
                found = 1;
                break;
            }
        }
        if (found)
            break; // 找到后退出循环
    }

    // 如果没找到有效终点，使用默认值

    // 连接起点和终点
    Angle_Lianxian(x_start, y_start, x_end, y_end);
    // for (int i = 0;i<15;i++)
    // {
    //     for (int j = 10; j <LCDW- 10; j++)
    //     {
    //        if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
    //        {
    //            vivid_row++;
    //            break;
    //        }
    //     }
    //     if (vivid_row > 10)
    //     {
    //         judge.Right_Flag = 0;
    //         break;
    //     }
    // }
    if ((judge.right_area_true == 1 && judge.left_area_true == 1) || (judge.right_area_true == 0 && judge.left_area_true == 0))
    {
        judge.Right_Flag = 0;
    }
}

/**
 * @brief 十字路口处理函数
 * @details 检测十字路口特征并生成穿越轨迹
 * @note 1. 使用现有中线底部点作为起点
 *       2. 在顶部区域双向搜索左右边界
 *       3. 计算中点作为终点
 *       4. 支持斜入处理和单边检测
 * 最后比赛时关掉了，在连续直角时会导致误判，导致直角拉线收到影响
 * @author 尘烟
 */
void Handle_Cross()
{
    if (ImageFlag.straight_long == 1)
    {
        return; // 直接退出拉线循环，避免破坏已有的正确中线数据
    }

    uint8 x_start = LCDW / 2, y_start = LCDH - 1; // 默认起点
    uint8 x_end = LCDW / 2, y_end = 5;            // 默认终点
    uint8 left_edge = 0, right_edge = 0;          // 顶部边界
    uint8 found_left = 0, found_right = 0;        // 检测标志

    // 底部起点检测：优先使用现有中线的最底部点
    if (Mid_line_count > 0)
    {
        // 寻找最底部5行中的中线点作为起点
        for (int row = LCDH - 1; row >= LCDH - 5; row--)
        {
            for (int i = 0; i < Mid_line_count; i++)
            {
                if (Mid_line[i].row == row)
                {
                    x_start = Mid_line[i].col;
                    y_start = Mid_line[i].row;
                    break;
                }
            }
            if (x_start != LCDW / 2) // 如果找到了，退出循环
                break;
        }
    }

    // 顶部边界检测：在最上面5行进行双向扫描
    for (int row = 5; row < 10; row++)
    {
        // 从左到右搜索黑白白跳变
        if (!found_left)
        {
            for (int col = 5; col < LCDW - 5; col++)
            {
                if (image[row][col] == BLACK &&
                    image[row][col + 1] == WHITE &&
                    image[row][col + 2] == WHITE)
                {
                    left_edge = col + 1; // 存储白色的第一个点
                    found_left = 1;
                    break;
                }
            }
        }

        // 从右到左搜索黑白白跳变
        if (!found_right)
        {
            for (int col = LCDW - 5; col > 5; col--)
            {
                if (image[row][col] == BLACK &&
                    image[row][col - 1] == WHITE &&
                    image[row][col - 2] == WHITE)
                {
                    right_edge = col - 1; // 存储白色的第一个点
                    found_right = 1;
                    break;
                }
            }
        }

        // 如果两边都找到了，可以提前退出
        if (found_left && found_right)
            break;
    }

    // 计算中点坐标
    if (found_left && found_right)
    {
        x_end = (left_edge + right_edge) >> 1; // 使用位运算计算中点

        // 斜入处理：当赛道宽度>30时，使用右边界补充策略
        uint8 road_width = right_edge - left_edge;
        if (road_width > 30)
        {
            // 斜入情况，优先使用右边界来补充中线计算
            x_end = right_edge - (road_width >> 2); // 偏向右边界
        }
    }
    else if (found_left)
    {
        x_end = left_edge + 10; // 只找到左边界，向右偏移
    }
    else if (found_right)
    {
        x_end = right_edge - 10; // 只找到右边界，向左偏移
    }
    // 如果都没找到，使用默认中心位置

    // 调用连线函数
    Lianxian(x_start, y_start, x_end, y_end);
}

/**
 * @brief 标志位清除函数
 */
void clear_flags(void)
{
    judge.Left_Flag = 0;
    judge.Right_Flag = 0;
    ImageFlag.Cross = 0;
    ImageFlag.straight_long = 0;
    ImageFlag.Bend_Road = 0;
    ImageFlag.break_road = 0;
}

/**
 * @brief 元素行列法
 * @details 扫描图像四个区域的黑白跳变特征，为元素识别提供数据
 * @note 检测区域：
 *       - 上方区域：第2-10行，检测黑白白和白白黑跳变
 *       - 下方区域：最后11行，检测黑白白和白白黑跳变
 *       - 左方区域：第20-40列，检测垂直方向跳变
 *       - 右方区域：第LCDW-40到LCDW-20列，检测垂直方向跳变
 * @author 尘烟(算法思路来自于青山龙邱直播)
 */
void judge_rowline(void)
{
    uint8 col = 0; // 暂存图像列
    uint8 row = 0; // 暂存图线行

    judge.up_0_1_num = 0; // 重置所有计数器
    judge.up_1_0_num = 0;
    judge.down_0_1_num = 0;
    judge.down_1_0_num = 0;
    judge.left_0_1_num = 0;
    judge.left_1_0_num = 0;
    judge.right_0_1_num = 0;
    judge.right_1_0_num = 0;
    judge.left_area_true = 0;
    judge.right_area_true = 0;
    judge.top_area_true = 0;
    judge.bottom_area_true = 0;
    judge.Left_Flag = 0;
    judge.Right_Flag = 0;

    /******************元素行上方****************/
    // 上面区域跳变点统计 - 只扫描10行，步长1，适应虚线路径
    for (int i = 2; i <= 10; i++)
    {
        col = 0; // 重置列值
        // 从左向右扫描
        for (int j = 20; j <= LCDW - 20; j++)
        {
            // 修改为黑白白模式
            if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
            {
                col = j;            // 找到某一行的左边界并记录
                judge.up_0_1_num++; // 黑白白跳变
                break;
            }
        }

        // 只有找到左边界才继续寻找右边界
        if (col > 0)
        {
            // 从左边界后继续向右扫描，找白白黑跳变
            for (int j = col + 3; j < LCDW - 20; j++)
            {
                if (image[i][j] == WHITE && image[i][j + 1] == WHITE && image[i][j + 2] == BLACK)
                {
                    judge.up_1_0_num++; // 白白黑跳变
                    break;
                }
            }
        }
    }

    // 上方区域判断逻辑 - 适应虚线，降低阈值要求
    uint8 top_threshold = 3; // 针对上区域的阈值
    judge.top_area_true = (judge.up_0_1_num >= top_threshold && judge.up_1_0_num >= top_threshold) ? 1 : 0;

    /******************元素行下方****************/
    // 下面区域跳变点统计 - 只扫描10行，步长1，适应虚线路径
    for (int i = LCDH - 1; i >= LCDH - 11; i--)
    {
        col = 0; // 重置列值
        // 从左向右扫描
        for (int j = 20; j < LCDW - 20; j++)
        {
            // 修改为黑白白模式
            if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
            {
                col = j;              // 找到某一行的左边界并记录
                judge.down_0_1_num++; // 黑白白跳变
                break;
            }
        }

        // 只有找到左边界才继续寻找右边界
        if (col > 0)
        {
            // 从左边界后继续向右扫描，找白白黑跳变
            for (int j = col + 3; j < LCDW - 6; j++)
            {
                if (image[i][j] == WHITE && image[i][j + 1] == WHITE && image[i][j + 2] == BLACK)
                {
                    judge.down_1_0_num++; // 白白黑跳变
                    break;
                }
            }
        }
    }

    // 下方区域判断逻辑 - 适应虚线
    uint8 bottom_threshold = 2; // 针对下区域的阈值
    judge.bottom_area_true = (judge.down_0_1_num >= bottom_threshold && judge.down_1_0_num >= bottom_threshold) ? 1 : 0;

    /******************元素列左方****************/
    // 左边区域跳变点统计 - 只扫描10列，步长1，适应虚线
    for (int j = 20; j <= 40; j++)
    {
        row = 0; // 重置行值
        // 从下向上扫描
        for (int i = LCDH - 5; i > 4; i--)
        {
            // 修改为黑白白模式（垂直方向）
            if (image[i][j] == BLACK && image[i - 1][j] == WHITE && image[i - 2][j] == WHITE)
            {
                row = i;
                judge.left_0_1_num++;
                break;
            }
        }

        // 只有找到下边界才继续寻找上边界
        if (row > 0)
        {
            // 从下边界后继续向上扫描，找白白黑跳变
            for (int i = row - 1; i > 4; i--)
            {
                if (image[i][j] == WHITE && image[i - 1][j] == WHITE && image[i - 2][j] == BLACK)
                {
                    judge.left_1_0_num++;
                    break;
                }
            }
        }
    }

    // 左方区域判断逻辑 - 适应虚线
    uint8 left_threshold = 5; // 针对左区域的阈值
    judge.left_area_true = (judge.left_0_1_num >= left_threshold && judge.left_1_0_num >= left_threshold) ? 1 : 0;

    /******************元素列右方****************/
    // 右边区域跳变点统计 - 只扫描10列，步长1，适应虚线
    for (int j = LCDW - 20; j >= LCDW - 40; j--)
    {
        row = 0; // 重置行值
        // 从下向上扫描
        for (int i = LCDH - 5; i > 4; i--)
        {
            // 修改为黑白白模式（垂直方向）
            if (image[i][j] == BLACK && image[i - 1][j] == WHITE && image[i - 2][j] == WHITE)
            {
                row = i;
                judge.right_0_1_num++;
                break;
            }
        }

        // 只有找到下边界才继续寻找上边界
        if (row > 0)
        {
            // 从下边界后继续向上扫描，找白白黑跳变
            for (int i = row - 1; i > 4; i--)
            {
                if (image[i][j] == WHITE && image[i - 1][j] == WHITE && image[i - 2][j] == BLACK)
                {
                    judge.right_1_0_num++;
                    break;
                }
            }
        }
    }

    // 右方区域判断逻辑 - 适应虚线
    uint8 right_threshold = 5; // 针对右区域的阈值
    judge.right_area_true = (judge.right_0_1_num >= right_threshold && judge.right_1_0_num >= right_threshold) ? 1 : 0;
}

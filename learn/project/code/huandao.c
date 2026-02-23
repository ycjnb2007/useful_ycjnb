#include "huandao.h"

struct L_HUAN l_huan;
struct R_HUAN r_huan;
struct CIRCLE Circle;

static circle_state_t circle_state = CIRCLE_IDLE;

void circle_init(void)
{
    // 初始化状态机变量
    Circle.left_circle_state = CIRCLE_IDLE;
    Circle.right_circle_state = CIRCLE_IDLE;

    // 初始化现有标志位（向后兼容）
    Circle.In_circle = 0;
    Circle.tracking = 0;
    Circle.now_yaw = 0.0f;
    Circle.Left_flag = 0;
    Circle.Right_flag = 0;
    Circle.out_flag = 0;
    circle_state = CIRCLE_IDLE;
    judge.Left_Flag = 0;
    judge.Right_Flag = 0;
}

/**
 * @brief 环岛进入退出检测函数
 * @details 通过检测顶部区域的黑白跳变点判断是否已进入环岛
 * @note - 扫描顶部5-25行区域
 *       - 统计第一条和第二条白色赛道线数量
 *       - 一条线多(>=3)且二条线少(<=2)时判定进入环岛
 *       - 300度出环岛
 * @author 尘烟
 */
void check_enter_circle(void)
{
    uint8 White_yindao_Lines1 = 0; // 第一条赛道线计数
    uint8 White_yindao_Lines2 = 0; // 第二条赛道线计数
    uint8 col = 0;                 // 用于记录第一条白色赛道结束的列位置

    // 扫描顶部区域的5-15行
    for (uint8 i = 5; i < 25; i++)
    {
        // 寻找第一条黑白跳变
        for (uint8 j = 0; j < LCDW - 2; j++)
        {
            if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
            {
                col = j;
                White_yindao_Lines1++;
                break;
            }
        }

        // 从第一条跳变点之后寻找第二条黑白跳变
        for (uint8 j = col + 3; j < LCDW - 2; j++)
        {
            if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
            {
                White_yindao_Lines2++;
                break;
            }
        }
    }

    // 判断是否进入环岛
    // 如果第一条线足够多(>=5)而第二条线几乎没有(<=1)，表示已经进入环岛

    // 只有在环岛状态且未出环时才能进入环岛
    if (White_yindao_Lines1 >= 3 && White_yindao_Lines2 <= 2 && !Circle.out_flag &&
        (Circle.Left_flag || Circle.Right_flag) && !Circle.In_circle)
    {
        Circle.In_circle = 1;  // 已经进入环岛
        Circle.Left_flag = 0;  // 清除左环岛标志位，停止拉线
        Circle.Right_flag = 0; // 清除右环岛标志位，停止拉线
        Circle.tracking = 1;
    }

    else if (Circle.In_circle) // 在环岛寻迹过程中，一直记录度数，当检测到陀螺仪记度数到达300度的时候开始准备出环岛
    {
        // 根据状态机状态判断左右环岛，而不是依赖已清除的标志位
        if (((Circle.left_circle_state >= CIRCLE_ENTERING) && Circle.now_yaw > 300) ||
            ((Circle.right_circle_state >= CIRCLE_ENTERING) && Circle.now_yaw < -300))
        {
            if (!Circle.out_flag) // 只在未设置时设置，避免重复设置
            {
                Circle.out_flag = 1;
                Circle.In_circle = 0;
                // Circle.tracking = 1;
                BEE_STA(0);
            }
        }
    }

    else if (Circle.out_flag) // 开始出环岛
    {
        // int16 l_high_flag = 0;
        // int y = 0, x = 0;
        if (Circle.now_yaw > 0)
        { // {
          //     for (uint8 j = 5; j < 30; j++)
          //     {
          //         for (uint8 i = 0; i < 25; i++)
          //         {

            //             if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
            //             {
            //                 l_high_flag = 5;
            //                 y = i;
            //                 break;
            //             }
            //         }
            //     }
            //     Lianxian(LCDW / 2, LCDH - 1, l_high_flag, y); // 连接右环切入点
            for (uint8 i = 0; i < LCDH; i++)
            {
                Mid_line[i].col = Right_line[i].col;
                Mid_line[i].row = Right_line[i].row;
            }
        }

        else if (Circle.now_yaw < 0)
        {
            // int16 l_high_flag = 0;
            // int y = 0, x = 0;
            // int16 row = 25; // 行

            // // 左环搜索算法
            // while (row > 5)
            // {
            //     // 左边区域搜索
            //     for (int col = 20; col <= 50; col++)
            //     {
            //         if (image[row][col] == BLACK &&
            //             image[row][col + 1] == WHITE &&
            //             image[row][col + 2] == WHITE)
            //         {
            //             l_high_flag = col + 5; // 左边界最高点
            //             y = row;
            //             break;
            //         }
            //     }

            //     row--;
            // }
            // Lianxian(LCDW / 2, LCDH - 1, l_high_flag, y); // 连接左环切入点
            for (uint8 i = 0; i < LCDH; i++)
            {
                Mid_line[i].col = Left_line[i].col;
                Mid_line[i].row = Left_line[i].row;
            }
        }

        // 检查是否完成出环岛 - 根据状态机状态判断左右环岛
        uint8 exit_complete = 0;
        if ((Circle.left_circle_state == CIRCLE_EXITING) && ImageFlag.straight_long == 1 && Circle.now_yaw > 350)
        {
            exit_complete = 1;
        }
        else if ((Circle.right_circle_state == CIRCLE_EXITING) && ImageFlag.straight_long == 1 && Circle.now_yaw < -350)
        {
            exit_complete = 1;
        }

        if (exit_complete)
        {
            Circle.out_flag = 0;
            Circle.In_circle = 0;
            Circle.Left_flag = 0;
            Circle.Right_flag = 0;
            Circle.tracking = 0;
            // Circle.now_yaw = 0;
        }
    }
}

/**
 * @brief 左环岛处理函数
 * @details 检测左环岛边界并进行连线处理
 * @note - 从第25行向上搜索到第5行
 *       - 在左侧区域(20-40列)寻找黑白白跳变
 *       - 找到最高点后连接到图像底部中心
 * @author 尘烟
 */
void left_circle_process(void)
{
    int16 l_high_flag = 0;
    int y = 0, x = 0;
    int16 row = 25; // 行

    while (row > 5) // 左环搜索算法
    {
        for (int col = 20; col <= 40; col++) // 左边区域搜索
        {
            if (image[row][col] == BLACK &&
                image[row][col + 1] == WHITE &&
                image[row][col + 2] == WHITE)
            {
                l_high_flag = col+5 ; // 左边界最高点
                y = row;
                break;
            }
        }
        row--;
    }
    Lianxian(LCDW / 2, LCDH - 1, l_high_flag, y); // 连接左环切入点
}
/**
 * @brief 右环岛处理函数
 * @details 检测右环岛边界并进行连线处理
 * @note - 从第25行向上搜索到第5行
 *       - 在右侧区域(LCDW-40到LCDW-20列)寻找白黑黑跳变
 *       - 找到最高点后连接到图像底部中心
 * @author 尘烟
 */
void right_circle_process(void)
{
    int16 r_high_flag = LCDW - 1; // 初始化右边界最高点为屏幕宽度
    int y = 0;
    int16 row = 25; // 行

    while (row > 5) // 右环搜索算法
    {
        int x = LCDW; // 初始值设为屏幕宽度

        for (int col = LCDW - 20; col >= LCDW - 40; col--) // 右边区域搜索，从右往左
        {
            if (image[row][col] == BLACK &&
                image[row][col - 1] == WHITE &&
                image[row][col - 2] == WHITE)
            {
                r_high_flag = col-5; // 右边界最高点
                y = row;
                break;
            }
        }
        row--;
    }
    Lianxian(LCDW / 2, LCDH - 1, r_high_flag, y); // 连接右环切入点
}

/**
 * @brief 环岛识别检测函数
 * @details 通过检测图像四个区域的黑白跳变特征识别环岛
 * @note - 上方区域：第3行和第9行
 *       - 下方区域：第60行和第66行
 *       - 左侧区域：第5-35列
 *       - 右侧区域：第85-115列
 *       支持虚线赛道的跳变检测
 * @author 尘烟
 */
void circle_detection(void)
{
    uint8 x = 0; // 暂存图像列
    uint8 y = 0; // 暂存图线行

    // 重置所有计数器
    Circle.up_0_1_num1 = 0;
    Circle.up_1_0_num1 = 0;
    Circle.up_0_1_num2 = 0;
    Circle.up_1_0_num2 = 0;
    Circle.down_0_1_num1 = 0;
    Circle.down_1_0_num1 = 0;
    Circle.down_0_1_num2 = 0;
    Circle.down_1_0_num2 = 0;
    Circle.left_0_1_num1 = 0;
    Circle.left_1_0_num1 = 0;
    Circle.left_0_1_num2 = 0;
    Circle.left_1_0_num2 = 0;
    Circle.right_0_1_num1 = 0;
    Circle.right_1_0_num1 = 0;
    Circle.right_0_1_num2 = 0;
    Circle.right_1_0_num2 = 0;

    // /******************元素行上方****************/
    // 上面区域跳变点统计 - 只扫描16行，步长1，适应虚线路径
    for (int i = 1; i <= 20; i++)
    {
        x = 0;        // 重置列值
        int16 x1 = 0; // 用于记录第二个跳变点
        int16 x2 = 0;

        // 从左向右扫描
        for (int j = 1; j < LCDW; j++)
        {
            // 检测黑白跳变，考虑虚线情况
            if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
            {
                x1 = j;               // 找到某一行的左边界并记录
                Circle.up_0_1_num1++; // 黑白跳变
                break;
            }
        }

        // 只有找到左边界才继续寻找右边界
        if (x1 > 0)
        {
            // 从左边界后继续向右扫描，找白黑跳变
            for (int j = x1 + 2; j < LCDW; j++)
            {
                if (image[i][j] == WHITE && image[i][j + 1] == WHITE && image[i][j + 2] == BLACK)
                {
                    x1 = j;               // 找到某一行的右边界并记录
                    Circle.up_1_0_num1++; // 白黑跳变
                    break;
                }
            }
        }

        if (Circle.up_0_1_num1 != 0 && Circle.up_1_0_num1 != 0)
        {
            for (int j = x1 + 1; j < LCDW; j++)
            {
                if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
                {
                    x2 = j;               // 找到某一行的第二个左边界并记录
                    Circle.up_0_1_num2++; // 第二个黑白跳变
                    break;
                }
            }
        }

        if (x2 > 0)
        {
            for (int j = x2 + 2; j < LCDW; j++)
            {
                if (image[i][j] == WHITE && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
                {
                    Circle.up_1_0_num2++; // 第二个白黑跳变
                    break;
                }
            }
        }
    }

    // 上方区域判断逻辑 - 适应虚线
    uint8 top_threshold = 2; // 针对上区域的阈值
    Circle.top_area_true = (Circle.up_0_1_num1 >= top_threshold && Circle.up_1_0_num1 >= top_threshold && Circle.up_0_1_num2 >= top_threshold && Circle.up_1_0_num2 >= top_threshold) ? 1 : 0;

    /******************元素行下方****************/
    // 下面区域跳变点统计 - 只扫描10行，步长1，适应虚线路径
    for (int i = LCDH - 1; i >= LCDH - 20; i--)
    {
        x = 0;        // 重置列值
        int16 x1 = 0; // 用于记录第二个跳变点
        int16 x2 = 0;
        // 从左向右扫描
        for (int j = 1; j < LCDW; j++)
        {
            // 检测黑白跳变，考虑虚线情况
            if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
            {
                x1 = j;                 // 找到某一行的左边界并记录
                Circle.down_0_1_num1++; // 黑白跳变
                break;
            }
        }

        // 只有找到左边界才继续寻找右边界
        if (x1 > 0)
        {
            // 从左边界后继续向右扫描，找白黑跳变
            for (int j = x1 + 2; j < LCDW; j++)
            {
                if (image[i][j] == WHITE && image[i][j + 1] == WHITE && image[i][j + 2] == BLACK)
                {
                    x1 = j;                 // 找到某一行的左边界并记录
                    Circle.down_1_0_num1++; // 白黑跳变
                    break;
                }
            }
        }
        if (Circle.down_0_1_num1 != 0 && Circle.down_1_0_num1 != 0)
        {
            for (int j = x1 + 1; j < LCDW; j++)
            {
                if (image[i][j] == BLACK && image[i][j + 1] == WHITE && image[i][j + 2] == WHITE)
                {
                    x2 = j;                 // 找到某一行的左边界并记录
                    Circle.down_0_1_num2++; // 白黑跳变
                    break;
                }
            }
        }
        if (x2 > 0)
        {
            for (int j = x2 + 2; j < LCDW; j++)
            {
                if (image[i][j] == WHITE && image[i][j + 1] == WHITE && image[i][j + 2] == BLACK)
                {
                    Circle.down_1_0_num2++; // 白黑跳变
                    break;
                }
            }
        }
    }

    // 下方区域判断逻辑 - 适应虚线
    uint8 bottom_threshold = 2; // 针对下区域的阈值
    Circle.down_area_true = (Circle.down_0_1_num1 >= bottom_threshold && Circle.down_1_0_num1 >= bottom_threshold && Circle.down_0_1_num2 >= bottom_threshold && Circle.down_1_0_num2 >= bottom_threshold) ? 1 : 0;

    /******************元素列左侧****************/
    // 左侧区域跳变点统计 - 扫描30-45列，步长1，适应虚线路径
    for (int j = 20; j <= 45; j++)
    {
        y = 0;        // 重置行值
        int16 y1 = 0; // 用于记录第二个跳变点
        int16 y2 = 0;

        // 从上向下扫描
        for (int i = 1; i < LCDH; i++)
        {
            // 检测黑白跳变，考虑虚线情况
            if (image[i][j] == BLACK && image[i + 1][j] == WHITE && image[i + 2][j] == WHITE)
            {
                y1 = i;                 // 找到某一列的上边界并记录
                Circle.left_0_1_num1++; // 黑白跳变
                break;
            }
        }

        // 只有找到上边界才继续寻找下边界
        if (y1 > 0)
        {
            // 从上边界后继续向下扫描，找白黑跳变
            for (int i = y1 + 1; i < LCDH; i++)
            {
                if (image[i][j] == WHITE && image[i + 1][j] == WHITE && image[i + 2][j] == BLACK)
                {
                    y1 = i;                 // 找到某一列的下边界并记录
                    Circle.left_1_0_num1++; // 白黑跳变
                    break;
                }
            }
        }

        // if (Circle.left_0_1_num1 != 0 && Circle.left_1_0_num1 != 0)
        // {
        //     for (int i = y1 + 1; i < LCDH; i++)
        //     {
        //         if (image[i][j] == BLACK && image[i + 1][j] == WHITE && image[i + 2][j] == WHITE)
        //         {
        //             y2 = i;                 // 找到某一列的第二个上边界并记录
        //             Circle.left_0_1_num2++; // 第二个黑白跳变
        //             break;
        //         }
        //     }
        // }

        // if (y2 > 0)
        // {
        //     for (int i = y2 + 1; i < LCDH; i++)
        //     {
        //         if (image[i][j] == WHITE && image[i + 1][j] == WHITE && image[i + 2][j] == BLACK)
        //         {
        //             Circle.left_1_0_num2++; // 第二个白黑跳变
        //             break;
        //         }
        //     }
        // }
    }

    // 左侧区域判断逻辑 - 适应虚线
    uint8 left_threshold = 3; // 针对左区域的阈值
    Circle.left_area_true = (Circle.left_0_1_num1 >= left_threshold && Circle.left_1_0_num1 >= left_threshold) ? 1 : 0;

    /******************元素列右侧****************/
    // 右侧区域跳变点统计 - 扫描75-90列（对称），步长1，适应虚线路径
    for (int j = LCDW-45; j <= LCDW-20; j++)

    {
        y = 0;        // 重置行值
        int16 y1 = 0; // 用于记录第二个跳变点
        int16 y2 = 0;

        // 从上向下扫描
        for (int i = 1; i < LCDH; i++)
        {
            // 检测黑白跳变，考虑虚线情况
            if (image[i][j] == BLACK && image[i + 1][j] == WHITE && image[i + 2][j] == WHITE)
            {
                y1 = i;                  // 找到某一列的上边界并记录
                Circle.right_0_1_num1++; // 黑白跳变
                break;
            }
        }

        // 只有找到上边界才继续寻找下边界
        if (y1 > 0)
        {
            // 从上边界后继续向下扫描，找白黑跳变
            for (int i = y1 + 1; i < LCDH; i++)
            {
                if (image[i][j] == WHITE && image[i + 1][j] == WHITE && image[i + 2][j] == BLACK)
                {
                    y1 = i;                  // 找到某一列的下边界并记录
                    Circle.right_1_0_num1++; // 白黑跳变
                    break;
                }
            }
        }

        // if (Circle.right_0_1_num1 != 0 && Circle.right_1_0_num1 != 0)
        // {
        //     for (int i = y1 + 1; i < LCDH; i++)
        //     {
        //         if (image[i][j] == BLACK && image[i + 1][j] == WHITE && image[i + 2][j] == WHITE)
        //         {
        //             y2 = i;                  // 找到某一列的第二个上边界并记录
        //             Circle.right_0_1_num2++; // 第二个黑白跳变
        //             break;
        //         }
        //     }
        // }

        // if (y2 > 0)
        // {
        //     for (int i = y2 + 1; i < LCDH; i++)
        //     {
        //         if (image[i][j] == WHITE && image[i + 1][j] == WHITE && image[i + 2][j] == BLACK)
        //         {
        //             Circle.right_1_0_num2++; // 第二个白黑跳变
        //             break;
        //         }
        //     }
        // }
    }

    // 右侧区域判断逻辑 - 适应虚线
    uint8 right_threshold = 3; // 针对右区域的阈值
    Circle.right_area_true = (Circle.right_0_1_num1 >= right_threshold && Circle.right_1_0_num1 >= right_threshold) ? 1 : 0;
}



/**
 * @brief 环岛状态机处理函数
 * @details 左右环岛的状态转换和处理逻辑
 * @note 状态流程：
 *       IDLE -> DETECTED -> ENTERING -> IN_CIRCLE -> EXITING -> COMPLETE -> IDLE
 * @author 尘烟
 */
void circle_state_machine(void)
{//Circle.top_area_true && Circle.down_area_true &&
    if (Circle.consider_circle_top == 1 && Circle.consider_circle == 1 && Circle.consider_circle_down == 1 &&!Circle.out_flag)
    {
        // 检测左环岛特征
        if ( Circle.left_area_true==1 && !Circle.Right_flag && judge.right_area_true == 0 && ImageFlag.Cross == 0 && top_left_white >= 15 && bottom_left_white >= 10 && Circle.left_circle_state == CIRCLE_IDLE && Circle.right_circle_state == CIRCLE_IDLE)
        {
            Circle.left_circle_state = CIRCLE_DETECTED;
            Circle.Left_flag = 1;
            BEE_STA(1);
        }
        // // 检测右环岛特征
        else if ( Circle.right_area_true ==1&& !Circle.Left_flag && judge.left_area_true == 0 && ImageFlag.Cross == 0 && top_right_white >= 15 && bottom_right_white >= 10 && Circle.left_circle_state == CIRCLE_IDLE && Circle.right_circle_state == CIRCLE_IDLE)
        {
            Circle.right_circle_state = CIRCLE_DETECTED;
            Circle.Right_flag = 1;
            BEE_STA(1);
        }
    }

    // 左环岛状态机
    switch (Circle.left_circle_state)
    {
    case CIRCLE_IDLE:
        // 空闲状态，等待检测
        break;

    case CIRCLE_DETECTED:
        Circle.left_circle_state = CIRCLE_ENTERING;
        huandao_yaw(); // 开始角度跟踪
        break;

    case CIRCLE_ENTERING:

        left_circle_process(); // 执行左环岛处理

        // left_circle_process(); // 执行左环岛处理
        // 检查是否进入环岛

        Circle.left_circle_state = CIRCLE_IN_CIRCLE;

        break;

    case CIRCLE_IN_CIRCLE:
        if (!Circle.In_circle)
        {
            left_circle_process(); // 继续左环岛处理
        }
        // 检查是否开始出环岛
        if (Circle.out_flag)
        {
            Circle.left_circle_state = CIRCLE_EXITING;
        }
        break;

    case CIRCLE_EXITING:
        // left_circle_process(); // 完成出环岛处理

        if (!Circle.out_flag && !Circle.In_circle)
        {
            Circle.left_circle_state = CIRCLE_COMPLETE;
        }
        break;

    case CIRCLE_COMPLETE:
        Circle.tracking = 0; // 清除tracking标志位，恢复正常元素识别
        Circle.left_circle_state = CIRCLE_IDLE;
        break;
    }

    // 右环岛状态机
    switch (Circle.right_circle_state)
    {
    case CIRCLE_IDLE:
        // 空闲状态，等待检测
        break;

    case CIRCLE_DETECTED:
        Circle.right_circle_state = CIRCLE_ENTERING;
        huandao_yaw(); // 开始角度跟踪
        break;

    case CIRCLE_ENTERING:

        right_circle_process(); // 执行右环岛处理

        // right_circle_process(); // 执行右环岛处理
        // 检查是否进入环岛

        Circle.right_circle_state = CIRCLE_IN_CIRCLE;

        break;

    case CIRCLE_IN_CIRCLE:
        if (!Circle.In_circle)
        {
            right_circle_process(); // 继续右环岛处理
        }
        // 检查是否开始出环岛
        if (Circle.out_flag)
        {
            Circle.right_circle_state = CIRCLE_EXITING;
        }
        break;

    case CIRCLE_EXITING:
        // right_circle_process(); // 完成出环岛处理
        // 检查是否完成环岛 - 当out_flag被清除时表示出环岛完成
        if (!Circle.out_flag && !Circle.In_circle)
        {
            Circle.right_circle_state = CIRCLE_COMPLETE;
        }
        break;

    case CIRCLE_COMPLETE:
        Circle.tracking = 0; // 清除tracking标志位，恢复正常元素识别
        Circle.right_circle_state = CIRCLE_IDLE;
        break;
    }

    // 执行环岛进入检查
    check_enter_circle();
}

/**
 * @brief 兼容标志位更新函数
 * @details 根据状态机状态维护现有标志位，确保向后兼容性
 * @note - 根据状态机状态维护Left_flag和Right_flag
 *       - 环岛状态保护机制，防止异常状态
 *       - 长直道时重置所有环岛相关标志位
 * @author 尘烟
 */
void update_legacy_flags(void)
{
    // Left_flag和Right_flag在检测时已经设置，这里根据状态维护
    if (Circle.left_circle_state == CIRCLE_IDLE)
        Circle.Left_flag = 0;
    if (Circle.right_circle_state == CIRCLE_IDLE)
        Circle.Right_flag = 0;

    // tracking安全检查：当所有环岛状态都为IDLE时，强制设置tracking=0
    if (Circle.left_circle_state == CIRCLE_IDLE && Circle.right_circle_state == CIRCLE_IDLE &&
        !Circle.In_circle && !Circle.out_flag)
    {
        Circle.tracking = 0; // 确保非环岛状态时tracking始终为0
    }

    // 环岛状态保护机制：只有在非环岛状态时才允许straight_long重置环岛状态
    if (ImageFlag.straight_long == 1 &&
        Circle.left_circle_state == CIRCLE_IDLE && Circle.right_circle_state == CIRCLE_IDLE &&
        !Circle.In_circle && !Circle.out_flag)
    {
        Circle.now_yaw = 0;  // 重置累计角度
        Circle.out_flag = 0; // 重置出环标志位
        Circle.tracking = 0; // 重置角度跟踪标志
        Circle.In_circle = 0;
        Circle.Left_flag = 0;
        Circle.Right_flag = 0;
    }
}

/**
 * @brief 环岛处理主函数
 * @details 环岛识别和处理的统一入口，整合所有环岛相关功能
 * @note 处理流程：
 *       1. circle_detection() - 环岛特征检测
 *       2. circle_state_machine() - 状态机处理
 *       3. update_legacy_flags() - 兼容标志位更新
 * @author 尘烟
 */
void circle_process(void)
{
    circle_detection(); // 环岛的元素行列

    circle_state_machine(); // 状态机处理

    update_legacy_flags();
}



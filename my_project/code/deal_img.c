#include "deal_img.h"

// ==================== 21届状态机与控制特有变量 ====================
RunState cur_state = STATE_NORMAL;       // 核心状态机：当前工作状态
uint8 Y_trigger = 0;                     // 触发动态检测框的起始行
uint8 is_blind_turning = 0;              // 盲转状态标志位（0:正常图像控制 1:陀螺仪接管切角）
float Yaw_Start = 0;                     // 盲转起始的陀螺仪 Yaw 角
float Yaw_Target = 0;                    // 盲转目标要到达的 Yaw 角

// 假定路径数组：0左转，1右转，2直行 (需要根据你实际跑的电路图提前标定)
uint8 Path_Array[20] = {1, 0, 1, 2, 0, 0};
uint8 node_index = 0;                    // 当前走到了第几个路口节点

// ==================== 图像处理基础变量 ====================
uint16_t histogram[256];                 // 灰度直方图
uint8_t previousThreshold = 0;           // 上一帧阈值
uint8_t nowThreshold = 0;                // 当前阈值
int minGrayscale = 40;                   // 最小灰度限制
int maxGrayscale = 160;                  // 最大灰度限制
int minGray = 0;                         // 图像本身存在的最小灰度
int maxGray = 0;                         // 图像本身存在的最大灰度
uint8_t minThreshold = 70;               // 允许的最小大津法阈值
uint8_t maxThreshold = 160;              // 允许的最大大津法阈值

// 图像分区阈值（用于对抗不均匀光照）
uint8 img_threshold_group[3];            // 分区阈值：近、中、远景
uint8 close_Threshold = 0;               // 近景阈值偏移量
uint8 mid_Threshold = 0;                 // 中景阈值偏移量
uint8 far_Threshold = 0;                 // 远景阈值偏移量

// ==================== 图像存储数组 ====================
uint8 imgGray[IMG_H][IMG_W];             // 原始灰度图像(120*160)
uint8 imgOSTU[YM][XM];                   // 压缩后的二值化图像
uint8 white_num_row[YM];                 // 每行白点总数
uint8 white_num_col[XM];                 // 每列白点总数
uint8 white_row_max[2], white_row_min[2];// 行白点极值
uint8 white_col_max[2], white_col_min[2];// 列白点极值
uint8 mid_line[YM] = {XM/2};             // 最终输出的引导中线数组

// ==================== 爬线起点与丢失统计 ====================
uint8 start_point_l = 0;                 // 左边界起点
uint8 start_point_r = XX;                // 右边界起点
uint8 start_center_y = 0;                // 寻线起点所在行
uint8 start_center_x = XM/2;             // 寻线起点所在列

uint8 l_lost_tip = 0;                    // 左边界遇到丢失特征的点数
uint8 r_lost_tip = 0;                    // 右边界遇到丢失特征的点数
uint8 t_lost_tip = 0;                    // 顶部遇到断层的丢失点数

// ==================== 八邻域爬线专用数组 ====================
int points_l[USE_num][2] = { {0} };      // 爬线寻得的左边界坐标点阵
int points_r[USE_num][2] = { {0} };      // 爬线寻得的右边界坐标点阵
float dir_r[USE_num] = { 0 };            // 右侧生长方向记录
float dir_l[USE_num] = { 0 };            // 左侧生长方向记录
uint32 l_data_statics = 0;               // 爬到的左边界有效点数量
uint32 r_data_statics = 0;               // 爬到的右边界有效点数量
uint8 hightest = 0;                      // 八邻域能爬到的最高有效行数

// 爬线种子点与偏移辅助
int center_point_l[2] = { 0 };
int center_point_r[2] = { 0 };
uint8 search_filds_l[8][2] = { {0} };
uint8 search_filds_r[8][2] = { {0} };
uint8 index_l = 0;
uint8 index_r = 0;
uint8 temp_l[8][2] = { {0} };
uint8 temp_r[8][2] = { {0} };
int8_t seeds_l[8][2] = { {0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1, 0},{1, -1} }; // 左手扶墙走规律
int8_t seeds_r[8][2] = { {0, -1},{1,-1},{1,0}, {1,1},{0,1},{-1,1}, {-1,0},{-1, -1} }; // 右手扶墙走规律

// ==================== 物理逆透视映射表 (需实车标定) ====================
// 【重要】这两组数据决定了动态框起框的精准度
float white_width[YM] = {17,17,17,17,17,16,16,16,16,16,
                         16,16,16,15,15,15,15,14,14,14,
                         14,14,14,13,13,13,13,13,13,13,
                         12,12,12,12,12,11,11,11,11,10,
                         10,10,10,10,10,10,10,10,10, 9,
                          9, 9, 9, 9, 9, 8, 8, 8, 8, 8,
                          7, 7, 7, 7, 6, 6, 6, 6, 5, 5};

float Length_5cm[YM] =  {30,29,28,28,27,27,26,26,25,25,
                         25,25,25,25,24,24,24,23,23,23,
                         22,22,22,22,21,21,21,20,20,20,
                         19,19,19,18,18,18,17,17,16,16,
                         15,15,15,14,14,14,13,13,12,12,
                         11,11,11,11,11,10,10,10,9, 9,
                          9, 8, 8, 8, 8, 7, 7, 7, 7, 7};

// ==================== 基础函数实现 ====================

// 拷贝原始图像到处理数组
void image_copy()
{
    memset(imgGray, 0, sizeof(imgGray));
    for(int i = 0; i < MT9V03X_H; i++) {
        for(int j = 0; j < MT9V03X_W; j++) {
            imgGray[i][j] = mt9v03x_image[i][j];
        }
    }
}

// 绝对值函数
int my_abs(int value) {
    if(value >= 0) return value;
    else return -value;
}

// 灰度直方图统计
void getGrayscaleHistogram(void)
{
    memset(histogram, 0, sizeof(histogram));
    minGray = 0;
    maxGray = 255;
    uint8_t *ptr = &imgGray[IMG_H-YM][(IMG_W-XM)/2];
    uint8_t *ptrEnd = &imgGray[IMG_H-1][(IMG_W-XM)/2 + XM - 1] + 1;
    while (ptr != ptrEnd)
    {
        minGray = *ptr < minGray ? *ptr:minGray;
        maxGray = *ptr > maxGray ? *ptr:maxGray;
        ++histogram[*ptr++];
    }
}

// 获取大津法阈值
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
    if(max < minThreshold) return minThreshold;
    if(min > maxThreshold) return maxThreshold;

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
        if (sigma >= maxSigma) {
            maxSigma = sigma;
            nowThreshold = i;
        } else {
            break;
        }
    }
    nowThreshold = nowThreshold < minThreshold ? minThreshold : nowThreshold;
    nowThreshold = nowThreshold > maxThreshold ? maxThreshold : nowThreshold;
    previousThreshold = nowThreshold;
    return nowThreshold;
}

// 二值化及场地边缘反光暴力消除
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

    // 腐蚀去噪与二值化
    for(int i = 0; i <= YY; i++)
    {
        if(i == YY/3) k = 1;
        else if(i == YY*2/3) k = 2;

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
                    if(x < 0 || x > XX || y < 0 || y > YY) White_num++;
                    else if(imgOSTU[y][x] == White) { White_num = 8; break; }
                    else if(imgGray[IMG_H-1-y][(IMG_W-XM)/2+x] >= nowThreshold) White_num++;
                    else Black_num++;

                    if(Black_num >= 2) break;
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
                        if(x < 0 || x > XX || y < 0 || y > YY) continue;
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

    // [反光暴力消除] 检查两侧纵向列是否有大面积反光发白带，若有则强制涂黑其以及周围外扩区域
    white_row_max[0] = 0; white_col_max[0] = 0;
    for(uint8 i = Deal_Left; i <= Deal_Right; i++) {
        if(white_num_col[i] > white_col_max[0]) {
            white_col_max[0] = white_num_col[i];
        }
    }
    // 如果发白列异常多，且并不是全部屏幕白茫茫，说明是侧边大面积反光膜
    if(white_col_max[0] >= YY && white_row_max[0] < white_width[0])  
    {
        for(uint8 x = 0; x <= XX; x++ ) {
            if(imgOSTU[Deal_Bottom-1][x] == White || imgOSTU[Deal_Bottom-2][x] == White) imgOSTU[Deal_Bottom][x] = White;
            imgOSTU[Deal_Bottom-1][x] = Black; imgOSTU[Deal_Bottom-2][x] = Black;

            if(imgOSTU[Deal_Top+1][x] == White || imgOSTU[Deal_Top+2][x] == White) imgOSTU[Deal_Top][x] = White;
            imgOSTU[Deal_Top+1][x] = Black; imgOSTU[Deal_Top+2][x] = Black;
        }
        for(uint8 y = 0; y <= Deal_Top; y++ ) {
            if(imgOSTU[y][Deal_Left-1] == White || imgOSTU[y][Deal_Left-2] == White) imgOSTU[y][Deal_Left] = White;
            imgOSTU[y][Deal_Left-1] = Black; imgOSTU[y][Deal_Left-2] = Black;

            if(imgOSTU[y][Deal_Right+1] == White || imgOSTU[y][Deal_Right+2] == White) imgOSTU[y][Deal_Right] = White;
            imgOSTU[y][Deal_Right+1] = Black; imgOSTU[y][Deal_Right+2] = Black;
        }
    }
}

// 寻找起点
uint8 get_start_point(void)
{
    start_center_y = 0;
    uint8 l_found = 0, r_found = 0;
    uint32 l_point = 0, r_point = 0;

    for(uint8 i = Deal_Bottom; i <= Deal_Bottom + 30; i++)
    {
        if(white_num_row[i] >= 5 && white_num_row[i+1] >= 5)
        {
            start_center_y = i;
            // 从左向右找黑白跳变点
            for(uint8 j = 0; j < XX-1; j++) {
                if(imgOSTU[start_center_y][j+1] == White && imgOSTU[start_center_y][j] == Black) {
                    l_point = j; l_found = 1; break;
                }
            }
            // 从右向左找黑白跳变点
            for(uint8 j = XX; j > 1; j--) {
                if(imgOSTU[start_center_y][j-1] == White && imgOSTU[start_center_y][j] == Black) {
                    r_point = j; r_found = 1; break;
                }
            }
            if(r_found && l_found) break;
        }
    }

    if(l_found && r_found) {
        start_center_x = (l_point + r_point) / 2;
        start_point_l = l_point;
        start_point_r = r_point;
        return 1;
    }
    return 0;
}

// =========================================================================
// 核心：八邻域爬线与 Trigger 2.0 异常打断
// =========================================================================
uint8 search_l_r(uint8 start_l_x, uint8 start_l_y, uint8 start_r_x, uint8 start_r_y)
{
    hightest = 0;
    l_lost_tip = 0;
    r_lost_tip = 0;
    t_lost_tip = 0;
    l_data_statics = 0;
    r_data_statics = 0;

    center_point_l[0] = start_l_x; center_point_l[1] = start_l_y;
    center_point_r[0] = start_r_x; center_point_r[1] = start_r_y;

    points_l[l_data_statics][0] = center_point_l[0];
    points_l[l_data_statics][1] = center_point_l[1];
    points_r[r_data_statics][0] = center_point_r[0];
    points_r[r_data_statics][1] = center_point_r[1];

    uint32 break_flag = USE_num;

    while(break_flag--)
    {
        // 1. 左侧边界八邻域生长
        for(uint8 i = 0; i < 8; i++) {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
        }
        l_data_statics++; index_l = 0;
        for(uint8 i = 0; i < 8; i++) {
            if((imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Black || imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Lost_line) &&
               (imgOSTU[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] != Black && imgOSTU[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] != Lost_line))
            {
                temp_l[index_l][0] = search_filds_l[i][0];
                temp_l[index_l][1] = search_filds_l[i][1];
                index_l++;
                dir_l[l_data_statics - 1] = i;
            }
        }
        if(index_l) {
            center_point_l[0] = temp_l[0][0]; center_point_l[1] = temp_l[0][1];
            for(uint8 j = 0; j < index_l; j++) {
                if(center_point_l[1] < temp_l[j][1]) {
                    center_point_l[0] = temp_l[j][0];
                    center_point_l[1] = temp_l[j][1];
                }
            }
        }

        // 丢失特征统计 (简化的边缘碰壁检测)
        if(center_point_l[0] <= Deal_Left) l_lost_tip++;
        if(center_point_l[1] >= Deal_Top) t_lost_tip++;

        if(center_point_l[1] > hightest) hightest = center_point_l[1];

        // 2. 右侧边界八邻域生长
        for(uint8 i = 0; i < 8; i++) {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
        }
        r_data_statics++; index_r = 0;
        for(uint8 i = 0; i < 8; i++) {
            if((imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Black || imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Lost_line) &&
               (imgOSTU[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] != Black && imgOSTU[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] != Lost_line))
            {
                temp_r[index_r][0] = search_filds_r[i][0];
                temp_r[index_r][1] = search_filds_r[i][1];
                index_r++;
                dir_r[r_data_statics - 1] = i;
            }
        }
        if(index_r) {
            center_point_r[0] = temp_r[0][0]; center_point_r[1] = temp_r[0][1];
            for(uint8 j = 0; j < index_r; j++) {
                if(center_point_r[1] < temp_r[j][1]) {
                    center_point_r[0] = temp_r[j][0];
                    center_point_r[1] = temp_r[j][1];
                }
            }
        }

        if(center_point_r[0] >= Deal_Right) r_lost_tip++;
        if(center_point_r[1] >= Deal_Top) t_lost_tip++;

        if(center_point_r[1] > hightest) hightest = center_point_r[1];

        // 储存边界
        points_l[l_data_statics][0] = center_point_l[0];
        points_l[l_data_statics][1] = center_point_l[1];
        points_r[r_data_statics][0] = center_point_r[0];
        points_r[r_data_statics][1] = center_point_r[1];

        // 基础闭环退出条件
        if(my_abs(points_r[r_data_statics][0] - points_l[l_data_statics][0]) <= 1 &&
           my_abs(points_r[r_data_statics][1] - points_l[l_data_statics][1]) <= 1) break;

        // =========================================================================
        // 【核心】Trigger 2.0 打断：遇到可疑干扰件或路口，立刻停止向外生长！
        // =========================================================================
        uint8 cur_y = hightest;
        int current_width = points_r[r_data_statics][0] - points_l[l_data_statics][0];

        // 特征 A：宽度突然膨胀大于标准宽度的1.5倍（遇到横向引脚或大路口）
        uint8 trigger_width = (current_width > (white_width[cur_y] * 1.5)) ? 1 : 0;

        // 特征 B：刚起步没多高，就大量碰到了屏幕边缘丢失线（遇到极宽的三岔路）
        uint8 trigger_fork = 0;
        if(cur_y < (Deal_Top - 20) && (l_lost_tip > 5 || r_lost_tip > 5)) {
            trigger_fork = 1;
        }

        // 触发判定，要求车子必须起步几行后，排除车头杂色死角
        if ((trigger_width || trigger_fork) && cur_y > Deal_Bottom + 5)
        {
            Y_trigger = cur_y;
            cur_state = STATE_CHECK_NODE; // 将控制权移交给外框判定函数
            break;
        }
    }

    // 绘制左右边线用于TFT显示
    for(int i = l_data_statics; i > 0; i--) imgOSTU[points_l[i][1]][points_l[i][0]] = Left_line;
    for(int i = r_data_statics; i > 0; i--) imgOSTU[points_r[i][1]][points_r[i][0]] = Right_line;

    return 0;
}

// =========================================================================
// 核心：动态框鉴别真假节点
// =========================================================================
uint8 Check_Node_Box(uint8 trigger_y)
{
    int box_h = 0;
    uint8 temp_y = trigger_y;

    // 利用数组推算，向上看约 15cm (累计3个5cm)
    for(int i = 0; i < 3; i++) {
        if(temp_y >= Deal_Top) break;
        box_h += Length_5cm[temp_y];
        temp_y += Length_5cm[temp_y];
    }
    
    int box_top = trigger_y + box_h; 
    if(box_top >= Deal_Top) box_top = Deal_Top - 1; // 极限保护，防止 y+1 越界

    int left_border_missing = 0; 
    int right_border_missing = 0;

    // 扫描动态框的左右边缘
    for (int y = trigger_y; y <= box_top; y++)
    {
        // 如果左侧框内连续两行都是白点，说明向左有通路
        if(imgOSTU[y][Deal_Left+5] == White && imgOSTU[y+1][Deal_Left+5] == White) {
            left_border_missing++;
        }
        // 同理看右侧
        if(imgOSTU[y][Deal_Right-5] == White && imgOSTU[y+1][Deal_Right-5] == White) {
            right_border_missing++;
        }
    }

    // 判定：只要有一侧出现大面积(比如超过5行)缺口，就是真节点（岔路）
    // 否则就是干扰元件（电感电阻），因为它们的线在框内绕，但不会跑到框外面去！
    if (left_border_missing > 5 || right_border_missing > 5) 
    {
        return 1; // 真节点 (REAL_NODE)
    }

    return 0; // 假干扰 (FAKE_BROKEN)
}

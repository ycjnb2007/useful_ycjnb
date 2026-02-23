#include "Image_processing.h"


int16   l_border[IMAGE_H];        //左边界数组
int16   r_border[IMAGE_H];        //右边界数组
int16   border[IMAGE_H];          //中线数组
int8    rightangle_row;           //检测到的直角行号
uint8   right_rightangle_flag=0;  //右直角标志位
uint8   left_rightangle_flag=0;   //左直角标志位
uint8   start_angle_flag=0;       //开始角度积分标志位


BorderStatus l_status[IMAGE_H], r_status[IMAGE_H];//边界状态数组








// 动态搜索窗口半径（每5行步进）
int get_search_window(int row)
{
    int base = 6;                // 最小窗口
    int max = SEARCH_WINDOW;      // 最大窗口（可与原SEARCH_WINDOW一致）
    int step = (row / 5);
    int window = base + step * 2; // 每5行窗口+2
    if(window > max) window = max;
    return window;
}


// 动态获取最大允许插值行数间隔（近大远小）
int16 get_interpolate_gap(int16 row)
{
    // 近处（下方）允许插值多，远处（上方）少
    if (row >= IMAGE_H - 40)            //50--80行
    {
        return 30;  // 近处
    } 
    else if (row >= IMAGE_H - 55)      //25--40行
    {
        return 22;  // 中间
    } 
    else                               //0--29行
    {
        return 10;  // 远处
    }
}


int16 start_h = IMAGE_H - 20;         // 默认第二阶段起始行
//搜索边界
void search_border()
{
   static int16 last_valid_l_row = -1;  // 上一次左边界有效的行号
   static int16 last_valid_l_col = 0;   // 对应的列位置
   static int16 last_valid_r_row = -1;  // 右边界同理
   static int16 last_valid_r_col = 0;

   
    // 每次调用函数时初始化静态变量（避免历史数据干扰）
    last_valid_l_row = -1;
    last_valid_l_col = 0;
    last_valid_r_row = -1;
    last_valid_r_col = 0;
   
   
    // 第一阶段：向上搜索直到找到第一个有效边界
    for (int16 i = IMAGE_H-3; i >= IMAGE_H - 20; i--)
     {
        l_status[i] = BORDER_NOT_FOUND;
        r_status[i] = BORDER_NOT_FOUND;

        // 左边界搜索
        for (int16 j = SEARCH_MIN; j <= SEARCH_MAX; j++)
        {
              if(j+JUMP >= IMAGE_W) continue; // 防止数组越界
            if (Thresholding_image[i][j+JUMP]==white&&Thresholding_image[i][j]==black) 
            {
                l_border[i] = j;
                l_status[i] = BORDER_VALID;
                break;
            }            
            
        }

        // 右边界搜索
        for (int16 j = SEARCH_MAX; j >= SEARCH_MIN; j--) 
        {
                if(j-JUMP < 0) continue; // 防止数组越界
            if (Thresholding_image[i][j-JUMP]==white&&Thresholding_image[i][j]==black) 
             {
                r_border[i] = j;
                r_status[i] = BORDER_VALID;
                break;
            }
        }

        // 边界两边边界都搜到线则立即终止第一阶段
        if (l_status[i] == BORDER_VALID && r_status[i] == BORDER_VALID)
         {
            start_h = i;
            break;
        }
    }
      
     // 第二阶段：动态回溯搜索
    for (int16 i = start_h-1; i >= 0; i--) 
    {
        l_status[i] = BORDER_NOT_FOUND;       //默认未找到边界
        r_status[i] = BORDER_NOT_FOUND;       //默认未找到边界
        l_border[i] = SEARCH_MIN;             // 默认左置边
        r_border[i] = SEARCH_MAX;             // 默认右置边

        /*---------- 左边界动态搜索 ----------*/
        // 回溯查找最近的有效左边界
        int16 valid_row_l = -1;
        for(int16 back = 1; back <= MAX_BACKTRACK_DEPTH; back++)
        {
            if(i+back >= IMAGE_H-1) break;
            if(l_status[i+back] == BORDER_VALID)
            {
                valid_row_l = i+back;
                break;
            }
        }
        
        if(valid_row_l != -1)
        {
            // 基于上一有效边界生成动态搜索范围
            int16 base_col = l_border[valid_row_l];
            

            uint8 search_window = get_search_window(i); // 动态窗口
            int16 predict_min = base_col - search_window;
            int16 predict_max = base_col + search_window;
            
            
            // 约束全局边界
            predict_min = (predict_min < SEARCH_MIN) ? SEARCH_MIN : predict_min;
            predict_max = (predict_max > SEARCH_MAX) ? SEARCH_MAX : predict_max;

            // 左边界预测搜索
            for (int16 j = predict_min; j <= predict_max; j++) 
            {
                if(j+JUMP >= IMAGE_W) continue; // 防止数组越界
                

                if (Thresholding_image[i][j+JUMP] == white && 
                    Thresholding_image[i][j] == black) 
                {
                    l_border[i] = j;
                    l_status[i] = BORDER_VALID;
                    break;
                }
            }
        }

        /*---------- 右边界动态搜索 ----------*/
        // 回溯查找最近的有效右边界
        int16 valid_row_r = -1;
        for(int16 back = 1; back <= MAX_BACKTRACK_DEPTH; back++)
        {
            if(i+back >= IMAGE_H-1) break;
            if(r_status[i+back] == BORDER_VALID)
            {
                valid_row_r = i+back;
                break;
            }
        }

        if(valid_row_r != -1)
        {
            // 基于上一有效边界生成动态搜索范围
            int16 base_col = r_border[valid_row_r];

            uint8 search_window = get_search_window(i); // 动态窗口
            int16 predict_min = base_col - search_window;
            int16 predict_max = base_col + search_window;
                  
            
            // 约束全局边界
            predict_min = (predict_min < SEARCH_MIN) ? SEARCH_MIN : predict_min;
            predict_max = (predict_max > SEARCH_MAX) ? SEARCH_MAX : predict_max;

            // 右边界预测搜索
            for (int16 j = predict_max; j >= predict_min; j--) 
            {
                if(j-JUMP < 0) continue; // 防止数组越界
                if (Thresholding_image[i][j-JUMP] == white && 
                    Thresholding_image[i][j] == black) 
                {
                    r_border[i] = j;
                    r_status[i] = BORDER_VALID;
                    break;
                }
            }
        }   

                    // 虚线检测和处理
                if (l_status[i] == BORDER_VALID&&right_rightangle_flag==0&&left_rightangle_flag==0) 
                {
                  
                    if (last_valid_l_row == i) continue;        // 避免除零
                    
                    int16 gap_down = get_interpolate_gap(i);
                    int16 gap_up = get_interpolate_gap(last_valid_l_row);
                    int16 max_gap = MIN(gap_down, gap_up);
                    
                    if (last_valid_l_row != -1 && 
                        i < last_valid_l_row - 1 &&             // 中间有间隔
                        (last_valid_l_row - i) <= max_gap)      //检测边界是否满足有效->无效->有效
                    {
                        // 计算插值步长
                        float delta_col = (float)(last_valid_l_col - l_border[i]) / (last_valid_l_row - i);
                        
                        // 填充中间行
                        for (int16 r = i + 1; r < last_valid_l_row; r++) 
                        {
                            l_border[r] = l_border[i] + (int16)(delta_col * (r - i));
                            l_status[r] = BORDER_INTERPOLATED;         //插值生成的边界状态标记为BORDER_INTERPOLATED，区别于直接检测到的BORDER_VALID。
                        }
                    }
                    // 更新上一次有效状态
                    last_valid_l_row = i;
                    last_valid_l_col = l_border[i];
                } 
                else
                {
                    // 当连续丢线超过阈值时重置
                    if (last_valid_l_row - i > get_interpolate_gap(i)) 
                    {
                        last_valid_l_row = -1;
                    }
                }
                    //右边同理
              if (r_status[i] == BORDER_VALID&&right_rightangle_flag==0&&left_rightangle_flag==0)
              {
                  if (last_valid_r_row != -1 && 
                      i < last_valid_r_row - 1 && 
                      (last_valid_r_row - i) <= get_interpolate_gap(i)) 
                  {
                      int16 delta_col = (last_valid_r_col - r_border[i]) / (last_valid_r_row - i);
                      for (int16 r = i + 1; r < last_valid_r_row; r++) 
                      {
                          r_border[r] = r_border[i] + delta_col * (r - i);
                          r_status[r] = BORDER_INTERPOLATED;
                      }
                  }
                  last_valid_r_row = i;
                  last_valid_r_col = r_border[i];
              } 
              else 
              {
                  if (last_valid_r_row - i > get_interpolate_gap(i)) 
                  {
                      last_valid_r_row = -1;
                  }
              } 

                        validate_border(i) ;            //进行行间差异检测和边界补偿//直角不补线
    }

//////////////////再次进行补偿
            for (int16 i = start_h-1; i >= 0; i--)
            {
                validate_border(i);
            }

}



void validate_border(int16 i) 
{
    // 行间差异检测;当该行边界与上一行边界差值大于阈值时，标记为BORDER_ROW_DIFF，可对其进行边界补偿（补偿后状态会被覆盖）
  //**想想如果不满足补偿逻辑该怎么处理？对其置边？
    if (i < IMAGE_H - 1) 
    {
        if (l_status[i] == BORDER_VALID && l_status[i+1] == BORDER_VALID && //当该行和上一行都为有效边界时，两行边界做差
            abs(l_border[i] - l_border[i+1]) > ROW_DELTA_THRESHOLD) 
        {
            l_status[i] = BORDER_ROW_DIFF;       
        }
        if (r_status[i] == BORDER_VALID && r_status[i+1] == BORDER_VALID &&
            abs(r_border[i] - r_border[i+1]) > ROW_DELTA_THRESHOLD) 
        {
            r_status[i] = BORDER_ROW_DIFF;
        }
    }
    
    // 边界补偿逻辑:一边有效且不是置边，一边无效，则将有效边界偏移作为另一边界，并标记为补偿边界。
    if (l_status[i] == BORDER_NOT_FOUND&&r_status[i] != BORDER_NOT_FOUND &&(r_border[i]!=SEARCH_MAX))//&&(l_border[i-1]!=SEARCH_MIN)
    {
        l_border[i] = r_border[i] - BORDER_OFFSET;
        l_status[i] = BORDER_COMPENSATED;
    } 
    else if (r_status[i] == BORDER_NOT_FOUND&&l_status[i] != BORDER_NOT_FOUND &&(l_border[i]!=SEARCH_MIN))//&&(r_border[i-1]!=SEARCH_MAX)
    {
        r_border[i] = l_border[i] + BORDER_OFFSET;
        r_status[i] = BORDER_COMPENSATED;       
    }
    
    
    
 
    //当左右边界差异都超过阈值时，对其置边
    if(r_status[i] == BORDER_ROW_DIFF&&l_status[i] == BORDER_ROW_DIFF)
    {
      r_border[i]=SEARCH_MAX;
      l_border[i]=SEARCH_MIN;
    }
}

int16 cross_row=-1;
void cross_check( )//十字检测
{
  cross_row=-1;
  if(max_white_value >cross_max_white_value)
  {
      if((Thresholding_image[max_white_row][MIDLINE-20]==white||Thresholding_image[max_white_row][MIDLINE-40]==white||Thresholding_image[max_white_row][MIDLINE-50]==white)
            &&(Thresholding_image[max_white_row][MIDLINE+20]==white||Thresholding_image[max_white_row][MIDLINE+40]==white||Thresholding_image[max_white_row][MIDLINE+50]==white))
      {
          cross_row=max_white_row;
      }
  } 
  
}

// 比较函数，用于qsort升序排序
int compare_function(const void *a, const void *b) 
{
    int16 arg1 = *(const int16*)a;
    int16 arg2 = *(const int16*)b;
    return (arg1 > arg2) - (arg1 < arg2);
}


// 环岛相关定义
#define ISLAND_CHECK_ROWS                   {72,65,60,55,25,23,20,18}   // 检测行集合                //正常情况下使用flash中的行白点数，若失败则使用该数组白点
#define ISLAND_CHECK_ROWS_normal_white       {11, 11,11,10,8,7,7,8}       // 检测行对应直道白点宽度集合 **必须和检测行个数相同
                                                               
#define WHITE_MULTIPLIER_MIN        1.6f                   // 白点倍数最小阈值
#define WHITE_MULTIPLIER_MAX        4.0                    // 白点倍数最大阈值

#define ISLAND_DIFF_THRESH      70                       // 边界差值阈值
#define ISLAND_CONFIRM_FRAMES   1                        // 连续确认帧数
#define ISLAND_DISTANCE_MAX   60000                      // 强制退出环岛距离

#define ISLAND_CONFIRM_RATIO    8                        // 环岛有效检测行比例阈值，8为0.8

IslandState island_state = ISLAND_NONE;
uint8 island_confirm_cnt = 0;
IslandState original_island;  // 记录原始环岛方向
int16 l_diff = 0, r_diff = 0;



RingType ringSequence[] = { RING_SMALL, RING_BIG, RING_MEDIUM };         //环岛大小顺序
int8 ringCount = sizeof(ringSequence) / sizeof(ringSequence[0]);       //计算环岛个数
int8 currentRingIndex = -1; 
RingType current_circle_type=RING_NONE;


// 环岛检测
void circle_check() 
{
    const uint8 check_rows[] = ISLAND_CHECK_ROWS;                                //检测行数组
    const uint8 rows_cnt = sizeof(check_rows)/sizeof(check_rows[0]);            //计算检测行数组 check_rows 的实际元素个数
    
  /*---- 第一步：统计有效检测行比例 ----*/
    uint8 valid_rows = 0;
    for(uint8 i=0; i<rows_cnt; i++) 
    {
        uint8 row = check_rows[i];
        if(row >= IMAGE_H||ting_circle<50) continue;            //防止越界和重复判定
        
        // 使用从Flash读取的正常白点值
        uint16 normal_white = normal_white_per_row[row];
        if(normal_white == 0)  // 如果Flash中没有数据，使用默认值
        {
            const uint8 default_white[] = ISLAND_CHECK_ROWS_normal_white;
            normal_white = default_white[i];
        }
        
        // 计算阈值时避免浮点
        uint16 threshold1 = (uint16)(normal_white * WHITE_MULTIPLIER_MIN);
        uint16 threshold2 = (uint16)(normal_white * WHITE_MULTIPLIER_MAX);

        if(white_per_row[row] >= threshold1&&white_per_row[row] < threshold2) 
        {
            valid_rows++;
        }
    }

    // 使用交叉相乘避免除法: valid/rows < 8/10 => valid*10 < rows*8
    if(valid_rows * 10 < rows_cnt * ISLAND_CONFIRM_RATIO) 
    {
        island_confirm_cnt = 0;
        l_diff = 0, r_diff = 0;
        return;
    }
    /*---- 第二步：对边界最值做差判断环岛左右 ----*/
     l_diff = 0, r_diff = 0;
    
    // 收集左边界有效值（VALID和INTERPOLATED状态）
    int16 l_values[IMAGE_H];
    int l_count = 0;
    for(int i = 0; i < IMAGE_H; i++) 
    {
        if(l_status[i] == BORDER_VALID || l_status[i] == BORDER_INTERPOLATED) 
        {
            l_values[l_count++] = l_border[i];
        }
    }
    
    // 收集右边界有效值
    int16 r_values[IMAGE_H];
    int r_count = 0;
    for(int i = 0; i < IMAGE_H; i++) 
    {
        if(r_status[i] == BORDER_VALID || r_status[i] == BORDER_INTERPOLATED) 
        {
            r_values[r_count++] = r_border[i];
        }
    }
    
    // 计算左边界差值（采用倒数第三大和第三小，抗干扰）
    if(l_count >= 3) 
    {
        qsort(l_values, l_count, sizeof(int16), compare_function);
        int16 l_third_max = l_values[l_count - 3]+l_values[l_count - 4]; // 倒数第三和第四大的值
        int16 l_third_min = l_values[2]+l_values[3];                     // 倒数第三和第四小的值
        l_diff = l_third_max - l_third_min;
    }
    
    // 计算右边界差值
    if(r_count >= 3) 
    {
        qsort(r_values, r_count, sizeof(int16), compare_function);
        int16 r_third_max = r_values[r_count - 3]+r_values[r_count - 4]; 
        int16 r_third_min = r_values[2]+r_values[3];                     
        r_diff = r_third_max - r_third_min;
    }
    
    /*---- 状态判断 ----*/
        // 右环岛条件：右差值超阈值且左差值小于阈值的三分之一
        if(r_diff > ISLAND_DIFF_THRESH && l_diff < ISLAND_DIFF_THRESH/2) 
        {
          island_confirm_cnt++;
            if(island_confirm_cnt >= ISLAND_CONFIRM_FRAMES) 
            {
                island_state = ISLAND_RIGHT;
                original_island = ISLAND_RIGHT;  // 新增赋值
                currentRingIndex = (currentRingIndex + 1) % ringCount;//实现顺序循环   
                current_circle_type = ringSequence[currentRingIndex];       //更新环岛大小
            }
        }
        // 左环岛条件：左差值超阈值且右差值小于阈值的三分之一
        else if(l_diff > ISLAND_DIFF_THRESH && r_diff < ISLAND_DIFF_THRESH/2) 
        {
            if(++island_confirm_cnt >= ISLAND_CONFIRM_FRAMES) 
            {
                island_state = ISLAND_LEFT;
                original_island = ISLAND_LEFT;  // 新增赋值
                currentRingIndex = (currentRingIndex + 1) % ringCount;//实现顺序循环
                current_circle_type = ringSequence[currentRingIndex];       //更新环岛大小
            }
        }
        else 
        {
            island_confirm_cnt = 0;
        }
}




#define POST_ISLAND_DISTANCE  7000    // 环后处理距离阈值
uint16_t post_island_distance = 0;    // 环后处理距离积分
// 环岛处理
uint16  island_distance=0;
void island_deal()
{

    if(island_state != ISLAND_NONE) 
  {

    static int16 anchor_row = -1;     // 锚定行（第三小值所在行）
    island_distance+=SPEED;     //开始积分记距离
    start_angle_flag=1;         //开始角度积分标志位
    
    
        // 检查是否进入出环状态
        if(func_abs(theta_total) > 280)            //角度大于这个值则准备出环岛
        {
            island_state = ISLAND_LEAVE;
        }
        
        // 出环状态处理
        if(island_state == ISLAND_LEAVE) 
        {            
            // 退出条件
            if(func_abs(theta_total)> 300||(island_distance>ISLAND_DISTANCE_MAX))            //想想还有什么条件能判断
            {
                // 进入环后处理状态
                island_state = ISLAND_POST_PROCESS;
                post_island_distance = 0;  // 重置距离积分
                theta_total = 0;

            }
            
            // 单边巡线逻辑（与入环方向相反）
            if(island_state == ISLAND_LEAVE)
            {
                if(original_island == ISLAND_LEFT) 
                {  // 原左环岛用右单边
                    for(int r=0; r<IMAGE_H; r++) 
                    {
                      if(r_status[r]==BORDER_VALID )
                        l_border[r] = r_border[r] - BORDER_OFFSET;
                        l_status[r] = BORDER_COMPENSATED;
                    }
                } 
              else  if(original_island == ISLAND_RIGHT)
                {  // 原右环岛用左单边
                    for(int r=0; r<IMAGE_H; r++) 
                    {                
                      if(l_status[r]==BORDER_VALID )
                        r_border[r] = l_border[r] + BORDER_OFFSET;
                        r_status[r] = BORDER_COMPENSATED;
                    }
                }
            }
        }
                // 环后处理状态，防止出环车身不稳
        else if(island_state == ISLAND_POST_PROCESS) 
        {
            // 积分距离
            post_island_distance += (func_abs(motor_value_left.receive_left_speed_data) + func_abs(motor_value_right.receive_right_speed_data)
             +func_abs(motor_value_left.receive_right_speed_data)+func_abs(motor_value_right.receive_left_speed_data)) / 4;;
            
            // 单边巡线逻辑（与入环方向相反）
            if(original_island == ISLAND_LEFT) 
            {  // 原左环岛用右单边
                for(int r=0; r<IMAGE_H; r++) 
                {
                    if(r_status[r]==BORDER_VALID )
                    {
                        l_border[r] = r_border[r] - BORDER_OFFSET;
                        l_status[r] = BORDER_COMPENSATED;
                    }
                }
            } 
            else if(original_island == ISLAND_RIGHT)
            {  // 原右环岛用左单边
                for(int r=0; r<IMAGE_H; r++) 
                {                
                    if(l_status[r]==BORDER_VALID )
                    {
                        r_border[r] = l_border[r] + BORDER_OFFSET;
                        r_status[r] = BORDER_COMPENSATED;
                    }
                }
            }
            
            // 退出条件1：检测到直角
            if(left_rightangle_flag || right_rightangle_flag)
            {
                island_state = ISLAND_NONE;
                original_island = ISLAND_NONE;
                post_island_distance = 0;
                start_angle_flag = 0;
                current_circle_type = RING_NONE;
                ting_circle = 0;
            }
            // 退出条件2：达到预定距离
            else if(post_island_distance > POST_ISLAND_DISTANCE)
            {
                island_state = ISLAND_NONE;
                original_island = ISLAND_NONE;
                post_island_distance = 0;
                start_angle_flag = 0;
                current_circle_type = RING_NONE;
                ting_circle = 0;
            }
        }

        
        if(island_state == ISLAND_LEFT || island_state == ISLAND_RIGHT)
          { 
            // 根据环岛方向处理边界
            switch (island_state)
            {
                case ISLAND_RIGHT: 
                  {
                    // 寻找右边界第三小值
                    int16 valid_r_values[IMAGE_H];
                    int16 valid_rows[IMAGE_H];
                    int valid_count = 0;

                    // 收集有效右边界（有效或插值状态），限制在白点值超过正常赛道的1.5倍的行范围内
                    for (int i = 40; i < IMAGE_H; i++) 
                    {
                        // 检查白点值是否超过正常赛道的1.5倍
                        uint16 normal_white = normal_white_per_row[i];
                        if(normal_white == 0)  // 如果Flash中没有数据，使用默认值
                        {
                            const uint8 default_white[] = ISLAND_CHECK_ROWS_normal_white;
                            // 找到对应的默认值（这里简化处理，使用第一个默认值）
                            normal_white = default_white[0];
                        }
                        uint16 threshold = (uint16)(normal_white * 1.5f);
                        
                        if (white_per_row[i] >= threshold && 
                            (r_status[i] == BORDER_VALID || r_status[i] == BORDER_INTERPOLATED))
                        {
                            valid_r_values[valid_count] = r_border[i];
                            valid_rows[valid_count] = i;
                            valid_count++;
                        }
                    }

                    // 至少需要3个有效值才能找到第三小
                    if (valid_count >= 3) 
                    {
                        // 找到第三小的索引
                        int16 min1 = IMAGE_W, min2 = IMAGE_W, min3 = IMAGE_W;
                        int16 idx1 = -1, idx2 = -1, idx3 = -1;

                        for (int i = 0; i < valid_count; i++)
                        {
                            if (valid_r_values[i] < min1) 
                            {
                                min3 = min2; idx3 = idx2;
                                min2 = min1; idx2 = idx1;
                                min1 = valid_r_values[i]; idx1 = i;
                            }
                            else if (valid_r_values[i] < min2)
                            {
                                min3 = min2; idx3 = idx2;
                                min2 = valid_r_values[i]; idx2 = i;
                            } 
                            else if (valid_r_values[i] < min3)
                            {
                                min3 = valid_r_values[i]; idx3 = i;
                            }
                        }

                        // 获取第三小的行号
                        if (idx3 != -1) 
                        {
                            anchor_row = valid_rows[idx3];
                        }
                    }

                    // 应用偏移（从锚定行向上处理）
                    if (anchor_row != -1) 
                    {
                        for (int r = anchor_row; r >= 0; r--)
                        {
                                l_border[r] = r_border[r] - BORDER_OFFSET; // 动态偏移//                        l_border[r] = base_offset + (anchor_row - r)/2; // 动态偏移
                                l_status[r] = BORDER_COMPENSATED;
                        }
                        for(int16 i = 25 ; i >= 0; i--)       //如果检测到环岛则将上30行置边，以免不入环。
                        {
                              l_border[i] = SEARCH_MAX; // 左置边
                              r_border[i] = SEARCH_MAX; // 右置边
                        }
                    }
                    break;
                }

                case ISLAND_LEFT: 
                  {
                    // 寻找左边界第三大值（与右边同理）
                    int16 valid_l_values[IMAGE_H];
                    int16 valid_rows[IMAGE_H];
                    int valid_count = 0;

                    // 收集有效左边界（有效或插值状态），限制在白点值超过正常赛道的1.5倍的行范围内
                    for (int i = 40; i < IMAGE_H; i++) 
                    {
                        // 检查白点值是否超过正常赛道的1.5倍
                        uint16 normal_white = normal_white_per_row[i];
                        if(normal_white == 0)  // 如果Flash中没有数据，使用默认值
                        {
                            const uint8 default_white[] = ISLAND_CHECK_ROWS_normal_white;
                            // 找到对应的默认值（这里简化处理，使用第一个默认值）
                            normal_white = default_white[0];
                        }
                        uint16 threshold = (uint16)(normal_white * 1.5f);
                        
                        if (white_per_row[i] >= threshold && 
                            (l_status[i] == BORDER_VALID || l_status[i] == BORDER_INTERPOLATED))
                        {
                            valid_l_values[valid_count] = l_border[i];
                            valid_rows[valid_count] = i;
                            valid_count++;
                        }
                    }

                    if (valid_count >= 3)
                    {
                        // 找第三大值（即右侧最远的第三个边界点）
                        int16 max1 = 0, max2 = 0, max3 = 0;
                        int16 idx1 = -1, idx2 = -1, idx3 = -1;

                        for (int i = 0; i < valid_count; i++)
                        {
                            if (valid_l_values[i] > max1)
                            {
                                max3 = max2; idx3 = idx2;
                                max2 = max1; idx2 = idx1;
                                max1 = valid_l_values[i]; idx1 = i;
                            } 
                            else if (valid_l_values[i] > max2)
                            {
                                max3 = max2; idx3 = idx2;
                                max2 = valid_l_values[i]; idx2 = i;
                            } 
                            else if (valid_l_values[i] > max3)
                            {
                                max3 = valid_l_values[i]; idx3 = i;
                            }
                        }

                        if (idx3 != -1)
                        {
                            anchor_row = valid_rows[idx3];
                        }
                    }

                    // 应用偏移
                    if (anchor_row != -1)
                    {
                        for (int r = anchor_row; r >= 0; r--) 
                        {
                                r_border[r] = l_border[r] + BORDER_OFFSET; // 动态递减偏移
                                r_status[r] = BORDER_COMPENSATED;
                        }
                        
                        
                                for(int16 i = 25 ; i >= 0; i--)       //如果检测到环岛则将上30行置边，以免不入环。
                        {
                              l_border[i] = SEARCH_MIN; // 左置边
                              r_border[i] = SEARCH_MIN; // 右置边
                        }
                        
                        

                    }
                    break;
                }

                default:
                    break;
            }
          }
    }
}


#define RIGHT_ANGLE_WHITE_THRESHOLD_MAX  95     // 直角行白点数量最大阈值
#define RIGHT_ANGLE_WHITE_THRESHOLD_MIN  40     //直角行白点数量最小阈值
#define START_RIGHT_ANGLE_ROW            75    //直角检测开始行（从下往上）
#define END_RIGHT_ANGLE_ROW              5    //直角检测结束行


#define BORDER_LOST_ROWS                 30     // 直角行上方无边界行数阈值
#define RIGHT_W                          3      // 直角行宽度


// 直角检测列定义（左侧和右侧）
#define LEFT_CHECK_COLS     {20, 22, 24,26}                                  // 左侧检测列
#define RIGHT_CHECK_COLS    {IMAGE_W-20, IMAGE_W-22, IMAGE_W-24,IMAGE_W-26}  // 右侧检测列
//#define CHECK_COLS_CNT      4                                              // 检测列数量



uint8 right_white_found_threshold=20;             //判断直角白点的阈值
// 直角检测函数
void detect_right_angle( ) 
{
    rightangle_row = -1;
    int16 lost_rows = 0;

    int16 candidate_row = -1;
    int16 sum_white = 0;
    
    if(max_white_value>40)
{
        /*---------- 车身稳定，图像较理想时，条件1：寻找是否连续满足RIGHT_W行阈值范围，并求中间行 ----------*/
    for (int i = START_RIGHT_ANGLE_ROW; i >= END_RIGHT_ANGLE_ROW; i--) 
    { 
        uint8 valid = 1;
        // 检查是否有连续RIGHT_W行满足白点条件
        for (int j = 0; j < RIGHT_W; j++) 
        {
            int current_row = i + j;
            if (current_row >= IMAGE_H) 
            {
                valid = 0;
                break;
            }
            if (white_per_row[current_row] <= RIGHT_ANGLE_WHITE_THRESHOLD_MIN || 
                white_per_row[current_row] >= RIGHT_ANGLE_WHITE_THRESHOLD_MAX) 
            {
                valid = 0;
                break;
            }
        }
        if (valid) 
        {
            // 计算连续行的中间行作为候选行
            candidate_row = i + (RIGHT_W / 2); 
            break; // 找到一个有效区域即退出
        }
    }
}   
else                     
{
    /*---------- 车身不稳定，图像倾斜时，条件1：max_white_row及其上下2行白点值之和大于120且小于500，并将max_white_row作为候选行 ----------*/
    for (int offset = -2; offset <= 2; offset++) 
    {
        int16 row = max_white_row + offset;
        if (row >= 0 && row < IMAGE_H) 
        {
            sum_white += white_per_row[row];
        }
    }
    if (sum_white > 100 && sum_white < 400) 
    {
        candidate_row = max_white_row;
    }
}



    if (candidate_row != -1) 
    {
        /*---------- 条件2：检查上方边界丢失情况 ----------*/
        // 向上遍历BORDER_LOST_ROWS行寻找无效边界
          lost_rows=0;
        for (int16 r = candidate_row-5; r >= candidate_row - BORDER_LOST_ROWS; r--) 
        {
            if (r < 0) break;
            if (l_status[r] == BORDER_NOT_FOUND && r_status[r] == BORDER_NOT_FOUND&&white_per_row[r]<4) 
            {
                lost_rows++;
            }
        }
    }
        
    if(lost_rows>10)
  {
        /*---------- 条件3：全区间白点统计，判断具体直角类型 ----------*/
        int left_white_found = 0, right_white_found = 0;
        // 左侧统计（0 ~ IMAGE_W/2-1）
        for (int16 col = 0; col < IMAGE_W/2; col++) 
        {
            for (int16 row = candidate_row+20; row >= candidate_row-20; row--)
            {
                if (row < 0 || row >= IMAGE_H) continue;
                if (Thresholding_image[row][col] == white)
                {
                    left_white_found++;
                    break; // 每列只计一次
                }
            }
        }
        // 右侧统计（IMAGE_W/2 ~ IMAGE_W-1）
        for (int16 col = IMAGE_W/2; col < IMAGE_W; col++)
        {
            for (int16 row = candidate_row+20; row >= candidate_row-20; row--)
            {
                if (row < 0 || row >= IMAGE_H) continue;
                if (Thresholding_image[row][col] == white) 
                {
                    right_white_found++;
                    break; // 每列只计一次
                }
            }
        }
        // 判断直角类型
        if (right_white_found-left_white_found > right_white_found_threshold)
        {
            rightangle_row = candidate_row;
            right_rightangle_flag = 1;
            left_rightangle_flag = 0;
        } else if (left_white_found-right_white_found > right_white_found_threshold) 
        {
            rightangle_row = candidate_row;
            left_rightangle_flag = 1;
            right_rightangle_flag = 0;
        } else 
        {
            right_rightangle_flag = 0;
            left_rightangle_flag = 0;
        }
  }
}

//float right_gain=-1;            //直角增益
uint8 RIGHT_DRAW_START=0 ;  //限制直角拉线起始行 
uint8 right_qianzhan_base=32;
//直角处理
void right_angle_deal(uint8 start_h)
{
    static uint8 rightangle_laline_enable = 0; // 拉线使能标志
    static int16 last_rightangle_row = -1;     // 上一次直角行


    if (rightangle_row != -1)
    {
        start_angle_flag=1;         //开始角度积分标志位          
        int16 sum_white = 0;
    

        if(max_white_value>40)
    {
            /*---------- 车身稳定，图像较理想时，条件1：寻找是否连续满足RIGHT_W行阈值范围，并求中间行 ----------*/
        for (int i = START_RIGHT_ANGLE_ROW; i >= END_RIGHT_ANGLE_ROW; i--) 
        { 
            uint8 valid = 1;
            // 检查是否有连续RIGHT_W行满足白点条件
            for (int j = 0; j < RIGHT_W; j++) 
            {
                int current_row = i + j;
                if (current_row >= IMAGE_H) 
                {
                    valid = 0;
                    break;
                }
                if (white_per_row[current_row] <= RIGHT_ANGLE_WHITE_THRESHOLD_MIN || 
                    white_per_row[current_row] >= RIGHT_ANGLE_WHITE_THRESHOLD_MAX) 
                {
                    valid = 0;
                    break;
                }
            }
            if (valid) 
            {
                // 计算连续行的中间行作为候选行
                rightangle_row = i + (RIGHT_W / 2); 
                break; // 找到一个有效区域即退出
            }
        }
        
        
       
        
    }   
    else                     
    {
        /*---------- 车身不稳定，图像倾斜时，条件1：max_white_row及其上下2行白点值之和大于120且小于500，并将max_white_row作为候选行 ----------*/
        for (int offset = -2; offset <= 2; offset++) 
        {
            int16 row = max_white_row + offset;
            if (row >= 0 && row < IMAGE_H) 
            {
                sum_white += white_per_row[row];
            }
        }
        if (sum_white > 80 && sum_white < 400) 
        {
            rightangle_row = max_white_row;
        }
    }
///////////////////////动态直角前瞻    
    if(speed <= 1000)
    {
        RIGHT_DRAW_START = right_qianzhan_base;
    } else 
    {
        int16 temp = right_qianzhan_base - ((speed - 1100) / 15);
        if(temp < 5) temp = 5;
        RIGHT_DRAW_START = temp;
    }   
//                     printf("%d\r\n",RIGHT_DRAW_START);                               

///////////////////////动态直角前瞻
    
       
        // 只要未启用拉线，且直角行小于RIGHT_DRAW_START，则只刷新，不拉线
        if(!rightangle_laline_enable && rightangle_row < RIGHT_DRAW_START)
        {
                      // 只允许rightangle_row比上一次大
            if(last_rightangle_row == -1 || rightangle_row > last_rightangle_row)
            {
                last_rightangle_row = rightangle_row;
            }
            else
            {
                rightangle_row = last_rightangle_row;
            }
            return;
        }
//        // 一旦直角行达到或超过IMAGE_H-30，启用拉线，后续不再限制
//        if(rightangle_row >= 30)
//        {
//            rightangle_laline_enable = 1;
//        }   
        
                    // 限制每次搜索到的直角行rightangle_row，只允许直角行比上一次大，防误判。
            if(last_rightangle_row == -1 || rightangle_row > last_rightangle_row)
            {
                last_rightangle_row = rightangle_row;
            }
            else
            {
                rightangle_row = last_rightangle_row;
            }
        
//        right_gain=(rightangle_row-(RIGHT_DRAW_START-2))/20.0f;
        
        
          
        /*----- 连接点1和点2进行拉线 -----*/
        
        //点1坐标（起始行对应的中线坐标）
        int16 point1_col =  border[start_h];//IMAGE_W/2;
        int16 point1_row = start_h-2;
        
        //点2坐标（直角行向上偏移5行）
        int16 point2_row = rightangle_row ;
        int16 point2_col = (left_rightangle_flag) ? 4 : (IMAGE_W - 4); //左直角为2，右直角为IMAGE_W-2

        /*----- 边界插值处理 -----*/
        //计算行范围和列差值
        int16 row_start = MIN(point1_row, point2_row);          //取 point1_row（起始行）和 point2_row（目标行）中的较小值，作为插值的起始行。
        int16 row_end = MAX(point1_row, point2_row);            //无论两点中哪一个行号更小（即更靠近图像顶部），都确保插值从更上方的行开始。
        int16 col_diff = point2_col - point1_col;
        int16 row_diff = point2_row - point1_row;
        
        //避免除零
        if(row_diff != 0) 
        {
            float slope = (float)col_diff / row_diff;           //计算斜率
            
            
            rightangle_laline_enable = 1;//拉线使能
            
            
        /*----- 插值生成新边界 -----*/
            for(int16 r = row_start; r <= row_end; r++)
            {
                int16 interpolated_col = point1_col + (int16)(slope * (r - point1_row));

                if(left_rightangle_flag)
                {
                    l_border[r] = CLAMP(interpolated_col, SEARCH_MIN, SEARCH_MAX);
                    l_status[r] = BORDER_INTERPOLATED;
                                        // 右边界跟随偏移
                    r_border[r] = l_border[r]+ BORDER_OFFSET;
                    r_status[r] = BORDER_INTERPOLATED;
                }
                else if(right_rightangle_flag)
                {
                    r_border[r] = CLAMP(interpolated_col, SEARCH_MIN, SEARCH_MAX);
                    r_status[r] = BORDER_INTERPOLATED;
                                        // 左边界跟随偏移
                    l_border[r] =r_border[r] - BORDER_OFFSET;
                    l_status[r] = BORDER_INTERPOLATED;
                }
            }
        }
   
               for(int16 i = rightangle_row-5 ; i >= 22; i--)       //如果检测到环岛则将上30行置边，以免不入环。
            {
                if(l_status[i]==BORDER_NOT_FOUND&&r_status[i]==BORDER_NOT_FOUND)
                {
                    if(left_rightangle_flag)
                    {
                      r_border[i] = SEARCH_MIN; // 右置边
                    }

                    else
                    {
                      l_border[i] = SEARCH_MAX; // 左置边
                    }


                }
            }
        
         /*----- 向上遍历BORDER_LOST_ROWS行寻找丢线行数 -----*/
            int16 lost_rows = 0;
        for (int16 r = rightangle_row-5; r >= rightangle_row - BORDER_LOST_ROWS; r--) 
        {
            if (r < 0) break;
            if (l_status[r] == BORDER_NOT_FOUND && r_status[r] == BORDER_NOT_FOUND&&white_per_row[r]<6) 
            {
                lost_rows++;
            }
        }
        
        /*----- 退出条件检查 -----*/
        if( lost_rows<10&&func_abs(theta_total) > 20)        //丢线行小于一定值则退出//增加转向角度判断
        {
            rightangle_row = -1;
            last_rightangle_row=-1;
            right_rightangle_flag = 0;
            left_rightangle_flag = 0;
            start_angle_flag=0;         //开始角度积分标志位
            theta_total=0;
            rightangle_laline_enable=0;
//            right_gain=-1;

        }
    }
}

// 断路检测相关定义
#define BREAK_CHECK_START_ROW     1      // 断路检测起始行
#define BREAK_CHECK_END_ROW       80     // 断路检测结束行
#define BREAK_LOST_RATIO          90     // 断路判定丢线比例//95%

BreakState break_state = BREAK_NONE;
uint16_t lost_percent = 0;
uint8 check_break_start=0;
uint8 break_confirm_cnt = 0;
uint8  break_angle_test=0;

void check_break_condition(void) 
{
    if(break_state == BREAK_NONE&&right_rightangle_flag==0&&left_rightangle_flag==0) 
    {
        uint16_t lost_count = 0;
        const uint16_t total_rows = BREAK_CHECK_END_ROW - BREAK_CHECK_START_ROW;    //计算总检测行
        
        for(int i = BREAK_CHECK_START_ROW; i < BREAK_CHECK_END_ROW; i++)            //在范围内寻找丢线行
        {
            if(l_status[i] == BORDER_NOT_FOUND && r_status[i] == BORDER_NOT_FOUND) 
            {
                lost_count++;
            }
        }
              
        
        lost_percent= (lost_count * 100) / total_rows;                               // 返回丢线百分比
      
        if(lost_percent > BREAK_LOST_RATIO)
        {
            break_confirm_cnt ++;
            if(break_confirm_cnt>=2)
            {
                break_state = BREAK_DETECTED;
                ting_check_break=0;
                replay_mode=1;                                                           //启用轨迹回放       
                
                if(break_angle_test)
                  finish_flag=2;
                
            }
        }
        else{break_confirm_cnt=0;}
    }
}

uint8 break_flag=0;
void check_break_condition_test(void) 
{
    if(break_state == BREAK_NONE&&right_rightangle_flag==0&&left_rightangle_flag==0) 
    {
        uint16_t lost_count = 0;
        const uint16_t total_rows = BREAK_CHECK_END_ROW - BREAK_CHECK_START_ROW;    //计算总检测行
        
        for(int i = BREAK_CHECK_START_ROW; i < BREAK_CHECK_END_ROW; i++)            //在范围内寻找丢线行
        {
            if(l_status[i] == BORDER_NOT_FOUND && r_status[i] == BORDER_NOT_FOUND ) 
            {
                lost_count++;
            }
        }
        lost_percent= (lost_count * 100) / total_rows;                               // 返回丢线百分比
      
        if(lost_percent > BREAK_LOST_RATIO)
        {
            break_flag = 1;
        }
        else
        {
            break_flag = 0;           
        }
    }
}

uint8 rest_count=0;
uint8 break_rest_threshold=20;  //断路恢复阈值
uint8 break_rest_enable=1;      //断路恢复巡线使能
void recover_search_line(void)//断路恢复
{
  if(break_rest_enable)
  {
      for(int i = 1; i < 78; i++)            //在范围内寻找丢线行
    {
        if(white_per_row[i]>=4) 
        {
            rest_count++;
        }
    }
      if(rest_count>break_rest_threshold)
      {
          stop_replay_flag=1;
      }
          rest_count=0;
  }
}

#define CHECK_STOP_START_ROW     1      // 断路检测起始行
#define CHECK_STOP_END_ROW       IMAGE_H-5     // 断路检测结束行
#define WHITE_FIND_RATIO         50     // 断路判定丢线比例//95%

uint16_t find_percent = 0;
uint8    stop_confirm_cnt = 0;
void check_stop(void)
{
        uint16_t white_row_count = 0;
        const uint16_t total_rows = CHECK_STOP_END_ROW - CHECK_STOP_START_ROW;    //计算总检测行
        
        for(int i = CHECK_STOP_START_ROW; i < CHECK_STOP_END_ROW; i++)            //在范围内寻找丢线行
        {
            if(white_per_row[i]>IMAGE_W-40) 
            {
                white_row_count++;
            }
        }
        find_percent= (white_row_count * 100) / total_rows;                               // 返回丢线百分比
      
        if(find_percent > WHITE_FIND_RATIO)
        {
            stop_confirm_cnt ++;
            if(stop_confirm_cnt>=1)
            {
                finish_flag = 2;
            }
        }
        else{stop_confirm_cnt=0;} 
}


void beep_sound(void)
{
  if (buzzer_enable)//
  {
            if((island_state != ISLAND_NONE)||(rightangle_row != -1)||(break_state ==BREAK_DETECTED)||straight_flag==1)//
          {
            gpio_set_level(P19_4,1);
          }
            else
          {
            gpio_set_level(P19_4,0);
          }
          
  }
}



#define ZHONGDIAN_WHITE_THRESHOLD_MAX  95     // 终点白点数量最大阈值
#define ZHONGDIAN_WHITE_THRESHOLD_MIN  60     // 终点白点数量最小阈值
#define START_CHECK_ZHONGDIAN          55     //终点检测开始行（从下往上）
#define END_CHECK_ZHONGDIAN            30     //终点检测结束行
#define ZHONGDIAN_W                    12     // 终点厚度

#define ZHONGDIAN_CHECK_COLS_LEFT     {2,5,9}                            // 左侧检测列
#define ZHONGDIAN_CHECK_COLS_RIGHT    {IMAGE_W-2, IMAGE_W-5, IMAGE_W-9}  // 右侧检测列

uint8 finish_flag = 0; 

void check_zhongdian(void)              //终点检测
{
        /*---------- 条件1：满足终点白点条件并寻找连续行ZHONGDIAN_W的中间行 ----------*/
    int16 zhongdian_row = -1;           //终点行
    for (int i = START_CHECK_ZHONGDIAN; i>= END_CHECK_ZHONGDIAN; i--) 
    { 
        uint8 valid = 1;
        // 检查是否有连续RIGHT_W行满足白点条件
        for (int j = 0; j < ZHONGDIAN_W; j++) 
        {
            int current_row = i + j;
            if (current_row >= IMAGE_H) 
            {
                valid = 0;
                break;
            }
            if (white_per_row[i + j] <= ZHONGDIAN_WHITE_THRESHOLD_MIN||
                white_per_row[i + j] >= ZHONGDIAN_WHITE_THRESHOLD_MAX)
            {
                valid = 0;
                break;
            }
        }
        if (valid) 
        {
            zhongdian_row = i + (RIGHT_W / 2);        // 计算连续行的中间行作为候选行
            break;                                   // 找到第一个有效区域即退出
        }
    }

    if (zhongdian_row != -1) 
    {
        /*---------- 条件2：检查终点行左右列白点情况 ----------*/

          const int16 left_cols[] = ZHONGDIAN_CHECK_COLS_LEFT;
          const int16 right_cols[] = ZHONGDIAN_CHECK_COLS_RIGHT;
          const uint8 CHECK_COLS_CNT = sizeof(left_cols)/sizeof(left_cols[0]);            //计算检测行数组 left_cols 的实际元素个数

          uint8 left_jump_found = 0, right_jump_found = 0;
      
                // 扫描左侧检测列
          for (uint8 i = 0; i < CHECK_COLS_CNT; i++) 
          {
              int16 col_l = left_cols[i];
              for (int16 row = zhongdian_row+20; row >= zhongdian_row-20; row--) 
              {
                  if (row < 0 || row >= IMAGE_H) continue;
                  // 检查连续两个跳变点
                  if (Thresholding_image[row][col_l] == white )
                  {
                      left_jump_found = 1;
                      break;
                  }
              }
              if (left_jump_found) break;
          }

          // 扫描右侧检测列
          for (uint8 i = 0; i < CHECK_COLS_CNT; i++)
          {
              int16 col_r = right_cols[i];
              for (int16 row = zhongdian_row+20; row >= zhongdian_row-20; row--) 
              {
                  if (row < 0 || row >= IMAGE_H) continue;
                  // 检查连续两个跳变点
                  if (Thresholding_image[row][col_r] == white)
                  {
                      right_jump_found = 1;
                      break;
                  }
              }
              if (right_jump_found) break;
          }
          if(!(left_jump_found&&right_jump_found))
          {
                finish_flag++;
                tings=0;                //重置计时，防止连续判定
          }
        zhongdian_row=-1;
    }
} 

// 保存白点值到Flash
void save_white_values_to_flash(void)
{
    WhiteValueStorage storage;
    storage.flag = WHITE_VALUE_FLAG;
    for(int i = 0; i < IMAGE_H; i++)                                      // 复制当前白点值到存储结构体
    {
        storage.normal_white_values[i] = normal_white_per_row[i];
    }
    flash_buffer_clear();                                                  // 清空缓冲区
    memcpy(flash_union_buffer, &storage, sizeof(storage));                // 将数据复制到缓冲区
    flash_write_page_from_buffer(0, WHITE_VALUE_PAGE, sizeof(storage)/4); // 从缓冲区写入Flash
}

// 从Flash读取白点值
uint8 load_white_values_from_flash(void)
{
    WhiteValueStorage storage;
    flash_read_page_to_buffer(0, WHITE_VALUE_PAGE, sizeof(storage)/4);  // 从Flash读取到缓冲区
    memcpy(&storage, flash_union_buffer, sizeof(storage));              // 从缓冲区复制数据到结构体
    if(storage.flag != WHITE_VALUE_FLAG)                                // 检查魔数是否正确
    {
        return 0;  // 数据无效
    }
    for(int i = 0; i < IMAGE_H; i++)                                    // 复制白点值到正常白点值数组
    {
        normal_white_per_row[i] = storage.normal_white_values[i];
    }
        return 1;  // 数据有效
}

// 在菜单中调用保存当前白点值
void save_current_white_values(void)
{
    // 将当前白点值复制到正常白点值数组
    for(int i = 0; i < IMAGE_H; i++)
    {
        normal_white_per_row[i] = white_per_row[i];
    }
    save_white_values_to_flash();   // 保存到Flash
}



// 保存摄像头参数到Flash
void save_camera_params_to_flash(void)
{
    CameraParamsStorage storage;
    storage.flag = CAMERA_PARAMS_FLAG;
    storage.exp_time = MT9V03X_EXP_TIME_DEF;
    storage.fps = MT9V03X_FPS_DEF;
    storage.otsu_limit = otsu_limit;
    storage.break_number = break_number; // 新增字段赋值    
    
    flash_buffer_clear();
    for(int i = 0; i < sizeof(storage)/4; i++)
    {
        flash_union_buffer[i].uint32_type = ((uint32_t*)&storage)[i];
    }
    flash_write_page_from_buffer(0, CAMERA_PARAMS_PAGE, sizeof(storage)/4);
}

// 从Flash读取摄像头参数
uint8 load_camera_params_from_flash(void)
{
    CameraParamsStorage storage;
    flash_read_page_to_buffer(0, CAMERA_PARAMS_PAGE, sizeof(storage)/4);
    for(int i = 0; i < sizeof(storage)/4; i++)
    {
        ((uint32_t*)&storage)[i] = flash_union_buffer[i].uint32_type;
    }
    
    if(storage.flag != CAMERA_PARAMS_FLAG)
    {
        MT9V03X_EXP_TIME_DEF = 350;
        MT9V03X_FPS_DEF = 100;
        otsu_limit = 130; // 读取失败设为默认值
        break_number = 1; // 读取失败时设为默认最大断路数
        return 0;  // 数据无效
    }
    MT9V03X_EXP_TIME_DEF = storage.exp_time;
    MT9V03X_FPS_DEF = storage.fps;
    otsu_limit = storage.otsu_limit;
    break_number = storage.break_number; // 新增字段赋值
    
    return 1;  // 数据有效
}





////////////偏差滤波////////////
#define    max_scale_factor      8.0f                     //偏差最大按有效行比例放大倍数
#define    FILTER_LIMIT          650                      //限幅滤波阈值
#define    FILTER_ALPHA          0.95f                    //低通滤波系数(0-1)，越大表示新数据权重越大

int16 image_position[IMAGE_H];                           //搜索的中线与摄像头中线的偏差值
int16 mtv9_position=0, mtv9_position_old=0;
uint16 valid_rows = 0;                              //有效行行数

// 限幅低通滤波器
int16 limit_filter(int16 new_value, int16 old_value) 
{
    // 限幅滤波
    if(func_abs(new_value - old_value) > FILTER_LIMIT) 
    {
        if(new_value > old_value) {
            new_value = old_value + FILTER_LIMIT;
        } else {
            new_value = old_value - FILTER_LIMIT;
        }
    }
    // 低通滤波
    return (int16)(FILTER_ALPHA * new_value + (1-FILTER_ALPHA) * old_value);
}

void calc_corner(uint8 start_row, uint8 end_row)        //偏差计算
{
    int16 i;
    int32 posi = 0; // 改用32位整数存储累加值
    valid_rows = 0;
    const int16 total_rows = start_row - end_row + 1; //搜线行数
    
    for (i = IMAGE_H-1; i >= 0; i--)
    {
          border[i] = (l_border[i] + r_border[i]) / 2;          //计算中线
    }
    for (i = start_row; i >= end_row; i--)
    {
        bool is_forced = (l_border[i] == SEARCH_MIN && r_border[i] == SEARCH_MAX); // 检查是否被置边
        if(!is_forced) 
        {
            image_position[i] = IMAGE_W / 2 - border[i];
            // 根据行号范围分配不同的权重
            uint16 weight;
            if (i <= (IMAGE_H - 20)&&i > (IMAGE_H - 30))    //[IMAGE_H - 30到IMAGE_H - 20]即50到60行
            {           
                weight = 110;                    // 1.3
            } else if (i < (IMAGE_H - 30)&&i > (IMAGE_H - 40) )
            {    
                weight = 110;                    // 1.2
            } else if (i < (IMAGE_H - 40)&&i > (IMAGE_H - 50) )
            {   
                weight = 110;                    // 1.1
            } else if (i <(IMAGE_H - 50)&&i > (IMAGE_H - 60) )
            {    
                weight = 110;                    // 1.0
            }
            else 
            {                             // 其余区域
                weight = 80;                     // 0.8
            }
                posi += image_position[i] * weight;
                valid_rows++;
        }
    }
   
    // 比例因子计算（整数运算）
    uint16 scale_factor = 100; // 1.0的缩放表示
    if (valid_rows > 0)
    {
        scale_factor = (total_rows * 100) / valid_rows; 
        
        
        scale_factor = MIN(scale_factor,(uint16)(max_scale_factor * 100));
    }
    else
    {
        posi = 0; // 无有效行时置零
    }
    posi = (posi * scale_factor) / (100 * 100); // 两次缩放补偿
    
    // 对计算出的偏差进行滤波处理
    int16 raw_position = (int16)(posi / 5);
//    if(right_gain!=-1){raw_position=(int16)(raw_position*(1+right_gain));}
    raw_position = (int16)func_limit(raw_position,900);
//    mtv9_position = limit_filter(raw_position, mtv9_position_old);
    mtv9_position =raw_position ;
    mtv9_position_old = mtv9_position;
    
//    printf("%d,%d\r\n", raw_position,mtv9_position); //打印偏差    

}




uint8 straight_flag=0;
void straight_detection_check(void)
{
    static uint8 straight_cnt = 0;
//    static uint8 straight_timeout = 0;
    #define STRAIGHT_CONFIRM_CNT  5   // 连续成立次数阈值
//    #define STRAIGHT_TIMEOUT_MAX  20  // 超时最大次数

    if(func_abs(mtv9_position)<100 && max_white_value<12 &&
       (right_rightangle_flag==0 && left_rightangle_flag==0) &&
       (break_state ==BREAK_NONE) && valid_rows>(IMAGE_H-10) &&
       (island_state==ISLAND_NONE))
    {
        straight_cnt++;
//        straight_timeout = 0;
        if(straight_cnt >= STRAIGHT_CONFIRM_CNT)
        {
            straight_flag = 1;
        }
    }
    else
    {
        straight_cnt = 0;
//        straight_timeout++;
//        if(straight_timeout > STRAIGHT_TIMEOUT_MAX)
//        {
             straight_flag = 0;
//            straight_timeout = 0;
//        }
    }
}

#define IMAGE_CUT_LEFT_threshold      0        //大津左裁剪大小
#define IMAGE_CUT_RIGHT_threshold     0        //大津右裁剪大小
#define IMAGE_CUT_UP_threshold        0        //大津上裁剪大小
#define IMAGE_CUT_DOWN_threshold      0        //大津下裁剪大小
//必须输入原始图像
uint8 my_adapt_threshold(uint8 *image, uint16 original_width, uint16 original_height) 
{
    #define GrayScale 256
    // 裁剪参数（与二值化范围一致）
    const uint16 start_row = IMAGE_CUT_UP_threshold;
    const uint16 end_row = original_height - IMAGE_CUT_DOWN_threshold - 1;
    const uint16 start_col = IMAGE_CUT_LEFT_threshold;
    const uint16 end_col = original_width - IMAGE_CUT_RIGHT_threshold - 1;

    // 采样后的尺寸计算（每隔2像素采样）
    const uint16 sampled_height = (end_row - start_row + 1) / 2;
    const uint16 sampled_width = (end_col - start_col + 1) / 2;
    uint32 pixelSum = sampled_height * sampled_width;

    int pixelCount[GrayScale] = {0};
    float pixelPro[GrayScale] = {0};
    uint32 gray_sum = 0;

    // 遍历裁剪区域并降采样
    for (uint16 i = start_row; i <= end_row; i += 2) 
    {
        for (uint16 j = start_col; j <= end_col; j += 2) 
        {
            uint8 pixel = image[i * original_width + j];
            pixelCount[pixel]++;
            gray_sum += pixel;
        }
    }
    // 计算概率分布
    for (int i = 0; i < GrayScale; i++) 
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    // 大津法计算阈值
    float deltaMax = 0;
    uint8 threshold = 0;
    float w0 = 0, u0 = 0;

    for (int t = 0; t < GrayScale; t++) 
    {
        w0 += pixelPro[t];
        float u0_tmp = u0 + t * pixelPro[t];
        float u1_tmp = (gray_sum / (float)pixelSum) - u0_tmp;
        float w1 = 1 - w0;

        if (w0 == 0 || w1 == 0) continue;

        u0 = u0_tmp / w0;
        float u1 = u1_tmp / w1;
        float delta = w0 * w1 * (u1 - u0) * (u1 - u0);

        if (delta > deltaMax) 
        {
            deltaMax = delta;
            threshold = t;
        }
    }

    return threshold;
}

// 固定阈值二值化
uint8 Thresholding_image[IMAGE_H][IMAGE_W];  //二值化图像数组
uint16 max_white_value;                      // 最大白点值
uint8 max_white_row;                         // 白点最多的行号
uint16 white_per_row[IMAGE_H] = {0};          // 存储每行的白点数量

// 添加变量定义
uint16 max_white_value_upper = 0;
uint16 max_white_value_lower = 0;
uint8 max_row_upper = 0;
uint8 max_row_lower = 0;

// 修改 image_Thresholding 函数
void image_Thresholding(uint8 value) 
{
    max_white_value = 0;
    max_white_row = 0;
    max_white_value_upper = 0;   // 初始化上半部分最大值
    max_white_value_lower = 0;   // 初始化下半部分最大值
    max_row_upper = 0;           // 初始化行号
    max_row_lower = 0;

    for(uint8 i = IMAGE_CUT_UP; i < MT9V03X_H-IMAGE_CUT_DOWN; i++) 
    {
        uint16 white_count = 0;              
        uint8* src_row = mt9v03x_image[i];
        uint8* dst_row = Thresholding_image[i-IMAGE_CUT_UP];
        
        for(uint8 j = IMAGE_CUT_LEFT; j < MT9V03X_W-IMAGE_CUT_RIGHT; j++)
        {
            const uint8 pixel = src_row[j];
            const uint8 binary = (pixel < value) ? 0 : 255;
            dst_row[j-IMAGE_CUT_LEFT] = binary;              
            white_count += (binary >> 7);                    
        }
      
        white_per_row[i-IMAGE_CUT_UP] = white_count;                      
        uint8 current_row = i - IMAGE_CUT_UP;  // 计算处理后的行号

        // 分区域统计最大值
        if (current_row < (IMAGE_H - 40)) 
        {
            // 上半部分（0 到 IMAGE_H-40-1）
            if (white_count > max_white_value_upper) 
            {
                max_white_value_upper = white_count;
                max_row_upper = current_row;
            }
        } 
        else 
        {
            // 下半部分（IMAGE_H-40到 IMAGE_H-1）
            if (white_count > max_white_value_lower) 
            {
                max_white_value_lower = white_count;
                max_row_lower = current_row;
            }
        }

        // 更新全局最大值
        if (white_count > max_white_value) 
        {
            max_white_value = white_count;
            max_white_row = current_row;
        }
    }  
}

uint16 ting_check_break=0;
uint16 tings=0;
uint16 ting=0;
uint16 ting_circle=0;
uint8  stop_flag=0;
uint8  otsu_limit=130;
uint16 normal_white_per_row[IMAGE_H] = {0};  // 直道正常时的白点值数组

void image_processing( )//图像处理流程
{
///////////这里大津计算出来的阈值有问题，算出来的阈值很低很低，最终还是靠限幅限住的（相当于固定阈值了），也是导致没完赛的主要原因。
              uint16 temp_otsu=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W,MT9V03X_H);           //大津法获取阈值
              uint16 otsu=func_limit_ab(temp_otsu,otsu_limit,255);                                 //对阈值限幅             
//              ips200_show_int(200,0,otsu,3);                                                     //显示阈值
              image_Thresholding(otsu);                                                            //二值化
              search_border();                                                                     //搜线（带虚线处理）
              //    cross_check( );                                                                //十字检测
              if(right_rightangle_flag==0&&left_rightangle_flag==0&&(island_state==ISLAND_NONE||island_state==ISLAND_POST_PROCESS))
              {
                  detect_right_angle();                              // 直角检测   
                  circle_check();                                    //环岛检测
               }
              
                  right_angle_deal(start_h);                         //直角处理
                  island_deal();                                     // 环岛处理
                  calc_corner(start_h,1);                            //偏差计算
              
//              if(tings>1000&&(right_rightangle_flag==0&&left_rightangle_flag==0&&island_state==ISLAND_NONE))
//              {
//                   check_zhongdian( );                                //终点检测
//              }
                 check_stop();
                 straight_detection_check();                    //直道检测
                 beep_sound();
              if(check_break_start&&right_rightangle_flag==0&&left_rightangle_flag==0&&break_state ==BREAK_NONE&&ting_check_break>200&&island_state==ISLAND_NONE)
              {
                  check_break_condition();                            //断路检测            //增加一个断路无效判定，当检测到断路开始计数若计数值小于一个阈值则判定为上个断路无效，将自增的current_replay_page减回来，但当只有一个断路时注意逻辑！
              }
              else
              {
                  check_break_condition_test();                       //发车前断路测试
              }
              if(break_state == BREAK_DETECTED)                      
              {
                  recover_search_line();                              //当若干行白点值大于阈值后结束回放，回到正常巡线
              }           
}



/*日期              作者                QQ
2025-8-22            ZQ            2897424468
*/


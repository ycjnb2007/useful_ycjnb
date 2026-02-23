#include "zf_common_headfile.h"

/**
函数简介：元素行和元素列

示例    :row_col_element_find();
返回   
**/
void row_col_element_find(void)
{
  
    up_row_num = Element_row_up();
    down_row_num = Element_row_down();
    left_col_num = Element_col_left();
    right_col_num = Element_col_right();  
    
    up_row_num1 = Element_row_up1();
    down_row_num1 = Element_row_down1();
    left_col_num1 = Element_col_left1();
    right_col_num1= Element_col_right1();    

}



/**
函数简介：Bresenham 直线算法，记录从 (x0, y0) 到 (x1, y1) 的所有点

示例    :uint16 num_points = bresenham_line(line_points, x_start, y_start, X_meet, Y_meet);
返回   ：返回实际存储的点数
**/
Point dotted_line_points[MAX_LINE_NUM];
Point line_points[MAX_LINE_NUM]; 
uint16 dotted_num_points;
uint16 num_points;
uint16 Bresenham_line(Point *points, uint8 x0, uint8 y0, uint8 x1, uint8 y1) 
{
    int16 dx = abs((int16_t)x1 - (int16_t)x0);
    int16 dy = -abs((int16_t)y1 - (int16_t)y0);
    int16 sx = (x0 < x1) ? 1 : -1;
    int16 sy = (y0 < y1) ? 1 : -1;
    int16 err = dx + dy;
    uint16 count = 0;

    while (1) {
        // 检查是否在图像范围内
        if (x0 < RIGHT_COL_LIMIT && y0 < DOWN_ROW_LIMIT) {
            points[count].x = x0;
            points[count].y = y0;
            count++;
        }

        // 到达终点则退出
        if (x0 == x1 && y0 == y1) break;

        int16_t e2 = 2 * err;
        if (e2 >= dy) { // 水平步进
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) { // 垂直步进
            err += dx;
            y0 += sy;
        }
    }

    return count;
}

/**
 * @brief 生成动态权重数组（中间偏上权重最高）
 * @param num_points 当前点数（40-200）
 * @param out_weights 输出权重数组（需预分配空间）
 */
float peak_center_use = 0.3f;
void generate_dynamic_weights(uint16 num_points, float out_weights[]) 
{
    const float peak_center = peak_center_use;  // 权重峰值位置（0.6=中间偏上）
    const float peak_width = 0.25f;    // 高权重区域宽度
    const float min_weight = 0.2f;    // 最小权重值
    const float max_weight = 1.0f;    // 最大权重值

    for (uint16 i = 0; i < num_points; i++) 
    {
        // 归一化位置：0=最下方（近端），1=最上方（远端）
        float normalized_pos = (float)i / (num_points - 1);
        // 高斯衰减计算（反向填充，确保line_points[0]对应最后一行）
        uint16 reversed_idx = num_points - 1 - i;
        float distance_weight_calculation = fabsf(normalized_pos - peak_center);
        
        if (distance_weight_calculation <= peak_width / 2) 
        {
            out_weights[reversed_idx] = max_weight;  // 核心区域最高权重
        } else 
        {
            // 边缘区域平滑衰减
            float falloff = (distance_weight_calculation - peak_width/2) / (0.5f - peak_width/2);
            out_weights[reversed_idx] = max_weight - (max_weight - min_weight) * falloff;
            out_weights[reversed_idx] = fmaxf(out_weights[reversed_idx], min_weight);
        }
    }
}






/**
 * @brief 计算加权中点坐标
 * @param line_points 中线点数组（line_points[0]对应最下方行）
 * @param num_points 当前点数
 * @return 加权中点坐标（若计算失败返回默认中点）
 */
float Mid_point;
float calculate_weighted_midpoint(Point line_points[], uint16 num_points) 
{
    static float dynamic_weights[MAX_LINE_NUM];  // 静态内存分配
    generate_dynamic_weights(num_points, dynamic_weights);

    float weight_sum = 0.0f, weighted_sum = 0.0f;
    for (uint16 i = 0; i < num_points; i++) 
    {
        weighted_sum += line_points[i].x * dynamic_weights[i];
        weight_sum += dynamic_weights[i];
    }

    // 防零除保护，失败时返回几何中点
    return (weight_sum > 1e-6f) ? (weighted_sum / weight_sum) : 
                                 (line_points[num_points / 2].x);
}

/**
函数：元素行和元素列检测

**/

/**左侧列——优化1**/
uint8 left_col_num = 0;
Point left_col_count[point_count]; 
uint8 Element_col_left(void) 
{
    uint8 total_count = 0;     
    
    for (uint8 i = col_start_line; i < col_start_line + many_count && total_count < point_count; i+= 2) 
    {
        uint8 col_jump_count = 0;
        Point temp_points[point_count]; 

        // 1. 扫描当前列跳变点（从下向上扫描）
        for (uint8 j = row_last_line; j > row_start_line+ 1; j--) 
        {
            if (Trip_point(image[j][i], image[j-1][i])) 
            {
                temp_points[col_jump_count].x = i;
                temp_points[col_jump_count].y = j;
                col_jump_count++;
                if (col_jump_count >= point_count) break;  // 防止临时数组溢出
            }
        }

        // 2. 严格偶数校验：奇数跳变点直接丢弃
        if (col_jump_count % 2 != 0 || col_jump_count == 0) 
        {
            continue; 
        }

        // 3. 距离校验（垂直方向）
        bool is_valid = true;
        for (uint8 k = 0; k < col_jump_count; k += 2) 
        {
            if (abs(temp_points[k].y - temp_points[k+1].y) > MAX_DISTANCE) 
            {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) continue;

        // 4. 保存有效数据
        for (uint8 k = 0; k < col_jump_count; k++) 
        {
            left_col_count[total_count] = temp_points[k];
            total_count++;
            if (total_count >= point_count) break;  // 防止结果数组溢出
        }

        // 5. 找到有效列后立即退出（可选）
        if (total_count > 0) break; 
    }
    return total_count; 
}

//左侧列单纯数
uint8 left_col_num1 = 0;
uint8 Element_col_left1(void)
{
  uint8 count = 0;
  for(uint8 i = col_start_line; i < col_start_line + many_count1;i+=2)
  {
    for(uint8 j = row_last_line; j > row_start_line + 1; j--)
    {
      if(Trip_point(image[j][i],image[j - 1][i]))
      {

        count++;
      }
    }
    
    if(count > 0)
    {
      break;
    }
  }
  
  return count;
}



////右侧列
//Point right_col_count[point_count];
//uint8 Element_col_right(void)
//{
//    uint8 count = 0;
//    for(uint8 i = col_last_line; i < col_last_line- many_count; i--)
//    {
//      for(uint8 j = row_last_line; i < row_start_line + 1; i--)
//      {
//        if(Trip_point(image[j][i],image[j - 1][i]))
//        {
//          right_col_count[count].x = i;
//          right_col_count[count].y = j;
//          count++;
//        }
//      }
//    }
//   
//    return count;
//}

/**右侧列——优化1**/
uint8 right_col_num = 0;
Point right_col_count[point_count];
uint8 Element_col_right(void) 
{
    uint8 total_count = 0;
    
    for (uint8 i = col_last_line; i > col_last_line - many_count && total_count < point_count; i-= 2) 
    {
        uint8 col_jump_count = 0;
        Point temp_points[point_count];

        // 1. 扫描当前列跳变点（从下向上扫描）
        for (uint8 j = row_last_line; j > row_start_line+ 1; j--) 
        {
            if (Trip_point(image[j][i], image[j-1][i])) 
            {
                temp_points[col_jump_count].x = i;
                temp_points[col_jump_count].y = j;
                col_jump_count++;
                if (col_jump_count >= point_count) break;  // 防止临时数组溢出
            }
        }

        // 2. 严格偶数校验：奇数跳变点直接丢弃
        if (col_jump_count % 2 != 0 || col_jump_count == 0) 
        {
            continue;
        }

        // 3. 距离校验（垂直方向）
        bool is_valid = true;
        for (uint8 k = 0; k < col_jump_count; k += 2) 
        {
            if (abs(temp_points[k].y - temp_points[k+1].y) > MAX_DISTANCE) 
            {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) continue;

        // 4. 保存有效数据
        for (uint8 k = 0; k < col_jump_count; k++) 
        {
            right_col_count[total_count] = temp_points[k];
            total_count++;
            if (total_count >= point_count) break;  // 防止结果数组溢出
        }

        // 5. 找到有效列后立即退出（可选）
        if (total_count > 0) break;
    }
    return total_count;
}

//右侧列单纯数
uint8 right_col_num1 = 0;
uint8 Element_col_right1(void)
{
  uint8 count = 0;
  for(uint8 i = col_last_line; i > col_last_line - many_count1; i-= 2)
  {
    for(uint8 j = row_last_line; j > row_start_line + 1; j--)
    {
      if(Trip_point(image[j][i],image[j - 1][i]))
      {

        count++;
      }
    }
    
    if(count > 0)
    {
      break;
    }
  }
  
  return count;
}



/**上方行——优化1**/
uint8 up_row_num = 0;
Point up_row_count[point_count]; 
uint8 Element_row_up(void) 
{
    uint8 total_count = 0;     
    
    for (uint8 i = row_start_line; i < row_start_line + many_count && total_count < point_count; i+= 2) 
    {
        uint8 row_jump_count = 0;
        Point temp_points[point_count]; 

        // 1. 扫描当前行跳变点
        for (uint8 j = col_start_line; j < col_last_line - 1; j++) 
        {
            if (Trip_point(image[i][j], image[i][j + 1])) 
            {
                temp_points[row_jump_count].x = j;
                temp_points[row_jump_count].y = i;
                row_jump_count++;
                // 防止数组越界
                if (row_jump_count >= point_count) break;
            }
        }

        // 2. 严格偶数校验：奇数跳变点直接丢弃整行
        if (row_jump_count % 2 != 0 || row_jump_count == 0) 
        {
            continue; // 跳过当前行
        }

        // 3. 距离校验（可选）
        bool is_valid = true;
        for (uint8 k = 0; k < row_jump_count; k += 2) 
        {
            if (abs(temp_points[k].x - temp_points[k+1].x) > MAX_DISTANCE) 
            {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) continue;

        // 4. 保存有效数据
        for (uint8 k = 0; k < row_jump_count && total_count < point_count; k++) 
        {
            up_row_count[total_count] = temp_points[k];
            total_count++;
        }

        // 5. 找到有效行后立即退出（根据需求可选）
        break; 
    }
    return total_count; 
}

//上方行单纯数
uint8 up_row_num1 = 0;
uint8 Element_row_up1(void) 
{
    uint8 count = 0;
    for(uint8 i = row_start_line; i < row_start_line + many_count1; i+=2)
    {
      for(uint8 j = col_start_line; j < col_last_line- 1;j++)
      {
         if(Trip_point(image[i][j], image[i][j + 1]))
         {
            count++;
         }
      }
      
      if(count > 0)
      {
        break;
      }
    }

    return count;
}





/**下方行——优化1**/
uint8 down_row_num = 0;
Point down_row_count[point_count]; 
uint8 Element_row_down(void) 
{
    uint8 total_count = 0;     
    
    for (uint8 i = row_last_line; i > row_last_line - many_count && total_count < point_count; i-=2) 
    {
        uint8 row_jump_count = 0;
        Point temp_points[point_count]; 

        // 1. 扫描当前行跳变点（从下向上扫描）
        for (uint8 j = col_start_line; j < col_last_line - 1; j++) 
        {
            if (Trip_point(image[i][j], image[i][j + 1])) 
            {
                temp_points[row_jump_count].x = j;
                temp_points[row_jump_count].y = i;
                row_jump_count++;
                if (row_jump_count >= point_count) break;  // 防止临时数组溢出
            }
        }

        // 2. 严格偶数校验：奇数跳变点直接丢弃整行
        if (row_jump_count % 2 != 0 || row_jump_count == 0) 
        {
            continue; 
        }

        // 3. 距离校验（仅校验成对跳变点）
        bool is_valid = true;
        for (uint8 k = 0; k < row_jump_count; k += 2) 
        {
            if (abs(temp_points[k].x - temp_points[k+1].x) > MAX_DISTANCE) 
            {
                is_valid = false;
                break;
            }
        }
        if (!is_valid) continue;

        // 4. 保存有效数据
        for (uint8 k = 0; k < row_jump_count; k++) 
        {
            down_row_count[total_count] = temp_points[k];
            total_count++;
            if (total_count >= point_count) break;  // 防止结果数组溢出
        }

        // 5. 找到有效行后立即退出（可选）
        if (total_count > 0) break; 
    }
    return total_count; 
}


//下方行单纯数
uint8 down_row_num1 = 0;
uint8 Element_row_down1(void) 
{
    uint8 count = 0;
    for(uint8 i = row_last_line; i > row_last_line - many_count1; i-=2 )
    {
      for(uint8 j = col_start_line; j < col_last_line- 1;j++)
      {
         if(Trip_point(image[i][j], image[i][j + 1]))
         {
            count++;
         }
      }
      
      if(count > 0)
      {
        break;
      }
    }

    return count;
}



#include "zf_common_headfile.h"

TrackingState Trackstate = LINE_TRACKING;


/**
*函数：巡线状态的确定
*
*返回：巡线状态
*/
TrackingState determine_tracking_state(void) 
{
    //每次判断先置当前为有线状态
    
    // ------------------------------
    // 优先级 1: 丢线检测
    // ------------------------------
    // 调用起始点检测函数（返回 0 表示未找到起始点）
    uint8_t start_point_result = Get_Start_Point(CUT_HEIGHT - 2, image, L_start_point, R_start_point);
#ifdef ENABLE_BREAK
    if (start_point_result == 0) 
    {
        // 连续未检测到起始点的帧数增加，超过阈值则判定为丢线
        if (wuxiao_start_point_count ++ >= ROAD_LOST_THRESHOLD) 
        {   
            if(roundabout_dir == 0)
            {
              current_has_line = false;   //只有在断路区才置当前无引导线状态
              return LOST_LINE;            
            
            }
        }
    } else 
    {
        // 检测到起始点，重置计数器
        wuxiao_start_point_count = 0;
    }
#endif    
    // ------------------------------
    // 优先级 2: 环岛检测
    // ------------------------------
    // 调用环岛检测函数（返回 0 表示未检测到环岛）
    uint8 new_dir = Detect_roundabout();
    Update_Roundabout_Dir(new_dir);
    if (roundabout_dir != 0) 
    {
        Pre_turn_roundabout_flag = 0;   //确定入环后把标志位置0
        current_has_line = true;
        return ROUNDABOUT;
    }  
    
#ifdef ENABLE_CROSS
    if(up_row_num1 == 2 && down_row_num1 == 2 && left_col_num1 == 2 && right_col_num1 == 2)
      {
            return CROSS;

      }

#endif
    
    
    // ------------------------------
    // 默认状态: 正常巡线
    // ------------------------------
    return LINE_TRACKING;
}


/**
*函数：正常寻迹的实现
*
*返回：
*/
void Line_track_solution(void)
{
      current_has_line = true;
      gpio_set_level(Buzzer,0);
     //起始点的中点
      x_start = (L_start_point[0] + R_start_point[0]) / 2;
      y_start = (L_start_point[1] + R_start_point[1]) / 2; 
      /******************************寻找左右边线*******************************/      
     //迷宫法爬边线来提取边线点
      Dir_Labyrinth_5(MAX_LINE,     image,
                      L_line,       R_line,
                      l_dir,        r_dir,
                      &L_Statics,   &R_Statics,
                      &X_meet,      &Y_meet,
                      L_start_point[0],L_start_point[1],
                      R_start_point[0],R_start_point[1],0);
      
      /******************************拟合中线*******************************/ 
      uint8 pre_flag_loop = Pre_roundabout_turn();      //获取入环前的标志位（防止入环前的正常寻迹不稳定）
      Pre_turn_flag_update(pre_flag_loop);              //更新标志位（锁存）        
      
      // 默认使用正常起点
      uint16 draw_start_x = x_start;
      uint16 draw_start_y = y_start;
      uint16 line_color = COLOR_NORMAL;
      bool is_roundabout_path = false;  // 标记是否使用环岛路径
      
      switch(Pre_turn_roundabout_flag)
      {
      case 1://左侧预入环
          if(Pre_left_roundabout_state())
          {
            draw_start_x = X_roundabout_start;
            draw_start_y = Y_roundabout_start;
            line_color = PRE_COLOR_ROUNDABOUT;
            is_roundabout_path = true;
            gpio_set_level(Buzzer, 1);  // 触发蜂鸣器
          
          }
        break;
      case 2://右侧预入环
        
          if(Pre_right_roundabout_state())
          {
            draw_start_x = X_roundabout_start;
            draw_start_y = Y_roundabout_start;
            line_color = PRE_COLOR_ROUNDABOUT;
            is_roundabout_path = true;
            gpio_set_level(Buzzer, 1);  // 触发蜂鸣器
          
          }             
        break;

      }
      
    // 当不是环岛路径时，确保使用正常起点
    if (!is_roundabout_path) 
    {
        draw_start_x = x_start;
        draw_start_y = y_start;
    }      
      
      
      // 统一生成并绘制拟合中线
    num_points = Bresenham_line(line_points, draw_start_x, draw_start_y, X_meet, Y_meet);
      
      
         
    if (start % 2 == 0) 
    {
        for (uint8 i = 0; i < num_points; i++) 
        {
            ips114_draw_point(line_points[i].x, line_points[i].y, line_color);
        }
    }

    /******************************权重求中点*******************************/ 
       Mid_point = calculate_weighted_midpoint(line_points, num_points);
       //更新巡线状态
      last_frame_has_line = true;
}

/**
*函数：六边形环岛的实现
*
*返回：
*/
void roundabout_solution(void)
{
      gpio_set_level(Buzzer,1);
   
     //起始点的中点
      x_start = (L_start_point[0] + R_start_point[0]) / 2;
      y_start = (L_start_point[1] + R_start_point[1]) / 2; 
      /******************************寻找左右边线*******************************/      
     //迷宫法爬边线来提取边线点
      Dir_Labyrinth_5(MAX_LINE,     image,
                      L_line,       R_line,
                      l_dir,        r_dir,
                      &L_Statics,   &R_Statics,
                      &X_meet,      &Y_meet,
                      L_start_point[0],L_start_point[1],
                      R_start_point[0],R_start_point[1],0);  
  
    /******************************拟合中线*******************************/     
    roundabout_num_points = Loop_tarck(); 
    if( start % 2 == 0)
    {     
        for(uint8 i = 0;i < roundabout_num_points;i++)
        {
          ips114_draw_point(roundabout_line_points[i].x,roundabout_line_points[i].y,RGB565_BLUE);//显示在六边形环岛时拟合出的中线
        }
    }
    
    /******************************权重求中点*******************************/     
    Mid_point = calculate_weighted_midpoint(roundabout_line_points, roundabout_num_points);
    //更新巡线状态
    last_frame_has_line = true;
}


/**
*函数：断路区通过的惯导实现
*
*返回：
*/
void open_road_solution(void)
{
  
      
      //首先判断是不是从有引导线到无引导线
     if(!current_has_line && last_frame_has_line)         //如果上次有引导线，这次没有引导线则将偏航角置0
     {
       Flash_data_read();       //把flash中的数据放到缓存中
       reset_yaw_to_zero(&estimator);   //重新置偏航角为 0
       final_yaw_num = 0;               //缓存区下标置0
//       gpio_set_level(Buzzer,1);        //设置蜂鸣器响代表断路区
     }  
     
    //从四元数中获取欧拉角
    angles = get_euler_angles(&estimator);
    
    //更新flash中储存的偏航角的下标并且重置中断编码器
     if(encoder_distance_flag == 1)
     {
         final_yaw_num++;
         L_motor.total_encoder = 0;
         L_motor.total_encoder = 0;
         encoder_distance_flag = 0;
     } 
     

     
     //将flash偏航角与实时偏航角做差，输入到舵机pid来调控舵机
     servo_open_road(flash_union_buffer[final_yaw_num].float_type,angles.yaw);     
     //更新巡线状态
     last_frame_has_line = false;
      //flash中没有数据则触发断言
     zf_assert(flash_check(FLASH_SECTION_INDEX,FLASH_YAW_PAGE_INDEX));


}


#ifdef ENABLE_CROSS
      
void cross_solution(void)
{
      current_has_line = true;
      gpio_set_level(Buzzer,1);
      uint8 X_start = (down_row_count[0].x + down_row_count[1].x) / 2;
      uint8 Y_start = (down_row_count[0].y + down_row_count[1].y) / 2;
      
      uint8 X_end = (up_row_count[0].x + up_row_count[1].x) / 2;
      uint8 Y_end = (up_row_count[0].y + up_row_count[1].y) / 2;
      
      num_points = Bresenham_line(line_points, X_start, Y_start, X_end, Y_end);
      
    if (start % 2 == 0) 
    {
        for (uint8 i = 0; i < num_points; i++) 
        {
            ips114_draw_point(line_points[i].x, line_points[i].y, RGB565_YELLOW);
        }
    } 

    Mid_point = calculate_weighted_midpoint(line_points, num_points);
     //更新巡线状态
    last_frame_has_line = true;
  
}


#endif




/**
* 函数：寻迹状态的处理
*
* 说明：使用状态机解决
*
*/
void line_tracking_state_machine()
{
    switch(Trackstate)          
    {
      case LINE_TRACKING:       //正常寻迹状态
            Line_track_solution();
            break;
        
      case ROUNDABOUT:          //六边形环岛状态
            roundabout_solution();
            break;
        
      case LOST_LINE:             //断路丢线状态
            open_road_solution();
            break;
#ifdef ENABLE_CROSS
      case CROSS:
            cross_solution(); 
            break;
#endif
    }

}


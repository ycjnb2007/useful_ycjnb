#include "zf_common_headfile.h"

Normal_track_mood normal_track_mood = link_up;

Normal_track_mood normal_track_last_mood = link_up;

uint8 X_dotted_end;              //虚线连接线的终点横坐标
uint8 Y_dotted_end;              //虚线连接线的终点纵坐标


Normal_track_mood determine_dotted_link_mood(void) 
{
    if(up_row_num == 4 && left_col_num == 2)
    {
      return link_up_roundabout_left;
    }
    
    if(up_row_num == 4 && right_col_num == 2)
    {
      return link_up_roundabout_right;
    }
  
        // 1. 优先检测直行/十字路口（上方有虚线）
    if (up_row_num == 2) 
    {
        return link_up;
    }
    
//    // 2. 处理T字路口（左右都有线但上方无线）
//    if (left_col_num == 2 && right_col_num == 2) 
//    {
//        
//        return link_up;
//    }
//    
    // 3. 处理单侧转向
    if (left_col_num == 2) 
    {
        return link_left;
    }
    if (right_col_num == 2) 
    {
        return link_right;
    }
    

    
//    // 4. 默认情况（无线或异常状态）
//    return link_up; // 或根据需求改为其他默认行为
    return normal_track_last_mood;
}


void Link_up_line(void)
{
    X_dotted_end = (up_row_count[0].x + up_row_count[1].x) / 2;
    Y_dotted_end = (up_row_count[0].y + up_row_count[1].y) / 2;
      
    dotted_num_points = Bresenham_line(dotted_line_points, x_start, y_start, X_dotted_end, Y_dotted_end);
    

}


void Link_left_line(void)
{
    X_dotted_end = (left_col_count[0].x + left_col_count[1].x) / 2;
    Y_dotted_end = (left_col_count[0].y + left_col_count[1].y) / 2;
      
    dotted_num_points = Bresenham_line(dotted_line_points, x_start, y_start, X_dotted_end, Y_dotted_end);
    

}

void Link_right_line(void)
{
    X_dotted_end = (right_col_count[0].x + right_col_count[1].x) / 2;
    Y_dotted_end = (right_col_count[0].y + right_col_count[1].y) / 2;
      
    dotted_num_points = Bresenham_line(dotted_line_points, x_start, y_start, X_dotted_end, Y_dotted_end);
    

}


void Link_up_per_roundabout_left(void)
{
    X_dotted_end = (up_row_count[2].x + up_row_count[3].x) / 2;
    Y_dotted_end = (up_row_count[2].y + up_row_count[3].y) / 2;
    dotted_num_points = Bresenham_line(dotted_line_points, x_start, y_start, X_dotted_end, Y_dotted_end);



}

void Link_up_per_roundabout_right(void)
{
    X_dotted_end = (up_row_count[0].x + up_row_count[1].x) / 2;
    Y_dotted_end = (up_row_count[0].y + up_row_count[1].y) / 2;
    dotted_num_points = Bresenham_line(dotted_line_points, x_start, y_start, X_dotted_end, Y_dotted_end);


}

void link_state_solution(Normal_track_mood mood)
{
    switch(mood)
    {
      case link_up:
            Link_up_line();
            break;
        
      case link_left:
            Link_left_line();
            break;    
        
      case link_right:
            Link_right_line();
            break;  
            
      case link_up_roundabout_left:
            Link_up_per_roundabout_left();
            break;  
            
      case link_up_roundabout_right:
            Link_up_per_roundabout_right();
            break;              
    }


}

void dotted_link_state_solution(void)
{
    normal_track_mood = determine_dotted_link_mood();
    link_state_solution(normal_track_mood);
    normal_track_last_mood = normal_track_mood;

}
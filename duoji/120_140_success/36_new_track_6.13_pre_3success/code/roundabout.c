#include "zf_common_headfile.h"

uint8 roundabout_dir = 0;   //环岛方向

LoopState roundabout_current_state = NORMAL_TRACK;


void Update_Roundabout_Dir(uint8 new_dir) 
{
    // 在方向为0，即正常的时候才能改变方向
    if (roundabout_dir == 0) 
    {
        roundabout_dir = new_dir;
    }
    
}


uint16 Loop_tarck(void)
{
  uint16 count_roundabout = 0;
    if(roundabout_dir == 1)
    {
        switch(roundabout_current_state)
        {
          
        case NORMAL_TRACK:
              
            
              break;
              
          
        case ENTER_ROUNDABOUT:
              count_roundabout =  Left_enter_roundabout();          //左侧入环处理
          
              break;
              
        case IN_RoundABOUT:
              count_roundabout =  In_roundabout();                  //环内正常寻迹 
              break;

        case EXIT_ROUNDABOUT1:
             count_roundabout =  Left_exit_roundabout1();
          
              break;  
              
        case EXIT_ROUNDABOUT2:
             count_roundabout =  Left_exit_roundabout2();
          
          
          break;
        
        }
    }else if(roundabout_dir == 2)
    {
        switch(roundabout_current_state)
        {
          
        case NORMAL_TRACK:
              

              break;
          
        case ENTER_ROUNDABOUT:
              count_roundabout =  Right_enter_roundabout();          //左侧入环处理
          
              break;
              
        case IN_RoundABOUT:
              count_roundabout =  In_roundabout();                  //环内正常寻迹 
              break;

        case EXIT_ROUNDABOUT1:
             count_roundabout =  Right_exit_roundabout1();
          
              break;  
              
        case EXIT_ROUNDABOUT2:
             count_roundabout =  Right_exit_roundabout2();
          
          
          break;
        
        }
        
    
    
    }
    
    
    return count_roundabout;

}



void Pre_turn_flag_update(uint8 new_flag) 
{
    // 在方向为0，即正常的时候才能改变方向
    if (Pre_turn_roundabout_flag == 0) 
    {
        Pre_turn_roundabout_flag = new_flag;
    }
    
}

uint8 Pre_turn_roundabout_flag = 0;
/**
函数：欲入环识别
说明：主要解决入环前正常寻迹因看到圆环边界导致转向问题前的识别左侧还是右侧
**/
uint8 Pre_roundabout_turn(void)
{
#ifdef ENABLE_PRE_LOOP
    if(up_row_num1 == 4 && down_row_num1 == 2)
    {
        if(left_col_num1 == 2 && right_col_num1 == 0)
        {
          return 1;     //代表左侧预入环
          
        }else if(right_col_num1 == 2 && left_col_num1 == 0)
        {
        
          return 2;     //代表右侧预备入环
        }
      
    }
#endif
  
    return 0;

}






/**
函数：左侧预入环
备注：主要解决入环前正常寻迹因看到圆环边界导致左转问题
**/
uint8 Pre_left_roundabout_state(void)
{
    if(down_row_num == 4)
    {
        if(up_row_num == 2 || up_row_num == 4)
        {
            X_roundabout_start = (down_row_count[2].x + down_row_count[3].x) / 2;
            Y_roundabout_start = (down_row_count[2].y + down_row_count[3].y) / 2;
          return 1;
          
        }
    }
  
  return 0;
  
}

/**
函数：左侧预入环
备注：主要解决入环前正常寻迹因看到圆环边界导致左转问题
**/
uint8 Pre_right_roundabout_state(void)
{
    if(down_row_num == 4)
    {
        if(up_row_num == 2 || up_row_num == 4)
        {
            X_roundabout_start = (down_row_count[0].x + down_row_count[1].x) / 2;
            Y_roundabout_start = (down_row_count[0].y + down_row_count[1].y) / 2;
          return 1;
          
        }
    }
  
  return 0;
  
}




/**

函数：环岛识别


**/
uint8 Detect_roundabout(void)
{

    row_col_element_find();
    
    uint8 left_white_count = 0;
    uint8 right_white_count = 0;
    uint8 detect_mid_flag = 0;
  if(up_row_num == 4 && down_row_num == 4 && left_col_num1 == 0 && right_col_num1 == 0)
  {

    uint8 left_mid_x = (down_row_count[0].x + down_row_count[1].x) / 2;
    uint8 right_mid_x = (down_row_count[2].x + down_row_count[3].x) / 2;
    uint8 left_mid_y = (down_row_count[0].y + down_row_count[1].y) / 2;
    uint8 right_mid_y = (down_row_count[2].y + down_row_count[3].y) / 2;
    
    uint8 detect_mid_x = (left_mid_x + right_mid_x) / 2;
    
    
    for(uint8 j = row_last_line; j > row_start_line+ 1; j--)
    {
      if(Trip_point(image[j][detect_mid_x],image[j - 1][detect_mid_x]))
      {
        detect_mid_flag = 1;
        break;
      }
    }
    
    
    
    if(detect_mid_flag == 1)
    {
        for(uint8 i = left_mid_y; i > row_start_line;i--)
        {
          if(image[i][left_mid_x] == 255)
          {
            left_white_count++;
          }
        
        }
        for(uint8 i = right_mid_y; i > row_start_line;i--)
        {
          if(image[i][right_mid_x] == 255)
          {
            right_white_count++;
          }
        
        }
        
        
        //起始点中心在左侧说明是左入环岛
        if(left_white_count <= right_white_count)
        {
          
          if(roundabout_current_state == NORMAL_TRACK)
            {
              roundabout_current_state = ENTER_ROUNDABOUT;
                     
            } 
          return 1; 
          
        
        }else //起始点中心在右侧说明是右入环岛
        {
          
          if(roundabout_current_state == NORMAL_TRACK)
            {
            roundabout_current_state = ENTER_ROUNDABOUT;

               
            }     
           return 2; 

        }
    }else
    {
      return 0;
    }

    
    
  }
  
  return 0;


}

//uint8 Detect_roundabout(void)
//{
//    row_col_element_find();     //元素行和元素列的扫线
//    
//    
//    if(up_row_num == 4 && down_row_num == 2)
//    {
//        if(left_col_num == 2)
//        {
//          
//          if(roundabout_current_state == NORMAL_TRACK)  //只有在正常寻迹状态下才置进入圆环的标志位，（有时候在出环状态下直接有变成了入环状态导致出环失败）
//          {
//            uint8 x_mid_guss = (left_col_count[0].x + down_row_count[0].x) / 2;
//            for(uint8 i = x_mid_guss - GUSS_MID_NUM; i < x_mid_guss + GUSS_MID_NUM;i++ )
//            {
//              for(uint8 j = row_last_line; j > row_start_line + 1; j--)
//              {
//                if(Trip_point(image[j][i],image[j - 1][i]))
//                {
//                  roundabout_current_state = ENTER_ROUNDABOUT;    
//                  return 1;                        
//
//                }
//              }
//            }              
//         
//          }
//        }else if(right_col_num == 2)
//        {
//          if(roundabout_current_state == NORMAL_TRACK)
//          {
//            
//            uint8 x_mid_guss = (right_col_count[0].x + down_row_count[0].x) / 2;
//            for(uint8 i = x_mid_guss - GUSS_MID_NUM; i < x_mid_guss + GUSS_MID_NUM;i++ )
//            {
//              for(uint8 j = row_last_line; j > row_start_line + 1; j--)
//              {
//                if(Trip_point(image[j][i],image[j - 1][i]))
//                {
//                    roundabout_current_state = ENTER_ROUNDABOUT;  
//                    return 2;                           
//
//                }
//              }
//             }               
//          }
//        }
//    }
//    
//    return 0;
//
//}

/**
函数：左侧入环处理

**/
uint16 roundabout_num_points;
Point roundabout_line_points[MT9V03X_W + CUT_HEIGHT];

uint8 X_roundabout_start;
uint8 Y_roundabout_start;

uint8 X_roundabout_end;
uint8 Y_roundabout_end;  
uint16 Left_enter_roundabout(void)
{

    if(up_row_num == 4)
    {
      if(down_row_num == 4 && left_col_num1 == 0)
      {
      X_roundabout_start = (down_row_count[2].x + down_row_count[3].x) / 2;
      Y_roundabout_start = (down_row_count[2].y + down_row_count[3].y) / 2;
      
      X_roundabout_end   = (up_row_count[0].x + up_row_count[1].x) / 2;
      Y_roundabout_end   = (up_row_count[0].y + up_row_count[1].y) / 2;
      roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
        
      }else if(down_row_num == 2)
      {
        
      X_roundabout_start = (down_row_count[0].x + down_row_count[1].x) / 2;
      Y_roundabout_start = (down_row_count[0].y + down_row_count[1].y) / 2;
      
      X_roundabout_end   = (up_row_count[2].x + up_row_count[3].x) / 2;
      Y_roundabout_end   = (up_row_count[2].y + up_row_count[3].y) / 2;
      roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
      }


    }else if(up_row_num == 2)
    {
        if(down_row_num == 4)
        {
          X_roundabout_start = (down_row_count[2].x + down_row_count[3].x) / 2;
          Y_roundabout_start = (down_row_count[2].y + down_row_count[3].y) / 2;
          
          X_roundabout_end   = (up_row_count[0].x + up_row_count[1].x) / 2;
          Y_roundabout_end   = (up_row_count[0].y + up_row_count[1].y) / 2;
          roundabout_num_points = Bresenham_line(roundabout_line_points,
                                                  X_roundabout_start,Y_roundabout_start,
                                                  X_roundabout_end,Y_roundabout_end);
          
        }
    
    }
    
    if(left_col_num == 2 && up_row_num == 2 && down_row_num == 4)
    {
          X_roundabout_start = (down_row_count[0].x + down_row_count[1].x) / 2;
          Y_roundabout_start = (down_row_count[0].y + down_row_count[1].y) / 2;
          
          X_roundabout_end   = (left_col_count[0].x + left_col_count[1].x) / 2;
          Y_roundabout_end   = (left_col_count[0].y + left_col_count[1].y) / 2;
          roundabout_num_points = Bresenham_line(roundabout_line_points,
                                                  X_roundabout_start,Y_roundabout_start,
                                                  X_roundabout_end,Y_roundabout_end);

    }
    
    
    if(up_row_num1 == 0)
    {
      roundabout_current_state = IN_RoundABOUT;
    
    }
    
    return roundabout_num_points;

}

/**
函数：环内正常寻迹处理

**/
uint16 In_roundabout(void)
{
      X_roundabout_start = x_start;
      Y_roundabout_start = y_start;
      
      
      X_roundabout_end   = X_meet;
      Y_roundabout_end   = Y_meet;
      roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
      
      
      if(roundabout_dir == 1)
      {
         if(right_col_num != 0)
          {
          
            roundabout_current_state = EXIT_ROUNDABOUT1;
          
          }
      }else
      {
          if(left_col_num != 0)
          {
            roundabout_current_state = EXIT_ROUNDABOUT1;
          }
      
      }
      
      

      
    return roundabout_num_points;
      

}

/**
函数：左侧出环处理1
**/
uint16 Left_exit_roundabout1(void)
{
  
      X_roundabout_start = x_start;
      Y_roundabout_start = y_start;
  
  
    if(left_col_num == 2)
    {
      X_roundabout_end = (left_col_count[0].x + left_col_count[1].x) / 2;
      Y_roundabout_end = (left_col_count[0].y + left_col_count[1].y) / 2;
    
    }else if(left_col_num == 4)
    {
      
      X_roundabout_end = (left_col_count[2].x + left_col_count[3].x) / 2;
      Y_roundabout_end = (left_col_count[2].y + left_col_count[3].y) / 2;
    
    }
    
     roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
    
    
    if(down_row_num == 4)
    {
      roundabout_current_state = EXIT_ROUNDABOUT2;
    
    }
    
   
        return roundabout_num_points;

}

/**
函数：左侧出环处理2

**/

uint16 Left_exit_roundabout2(void)
{
  
    if(down_row_num == 4)
    {
       X_roundabout_start = (down_row_count[2].x + down_row_count[3].x) / 2;
       Y_roundabout_start = (down_row_count[2].y + down_row_count[3].y) / 2;  
      
    }else
    {
      
      X_roundabout_start = x_start;
      Y_roundabout_start = y_start;    
    }
  
      X_roundabout_start = (down_row_count[2].x + down_row_count[3].x) / 2;
      Y_roundabout_start = (down_row_count[2].y + down_row_count[3].y) / 2;   


    
    if(left_col_num == 4)
    {
      
      X_roundabout_end = (left_col_count[2].x + left_col_count[3].x) / 2;
      Y_roundabout_end = (left_col_count[2].y + left_col_count[3].y) / 2;
    }else
    {
      
      X_roundabout_end   = (up_row_count[0].x + up_row_count[1].x) / 2;
      Y_roundabout_end   = (up_row_count[0].y + up_row_count[1].y) / 2;
    }
    
     roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
    
    if(up_row_num1 == 2 && down_row_num1 == 2 && left_col_num1 == 0 && right_col_num1 == 0)
    {
          
        roundabout_dir = 0;
        roundabout_current_state = NORMAL_TRACK;        
    }
    
    
    return roundabout_num_points;


}

/**
函数：右侧入环处理

**/
uint16 Right_enter_roundabout(void)
{

    if(up_row_num == 4)
    {
      if(down_row_num == 4 && right_col_num1 == 0)
      {
      X_roundabout_start = (down_row_count[0].x + down_row_count[1].x) / 2;
      Y_roundabout_start = (down_row_count[0].y + down_row_count[1].y) / 2;
      
      X_roundabout_end   = (up_row_count[2].x + up_row_count[3].x) / 2;
      Y_roundabout_end   = (up_row_count[2].y + up_row_count[3].y) / 2;
      roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
        
      }else if(down_row_num == 2)
      {
        
      X_roundabout_start = (down_row_count[0].x + down_row_count[1].x) / 2;
      Y_roundabout_start = (down_row_count[0].y + down_row_count[1].y) / 2;
      
      X_roundabout_end   = (up_row_count[0].x + up_row_count[1].x) / 2;
      Y_roundabout_end   = (up_row_count[0].y + up_row_count[1].y) / 2;
      roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
      
      }


    }else if(up_row_num == 2)
    {
        if(down_row_num == 4)
        {
          X_roundabout_start = (down_row_count[0].x + down_row_count[1].x) / 2;
          Y_roundabout_start = (down_row_count[0].y + down_row_count[1].y) / 2;
          
          X_roundabout_end   = (up_row_count[0].x + up_row_count[1].x) / 2;
          Y_roundabout_end   = (up_row_count[0].y + up_row_count[1].y) / 2;
          roundabout_num_points = Bresenham_line(roundabout_line_points,
                                                  X_roundabout_start,Y_roundabout_start,
                                                  X_roundabout_end,Y_roundabout_end);
          
        }
    
    }
    
    if(right_col_num == 2 && up_row_num == 2 && down_row_num == 4)
    {
          X_roundabout_start = (down_row_count[2].x + down_row_count[3].x) / 2;
          Y_roundabout_start = (down_row_count[2].y + down_row_count[3].y) / 2;
          
          X_roundabout_end   = right_col_count[0].x;
          Y_roundabout_end   = right_col_count[0].y;
          roundabout_num_points = Bresenham_line(roundabout_line_points,
                                                  X_roundabout_start,Y_roundabout_start,
                                                  X_roundabout_end,Y_roundabout_end);

    }
    
    
    if(up_row_num1 == 0)
    {
      roundabout_current_state = IN_RoundABOUT;
    
    }
    
    return roundabout_num_points;

}


/**
函数：右侧出环处理1

**/
uint16 Right_exit_roundabout1(void)
{
  
      X_roundabout_start = x_start;
      Y_roundabout_start = y_start;
  
  
    if(right_col_num == 2)
    {
      X_roundabout_end = (right_col_count[0].x + right_col_count[1].x) / 2;
      Y_roundabout_end = (right_col_count[0].y + right_col_count[1].y) / 2;
    
    }else if(left_col_num == 4)
    {
      
      X_roundabout_end = (right_col_count[2].x + right_col_count[3].x) / 2;
      Y_roundabout_end = (right_col_count[2].y + right_col_count[3].y) / 2;
    
    }
    
     roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
    
    
    if(down_row_num == 4)
    {
      roundabout_current_state = EXIT_ROUNDABOUT2;
    
    }
    
   
        return roundabout_num_points;

}


/**
函数：右侧出环处理2

**/

uint16 Right_exit_roundabout2(void)
{
  
    if(down_row_num == 4)
    {
       X_roundabout_start = (down_row_count[2].x + down_row_count[3].x) / 2;
       Y_roundabout_start = (down_row_count[2].y + down_row_count[3].y) / 2;  
      
    }else
    {
      
      X_roundabout_start = x_start;
      Y_roundabout_start = y_start;    
    }

    
    if(right_col_num == 4)
    {
      
      X_roundabout_end = (right_col_count[2].x + right_col_count[3].x) / 2;
      Y_roundabout_end = (right_col_count[2].y + right_col_count[3].y) / 2;
    }else 
    {
      
      X_roundabout_end   = (up_row_count[0].x + up_row_count[1].x) / 2;
      Y_roundabout_end   = (up_row_count[0].y + up_row_count[1].y) / 2;
    }
    
     roundabout_num_points = Bresenham_line(roundabout_line_points,
                                              X_roundabout_start,Y_roundabout_start,
                                              X_roundabout_end,Y_roundabout_end);
    
    if(up_row_num1 == 2 && down_row_num1 == 2 && left_col_num1 == 0 && right_col_num1 == 0)
    {
          
        roundabout_dir = 0;
        roundabout_current_state = NORMAL_TRACK;        
    }
    
    
    return roundabout_num_points;


}
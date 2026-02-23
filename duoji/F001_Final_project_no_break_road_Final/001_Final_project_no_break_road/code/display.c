#include "zf_common_headfile.h"



void display_image_page1(void)
{

       
       //显示画面
      ips114_show_gray_image(0, 0, image[0], CUT_W, CUT_H, CUT_W, CUT_H, 0);//显示图像
      
      //显示阈值
      ips114_show_uint(190,7*16,th,3);                                 //显示阈值        

    
       //显示元素行和元素列
      ips114_show_string(0*16,6*16,"Y");
      ips114_show_uint(1*16,6*16,up_row_num,1);  
      ips114_show_uint(2*16,6*16,down_row_num,1);  
      ips114_show_uint(1*16,7*16,left_col_num,1);  
      ips114_show_uint(2*16,7*16,right_col_num,1);  
  
      ips114_show_string(4*16,6*16,"N");
      ips114_show_uint(5*16,6*16,up_row_num1,1);
      ips114_show_uint(6*16,6*16,down_row_num1,1);
      ips114_show_uint(5*16,7*16,left_col_num1,1);
      ips114_show_uint(6*16,7*16,right_col_num1,1);
      
      //显示圆环状态
      ips114_show_string(190,0*16,"Dir");
      ips114_show_uint(190+2*16,0*16,roundabout_dir,1);
      ips114_show_string(190,1*16,"stat");
      ips114_show_uint(190+2*16,1*16,roundabout_current_state,1);
      
      //显示电机状态
      ips114_show_string(190,2*16,"mo");
      ips114_show_uint(190+2*16,2*16,start_motor,1);

      ips114_show_string(190,3*16,"so");
      ips114_show_uint(190+2*16,3*16,start_servo,1);
      
      //显示拟合出的线的点数
      ips114_show_uint(190,4*16,num_points,3);
      ips114_show_uint(190,5*16,dotted_num_points,3);
      ips114_show_uint(190,6*16,roundabout_num_points,3);
      

}

void display_pid_page2(void)
{

    

    
    ips114_show_string(4*16,0*16,"DATA_1");
    ips114_show_uint(0*16,0*16,exchange_choose_option,1);
    
    ips114_show_string(0*16,1*16,"1Servo.Kp:");
    ips114_show_float(6*16,1*16,(double)Servo_pid.Kp,2,1);
    ips114_show_string(0*16,2*16,"2Servo.Kd:");
    ips114_show_float(6*16,2*16,(double)Servo_pid.Kd,2,1);    
    ips114_show_string(0*16,3*16,"3K_dif:");
    ips114_show_float(6*16,3*16,(double)differential_k,2,1);
    ips114_show_string(0*16,4*16,"4Speed:");
    ips114_show_int(6*16,4*16,normol_speed,3);



}


//显示屏幕不同画面
void UI_display_page(void )
{
    // 显示前一页的画面
    
    
    switch(exchange_show_page)
    {
      case 1: 
        display_image_page1();
        break;

      case 2:
        display_pid_page2();
        break; 
 
    }
}


#ifdef ENABLE_BREAK
/*************************************断路显示*************************************/

//显示路线记录信息
void Display_duanlu_memery(void)
{
    
    ips114_show_string(4*16,0*16,"Angle_Memery:");
    ips114_show_uint(0*16,0*16,exchange_choose_option,1);
    
    ips114_show_string(0*16,1*16,"1area_remery:");
    ips114_show_uint(7*16,1*16,area_memery,3);
    
    ips114_show_string(0*16,2*16,"2angel_pwm:");
    ips114_show_uint(7*16,2*16,servo_angle_break_road,3);
    
    ips114_show_string(0*16,3*16,"3write:");
    ips114_show_uint(5*16,3*16,write_flag,1);
    
    ips114_show_uint(7*16,3*16,write_flag_finish1,1);
    ips114_show_uint(8*16,3*16,write_flag_finish2,1);
    ips114_show_uint(9*16,3*16,write_flag_finish3,1);
    
}


   

/*************************************断路显示*************************************/


#endif
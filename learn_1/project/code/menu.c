#include "menu.h"


/*********************************************************************************************************************
  *菜单逻辑判断和屏幕显示
 ********************************************************************************************************************/




uint8 Choice = 0;        // 初始状态为未选中
page_flag page=0;
uint8 Choice_MAX;        //可选择最大个数
uint8 buzzer_enable = 0; // 蜂鸣器使能，1表示允许高电平，0表示禁止高电平，默认关闭
uint8 fan_enable = 1;    // 负压风扇使能，1表示开启，0表示关闭，默认关闭


uint16 MT9V03X_EXP_TIME_DEF= 0;                // 摄像头曝光时间
uint16 MT9V03X_FPS_DEF     = 0;                // 图像帧率设置   摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS

uint8 run_flag=0;               //开跑！
uint8_t record_mode = 0;
uint8_t replay_mode = 0;


//////////////////////test
//float kp=1.0;
//float ki=2.0;
//float kd=3.0;
//uint8 save_flag=0;
//////////////////////test

uint8_t encoder_knob_unlocked = 0; // 旋钮开关标志
int16_t encoder_left_last = 0;
int16_t encoder_right_last = 0;
uint8_t knob_key_number = 0; // 旋钮等效按键

// 旋钮步进
#define ENCODER_KNOB_STEP 10

// 旋钮等效按键宏
void encoder_knob_menu_control(void)
{
    static int16_t left_last = 0, right_last = 0;
    static uint8_t encoder_knob_cooldown = 0; // 冷却计数

    if (encoder_knob_cooldown > 0)
    {
        encoder_knob_cooldown--;
        return;
    }

    if (gpio_get_level(KEY1_PIN)) 
    {
        left_last = motor_value_right.receive_left_speed_data;
        right_last = motor_value_right.receive_right_speed_data;
        return;
    }
    int16_t left_now = motor_value_right.receive_left_speed_data;
    int16_t right_now = motor_value_right.receive_right_speed_data;
    int16_t left_diff = left_now - left_last;
    int16_t right_diff = right_now - right_last;

    if (left_diff >= 20)
    {
//        left_last = left_now;
        knob_key_number = KEY_PAGE;
        encoder_knob_cooldown = 80; // 80个周期（约50ms）
    }
    else if (left_diff <= -ENCODER_KNOB_STEP)
    {
//        left_last = left_now;
        knob_key_number = KEY_Choice;
        encoder_knob_cooldown = 40;
    }
    if (right_diff >= ENCODER_KNOB_STEP)
    {
//        right_last = right_now;
        knob_key_number = KEY_ADD;
        encoder_knob_cooldown = 10;
    } else if (right_diff <= -ENCODER_KNOB_STEP)
    {
//        right_last = right_now;
        knob_key_number = KEY_REDUCE;
        encoder_knob_cooldown = 10;
    }
}

// 菜单按键事件分发（可直接调用原有处理逻辑）
void menu_key_event(uint8_t key)
{
    // 这里直接调用menu()内的按键处理逻辑，或将Key_number赋值后再调用menu
    // 你可以根据实际情况调整
    extern void menu_key_handler(uint8_t key); // 需在menu.h声明
    menu_key_handler(key);
}


void menu( )
{	

	static uint8 Key_number;
	Key_number = scanf_key(); // 按键扫描
	// 菜单切换时归零
	static page_flag last_page = 0;
	if (page != last_page) 
        {
		encoder_left_last = motor_value_right.receive_left_speed_data;
		encoder_right_last = motor_value_right.receive_right_speed_data;
		last_page = page;
	}
        
        
	// 调用旋钮控制
	 encoder_knob_menu_control();
        
          if (knob_key_number) 
       {
		Key_number = knob_key_number;
		knob_key_number = 0;
		clear_flag = 1; // 触发清屏
	}
        
	// 旋钮等效按键注入
	if (knob_key_number)
        {
		Key_number = knob_key_number;
		knob_key_number = 0;
	}

		 
	switch(page)//页面不同，可选择个数不同
	{
			case PAGE_HOME:	Choice_MAX=2;break; 
			case PAGE_2:	Choice_MAX=8;break;
			case PAGE_3:	Choice_MAX=12;break;
			case PAGE_4:	Choice_MAX=12;break;                        
			default:	Choice=0;break;
	}
        if (Choice > Choice_MAX) Choice = 1; // 循环小箭头

	
	if (Key_number == KEY_PAGE)
	{
		page = (page == 3) ? 0 : (page + 1);//页在 0 到 2 之间循环
		Choice = 1;                         // 切换页面时重置为1
	}
	else if(Key_number == KEY_Choice)
		Choice = (Choice == Choice_MAX) ? 1 : (Choice + 1);  

//主页面	
		if(page==PAGE_HOME&&Key_number==KEY_ADD)
	{
		switch(Choice)
		{
			case 1:run_flag=1;break;
                        case 2:run_test_enable=1;run_flag=1;buzzer_enable=1;break;
//			case 2:turn_d+=0.05;break;
//			case 3:speed+=50;break;
		}		
	}
	else if(page==PAGE_HOME&&Key_number==KEY_REDUCE)
	{
		switch(Choice)
		{
			case 1:run_flag=1;break;
                        case 2:run_test_enable=0;break;

//			case 2:turn_d-=0.05;break;
//			case 3:speed-=50;break;
		}
	}	
//页2	
	if(page==PAGE_2&&Key_number==KEY_ADD)
	{
		switch(Choice)
		{
			case 1:posi_KP +=0.1;break;
			case 2:posi_KD +=0.5;break;
			case 3:posi_KD_2 +=0.01;break;
                        case 4:except_speed_base +=50;break;
                        
//                        case 5:save_flag=1;break;
                        case 6:fan_enable = 1;break;
                        case 7:fan_duty +=500;break;
                        case 8:buzzer_enable = 1;break;
		}
			
	}
	else if(page==PAGE_2&&Key_number==KEY_REDUCE)
	{
		switch(Choice)
		{
			case 1:posi_KP -=0.1;break;
			case 2:posi_KD -=0.5;break;
			case 3:posi_KD_2 -=0.01;break;
                        case 4:except_speed_base -=50;break;
                        
//                        case 5:save_flag=1;break;
                        case 6:fan_enable = 0;break;
                        case 7:fan_duty -=500;break;
                        case 8:buzzer_enable = 0;break;
		}
	}
 //页3	       
        	if(page==PAGE_3&&Key_number==KEY_ADD)
	{
		switch(Choice)
		{
                        case 1:break_angle_test = (break_angle_test == 0) ? 1 : 0;break;
                        case 2:break_rest_enable = (break_rest_enable == 0) ? 1 : 0;break;
                        case 3:break_rest_threshold+=2;break;
			case 4:replay_mode =1;break;
			case 5:record_mode =1;break;
                        case 6:stop_record_flag =1;break;
                        case 8: break_number = (break_number <= 1) ? 3 : (break_number + 1); save_camera_params_to_flash(); break;
                        case 9:save_current_white_values();break;
                        case 10:MT9V03X_EXP_TIME_DEF += 50; save_camera_params_to_flash(); break;
                        case 11:MT9V03X_FPS_DEF += 50; save_camera_params_to_flash(); break;
                        case 12:otsu_limit = (otsu_limit < 200) ? (otsu_limit + 5) : 200; save_camera_params_to_flash(); break;

		}
			
	}
        	else if(page==PAGE_3&&Key_number==KEY_REDUCE)
	{
		switch(Choice)
		{
                  
                        case 1:break_angle_test = (break_angle_test == 0) ? 1 : 0;break;
                        case 2:break_rest_enable = (break_rest_enable == 0) ? 1 : 0;break;
                        case 3:break_rest_threshold-=2;break;
                        case 4:replay_mode =0;break;
                        case 5:record_mode =0;break;
                        case 6:stop_record_flag =1;break;
                        case 8: break_number = (break_number <= 1) ? 3 : (break_number - 1); save_camera_params_to_flash(); break;
                        case 9:save_current_white_values();break;
                        case 10:MT9V03X_EXP_TIME_DEF = (MT9V03X_EXP_TIME_DEF > 50) ? (MT9V03X_EXP_TIME_DEF - 50) : 50; save_camera_params_to_flash(); break;
                        case 11:MT9V03X_FPS_DEF = (MT9V03X_FPS_DEF > 50) ? (MT9V03X_FPS_DEF - 50) : 50; save_camera_params_to_flash(); break;
                        case 12:otsu_limit = (otsu_limit > 1) ? (otsu_limit - 5) : 1; save_camera_params_to_flash(); break;
		}
	}
//页4       
       
        
                	if(page==PAGE_4&&Key_number==KEY_ADD)
	{
		switch(Choice)
		{
                        case 1:k_ratio += 0.01;break;
                        case 2:speed_ring+=25;break;
			case 3:posi_KP_NORMAL += 0.1;break;
			case 4:posi_KD_NORMAL+=0.1;break;
                        case 5:posi_KP_STRAIGHT += 0.1;break;
                        case 6:posi_KD_STRAIGHT += 0.1;break;
                        case 7:posi_KP_RIGHT += 0.1;break;
                        case 8:posi_KD_RIGHT += 0.1;break;
                        case 9:posi_KP_RING += 0.1;break;
                        case 10:posi_KD_RING += 0.1;break;
                        case 11:right_white_found_threshold += 2;break;
                        case 12:right_qianzhan_base += 2;break;

		}
			
	}
        	else if(page==PAGE_4&&Key_number==KEY_REDUCE)
	{
		switch(Choice)
		{
                  
                        case 1:k_ratio -= 0.01;break;
                        case 2:speed_ring-=25;break;
			case 3:posi_KP_NORMAL -= 0.1;break;
			case 4:posi_KD_NORMAL-=0.1;break;
                        case 5:posi_KP_STRAIGHT -= 0.1;break;
                        case 6:posi_KD_STRAIGHT -= 0.1;break;
                        case 7:posi_KP_RIGHT -= 0.1;break;
                        case 8:posi_KD_RIGHT -= 0.1;break;
                        case 9:posi_KP_RING -= 0.1;break;
                        case 10:posi_KD_RING -= 0.1;break;
                        case 11:right_white_found_threshold -= 2;break;
                        case 12:right_qianzhan_base -= 2;break;
		}
	}
        	
}		


uint16  knob_value=0;
uint16 get_knob_value( )	
{
    return adc_convert(ADC0_CH21_P07_5);  
}


void display( )	
{
//        ips200_show_int(64, 140,get_knob_value()/2,3);  //   ips200_show_chinese(0, 140, 16, chinese_page[0], 3, RGB565_RED);   //显示当前页数
	switch(page)
	{
		case PAGE_HOME:
                        ips200_show_gray_image(IMAGE_CUT_LEFT, 0, (Thresholding_image[0]), IMAGE_W, IMAGE_H, (IMAGE_W), (IMAGE_H), 0);

                        // 显示其他信息
                        ips200_show_int(0,80,left_rightangle_flag,3);
                        ips200_show_int(0,100,right_rightangle_flag,3);
                        ips200_show_int(0,120,imu660ra_gyro_z,4); 
                        
                        ips200_show_int(160,80, motor_value_left.receive_left_speed_data,4);
                        ips200_show_int(200,80,motor_value_left.receive_right_speed_data,4);

                        ips200_show_int(160,100,-motor_value_right.receive_left_speed_data,4);
                        ips200_show_int(200,100, -motor_value_right.receive_right_speed_data,4);
                        
                        ips200_show_int(200,140,mtv9_position,4);
                        
                        ips200_show_int(0,160, gpio_get_level(KEY1_PIN),1);
                        ips200_show_int(20,160,gpio_get_level(KEY2_PIN),1);
                        ips200_show_int(40,160,gpio_get_level(KEY3_PIN),1);
                        ips200_show_int(60,160,gpio_get_level(KEY4_PIN),1);
                        
                        ips200_show_int(200,300,max_row_lower,3);
                        ips200_show_int(200,280,max_white_value,3);


                        ips200_show_int(80,180,run_flag,3);
                        ips200_show_string(20,180,"run");


                        
                        ips200_show_string(20,200,"run_test");
                        ips200_show_int(100,200,run_test_enable,1); // 显示旋钮开关状态
                        // 显示陀螺仪校准状态
                        Quaternion* euler = get_euler_angles();
                        ips200_show_string(0,220,"Gyro Calib:");
                        ips200_show_int(100,220,euler->calibrated,1);
                        ips200_show_float(140,220,euler->yaw,3,2);
                        
                        


			break;		
		case PAGE_2:
			 
                        ips200_displayimage03x(mt9v03x_image[0], MT9V03X_W, MT9V03X_H); 
                  
                        ips200_show_string(30,160,"posi_KP");
			ips200_show_string(30,180,"posi_KD");
                        ips200_show_string(30,200,"posi_KD_2");
                        ips200_show_string(30,220,"sudu");
                        ips200_show_string(30,240,"save");
                        ips200_show_string(30,260,"fan_enable");
                        ips200_show_string(30,280,"fan_duty");
                        ips200_show_string(30,300,"buzzer_enable");

                        ips200_show_float(150,160,posi_KP,3,3);  
                        ips200_show_float(150,180,posi_KD,1,3); 
                        ips200_show_float(150,200,posi_KD_2,1,3); 
                        ips200_show_int(150,220,except_speed_base,4); 
                        ips200_show_int(150,260,fan_enable,1);
                        ips200_show_int(150,280,fan_duty,5);
                        ips200_show_int(150,300,buzzer_enable,1);

                 

			break;		
		case PAGE_3:
                  
                         // 显示当前白点值
                        ips200_show_string(0, 280, "Current:");
                        ips200_show_int(80, 280, white_per_row[40], 3);  // 显示第40行的白点值
                        ips200_show_int(120, 280, white_per_row[60], 3); // 显示第50行的白点值
                        ips200_show_int(160, 280, white_per_row[75], 3); // 显示第60行的白点值
                        
                        // 显示正常白点值
                        ips200_show_string(0, 300, "Normal:");
                        ips200_show_int(80, 300, normal_white_per_row[40], 3);  // 显示第40行的正常白点值
                        ips200_show_int(120, 300, normal_white_per_row[60], 3); // 显示第50行的正常白点值
                        ips200_show_int(160, 300, normal_white_per_row[75], 3); // 显示第60行的正常白点值
                  
                        ips200_show_string(20,100,"replay_mode");
			ips200_show_string(20,120,"record_mode");
                        ips200_show_string(20,140,"stop_record_flag");
                        ips200_show_string(20,160,"break_flag");
                        ips200_show_string(20,180,"break_num:");
                        ips200_show_string(20,200,"save_white_values");

                        ips200_show_int(150,100,replay_mode,3);  
                        ips200_show_int(150,120,record_mode,1); 
                        ips200_show_int(150,140,stop_record_flag,1); 
                        ips200_show_int(150,160,break_flag,1); 
                        ips200_show_int(150,180,break_number,2);

                        
                        ips200_show_string(20,220,"exp_time");
                        ips200_show_string(20,240,"fps");
                        ips200_show_string(20,260,"otsu_limit");
                        
                        ips200_show_int(150,220,MT9V03X_EXP_TIME_DEF,4);
                        ips200_show_int(150,240,MT9V03X_FPS_DEF,4);
                        ips200_show_int(150,260,otsu_limit,4);

                        
                        ips200_show_int(0,20,total_pulses,4); 
                        ips200_show_string(0,0,"currnt_recoder");
                        ips200_show_int(120,0,currnt_recoder,2); 
                        
                        ips200_show_int(170,60,break_rest_enable,4);
                        ips200_show_string(20,60,"bk_rst_enable");
                        ips200_show_int(170,80,break_rest_threshold,4);
                        ips200_show_string(20,80,"bk_rst_threshold");   
                        
                        ips200_show_string(20,40,"bk_angle_test");
                        ips200_show_int(170,40,break_angle_test,3);


                        
			break;	
                        
		case PAGE_4:
                  
                        ips200_show_float(170,60,k_ratio,2,2);
                        ips200_show_string(20,60,"K_BASE");
                        ips200_show_int(170,80,speed_ring,4);
                        ips200_show_string(20,80,"speed_ring");  
                  
                        ips200_show_string(20,100,"KP_NORMAL");
			ips200_show_string(20,120,"KD_NORMAL");
                        ips200_show_string(20,140,"KP_STRAIGHT");
                        ips200_show_string(20,160,"KD_STRAIGHT");
                        ips200_show_string(20,180,"KP_RIGHT:");
                        ips200_show_string(20,200,"KD_RIGHT");


                        ips200_show_float(150,100,posi_KP_NORMAL,3,1);  
                        ips200_show_float(150,120,posi_KD_NORMAL,1,1); 
                        ips200_show_float(150,140,posi_KP_STRAIGHT,1,1); 
                        ips200_show_float(150,160,posi_KD_STRAIGHT,1,1); 
                        ips200_show_float(150,180,posi_KP_RIGHT,1,1);
                        ips200_show_float(150,200,posi_KD_RIGHT,1,1);

                        
                        ips200_show_string(20,220,"KP_RING");
                        ips200_show_string(20,240,"KD_RING");
                        
                        ips200_show_float(150,220,posi_KP_RING,1,1);
                        ips200_show_float(150,240,posi_KD_RING,1,1);
                        
                        
                        ips200_show_string(20,260,"right_threshold");
                        ips200_show_int(150,260,right_white_found_threshold,4);
                        
                        ips200_show_string(20,280,"right_qianzhan");
                        ips200_show_int(150,280,right_qianzhan_base,4);


                        
			break;	                  
                        
	}
			if(Choice)		////小光标
		{
				switch (page)
				{
					case PAGE_HOME:		ips200_show_string(0, 160+(Choice*20), "->");break;
					case PAGE_2:	        ips200_show_string(0, 140+(Choice*20), "->");break;
					case PAGE_3:		ips200_show_string(0, 20+(Choice*20), "->");break;
                                        case PAGE_4:		ips200_show_string(0, 40+(Choice*20), "->");break;

				}				
		}
			if(clear_flag==1)	////清屏
		{
				ips200_clear();
				clear_flag=0;
		}
}






/*日期              作者                QQ
2025-8-22            ZQ            2897424468
*/

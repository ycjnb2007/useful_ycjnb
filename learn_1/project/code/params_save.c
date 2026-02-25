#include "params_save.h"

#define PID_FLASH_PAGE       3          //使用的扇区页
#define Number_of_parameters 3          //储存的参数个数



/*********************************************************************************************************************
  *用来保存参数到flash
 ********************************************************************************************************************/


// 存储参数到Flash
void save_pid_to_flash(float params1, float params2, int16 params3)
{
    flash_erase_page(0, PID_FLASH_PAGE); // 擦除目标页
    
    flash_buffer_clear();                   // 清空缓冲区
    flash_union_buffer[0].float_type = params1;   // 存储第一个参数到缓冲区位置0
    flash_union_buffer[1].float_type = params2;   // 存储第二个参数到缓冲区位置1
    flash_union_buffer[2].int16_type = params3;   // 存储第三个参数到缓冲区位置2
    
    // 写入数据
    flash_write_page_from_buffer(0, PID_FLASH_PAGE, Number_of_parameters);
}


// 从Flash读取参数
void read_pid_from_flash(float *params1, float *params2, int16 *params3)
{
    // 读取数据
    flash_read_page_to_buffer(0, PID_FLASH_PAGE, Number_of_parameters);
    
    *params1 = flash_union_buffer[0].float_type;  // 从缓冲区位置0读取第一个参数
    *params2 = flash_union_buffer[1].float_type;  // 从缓冲区位置1读取第二个参数
    *params3 = flash_union_buffer[2].int16_type;  // 从缓冲区位置2读取第三个参数
}


void load_params_from_flash(void)
{
  
      //    if(gpio_get_level(KEY4_PIN))        //按住"KEY4_PIN"再开机，不从flash读取数据     
      //    {
      //        read_pid_from_flash(&position_p,&position_d,&except_speed);
      //    }   
          
          // 从Flash读取白点值
          if(!load_white_values_from_flash())
          {
              // 如果读取失败，初始化正常白点值为0
              for(int i = 0; i < IMAGE_H; i++)
              {
                  normal_white_per_row[i] = 0;
              }
          }  
          
              // 从Flash读取摄像头参数
          if(!load_camera_params_from_flash())
          {
              // 如果读取失败，使用默认值
              MT9V03X_EXP_TIME_DEF = 400;
              MT9V03X_FPS_DEF = 200;
          }
          
}


/**
 * @file my_flash.c
 * @brief Flash存储
 * @author 尘烟
 */
#ifndef __MY_FLASH_H__
#define __MY_FLASH_H__

#include "zf_common_headfile.h"
#include "zf_driver_flash.h"

// Flash 存储配置
#define FLASH_SECTION_INDEX 0 // flash 扇区索引 - 保持0即可，仅为接口预留
#define FLASH_PAGE_INDEX 5    // flash 页索引 - 可根据需要修改，范围<0-47>

// 参数数据结构体 - 仿照Flash_save.h的DataInfomationType
typedef struct
{
  // 角度环PID参数
  float angle_kp;
  float angle_ki;
  float angle_kd;

  // 角速度环PID参数
  float gyro_kp;
  float gyro_ki;
  float gyro_kd;

  // 速度环PID参数
  float speed_kp;
  float speed_ki;
  float speed_kd;

  // 期望速度参数
  uint16 normal_speed;   // 正常速度
  uint16 straight_speed; // 直道速度
  uint16 curve_speed;    // 弯道速度
  uint16 circle_speed;   // 环岛速度
  uint16 break_speed;    // 断路速度

  // 标志位变量(预留8个)
  uint8 flag_1;
  uint8 flag_2;
  uint8 flag_3;
  uint8 flag_4;
  uint8 flag_5;
  uint8 flag_6;
  uint8 flag_7;
  uint8 flag_8;

  // 校验码
  uint32 check_code;
} ParameterDataType;

extern ParameterDataType ParamData;
extern uint8 Write_Flash_Flag;
extern uint8 Read_Flash_Flag;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     将参数写入flash
  @param     null
  @return    null
  Sample     flash_write_params();
  @note      将各项参数写入flash
-------------------------------------------------------------------------------------------------------------------*/
void flash_write_params(void);

/*-------------------------------------------------------------------------------------------------------------------
  @brief     从flash读取参数
  @param     null
  @return    null
  Sample     flash_read_params();
  @note      将各项参数从flash中读取
-------------------------------------------------------------------------------------------------------------------*/
void flash_read_params(void);
void flash_param_init(void);
void save_all_data(void); // 保存所有数据(仿照Flash_save.h)
void read_all_data(void); // 读取所有数据(仿照Flash_save.h)

#endif /* __MY_FLASH_H__ */

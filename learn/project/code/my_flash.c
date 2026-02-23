/**
 * @file my_flash.c
 * @brief Flash存储
 * @author 尘烟
 */
#include "my_flash.h"

uint8 Write_Flash_Flag = 0;
uint8 Read_Flash_Flag = 0;
// 参数数据实例
ParameterDataType ParamData;

// 校验码定义
#define PARAM_CHECK_CODE 0x12345678

/*-------------------------------------------------------------------------------------------------------------------
  @brief     保存所有数据 - 仿照Flash_save.h的Save_All_Data函数
  @param     null
  @return    null
  Sample     save_all_data();
  @note      将所有参数保存到结构体并写入flash
-------------------------------------------------------------------------------------------------------------------*/
void save_all_data(void)
{
    // 将当前参数保存到数据结构体
    ParamData.angle_kp = angle_kp;
    ParamData.angle_ki = angle_ki;
    ParamData.angle_kd = angle_kd;

    ParamData.gyro_kp = gyro_kp;
    ParamData.gyro_ki = gyro_ki;
    ParamData.gyro_kd = gyro_kd;

    ParamData.speed_kp = speed_kp;
    ParamData.speed_ki = speed_ki;
    ParamData.speed_kd = speed_kd;

    ParamData.normal_speed = normal_speed;
    ParamData.straight_speed = straight_speed;
    ParamData.curve_speed = curve_speed;
    ParamData.circle_speed = circle_speed;
    ParamData.break_speed = break_speed;

    ParamData.flag_1 = flag_1;
    ParamData.flag_2 = flag_2;
    ParamData.flag_3 = flag_3;
    ParamData.flag_4 = flag_4;
    ParamData.flag_5 = flag_5;
    ParamData.flag_6 = flag_6;
    ParamData.flag_7 = flag_7;
    ParamData.flag_8 = flag_8;
    ParamData.check_code = PARAM_CHECK_CODE;

    // 清空Flash缓冲区
    flash_buffer_clear();
    
    // 将数据复制到Flash缓冲区
    memcpy(flash_union_buffer, &ParamData, sizeof(ParameterDataType));

    // 检查并擦除页面
    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) == 1)
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    }

    // 写入Flash
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX,
                                 sizeof(ParameterDataType) / sizeof(uint32));
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     读取所有数据 - 仿照Flash_save.h的Read_All_Data函数
  @param     null
  @return    null
  Sample     read_all_data();
  @note      从flash读取所有参数到结构体
-------------------------------------------------------------------------------------------------------------------*/
void read_all_data(void)
{
    // 从Flash读取数据到缓冲区
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX,
                              sizeof(ParameterDataType) / sizeof(uint32));

    // 从Flash缓冲区复制数据
    memcpy(&ParamData, flash_union_buffer, sizeof(ParameterDataType));

    // 检查校验码
    if (ParamData.check_code != PARAM_CHECK_CODE)
    {
        // 校验失败，使用默认值
        return;
    }

    // 校验成功，更新参数
    angle_kp = ParamData.angle_kp;
    angle_ki = ParamData.angle_ki;
    angle_kd = ParamData.angle_kd;

    gyro_kp = ParamData.gyro_kp;
    gyro_ki = ParamData.gyro_ki;
    gyro_kd = ParamData.gyro_kd;

    speed_kp = ParamData.speed_kp;
    speed_ki = ParamData.speed_ki;
    speed_kd = ParamData.speed_kd;

    normal_speed = ParamData.normal_speed;
    straight_speed = ParamData.straight_speed;
    curve_speed = ParamData.curve_speed;
    circle_speed = ParamData.circle_speed;
    break_speed = ParamData.break_speed;

    flag_1 = ParamData.flag_1;
    flag_2 = ParamData.flag_2;
    flag_3 = ParamData.flag_3;
    flag_4 = ParamData.flag_4;
    flag_5 = ParamData.flag_5;
    flag_6 = ParamData.flag_6;
    flag_7 = ParamData.flag_7;
    flag_8 = ParamData.flag_8;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     将参数写入flash
  @param     null
  @return    null
  Sample     flash_write_params();
  @note      将各项参数写入flash - 主循环调用的版本
-------------------------------------------------------------------------------------------------------------------*/
void flash_write_params(void)
{
    // 先检查标志位
    if (Write_Flash_Flag == 0)
        return;
    Write_Flash_Flag = 0; // 清除标志位

    // 显示正在保存
    tft180_show_string(0, 80, "Saving...");

    // 检查页面是否有数据，如果有则先擦除
    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    }

    // 保存所有数据
    save_all_data();

    // 显示保存结果
    tft180_show_string(0, 80, "Save Success!");
    system_delay_ms(500);

    show_menu_flag = 1; // 重新显示菜单
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     从flash读取参数
  @param     null
  @return    null
  Sample     flash_read_params();
  @note      将各项参数从flash中读取 - 主循环调用的版本
-------------------------------------------------------------------------------------------------------------------*/
void flash_read_params(void)
{
    if (Read_Flash_Flag == 0)
        return;
    Read_Flash_Flag = 0;

    // 显示正在读取
    tft180_show_string(0, 80, "Loading...");

    // 检查该页是否有数据
    if (!flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        // 页为空，没有数据可读
        tft180_show_string(0, 80, "No Data!");
        system_delay_ms(500);
        show_menu_flag = 1;
        return;
    }

    // 读取所有数据
    read_all_data();

    // 检查是否读取成功(通过校验码)
    if (ParamData.check_code == PARAM_CHECK_CODE)
    {
        tft180_show_string(0, 80, "Load Success!");
    }
    else
    {
        tft180_show_string(0, 80, "Invalid Data!");
    }

    system_delay_ms(500);
    show_menu_flag = 1;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     初始化Flash并加载参数 - 仿照Flash_save.h的FlashData_Init函数
  @param     null
  @return    null
  Sample     flash_param_init();
  @note      初始化Flash并尝试加载参数
-------------------------------------------------------------------------------------------------------------------*/
void flash_param_init(void)
{
    // 初始化 Flash
    flash_init();

    // 检查是否有参数数据
    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        // 有数据，尝试读取
        tft180_show_string(0, 80, "Loading params...");
        system_delay_ms(300);

        read_all_data();

        // 检查校验码
        if (ParamData.check_code == PARAM_CHECK_CODE)
        {
            tft180_show_string(0, 80, "Params loaded");
            system_delay_ms(500);
        }
        else
        {
            tft180_show_string(0, 80, "Use defaults");
            system_delay_ms(500);
        }
    }
    else
    {
        // 没有数据，使用默认参数
        tft180_show_string(0, 80, "Use defaults");
        system_delay_ms(500);
    }
}

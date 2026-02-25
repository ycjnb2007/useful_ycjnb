#include "zf_common_headfile.h"




CarStateForBreak car_state_current = STATE_WRITE;
uint16 Break_road_write_select[BREAK_ROAD_MAX_COUNT];          //写入flash的舵机不同区域打角的数组
uint16 Break_road_read_select[BREAK_ROAD_MAX_COUNT];           //读出flash的舵机不同区域打角的数组


//写入断路区1的舵机打角
void flash_break_road_1_memery_angle(void)
{
    // 第一步清除缓冲区
    flash_buffer_clear();
    // 第二步把数据存到缓冲区
    flash_union_buffer[0].uint16_type = Break_road_write_select[0]; 
    // 第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
    if (flash_check(FLASH_SECTION_INDEX, ROAD_ANGLE_1))
    { // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, ROAD_ANGLE_1);
    } // 擦除这一页
    // 第四步存到FLASH指定扇区
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, ROAD_ANGLE_1, 1);
    
    //置写入完成标志位
    write_flag_finish1 = 1;
    
}

//写入断路区2的舵机打角
void flash_break_road_2_memery_angle(void)
{
    // 第一步清除缓冲区
    flash_buffer_clear();
    // 第二步把数据存到缓冲区
    flash_union_buffer[0].uint16_type = Break_road_write_select[1]; 
    // 第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
    if (flash_check(FLASH_SECTION_INDEX, ROAD_ANGLE_2))
    { // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, ROAD_ANGLE_2);
    } // 擦除这一页
    // 第四步存到FLASH指定扇区
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, ROAD_ANGLE_2, 1);
    
    //置写入完成标志位
    write_flag_finish2 = 1;
    
}

//写入断路区3的舵机打角
void flash_break_road_3_memery_angle(void)
{
    // 第一步清除缓冲区
    flash_buffer_clear();
    // 第二步把数据存到缓冲区
    flash_union_buffer[0].uint16_type = Break_road_write_select[2]; 
    // 第三步判断FLASH里有没有数据，有就把FLASH数据擦除/把数据存到缓冲区
    if (flash_check(FLASH_SECTION_INDEX, ROAD_ANGLE_3))
    { // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX, ROAD_ANGLE_3);
    } // 擦除这一页
    // 第四步存到FLASH指定扇区
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, ROAD_ANGLE_3, 1);
    
    //置写入完成标志位
    write_flag_finish3 = 1;
    
}

// 取断路区1的舵机打角数据
void flash_road_1_read_get(void)
{
    // 检查flash里面是否存了数据
    if (flash_check(FLASH_SECTION_INDEX, ROAD_ANGLE_1)) 
    {
        //把舵机打角放到缓存区中
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, ROAD_ANGLE_1, 1);
        //赋值
        Break_road_read_select[0] = flash_union_buffer[0].uint16_type; // 取出历史偏航角

    }
    
}

// 取断路区2的舵机打角数据
void flash_road_2_read_get(void)
{
    // 检查flash里面是否存了数据
    if (flash_check(FLASH_SECTION_INDEX, ROAD_ANGLE_2)) 
    {
        //把舵机打角放到缓存区中
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, ROAD_ANGLE_2, 1);
        //赋值
        Break_road_read_select[1] = flash_union_buffer[0].uint16_type; // 取出历史偏航角

    }
    
}

// 取断路区2的舵机打角数据
void flash_road_3_read_get(void)
{
    // 检查flash里面是否存了数据
    if (flash_check(FLASH_SECTION_INDEX, ROAD_ANGLE_3)) 
    {
        //把舵机打角放到缓存区中
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, ROAD_ANGLE_3, 1);
        //赋值
        Break_road_read_select[2] = flash_union_buffer[0].uint16_type; // 取出历史偏航角

    }
    
}

void flash_road_read_get_all(void)
{
    flash_road_1_read_get();

    flash_road_2_read_get();

    flash_road_3_read_get();
}

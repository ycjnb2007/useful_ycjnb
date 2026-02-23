#ifndef _flash_h_
#define _flash_h_

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_YAW_PAGE_INDEX          (10)                                // 存储偏航角用的页码 

#define Flash_data_count        90     


//把数据从Flash放到缓存区中
void Flash_data_read();

#endif
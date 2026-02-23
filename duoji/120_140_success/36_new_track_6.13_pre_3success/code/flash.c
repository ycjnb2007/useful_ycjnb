#include "zf_common_headfile.h"


  
void Flash_data_read(void)
{
    if(flash_check(FLASH_SECTION_INDEX,FLASH_YAW_PAGE_INDEX))
    {

     flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_YAW_PAGE_INDEX,Flash_data_count);        // 将数据从 flash 读取到缓冲区
                    
    }

}
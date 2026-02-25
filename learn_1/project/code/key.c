#include "key.h"

uint8 clear_flag=0;
uint8 press_flag=0;

void mykey_init(void)
{

        gpio_init(KEY1_PIN, GPI, 0, GPI_PULL_UP);
        gpio_init(KEY2_PIN, GPI, 0, GPI_PULL_UP);
        gpio_init(KEY3_PIN, GPI, 0, GPI_PULL_UP);
        gpio_init(KEY4_PIN, GPI, 0, GPI_PULL_UP);
}

uint8 scanf_key()
{
	uint8 KeyNum=0;
	if((!gpio_get_level(KEY1_PIN)) || (!gpio_get_level(KEY2_PIN)) || (!gpio_get_level(KEY3_PIN))  || (!gpio_get_level(KEY4_PIN)))
	{
				if(press_flag >= 60)            //抖动计数，若按键按下较迟钝可减小此值
			{
					press_flag=0;
//					if(!gpio_get_level(KEY1_PIN))//由于KEY1_PIN被拿去当做旋钮解锁，所以注释
//					{
//						KeyNum=1;
//						clear_flag=1;
//					}						
					if(!gpio_get_level(KEY2_PIN))
					{
						KeyNum=2;
						clear_flag=1;
					}						
					if(!gpio_get_level(KEY3_PIN)) KeyNum=3;
					if(!gpio_get_level(KEY4_PIN)) KeyNum=4;         
			}
			press_flag += 1;
	}
	return KeyNum;
}



/*日期              作者                QQ
2025-8-22            ZQ            2897424468
*/

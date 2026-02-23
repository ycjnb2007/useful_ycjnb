#ifndef _flash_h_
#define _flash_h_



typedef enum 
{
    STATE_NORMAL,      // 正常寻迹
    STATE_WRITE,       // 记录惯性导航数据

} CarStateForBreak;

extern CarStateForBreak car_state_current;


#define FLASH_SECTION_INDEX (0)     // 存储数据用的扇区

#define ROAD_ANGLE_1            (11)                            //写入第一个断路的舵机打角（舵机PWM）
#define ROAD_ANGLE_2            (12)                            //写入第二个断路的舵机打角（舵机PWM）
#define ROAD_ANGLE_3            (13)                            //写入第三个断路的舵机打角（舵机PWM）        

#define BREAK_ROAD_MAX_COUNT    (3)

extern uint16 Break_road_write_select[BREAK_ROAD_MAX_COUNT];          //写入flash的舵机不同区域打角的数组
extern uint16 Break_road_read_select[BREAK_ROAD_MAX_COUNT];           //读出flash的舵机不同区域打角的数组


void flash_break_road_1_memery_angle(void);
void flash_break_road_2_memery_angle(void);
void flash_break_road_3_memery_angle(void);

void flash_road_1_read_get(void);
void flash_road_2_read_get(void);
void flash_road_3_read_get(void);
void flash_road_read_get_all(void);
                                   
#endif
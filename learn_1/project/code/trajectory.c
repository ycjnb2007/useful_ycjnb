#include "trajectory.h"

//#define  break_number  1       //断路个数
#define  MAX_BREAK_NUMBER 3    //最大断路数

float yaw_record[MAX_TRAJECTORY_POINTS];         //储存的yaw数组
float yaw_record1[MAX_TRAJECTORY_POINTS];        //第1断路区
float yaw_record2[MAX_TRAJECTORY_POINTS];        //第2断路区
float yaw_record3[MAX_TRAJECTORY_POINTS];        //第3断路区
uint16_t yaw_record_count[MAX_BREAK_NUMBER] = {0}; //每个断路区的轨迹点数
float target_yaw = 0;                            //目标偏航角
uint8_t current_replay_page = 4;                 // 当前回放页（从第四页开始存数据）
uint8_t current_break_index = 0;                // 当前回放断路区编号（0/1/2）
uint8_t flash_page=0;                            //写入flash的页数（从第四页开始存数据）
uint16 record_index = 0;                         //记录索引
uint16 replay_index = 0;                         //回放索引
uint8 stop_record_flag=0;                        //停止记录标志
uint8 stop_replay_flag=0;                        //停止回放标志
uint8 currnt_recoder=0;                         //当前记录轨迹的个数

uint8_t break_number = 1;

int32 total_pulses = 0;  // 添加全局变量

void trajectory_processing(void)
{
    static uint8_t break_count = 0;               // 断路区计数器（每储存完一个断路区加一）
    // 记录模式处理逻辑

//      printf("%d,%d\r\n",record_index,replay_index);

    if(record_mode||replay_mode)
    {
            encoder_add();
    }

    if(record_mode && total_pulses >= 2500)        //大于脉冲计数值则记录一个轨迹点
    {
      
        if(record_index == 0)  // 在开始记录第一个点时重置姿态
        {
            quaternion_reset();  // 重置姿态数据，以当前位置为基准点
        }
      
      
        if(record_index < MAX_TRAJECTORY_POINTS) 
        {
            yaw_record[record_index++] = get_euler_angles()->yaw;
            total_pulses = 0;                    // 重置脉冲计数
        } 
        else                                      // 当记录完成时存储到flash
        {
           stop_record_flag=1;                   // 写满则停止记录 
        }
    }
    
    
    if(stop_record_flag)                         //记录完成后将轨迹点写入flash
    {
            stop_record_flag=0;
            record_mode = 0;                     
            flash_page = 4 + break_count;        // 根据断路区编号选择存储页（4,5,6）
            flash_buffer_clear();                //清空缓冲区
            for(int i=0; i<record_index; i++)
            {
                flash_union_buffer[i].float_type = yaw_record[i];                // 写入轨迹数据到缓冲区
            }
            
            flash_union_buffer[MAX_TRAJECTORY_POINTS].uint32_type = record_index;// 在最后一个位置存储轨迹点数（uint32类型）
            
            if(flash_check(0, flash_page))
                flash_erase_page(0, flash_page);
            
            flash_write_page_from_buffer(0, flash_page, MAX_TRAJECTORY_POINTS + 1);// 写入512个数据（510轨迹点 + 1个点数，最后一位保留）
            
            break_count = (break_count + 1) % 3;
            currnt_recoder=break_count;
            record_index = 0;
    }
    
    
        // 回放逻辑
    if(replay_mode)
    {
        uint16_t cur_count = yaw_record_count[current_break_index];
        float* cur_yaw_record = NULL;
        if(current_break_index == 0) cur_yaw_record = yaw_record1;
        else if(current_break_index == 1) cur_yaw_record = yaw_record2;
        else if(current_break_index == 2) cur_yaw_record = yaw_record3;

        if(replay_index == 0)
        {
            quaternion_reset();  // 在开始回放时重置姿态数据，以当前位置为基准点
        }

        if(total_pulses >= 2500)
        {
            total_pulses = 0;
            if(replay_index < cur_count-1)
                replay_index++;
        }

        if(cur_yaw_record && cur_count > 0)
            target_yaw = cur_yaw_record[replay_index];
        else
            target_yaw = 0;

        if(replay_index >= cur_count-1 || (stop_replay_flag==1))
        {
            current_break_index++;
            if(current_break_index >= break_number)
                current_break_index = 0; // 循环回放
            replay_index = 0;
            replay_mode = 0;
            break_state = BREAK_NONE;
            stop_replay_flag=0;
        }
    }

}


// 轨迹初始化函数，从flash读取所有断路区轨迹点
void trajectory_init(void)
{
    for(int break_idx = 0; break_idx < break_number && break_idx < MAX_BREAK_NUMBER; break_idx++)
    {
        uint8_t page = 4 + break_idx;
        flash_read_page_to_buffer(0, page, FLASH_PAGE_LENGTH);
        uint32_t stored_points = flash_union_buffer[MAX_TRAJECTORY_POINTS].uint32_type;
        uint16_t count = (stored_points <= MAX_TRAJECTORY_POINTS) ? stored_points : 0;
        yaw_record_count[break_idx] = count;
        for(int i = 0; i < count; i++)
        {
            if(break_idx == 0)
                yaw_record1[i] = flash_union_buffer[i].float_type;
            else if(break_idx == 1)
                yaw_record2[i] = flash_union_buffer[i].float_type;
            else if(break_idx == 2)
                yaw_record3[i] = flash_union_buffer[i].float_type;
        }
    }
}


// 计算yaw_error差值修正函数，解决359°到0°yaw跳变导致erro计算出为其补角
float angle_difference(float current, float target) 
{
    float diff = target - current;
    return fmodf(diff + 540.0f, 360.0f) - 180.0f;
}



//int16 trajectory_replay_control(void)
//{
//    float current_yaw = get_euler_angles()->yaw;
//    
//    // 计算未来10个记录点的索引
//    uint16 future_index = replay_index + 2;
//    // 处理索引越界情况
//    if (future_index >= record_index)
//    {
//        future_index = (record_index > 0) ? (record_index - 1) : 0;
//    }
//    // 计算加权目标yaw（0.3*当前 + 0.7*未来）
//    float weighted_target = 0.5f * yaw_record[replay_index] + 0.5f * yaw_record[future_index];
//    // 计算yaw误差
//    float yaw_error = angle_difference(current_yaw, weighted_target);
//    // 将误差调整到[-180, 180)范围
//    yaw_error = fmodf(yaw_error + 180.0f, 360.0f) - 180.0f;
//    
//    return yaw_error;
//}


int16 trajectory_replay_control(void)
{
    float current_yaw = get_euler_angles()->yaw;
    float* cur_yaw_record = NULL;
    uint16_t cur_count = yaw_record_count[current_break_index];
    if(current_break_index == 0) cur_yaw_record = yaw_record1;
    else if(current_break_index == 1) cur_yaw_record = yaw_record2;
    else if(current_break_index == 2) cur_yaw_record = yaw_record3;
    float cur_target_yaw = 0;
    if(cur_yaw_record && cur_count > 0)
        cur_target_yaw = cur_yaw_record[replay_index];
    float yaw_error = angle_difference(current_yaw, cur_target_yaw);
    yaw_error = fmodf(yaw_error + 180.0f, 360.0f) - 180.0f;    // 将误差调整到[-180, 180)范围
    return yaw_error;
}

/*日期              作者                QQ
2025-8-22            ZQ            2897424468
*/





/**
 * @file break_road.c
 * @brief 断路检测与处理模块
 * @details 实现多段断路记录、断路检测等功能
 * @note - 多段断路数据管理和自动切换
 *       - 断路检测和状态机控制
 *       - 断路冲刺速度和角度控制
 *       - 断路距离记录和回放
 * @author 尘烟
 */
#include "break_road.h"

#define stance 4.0f // 断路检测距离阈值

// 多段断路管理结构
MULTI_BREAK_ROAD MultiBreak;

// 兼容性变量（指向当前活跃断路段）
BREAK_ROAD Break;

// 简单计数器方案
uint8 break_usage_counter = 0;  // 开机初始化为0

/*
 *   断路数据自动切换逻辑：
 * - 开机时：break_usage_counter = 0，准备使用B1
 * - 第一次断路完成后：counter变为1，下次使用B2
 * - 第二次断路完成后：counter变为2，下次使用B3
 * - 第三次断路完成后：counter变为3，下次使用B4
 * - 第四次断路完成后：counter变为0，下次使用B1（循环）
 * 
 * 如果目标段没有数据，自动回退到B1
 */

// 调试和控制变量
float target_yaw;
float current_yaw;

// 当前活跃段的工作变量
static float distance_for_yaw = 0.0f;
static float last_break_dis = 0.0f;
static float break_cha_yaw = 0.0f; // 断路开始时的偏航角基准值

// 断路控制距离跟踪变量（与记录阶段对应）
#define BREAK_DETECTION_DELAY_OFFSET 2                         // 断路检测延迟偏移索引
#define USE_DYNAMIC_OFFSET 0                                   // 1:使用动态偏移, 0:使用固定偏移
static uint8 current_yaw_index = BREAK_DETECTION_DELAY_OFFSET; // 当前yaw索引（考虑检测延迟）
static float break_control_distance = 0.0f;                    // 控制距离累积器
static float last_break_control_dis = 0.0f;                    // 上次控制距离

void break_road_init(void)
{
    // 简单计数器方案：开机初始化
    break_usage_counter = 0;  // 0表示第一次断路使用B1
    
    // 调用多段断路初始化（仍需要加载各段数据）
    multi_break_init();
    
    // 设置初始活跃段为B1
    multi_break_set_active_segment(1);
    
    printf("Break system initialized: B1 ready (counter: %d)\n", break_usage_counter);
}

void break_road_check(void)
{
    // 检测到断路的处理逻辑
    if (ImageFlag.break_road == 1)
    {
        // 如果在提前记录状态下检测到断路，标记原断路已检测到
        if (Break.pre_break_state == 1)
        {
            Break.original_break_detected = 1;
            Break.pre_break_state = 0; // 退出提前状态
            printf("Original break road detected, exiting pre-break state\n");
        }

        // 只在第一次检测到断路时开始记录距离
        if (Break.have == 0 && top_right_white < 8 && top_left_white < 8 && bottom_right_white < 8 && bottom_left_white < 8)
        {
            Break.have = 1;
            BEE_STA(1);
            // 检查是否为记录模式（停车且在记录页面）还是控制模式（跑车）
            if (system_running == 0 && current_page == MENU_BREAK_RECORD_PAGE)
            {
                // 记录模式：清零数据，重新记录
                start_break_road_recording();
                if (MultiBreak.current_active_segment > 0)
                {
                    MultiBreak.segments[MultiBreak.current_active_segment - 1].yaw_record_count = 0;
                }
                distance_for_yaw = 0.0f;
                last_break_dis = 0.0f;
            }
            else
            {
                // 控制模式：使用已有数据，不清零
                start_break_road_recording(); // 仍需要记录距离
                // 不清零yaw_record_count，保持已加载的数据
                distance_for_yaw = 0.0f;
                last_break_dis = 0.0f;
                // 验证数据加载情况
                if (MultiBreak.current_active_segment > 0)
                {
                    BREAK_ROAD_SEGMENT* active_segment = &MultiBreak.segments[MultiBreak.current_active_segment - 1];
                    // printf("Control mode: loaded %d yaw records\n", active_segment->yaw_record_count);
                }
                // 计算初始距离偏移（如果需要动态调整）
                float initial_distance = break_dis;
                uint8 calculated_offset = (uint8)(initial_distance / 4.0f); // 每4cm一个索引
                // printf("Initial distance: %.1fcm, calculated offset: %d, using offset: %d\n",
                    //    initial_distance, calculated_offset, BREAK_DETECTION_DELAY_OFFSET);
            }

            break_cha_yaw = eulerAngle.yaw; // 记录断路开始时的偏航角

            reset_break_control_tracking(); // 初始化断路控制距离跟踪
        }
    }

    if (Break.have == 1)
    {
        if (system_running == 0 && current_page == MENU_BREAK_RECORD_PAGE)
        {
            // 更新偏航角记录
            update_yaw_records();

            // 提前断路记录状态检测机制
            if (Break.pre_break_state == 1)
            {
                // 提前状态下等待原断路出现，不进行退出判断
                return; // 跳过退出条件检查
            }

            // 手动启动的延迟检测机制（保留作为备用）
            if (Break.manual_started == 1)
            {
                Break.exit_delay_counter++;
                // 延迟3秒(600个5ms周期)再开始检测退出条件
                if (Break.exit_delay_counter < 600) 
                {
                    return; // 跳过退出条件检查
                }
                else
                {
                    // 延迟结束，转为正常检测模式
                    Break.manual_started = 0;
                    Break.exit_delay_counter = 0;
                }
            }

            // 检查是否退出断路状态 - 只有在检测到原断路后才能退出
            if ((Break.original_break_detected == 1 || Break.manual_started == 0) && 
                Right_line_count > 30 && Left_line_count > 30 && judge.bottom_area_true == 1)
            {
                Break.have = 0;
                Break.manual_started = 0;            // 重置手动启动标志
                Break.exit_delay_counter = 0;        // 重置延迟计数器
                Break.pre_break_state = 0;           // 重置提前状态
                Break.original_break_detected = 0;   // 重置原断路检测标志
                stop_break_road_recording();         // 停止记录距离
                
                // 保存总距离到当前活跃段
                if (MultiBreak.current_active_segment > 0)
                {
                    MultiBreak.segments[MultiBreak.current_active_segment - 1].last_total_distance = break_dis * 2;
                }
                
                print_yaw_records();                 // 打印偏航角数据
                yaw_data_storage();                  // 存储偏航角数据到Flash
                
                // 断路完成后推进到下一个断路段
                simple_break_advance();
                
                BEE_STA(0);
            }
        }
        else if (system_running == 1)
        {
            // get_current_target_yaw();
            // 如果不在断路记录页面，则停止记录
            if (Right_line_count > 30 && Left_line_count > 30)
            {
                Break.have = 0;
                stop_break_road_recording();         // 停止记录距离
                
                // 保存总距离到当前活跃段（单位：cm）
                if (MultiBreak.current_active_segment > 0)
                {
                    MultiBreak.segments[MultiBreak.current_active_segment - 1].last_total_distance = break_dis * 2;
                }
                
                // 断路完成后推进到下一个断路段
                simple_break_advance();
                
                // print_yaw_records();                 // 打印偏航角数据
                BEE_STA(0);
            }
        }
    }
}

// 偏航角记录更新函数（兼容性函数，操作当前活跃段）
void update_yaw_records(void)
{
    if (MultiBreak.current_active_segment == 0)
        return;
    
    BREAK_ROAD_SEGMENT* active_segment = &MultiBreak.segments[MultiBreak.current_active_segment - 1];
    
    // 检查距离是否增加了
    float current_distance = break_dis;
    float distance_increment = current_distance - last_break_dis;
    distance_for_yaw += distance_increment;
    last_break_dis = current_distance;

    // 每2cm记录一次偏航角（与控制频率保持一致）
    if (distance_for_yaw >= stance && active_segment->yaw_record_count < MAX_YAW_RECORDS)
    {
        active_segment->yaw_records[active_segment->yaw_record_count] = eulerAngle.yaw - break_cha_yaw;
        active_segment->yaw_record_count++;
        distance_for_yaw = 0.0f; // 重置计数器
        
        // 同步更新Break兼容性变量（如果需要）
        Break = *active_segment;
    }
}

// 打印偏航角数据（兼容性函数，打印当前活跃段）
void print_yaw_records(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        printf("No active breakroad segment\n");
        return;
    }
    
    multi_break_print_data(MultiBreak.current_active_segment);
}

// 偏航角数据存储到Flash（兼容性函数，存储当前活跃段）
void yaw_data_storage(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        printf("No active breakroad segment to save\n");
        return;
    }
    
    multi_break_save_data(MultiBreak.current_active_segment);
}

// 从Flash读取偏航角数据（兼容性函数，加载当前活跃段）
void yaw_data_load(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        printf("No active breakroad segment to load\n");
        return;
    }
    
    multi_break_load_data(MultiBreak.current_active_segment);
    
    // 更新兼容性变量Break
    Break = MultiBreak.segments[MultiBreak.current_active_segment - 1];
}

// 打印从Flash读取的偏航角数据（兼容性函数，打印当前活跃段）
void print_loaded_yaw_records(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        printf("No active breakroad segment\n");
        return;
    }
    
    printf("Loaded ");
    multi_break_print_data(MultiBreak.current_active_segment);
}

// 断路控制专用PID结构体
typedef struct
{
    float Kp, Ki, Kd;
    float error, last_error, integral;
    float output_max, output_min;
    float output;
} Break_PID_TypeDef;

// 断路偏航角PID控制器
static Break_PID_TypeDef break_yaw_pid = {3.0f, 0.00f, 0.1f, 0, 0, 0, 60.0f, -60.0f, 0};

// 断路控制变量
static float break_target_speed = 60.0f; // 断路基础速度（降低速度提高精度）
float break_speed_diff = 0.0f;           // 偏航角PID输出的速度差值

// 调参用改成了全局变量
float target_yaw = 0.0f;
float current_yaw = 0.0f;

/**
 * @brief 断路PID控制算法
 * @param pid PID控制器指针
 * @param error 误差值
 * @return PID输出
 */
float break_pid_calculate(Break_PID_TypeDef *pid, float error)
{
    pid->error = error;

    // 积分计算
    pid->integral += error * 0.005f; // 5ms控制周期

    // 积分限幅防饱和
    if (pid->integral > 10.0f)
        pid->integral = 10.0f;
    if (pid->integral < -10.0f)
        pid->integral = -10.0f;

    // 微分计算
    float derivative = (error - pid->last_error) / 0.005f;

    // PID输出计算
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 输出限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    pid->last_error = error;
    return pid->output;
}

// 断路控制主函数
void break_control(void)
{
    // 检查断路状态和数据有效性
    if (!Break.have)
    {
        break_speed_diff = 0.0f;
        return;
    }

    // 获取目标偏航角（采用类似偏航角记录的逻辑）
    target_yaw = get_current_target_yaw();

    // 获取当前偏航角（相对于断路开始时的角度）
    current_yaw = eulerAngle.yaw - break_cha_yaw;

    // 处理角度跳变
    float yaw_error = target_yaw - current_yaw;
    if (yaw_error > 180.0f)
        yaw_error -= 360.0f;
    if (yaw_error < -180.0f)
        yaw_error += 360.0f;

    // 偏航角PID控制 -> 速度差值
    break_speed_diff = break_pid_calculate(&break_yaw_pid, yaw_error);

    // 计算左右轮目标速度
    float break_left_speed = break_speed - break_speed_diff;
    float break_right_speed = break_speed + break_speed_diff;

    // 速度限幅（防止轮子停转或过快）
    if (break_left_speed > 80.0f)
        break_left_speed = 80.0f;
    if (break_left_speed < 20.0f)
        break_left_speed = 20.0f;
    if (break_right_speed > 80.0f)
        break_right_speed = 80.0f;
    if (break_right_speed < 20.0f)
        break_right_speed = 20.0f;

    // 调用现有的速度PID控制器
    extern Incremental_PID_TypeDef pid_left, pid_right;
    extern float output_left, output_right;

    float left_speed_error = break_left_speed - Encoer_Speed[0];
    float right_speed_error = break_right_speed - Encoer_Speed[1];

    output_left = Get_Incremental_PID_Value(&pid_left, left_speed_error);
    output_right = Get_Incremental_PID_Value(&pid_right, right_speed_error);
}

// 清零yaw_records数组（兼容性函数，清零当前活跃段）
void clear_yaw_records(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        printf("No active breakroad segment to clear\n");
        return;
    }
    
    multi_break_clear_data(MultiBreak.current_active_segment);
    
    // 重置工作变量
    distance_for_yaw = 0.0f;
    last_break_dis = 0.0f;
    
    // 更新兼容性变量Break
    Break = MultiBreak.segments[MultiBreak.current_active_segment - 1];
}

/**
 * @brief 获取当前目标偏航角 - 采用与记录阶段相同的距离跟踪逻辑
 * @return 当前距离对应的目标偏航角
 */
float get_current_target_yaw(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        return 0.0f; // 没有激活的段
    }
    
    BREAK_ROAD_SEGMENT* active_segment = &MultiBreak.segments[MultiBreak.current_active_segment - 1];
    
    // 距离计算
    float current_distance = break_dis;
    float distance_increment = current_distance - last_break_control_dis;
    break_control_distance += distance_increment;
    last_break_control_dis = current_distance;

    if (break_control_distance >= stance) // 每2cm前进一个索引
    {
        if (current_yaw_index < active_segment->yaw_record_count - 1)
        {
            current_yaw_index++;
        }

        break_control_distance = 0.0f; // 重置计数器
    }

    // 边界检查并返回目标值
    if (current_yaw_index >= active_segment->yaw_record_count)
    {
        current_yaw_index = active_segment->yaw_record_count - 1;
    }

    // 调试输出（验证距离对应关系）
    // static uint8 debug_counter = 0;
    // if (++debug_counter >= 20)
    // { // 每100ms输出一次（5ms*20）
    //     printf("[BREAK%d] idx=%d/%d, dist=%.1f, target=%.1f\n",
    //            MultiBreak.current_active_segment, current_yaw_index, active_segment->yaw_record_count, 
    //            break_control_distance, active_segment->yaw_records[current_yaw_index]);
    //     debug_counter = 0;
    // }

    if (active_segment->yaw_record_count == 0)
    {
        return 0.0f;
    }

    return active_segment->yaw_records[current_yaw_index];
}

/**
 * @brief 重置断路控制距离跟踪变量，有更好的位置，待优化
 */
void reset_break_control_tracking(void)
{
#if USE_DYNAMIC_OFFSET
    // 动态计算偏移（基于当前距离）
    float initial_distance = break_dis;
    uint8 dynamic_offset = (uint8)(initial_distance / 4.0f); // 每4cm一个索引
    current_yaw_index = dynamic_offset;
    printf("Dynamic offset: %.1fcm -> index %d\n", initial_distance, dynamic_offset);
#else
    // 使用固定偏移
    current_yaw_index = BREAK_DETECTION_DELAY_OFFSET;
#endif
    break_control_distance = 0.0f; // 重置距离累积器
    last_break_control_dis = 0.0f; // 重置上次距离
}

void manual_start_break_recording(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        printf("No active breakroad segment for manual recording\n");
        return;
    }
    
    BREAK_ROAD_SEGMENT* active_segment = &MultiBreak.segments[MultiBreak.current_active_segment - 1];
    
    // 手动开始断路记录的专用函数
    active_segment->have = 1;
    active_segment->manual_started = 1;        // 标记为手动启动
    active_segment->exit_delay_counter = 0;    // 重置延迟计数器
    
    // 初始化记录状态
    start_break_road_recording();
    active_segment->yaw_record_count = 0;
    distance_for_yaw = 0.0f;
    last_break_dis = 0.0f;
    break_cha_yaw = eulerAngle.yaw;  // 记录起始偏航角
    reset_break_control_tracking();
    
    // 同步兼容性变量
    Break = *active_segment;
}

void start_pre_break_recording(void)
{
    if (MultiBreak.current_active_segment == 0)
    {
        printf("No active breakroad segment for pre-break recording\n");
        return;
    }
    
    BREAK_ROAD_SEGMENT* active_segment = &MultiBreak.segments[MultiBreak.current_active_segment - 1];
    
    // 开始提前断路记录状态
    active_segment->have = 1;
    active_segment->pre_break_state = 1;       // 标记为提前记录状态
    active_segment->original_break_detected = 0; // 重置原断路检测标志
    active_segment->manual_started = 0;        // 不使用手动启动机制
    active_segment->exit_delay_counter = 0;    // 重置延迟计数器
    
    // 初始化记录状态
    start_break_road_recording();
    active_segment->yaw_record_count = 0;
    distance_for_yaw = 0.0f;
    last_break_dis = 0.0f;
    break_cha_yaw = eulerAngle.yaw;  // 记录起始偏航角
    reset_break_control_tracking();
    
    // 同步兼容性变量
    Break = *active_segment;
    
    BEE_STA(1); // 蜂鸣器提示
    // printf("Pre-break recording started, waiting for original break detection\n");
}

// =================================================================
// 多段断路管理函数实现
// =================================================================

/**
 * @brief 多段断路系统初始化
 */
void multi_break_init(void)
{
    // 初始化多段断路结构
    memset(&MultiBreak, 0, sizeof(MULTI_BREAK_ROAD));
    
    // 设置默认执行序列：1,2,3,4
    MultiBreak.execution_sequence[0] = 1;
    MultiBreak.execution_sequence[1] = 2;
    MultiBreak.execution_sequence[2] = 3;
    MultiBreak.execution_sequence[3] = 4;
    MultiBreak.total_segments_enabled = 4;
    MultiBreak.sequence_index = 0;
    MultiBreak.current_active_segment = 0;
    MultiBreak.sequence_mode_active = 0;  // 不需要序列模式了
    
    // 初始化各个断路段
    for (uint8 i = 0; i < MAX_BREAKROADS; i++)
    {
        MultiBreak.segments[i].have = 0;
        MultiBreak.segments[i].manual_started = 0;
        MultiBreak.segments[i].exit_delay_counter = 0;
        MultiBreak.segments[i].pre_break_state = 0;
        MultiBreak.segments[i].original_break_detected = 0;
        MultiBreak.segments[i].recorded = 0;
        MultiBreak.segments[i].yaw_record_count = 0;
        MultiBreak.segments[i].last_total_distance = 0.0f;
        memset(MultiBreak.segments[i].yaw_records, 0, sizeof(MultiBreak.segments[i].yaw_records));
    }
    
    // 从Flash加载各段数据
    for (uint8 i = 1; i <= MAX_BREAKROADS; i++)
    {
        multi_break_load_data(i);
    }
    
    // 初始化工作变量
    distance_for_yaw = 0.0f;
    last_break_dis = 0.0f;
    break_cha_yaw = 0.0f;
}

/**
 * @brief 设置当前活跃的断路段
 * @param segment_id 断路段ID (1-4)
 */
void multi_break_set_active_segment(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
    {
        MultiBreak.current_active_segment = 0;
        memset(&Break, 0, sizeof(BREAK_ROAD));
        return;
    }
    
    MultiBreak.current_active_segment = segment_id;
    // 将选中的段复制到兼容性变量Break中
    Break = MultiBreak.segments[segment_id - 1];
}

/**
 * @brief 设置断路执行序列
 * @param seg1, seg2, seg3, seg4 执行序列 (1-4)
 */
void multi_break_set_sequence(uint8 seg1, uint8 seg2, uint8 seg3, uint8 seg4)
{
    MultiBreak.execution_sequence[0] = seg1;
    MultiBreak.execution_sequence[1] = seg2;
    MultiBreak.execution_sequence[2] = seg3;
    MultiBreak.execution_sequence[3] = seg4;
    
    // 计算启用的段数（非0值）
    MultiBreak.total_segments_enabled = 0;
    for (uint8 i = 0; i < MAX_BREAKROADS; i++)
    {
        if (MultiBreak.execution_sequence[i] > 0 && MultiBreak.execution_sequence[i] <= MAX_BREAKROADS)
        {
            MultiBreak.total_segments_enabled++;
        }
    }
}

/**
 * @brief 获取下一个要执行的断路段ID
 * @return 断路段ID (1-4)，0表示序列结束
 */
uint8 multi_break_get_next_segment(void)
{
    if (MultiBreak.sequence_index >= MultiBreak.total_segments_enabled)
    {
        return 0; // 序列结束
    }
    
    return MultiBreak.execution_sequence[MultiBreak.sequence_index];
}

/**
 * @brief 前进到下一个断路段
 */
void multi_break_advance_sequence(void)
{
    if (MultiBreak.sequence_index < MultiBreak.total_segments_enabled - 1)
    {
        MultiBreak.sequence_index++;
        uint8 next_segment = multi_break_get_next_segment();
        if (next_segment > 0)
        {
            multi_break_set_active_segment(next_segment);
        }
    }
}

/**
 * @brief 重置执行序列到开始
 */
void multi_break_reset_sequence(void)
{
    MultiBreak.sequence_index = 0;
    if (MultiBreak.total_segments_enabled > 0)
    {
        uint8 first_segment = multi_break_get_next_segment();
        if (first_segment > 0)
        {
            multi_break_set_active_segment(first_segment);
        }
    }
}

/**
 * @brief 启用/禁用序列模式
 * @param enable 1=启用序列模式，0=禁用
 */
void multi_break_enable_sequence_mode(uint8 enable)
{
    MultiBreak.sequence_mode_active = enable;
    if (enable)
    {
        multi_break_reset_sequence(); // 启用时重置序列
    }
}

/**
 * @brief 简单计数器断路推进方案
 */
/**
 * @brief 简单计数器断路推进方案 - 自动切换断路数据
 * @note  开机初始化为0，第一次断路使用B1，第二次使用B2，依此类推
 */
void simple_break_advance(void)
{
    // 推进计数器：0->1->2->3->0 (循环)
    break_usage_counter++;
    if (break_usage_counter > 3) {  // 计数范围：0,1,2,3
        break_usage_counter = 0;    // 循环回到0
    }
    
    // 计算要使用的段号：counter 0->B1, 1->B2, 2->B3, 3->B4
    uint8 target_segment = break_usage_counter + 1;
    
    // 检查目标段是否有数据，如果没有则回退到B1
    if (!multi_break_is_recorded(target_segment)) {
        printf("B%d has no data, falling back to B1\n", target_segment);
        break_usage_counter = 0;  // 重置计数器
        target_segment = 1;       // 使用B1
    }
    
    // 加载目标段数据
    multi_break_set_active_segment(target_segment);
    printf("Next breakroad: B%d (counter: %d)\n", target_segment, break_usage_counter);
}

/**
 * @brief 开始记录指定断路段
 * @param segment_id 断路段ID (1-4)
 */
void multi_break_start_recording(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return;
    
    multi_break_set_active_segment(segment_id);
    manual_start_break_recording(); // 调用现有的手动开始记录函数
}

/**
 * @brief 清除指定断路段的数据
 * @param segment_id 断路段ID (1-4)
 */
void multi_break_clear_data(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return;
    
    BREAK_ROAD_SEGMENT* segment = &MultiBreak.segments[segment_id - 1];
    segment->recorded = 0;
    segment->yaw_record_count = 0;
    segment->last_total_distance = 0.0f;
    memset(segment->yaw_records, 0, sizeof(segment->yaw_records));
    
    // 清除Flash中的数据
    uint8 flash_page = get_segment_flash_page(segment_id);
    if (flash_check(0, flash_page))
    {
        flash_erase_page(0, flash_page);
    }
    
    printf("Breakroad %d data cleared\n", segment_id);
}

/**
 * @brief 加载指定断路段的数据
 * @param segment_id 断路段ID (1-4)
 */
void multi_break_load_data(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return;
    
    yaw_data_load_segment(segment_id);
}

/**
 * @brief 保存指定断路段的数据
 * @param segment_id 断路段ID (1-4)
 */
void multi_break_save_data(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return;
    
    yaw_data_storage_segment(segment_id);
}

/**
 * @brief 打印指定断路段的数据
 * @param segment_id 断路段ID (1-4)
 */
void multi_break_print_data(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return;
    
    BREAK_ROAD_SEGMENT* segment = &MultiBreak.segments[segment_id - 1];
    
    if (segment->yaw_record_count == 0)
    {
        printf("Breakroad %d: No data\n", segment_id);
        return;
    }
    
    printf("Breakroad %d YAW[%d]:", segment_id, segment->yaw_record_count);
    for (uint8 i = 0; i < segment->yaw_record_count; i++)
    {
        printf("%.1f ", segment->yaw_records[i]);
    }
    printf("\n");
    printf("Total distance: %.1fcm\n", segment->last_total_distance);
}

/**
 * @brief 检查指定断路段是否已记录数据
 * @param segment_id 断路段ID (1-4)
 * @return 1=已记录，0=未记录
 */
uint8 multi_break_is_recorded(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return 0;
    
    return MultiBreak.segments[segment_id - 1].recorded;
}

/**
 * @brief 获取断路段对应的Flash页面
 * @param segment_id 断路段ID (1-4)
 * @return Flash页面号
 */
uint8 get_segment_flash_page(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return YAW_FLASH_PAGE_START; // 默认返回起始页
    
    return YAW_FLASH_PAGE_START + segment_id - 1;
}

/**
 * @brief 将指定断路段的偏航角数据存储到Flash
 * @param segment_id 断路段ID (1-4)
 */
void yaw_data_storage_segment(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return;
    
    BREAK_ROAD_SEGMENT* segment = &MultiBreak.segments[segment_id - 1];
    uint8 flash_page = get_segment_flash_page(segment_id);
    
    // 检查并擦除页面
    if (flash_check(0, flash_page))
    {
        flash_erase_page(0, flash_page);
    }
    
    flash_buffer_clear(); // 清空缓冲区
    
    // 将偏航角数据放入缓冲区
    for (uint8 i = 0; i < segment->yaw_record_count && i < MAX_YAW_RECORDS; i++)
    {
        flash_union_buffer[i].float_type = segment->yaw_records[i];
    }
    
    // 在缓冲区的最后一个位置存储记录数量
    flash_union_buffer[MAX_YAW_RECORDS].uint8_type = segment->yaw_record_count;
    
    // 写入Flash
    flash_write_page_from_buffer(0, flash_page, MAX_YAW_RECORDS + 1);
    
    // 标记为已记录
    segment->recorded = 1;
    
    printf("Breakroad %d data saved to flash page %d\n", segment_id, flash_page);
}

/**
 * @brief 从Flash读取指定断路段的偏航角数据
 * @param segment_id 断路段ID (1-4)
 */
void yaw_data_load_segment(uint8 segment_id)
{
    if (segment_id == 0 || segment_id > MAX_BREAKROADS)
        return;
    
    BREAK_ROAD_SEGMENT* segment = &MultiBreak.segments[segment_id - 1];
    uint8 flash_page = get_segment_flash_page(segment_id);
    
    // 检查Flash是否有数据
    if (flash_check(0, flash_page) == 0)
    {
        // 没有数据，初始化为0
        segment->yaw_record_count = 0;
        segment->last_total_distance = 0.0f;
        segment->recorded = 0;
        return;
    }
    
    // 读取Flash数据到缓冲区
    flash_read_page_to_buffer(0, flash_page, MAX_YAW_RECORDS + 1);
    
    // 从缓冲区恢复偏航角数据
    for (uint8 i = 0; i < MAX_YAW_RECORDS; i++)
    {
        segment->yaw_records[i] = flash_union_buffer[i].float_type;
    }
    
    // 恢复记录数量
    segment->yaw_record_count = flash_union_buffer[MAX_YAW_RECORDS].uint8_type;
    
    // 计算总距离
    segment->last_total_distance = segment->yaw_record_count * 2.0f;
    segment->recorded = (segment->yaw_record_count > 0) ? 1 : 0;
}

/**
 * @brief 显示Flash页面使用情况（调试用）
 */
void multi_break_show_flash_usage(void)
{
    printf("\n=== Flash Page Usage Report ===\n");
    printf("Page 5:  System Parameters\n");
    
    for (uint8 i = 1; i <= MAX_BREAKROADS; i++)
    {
        uint8 page = get_segment_flash_page(i);
        uint8 has_data = flash_check(0, page);
        BREAK_ROAD_SEGMENT* segment = &MultiBreak.segments[i - 1];
        
        printf("Page %d:  Breakroad %d - %s (%d records, %.1fcm)\n", 
               page, i, has_data ? "Used" : "Empty", 
               segment->yaw_record_count, segment->last_total_distance);
    }
    
    printf("Page 10-47: Available for future use\n");
    printf("===============================\n\n");
}

/**
 * @brief 多段断路系统诊断函数
 */
void multi_break_diagnostics(void)
{
    printf("\n=== Multi-Breakroad Diagnostics ===\n");
    
    // 显示系统状态
    printf("Current active segment: %d\n", MultiBreak.current_active_segment);
    printf("Sequence mode active: %s\n", MultiBreak.sequence_mode_active ? "Yes" : "No");
    printf("Total segments enabled: %d\n", MultiBreak.total_segments_enabled);
    printf("Current sequence index: %d\n", MultiBreak.sequence_index);
    
    // 显示执行序列
    printf("Execution sequence: [%d, %d, %d, %d]\n", 
           MultiBreak.execution_sequence[0], MultiBreak.execution_sequence[1],
           MultiBreak.execution_sequence[2], MultiBreak.execution_sequence[3]);
    
    // 显示各段状态
    printf("\nSegment Status:\n");
    for (uint8 i = 1; i <= MAX_BREAKROADS; i++)
    {
        BREAK_ROAD_SEGMENT* segment = &MultiBreak.segments[i - 1];
        printf("  Break %d: %s, %d records, %.1fcm, Flash Page %d\n", 
               i, segment->recorded ? "Recorded" : "Empty", 
               segment->yaw_record_count, segment->last_total_distance,
               get_segment_flash_page(i));
    }
    
    // 显示Flash使用情况
    multi_break_show_flash_usage();
    
    printf("===================================\n\n");
}

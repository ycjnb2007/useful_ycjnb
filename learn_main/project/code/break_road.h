/**
 * @file break_road.h
 * @brief 断路检测与处理模块头文件
 * @author 尘烟
 */
#ifndef _BREAK_ROAD_H_
#define _BREAK_ROAD_H_
#include "zf_common_headfile.h"

// 多段断路配置
#define MAX_BREAKROADS 4
#define YAW_FLASH_PAGE_START 6  // 断路数据存储起始页 (页6-9分别存储断路1-4)
#define MAX_YAW_RECORDS 50

/*
 * 简化的断路数据使用方案：
 * - 使用简单计数器自动切换断路数据，无需复杂的序列管理
 * - 开机初始化counter=0，第一次断路使用B1数据
 * - 每次断路完成后counter自动+1，下次使用下一个断路数据
 * - 顺序：B1 -> B2 -> B3 -> B4 -> B1（循环）
 * - 如果目标断路段没有数据，自动回退使用B1
 * 
 * 优势：
 * 1. 逻辑简单清晰，易于理解和维护
 * 2. 不需要复杂的序列配置和Flash管理
 * 3. 自动处理数据缺失的情况
 * 4. 开机即用，无需额外设置
 */

// Flash页面分配策略：
// 页面0-4：  系统保留（可能被SDK或其他模块使用）
// 页面5：    系统参数存储（my_flash.h中定义的PID参数、速度参数等）
// 页面6：    断路段1的偏航角数据存储
// 页面7：    断路段2的偏航角数据存储  
// 页面8：    断路段3的偏航角数据存储
// 页面9：    断路段4的偏航角数据存储
// 页面10-47：未使用，可用于未来扩展

// 断路控制偏移配置说明
// BREAK_DETECTION_DELAY_OFFSET: 固定偏移索引，补偿断路检测延迟
// USE_DYNAMIC_OFFSET: 1=动态计算偏移, 0=使用固定偏移
// 动态偏移公式: offset = (int)(break_dis / 4.0f)

// 单个断路段数据结构
typedef struct BREAK_ROAD_SEGMENT
{
    uint8 have;
    uint8 manual_started;      // 手动启动标志：1=手动启动，0=自动检测启动
    uint8 exit_delay_counter;  // 退出延迟计数器，用于手动启动后的延迟检测
    uint8 pre_break_state;     // 提前断路记录状态标志：1=提前记录状态，0=正常状态
    uint8 original_break_detected; // 原断路检测标志：1=检测到原断路，0=未检测到
    
    // 段特定数据
    uint8 recorded;            // 是否已记录数据：1=已记录，0=未记录
    uint8 yaw_record_count;    // 该段记录的偏航角数量
    float last_total_distance; // 该段记录的总距离
    float yaw_records[MAX_YAW_RECORDS]; // 该段的偏航角记录
} BREAK_ROAD_SEGMENT;

// 多段断路管理结构
typedef struct MULTI_BREAK_ROAD
{
    BREAK_ROAD_SEGMENT segments[MAX_BREAKROADS]; // 4个断路段
    uint8 current_active_segment;    // 当前激活的断路段ID (1-4, 0表示无)
    uint8 execution_sequence[MAX_BREAKROADS]; // 执行顺序数组 [1,2,3,4]
    uint8 sequence_index;            // 当前执行到的序列索引 (0-3)
    uint8 total_segments_enabled;    // 启用的断路段总数
    uint8 sequence_mode_active;      // 序列模式激活标志：1=按序列执行，0=单独执行
} MULTI_BREAK_ROAD;

// 兼容性别名（保持现有代码兼容）
#define BREAK_ROAD BREAK_ROAD_SEGMENT

// 全局变量声明
extern MULTI_BREAK_ROAD MultiBreak;        // 多段断路管理结构
extern BREAK_ROAD Break;                   // 兼容性变量（指向当前活跃断路段）
extern float break_speed_diff;
extern uint8 break_usage_counter;          // 简单计数器：0->第一次用B1, 1->第二次用B2, 2->第三次用B3, 3->第四次用B4, 然后循环

// 调试变量
extern float target_yaw;
extern float current_yaw;

//============================ 函数声明区域 ============================//
// 简化的断路管理函数
void simple_break_advance(void);                       // 简单计数器推进到下一个断路段
void multi_break_set_active_segment(uint8 segment_id); // 设置当前使用的断路段
uint8 multi_break_is_recorded(uint8 segment_id);       // 检查断路段是否有数据

// 多段断路管理函数
void multi_break_init(void);                           // 多段断路初始化
void multi_break_load_data(uint8 segment_id);          // 加载断路段数据
void multi_break_save_data(uint8 segment_id);          // 保存断路段数据
void multi_break_print_data(uint8 segment_id);         // 打印断路段数据
void multi_break_clear_data(uint8 segment_id);         // 清除断路段数据

// Flash操作函数
void yaw_data_storage_segment(uint8 segment_id);       // 存储断路段偏航角数据
void yaw_data_load_segment(uint8 segment_id);          // 加载断路段偏航角数据
uint8 get_segment_flash_page(uint8 segment_id);        // 获取断路段Flash页面

// 调试和诊断函数
void multi_break_show_flash_usage(void);               // 显示Flash使用情况
void multi_break_diagnostics(void);                    // 断路系统诊断

// 兼容性函数
void break_road_check(void);                           // 断路检测主函数
void break_road_init(void);                            // 断路系统初始化
void update_yaw_records(void);                         // 更新偏航角记录
void print_yaw_records(void);                          // 打印偏航角记录
void yaw_data_storage(void);                           // 存储偏航角数据
void yaw_data_load(void);                              // 加载偏航角数据
void print_loaded_yaw_records(void);                   // 打印已加载的偏航角记录
float get_current_target_yaw(void);                    // 获取当前目标偏航角
void reset_break_control_tracking(void);               // 重置断路控制跟踪
void flash_diagnostic(void);                           // Flash诊断
void flash_test(void);                                 // Flash测试
void clear_yaw_records(void);                          // 清除偏航角记录
void manual_start_break_recording(void);               // 手动开始断路记录
void start_pre_break_recording(void);                  // 开始提前断路记录

#endif
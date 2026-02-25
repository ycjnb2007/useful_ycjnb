
/**
 * @file key.c
 * @brief 按键菜单
 * @note - PID参数调试（角度环、角速度环、速度环）
 *       - 速度参数设置（正常速度、环岛速度、直角速度等）
 *       - 图像显示（二值化阈值、权重数组等）
 *       - 断路区记录
 * @author 尘烟
 */

#include "key.h"

// 角度环PID参数
float angle_kp = 7.9f; // 对应 Kp_outer = 7.9
float angle_ki = 0.0f; // 未使用积分项
float angle_kd = 0.5f; // 对应 Kd_outer = 0.5

// 角速度环PID参数 
float gyro_kp = 0.26f; // 对应 Kp_middle = 0.26
float gyro_ki = 0.0f;  // 未使用积分项
float gyro_kd = 0.09f; // 对应 Kd_middle = 0.09

// 速度环PID参数
float speed_kp = 110.0f; // 对应初始化的110
float speed_ki = 3.2f;   // 对应初始化的3.2
float speed_kd = 0.0f;   // 对应初始化的0

// 期望速度参数
uint16 normal_speed = 63;   // 对应 target_speed = 63
uint16 straight_speed = 80; // 直角时可能需要稍高速度
uint16 curve_speed = 50;    // 弯道时降低速度
uint16 circle_speed = 45;   // 环岛时更低速度
uint16 break_speed = 30;    // 断路检测时最低速度
// 标志位变量(预留8个)
uint8 flag_1 = 0;
uint8 flag_2 = 0;
uint8 flag_3 = 0;
uint8 flag_4 = 0;
uint8 flag_5 = 0;
uint8 flag_6 = 0;
uint8 flag_7 = 0;
uint8 flag_8 = 0;

// 菜单状态变量
uint8 current_page = MENU_MAIN_PAGE; // 开机时进入主菜单页面
uint8 param_select_index = 0;        // 当前选择的参数索引
uint8 show_menu_flag = 1;            // 菜单显示标志
uint8 system_running = 0;            // 开机时系统停止状态（菜单模式）
uint8 edit_mode = 0;                 // 编辑模式：0=选择模式，1=编辑模式
uint8 image_display_mode = 0;        // 图像显示模式标志
uint8 start_ready_flag = 0;          // 启动准备标志：开机时不可启动
uint8 track_lost_flag = 0;           // 赛道丢失标志：初始化为正常状态
uint8 emergency_stop_flag = 0;       // 紧急停车标志：初始化为正常状态

// 多段断路菜单状态变量
uint8 multi_break_select_mode = 0;   // 多段断路选择模式：0=选择断路，1=操作断路，2=序列设置
uint8 selected_break_id = 1;         // 选中的断路ID (1-4)
uint8 sequence_edit_mode = 0;        // 序列编辑模式：0=查看，1=编辑
uint8 sequence_edit_index = 0;       // 序列编辑索引 (0-3)

// 按键状态
static uint8 key_state[4] = {1, 1, 1, 1};      // 记录按键当前状态，1表示释放，0表示按下
static uint8 key_last_state[4] = {1, 1, 1, 1}; // 记录按键上一次状态

// 按键初始化
void my_key_init(void)
{
    // 初始化四个按键为上拉输入
    gpio_init(KEY_UP, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_DOWN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_RIGHT, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_SET, GPI, GPIO_HIGH, GPI_PULL_UP);
}

// 完整的按键扫描和处理函数 - 只在调参模式下调用
void key_scan(void)
{
    // 如果系统正在运行，只检测停止按键（参考CHIMU逻辑）
    if (system_running)
    {
        // 运行模式下的简单按键检测：只检济SET键停止
        static uint8 stop_key_state = 1;
        static uint8 stop_key_last_state = 1;

        uint8 current_set_state = gpio_get_level(KEY_UP);
        uint8 set_key_pressed = (current_set_state == 0 && stop_key_last_state == 1);

        if (set_key_pressed)
        {
            // 停止车辆，返回主菜单（手动停止或出赛道保护）
            system_running = 0;
            current_page = MENU_MAIN_PAGE;
            param_select_index = 0;
            show_menu_flag = 1;
            start_ready_flag = 0; // 重置准备标志

            // 显示停止原因
            tft180_clear();

            tft180_show_string(0, 40, "Manual Stop");

            system_delay_ms(1000); // 显示2秒停止信息
        }

        stop_key_last_state = current_set_state;
        return;
    }

    // 防抖计数器
    static uint8 debounce_counter[4] = {0, 0, 0, 0};

    // 读取物理按键状态
    uint8 raw_state[4];
    raw_state[0] = gpio_get_level(KEY_UP);    // UP
    raw_state[1] = gpio_get_level(KEY_DOWN);  // DOWN
    raw_state[2] = gpio_get_level(KEY_RIGHT); // RIGHT
    raw_state[3] = gpio_get_level(KEY_SET);   // SET

    // 处理按键防抖
    for (int i = 0; i < 4; i++)
    {
        if (raw_state[i] != key_state[i])
        {
            debounce_counter[i]++;
            if (debounce_counter[i] >= 3) // 连续3次检测到变化才认为是有效变化
            {
                key_state[i] = raw_state[i];
                debounce_counter[i] = 0;
            }
        }
        else
        {
            debounce_counter[i] = 0;
        }
    }

    // 检测按键按下事件(下降沿)
    uint8 key_up_pressed = (key_state[0] == 0 && key_last_state[0] == 1);
    uint8 key_down_pressed = (key_state[1] == 0 && key_last_state[1] == 1);
    uint8 key_right_pressed = (key_state[2] == 0 && key_last_state[2] == 1);
    uint8 key_set_pressed = (key_state[3] == 0 && key_last_state[3] == 1);

    // 处理调参模式下的按键逻辑
    switch (current_page)
    {
    case MENU_MAIN_PAGE: // 主菜单页（发车/调参/断路记录/图像显示）
        if (key_up_pressed || key_down_pressed)
        {
            param_select_index = (param_select_index + 1) % 4; // 四个选项：发车、调参、断路记录、图像显示
            show_menu_flag = 1;
        }
        if (key_set_pressed)
        {
            if (param_select_index == MAIN_OPTION_START) // 发车选项
            {
                // 参考CHIMU的安全启停机制：只有在准备状态下才能启动
                if (start_ready_flag == 0)
                {
                    // 第一次按下：进入准备状态
                    start_ready_flag = 1;
                    tft180_show_string(0, 90, "READY! Press again");
                    tft180_show_string(0, 106, "to START!");
                    tft180_show_float(100, 80, Err, 2, 2);
                    system_delay_ms(1000); // 显示提示信息
                    show_menu_flag = 1;    // 刷新菜单
                }
                else
                {
                    // 第二次按下：启动倒计时，然后启动车辆
                    tft180_clear();

                    // 倒计时显示

                    tft180_clear();
                    tft180_show_string(40, 50, "GO!");
                    system_delay_ms(500);

                    // 启动前重置控制系统状态，防止随机冲击
                    control_system_init();

                    system_running = 1;   // 启动车辆运行
                    start_ready_flag = 0; // 重置准备标志
                    current_page = MENU_CAMERA_PAGE;
                    param_select_index = 0;
                    show_menu_flag = 1;

                    // 清除所有按键状态，避免干扰
                    for (int i = 0; i < 4; i++)
                    {
                        key_state[i] = 1;
                        key_last_state[i] = 1;
                    }
                }
            }
            else if (param_select_index == MAIN_OPTION_PARAM) // 调参选项
            {
                current_page = MENU_PARAM_SELECT_PAGE;
                param_select_index = 0;
                show_menu_flag = 1;
            }
            else if (param_select_index == MAIN_OPTION_MULTI_BREAK) // 多段断路选项
            {
                current_page = MENU_MULTI_BREAK_SELECT_PAGE;
                param_select_index = 0;
                multi_break_select_mode = 0; // 重置为断路选择模式
                selected_break_id = 1;       // 重置选中的断路ID
                show_menu_flag = 1;
            }
            else if (param_select_index == MAIN_OPTION_IMAGE) // 图像显示选项
            {
                image_display_mode = 1; // 启用图像显示模式
                current_page = MENU_IMAGE_DISPLAY_PAGE;
                param_select_index = 0;
                show_menu_flag = 1;
            }
        }
        // KEY_RIGHT 在主菜单页面直接启动车辆（快捷键）
        // if (key_right_pressed)
        // {
        //     tft180_clear();
        //     system_delay_ms(300);

        //     // 快捷启动前也重置控制系统状态
        //     control_system_init();

        //     system_running = 1;
        //     current_page = MENU_CAMERA_PAGE;
        //     param_select_index = 0;
        //     show_menu_flag = 1;

        //     // 清除所有按键状态，避免干扰
        //     for (int i = 0; i < 4; i++)
        //     {
        //         key_state[i] = 1;
        //         key_last_state[i] = 1;
        //     }
        // }
        break;
    case MENU_PARAM_SELECT_PAGE:
        if (key_up_pressed)
        {
            param_select_index = (param_select_index == 0) ? PARAM_MAX_OPTION : param_select_index - 1;
            show_menu_flag = 1;
        }
        if (key_down_pressed)
        {
            param_select_index = (param_select_index + 1) % (PARAM_MAX_OPTION + 1);
            show_menu_flag = 1;
        }
        if (key_set_pressed)
        {
            switch (param_select_index)
            {
            case PARAM_OPTION_ANGLE:
                current_page = MENU_ANGLE_PID_PAGE;
                break;
            case PARAM_OPTION_GYRO:
                current_page = MENU_GYRO_PID_PAGE;
                break;
            case PARAM_OPTION_SPEED:
                current_page = MENU_SPEED_PID_PAGE;
                break;
            case PARAM_OPTION_EXPECT:
                current_page = MENU_EXPECT_SPEED_PAGE;
                break;
            case PARAM_OPTION_EXIT:
                current_page = MENU_MAIN_PAGE;
                break;
            }
            param_select_index = 0;
            edit_mode = 0; // 重置编辑模式
            show_menu_flag = 1;
        }
        // KEY_RIGHT 返回到主菜单页面
        if (key_right_pressed)
        {
            current_page = MENU_MAIN_PAGE;
            param_select_index = 0;
            edit_mode = 0;
            show_menu_flag = 1;
        }
        break;
    case MENU_ANGLE_PID_PAGE:
        if (!edit_mode) // 选择模式
        {
            if (key_up_pressed)
            {
                param_select_index = (param_select_index == 0) ? 4 : param_select_index - 1;
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                param_select_index = (param_select_index + 1) % 5;
                show_menu_flag = 1;
            }
            if (key_set_pressed)
            {
                if (param_select_index == 4) // Exit选项
                {
                    current_page = MENU_PARAM_SELECT_PAGE;
                    param_select_index = PARAM_OPTION_ANGLE;
                    edit_mode = 0;
                    show_menu_flag = 1;
                }
                else if (param_select_index == 3) // Save选项
                {
                    save_params_to_flash();
                }
                else // 参数选项 (0,1,2)
                {
                    edit_mode = 1;
                    show_menu_flag = 1;
                }
            }
            // KEY_RIGHT 在非编辑模式下返回参数选择页面
            if (key_right_pressed)
            {
                current_page = MENU_PARAM_SELECT_PAGE;
                param_select_index = PARAM_OPTION_ANGLE;
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        else // 编辑模式
        {
            if (key_up_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    angle_kp += PID_INCREMENT_SMALL;
                    break;
                case 1:
                    angle_ki += SPEED_INCREMENT;
                    break;
                case 2:
                    angle_kd += PID_INCREMENT_SMALL;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    angle_kp = (angle_kp > PID_INCREMENT_SMALL) ? angle_kp - PID_INCREMENT_SMALL : 0;
                    break;
                case 1:
                    angle_ki = (angle_ki > SPEED_INCREMENT) ? angle_ki - SPEED_INCREMENT : 0;
                    break;
                case 2:
                    angle_kd = (angle_kd > PID_INCREMENT_SMALL) ? angle_kd - PID_INCREMENT_SMALL : 0;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_right_pressed || key_set_pressed)
            {
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        break;
    case MENU_GYRO_PID_PAGE:
        if (!edit_mode) // 选择模式
        {
            if (key_up_pressed)
            {
                param_select_index = (param_select_index == 0) ? 4 : param_select_index - 1;
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                param_select_index = (param_select_index + 1) % 5;
                show_menu_flag = 1;
            }
            if (key_set_pressed)
            {
                if (param_select_index == 4) // Exit选项
                {
                    current_page = MENU_PARAM_SELECT_PAGE;
                    param_select_index = PARAM_OPTION_GYRO;
                    edit_mode = 0;
                    show_menu_flag = 1;
                }
                else if (param_select_index == 3) // Save选项
                {
                    save_params_to_flash();
                }
                else // 参数选项 (0,1,2)
                {
                    edit_mode = 1;
                    show_menu_flag = 1;
                }
            }
            // KEY_RIGHT 在非编辑模式下返回参数选择页面
            if (key_right_pressed)
            {
                current_page = MENU_PARAM_SELECT_PAGE;
                param_select_index = PARAM_OPTION_GYRO;
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        else // 编辑模式
        {
            if (key_up_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    gyro_kp += PID_INCREMENT_SMALL;
                    break;
                case 1:
                    gyro_ki += PID_INCREMENT_SMALL;
                    break;
                case 2:
                    gyro_kd += PID_INCREMENT_SMALL;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    gyro_kp = (gyro_kp > PID_INCREMENT_SMALL) ? gyro_kp - PID_INCREMENT_SMALL : 0;
                    break;
                case 1:
                    gyro_ki = (gyro_ki > PID_INCREMENT_SMALL) ? gyro_ki - PID_INCREMENT_SMALL : 0;
                    break;
                case 2:
                    gyro_kd = (gyro_kd > PID_INCREMENT_SMALL) ? gyro_kd - PID_INCREMENT_SMALL : 0;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_right_pressed || key_set_pressed)
            {
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        break;
    case MENU_SPEED_PID_PAGE:
        if (!edit_mode) // 选择模式
        {
            if (key_up_pressed)
            {
                param_select_index = (param_select_index == 0) ? 4 : param_select_index - 1;
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                param_select_index = (param_select_index + 1) % 5;
                show_menu_flag = 1;
            }
            if (key_set_pressed)
            {
                if (param_select_index == 4) // Exit选项
                {
                    current_page = MENU_PARAM_SELECT_PAGE;
                    param_select_index = PARAM_OPTION_SPEED;
                    edit_mode = 0;
                    show_menu_flag = 1;
                }
                else if (param_select_index == 3) // Save选项
                {
                    save_params_to_flash();
                }
                else // 参数选项 (0,1,2)
                {
                    edit_mode = 1;
                    show_menu_flag = 1;
                }
            }
            // KEY_RIGHT 在非编辑模式下返回参数选择页面
            if (key_right_pressed)
            {
                current_page = MENU_PARAM_SELECT_PAGE;
                param_select_index = PARAM_OPTION_SPEED;
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        else // 编辑模式
        {
            if (key_up_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    speed_kp += SPEED_INCREMENT;
                    break;
                case 1:
                    speed_ki += PID_INCREMENT_SMALL;
                    break;
                case 2:
                    speed_kd += PID_INCREMENT;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    speed_kp = (speed_kp > SPEED_INCREMENT) ? speed_kp - SPEED_INCREMENT : 0;
                    break;
                case 1:
                    speed_ki = (speed_ki > PID_INCREMENT_SMALL) ? speed_ki - PID_INCREMENT_SMALL : 0;
                    break;
                case 2:
                    speed_kd = (speed_kd > PID_INCREMENT) ? speed_kd - PID_INCREMENT : 0;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_right_pressed || key_set_pressed)
            {
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        break;
    case MENU_EXPECT_SPEED_PAGE:
        if (!edit_mode) // 选择模式
        {
            if (key_up_pressed)
            {
                param_select_index = (param_select_index == 0) ? 6 : param_select_index - 1;
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                param_select_index = (param_select_index + 1) % 7;
                show_menu_flag = 1;
            }
            if (key_set_pressed)
            {
                if (param_select_index == 6) // Exit选项
                {
                    current_page = MENU_PARAM_SELECT_PAGE;
                    param_select_index = PARAM_OPTION_EXPECT;
                    edit_mode = 0;
                    show_menu_flag = 1;
                }
                else if (param_select_index == 5) // Save选项
                {
                    save_params_to_flash();
                }
                else // 参数选项 (0,1,2,3,4)
                {
                    edit_mode = 1;
                    show_menu_flag = 1;
                }
            }
            // KEY_RIGHT 在非编辑模式下返回参数选择页面
            if (key_right_pressed)
            {
                current_page = MENU_PARAM_SELECT_PAGE;
                param_select_index = PARAM_OPTION_EXPECT;
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        else // 编辑模式
        {
            if (key_up_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    normal_speed += SPEED_INCREMENT1;
                    break;
                case 1:
                    straight_speed += SPEED_INCREMENT1;
                    break;
                case 2:
                    curve_speed += SPEED_INCREMENT1;
                    break;
                case 3:
                    circle_speed += SPEED_INCREMENT1;
                    break;
                case 4:
                    break_speed += SPEED_INCREMENT1;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_down_pressed)
            {
                switch (param_select_index)
                {
                case 0:
                    normal_speed = (normal_speed > SPEED_INCREMENT1) ? normal_speed - SPEED_INCREMENT1 : 0;
                    break;
                case 1:
                    straight_speed = (straight_speed > SPEED_INCREMENT1) ? straight_speed - SPEED_INCREMENT1 : 0;
                    break;
                case 2:
                    curve_speed = (curve_speed > SPEED_INCREMENT1) ? curve_speed - SPEED_INCREMENT1 : 0;
                    break;
                case 3:
                    circle_speed = (circle_speed > SPEED_INCREMENT1) ? circle_speed - SPEED_INCREMENT1 : 0;
                    break;
                case 4:
                    break_speed = (break_speed > SPEED_INCREMENT1) ? break_speed - SPEED_INCREMENT1 : 0;
                    break;
                }
                show_menu_flag = 1;
            }
            if (key_right_pressed || key_set_pressed)
            {
                edit_mode = 0;
                show_menu_flag = 1;
            }
        }
        break;
    case MENU_MULTI_BREAK_SELECT_PAGE: // 多段断路选择页
        // 处理返回键：返回主菜单
        if (key_right_pressed)
        {
            // 如果在断路操作模式下，返回到断路选择模式
            if (multi_break_select_mode == 1) {
                multi_break_select_mode = 0; // 返回断路选择模式
                param_select_index = selected_break_id - 1; // 保持在选中的断路
                show_menu_flag = 1;
            } else {
                // 返回主菜单
                current_page = MENU_MAIN_PAGE;
                param_select_index = MAIN_OPTION_MULTI_BREAK; // 返回时保持在多段断路选项
                show_menu_flag = 1;
                multi_break_select_mode = 0; // 重置选择模式
                selected_break_id = 1; // 重置选中的断路ID
            }
        }

        // 根据当前模式处理其他按键
        if (multi_break_select_mode == 0) {
            // 断路选择模式
            if (key_up_pressed || key_down_pressed) {
                if (key_up_pressed) {
                    param_select_index = (param_select_index == 0) ? MULTI_BREAK_MAX_OPTION : param_select_index - 1;
                } else {
                    param_select_index = (param_select_index == MULTI_BREAK_MAX_OPTION) ? 0 : param_select_index + 1;
                }
                show_menu_flag = 1;
            }

            // 确认键：选择断路或进入序列设置
            if (key_set_pressed) {
                if (param_select_index == MULTI_BREAK_OPTION_SEQUENCE) {
                    // 进入序列设置页面
                    current_page = MENU_BREAK_SEQUENCE_PAGE;
                    param_select_index = 0;
                    sequence_edit_mode = 0;
                    sequence_edit_index = 0;
                    show_menu_flag = 1;
                } else {
                    // 选择了一个断路
                    selected_break_id = param_select_index + 1;
                    multi_break_select_mode = 1; // 进入断路操作模式
                    param_select_index = 0; // 重置操作菜单的选项索引
                    show_menu_flag = 1;
                }
            }
        } else {
            // 断路操作模式
            if (key_up_pressed || key_down_pressed) {
                if (key_up_pressed) {
                    param_select_index = (param_select_index == 0) ? BREAK_OP_MAX_OPTION : param_select_index - 1;
                } else {
                    param_select_index = (param_select_index == BREAK_OP_MAX_OPTION) ? 0 : param_select_index + 1;
                }
                show_menu_flag = 1;
            }

            // 确认键：执行断路操作
            if (key_set_pressed) {
                switch (param_select_index) {
                    case BREAK_OP_RECORD:  // 记录断路数据
                        current_page = MENU_BREAK_RECORD_PAGE;
                        multi_break_set_active_segment(selected_break_id);
                        param_select_index = 0;
                        show_menu_flag = 1;
                        break;
                    case BREAK_OP_VIEW:    // 查看断路数据
                        multi_break_print_data(selected_break_id);
                        system_delay_ms(2000); // 显示2秒
                        show_menu_flag = 1;
                        break;
                    case BREAK_OP_CLEAR:   // 清除断路数据
                        multi_break_clear_data(selected_break_id);
                        system_delay_ms(1000); // 显示1秒
                        show_menu_flag = 1;
                        break;
                    case BREAK_OP_MANUAL:  // 手动开始记录
                        multi_break_start_recording(selected_break_id);
                        current_page = MENU_BREAK_RECORD_PAGE;
                        param_select_index = 0;
                        show_menu_flag = 1;
                        break;
                    case BREAK_OP_PRE_RECORD: // 提前记录模式
                        multi_break_set_active_segment(selected_break_id);
                        start_pre_break_recording();
                        current_page = MENU_BREAK_RECORD_PAGE;
                        param_select_index = 0;
                        show_menu_flag = 1;
                        break;
                    case BREAK_OP_BACK:    // 返回
                        multi_break_select_mode = 0;
                        param_select_index = selected_break_id - 1;
                        show_menu_flag = 1;
                        break;
                }
            }
        }
        break;
        
    case MENU_BREAK_SEQUENCE_PAGE: // 断路执行序列设置页
        // 处理返回键：返回多段断路选择页面
        if (key_right_pressed)
        {
            current_page = MENU_MULTI_BREAK_SELECT_PAGE;
            multi_break_select_mode = 0; // 返回断路选择模式
            param_select_index = MULTI_BREAK_OPTION_SEQUENCE; // 保持在序列设置选项
            show_menu_flag = 1;
        }
        
        // 处理上下键：选择序列位置
        if (key_up_pressed || key_down_pressed) {
            if (key_up_pressed) {
                param_select_index = (param_select_index == 0) ? 3 : param_select_index - 1;
            } else {
                param_select_index = (param_select_index == 3) ? 0 : param_select_index + 1;
            }
            show_menu_flag = 1;
        }
        
        // 处理SET键：修改序列值
        if (key_set_pressed) {
            uint8 current_value = MultiBreak.execution_sequence[param_select_index];
            current_value = (current_value == 4) ? 1 : current_value + 1;
            MultiBreak.execution_sequence[param_select_index] = current_value;
            
            // 重新计算启用的段数
            multi_break_set_sequence(MultiBreak.execution_sequence[0], MultiBreak.execution_sequence[1],
                                   MultiBreak.execution_sequence[2], MultiBreak.execution_sequence[3]);
            show_menu_flag = 1;
        }
        break;
        
    case MENU_BREAK_RECORD_PAGE: // 断路记录页
        // 处理KEY_RIGHT：如果正在记录则提前退出，否则返回主菜单
        if (key_right_pressed)
        {
            if (Break.have == 1) // 正在记录中
            {
                // 提前退出记录，保存已记录的数据
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
                BEE_STA(0);                          // 关闭蜂鸣器
                
                // 显示提前退出信息
                tft180_clear();
                tft180_show_string(0, 40, "Recording stopped");
                tft180_show_string(0, 60, "by KEY_RIGHT");
                tft180_show_string(0, 80, "Data saved!");
                system_delay_ms(1500); // 显示信息1.5秒
                
                // 返回多段断路选择页面
                current_page = MENU_MULTI_BREAK_SELECT_PAGE;
                multi_break_select_mode = 1; // 返回断路操作模式
                param_select_index = BREAK_OP_RECORD; // 返回到记录选项
                show_menu_flag = 1;     // 刷新显示
            }
            else
            {
                // 没有记录，返回多段断路选择页面
                current_page = MENU_MULTI_BREAK_SELECT_PAGE;
                multi_break_select_mode = 1; // 返回断路操作模式
                param_select_index = BREAK_OP_RECORD; // 返回到记录选项
                show_menu_flag = 1;
            }
        }
        
        // 处理SET键：启动提前断路记录状态
        if (key_set_pressed)
        {
            // 检查是否可以开始提前记录
            uint8 can_record = (system_running == 0 && current_page == MENU_BREAK_RECORD_PAGE);
            
            if (can_record && Break.have == 0) // 未在记录状态
            {
                // 启动提前断路记录
                start_pre_break_recording();
                
                // 显示提前记录开始提示
                tft180_clear();
                tft180_show_string(0, 40, "Pre-Break Recording");
                tft180_show_string(0, 60, "Started by KEY_SET");
                tft180_show_string(0, 80, "Wait for break road");
                system_delay_ms(1500);
                show_menu_flag = 1;
            }
            else if (Break.have == 1)
            {
                // 如果已经在记录，提示当前状态
                tft180_clear();
                if (Break.pre_break_state == 1)
                {
                    tft180_show_string(0, 40, "Pre-Recording Active");
                    tft180_show_string(0, 60, "Waiting for break...");
                }
                else
                {
                    tft180_show_string(0, 40, "Already Recording!");
                }
                tft180_show_string(0, 80, "Use KEY_RIGHT to stop");
                system_delay_ms(1500);
                show_menu_flag = 1;
            }
            else
            {
                // 不满足记录条件，返回多段断路选择页面
                current_page = MENU_MULTI_BREAK_SELECT_PAGE;
                multi_break_select_mode = 1; // 返回断路操作模式
                param_select_index = BREAK_OP_PRE_RECORD; // 返回到提前记录选项
                show_menu_flag = 1;
            }
        }

        // UP键：手动加载Flash数据
        if (key_up_pressed)
        {
            if (Break.have == 0) // 只有在非记录状态下才允许加载数据
            {
                yaw_data_load();
                show_menu_flag = 1; // 强制刷新显示
            }
        }

        // DOWN键：手动读取Flash数据并刷新显示
        if (key_down_pressed)
        {
            if (Break.have == 0) // 只有在非记录状态下才允许加载数据
            {
                yaw_data_load();
                print_loaded_yaw_records();
                show_menu_flag = 1; // 强制刷新显示
            }
        }
        break;
    case MENU_IMAGE_DISPLAY_PAGE: // 图像显示页
        // 处理返回键：返回主菜单
        if (key_right_pressed || key_set_pressed)
        {
            image_display_mode = 0; // 关闭图像显示模式
            current_page = MENU_MAIN_PAGE;
            param_select_index = MAIN_OPTION_IMAGE; // 返回时保持在图像显示选项
            show_menu_flag = 1;
        }
        // 其他按键可用于图像显示相关控制（如果需要）
        // 目前保持简单，不添加复杂功能避免断言警告
        break;

    default:
        break;
    }

    // 更新按键状态记录
    for (int i = 0; i < 4; i++)
    {
        key_last_state[i] = key_state[i];
    }
}

// 显示菜单
void menu_display(void)
{
    // 断路记录页面需要实时刷新数据，但不需要清屏
    static uint8 last_break_have = 0xFF;     // 记录上次的Break.have状态
    static uint8 break_page_initialized = 0; // 记录断路页面是否已初始化

    uint8 need_refresh = show_menu_flag;

    // 断路记录页面特殊处理：状态变化、首次进入或手动刷新时显示
    if (current_page == MENU_BREAK_RECORD_PAGE)
    {
        if (!break_page_initialized || Break.have != last_break_have || show_menu_flag)
        {
            need_refresh = 1;
            last_break_have = Break.have;
            break_page_initialized = 1;
        }
    }
    else
    {
        break_page_initialized = 0; // 离开断路页面时重置
    }

    if (!need_refresh)
        return;

    show_menu_flag = 0; // 清除刷新标志

    // 根据当前页面显示相应的菜单内容
    switch (current_page)
    {
    case MENU_MAIN_PAGE: // 主菜单页（发车/调参/断路记录/图像显示）
        tft180_clear();
        tft180_show_string(0, 0, "Smart Car System");
        tft180_show_string(0, 20, "================");

        tft180_show_string(0, 35, param_select_index == MAIN_OPTION_START ? "->Start Car" : "  Start Car");
        tft180_show_string(0, 50, param_select_index == MAIN_OPTION_PARAM ? "->Parameter Set" : "  Parameter Set");
        tft180_show_string(0, 65, param_select_index == MAIN_OPTION_MULTI_BREAK ? "->Multi Break" : "  Multi Break");
        tft180_show_string(0, 80, param_select_index == MAIN_OPTION_IMAGE ? "->Image Display" : "  Image Display");

        // 显示当前系统状态
        tft180_show_string(0, 110, "Status:");
        tft180_show_uint(50, 110, system_running, 1);

        // 显示启动准备状态
        if (start_ready_flag)
        {
            tft180_show_string(70, 110, "READY!");
        }
        break;
    case MENU_PARAM_SELECT_PAGE: // 调参选择页
        tft180_clear();
        tft180_show_string(0, 0, "Parameter Setup");
        tft180_show_string(0, 15, param_select_index == PARAM_OPTION_ANGLE ? "->Angle PID" : "  Angle PID");
        tft180_show_string(0, 30, param_select_index == PARAM_OPTION_GYRO ? "->Gyro PID" : "  Gyro PID");
        tft180_show_string(0, 45, param_select_index == PARAM_OPTION_SPEED ? "->Speed PID" : "  Speed PID");
        tft180_show_string(0, 60, param_select_index == PARAM_OPTION_EXPECT ? "->Expect Speed" : "  Expect Speed");
        tft180_show_string(0, 75, param_select_index == PARAM_OPTION_EXIT ? "->Exit" : "  Exit");
        break;

    case MENU_ANGLE_PID_PAGE: // 角度环PID页
        tft180_clear();
        tft180_show_string(0, 0, "Angle PID Setup");

        if (edit_mode)
        {
            tft180_show_string(0, 16, "EDIT MODE");
        }

        tft180_show_string(0, 32, param_select_index == 0 ? "->KP:" : "  KP:");
        tft180_show_float(40, 32, angle_kp, 2, 3);
        if (edit_mode && param_select_index == 0)
            tft180_show_string(100, 32, "<-");

        tft180_show_string(0, 48, param_select_index == 1 ? "->KI:" : "  KI:");
        tft180_show_float(40, 48, angle_ki, 2, 3);
        if (edit_mode && param_select_index == 1)
            tft180_show_string(100, 48, "<-");

        tft180_show_string(0, 64, param_select_index == 2 ? "->KD:" : "  KD:");
        tft180_show_float(40, 64, angle_kd, 2, 3);
        if (edit_mode && param_select_index == 2)
            tft180_show_string(100, 64, "<-");

        tft180_show_string(0, 80, param_select_index == 3 ? "->Save" : "  Save");
        tft180_show_string(0, 96, param_select_index == 4 ? "->Exit" : "  Exit");
        break;

    case MENU_GYRO_PID_PAGE: // 角速度环PID页
        tft180_clear();
        tft180_show_string(0, 0, "Gyro PID Setup");

        if (edit_mode)
        {
            tft180_show_string(0, 16, "EDIT MODE");
        }

        tft180_show_string(0, 32, param_select_index == 0 ? "->KP:" : "  KP:");
        tft180_show_float(40, 32, gyro_kp, 2, 3);
        if (edit_mode && param_select_index == 0)
            tft180_show_string(100, 32, "<-");

        tft180_show_string(0, 48, param_select_index == 1 ? "->KI:" : "  KI:");
        tft180_show_float(40, 48, gyro_ki, 2, 3);
        if (edit_mode && param_select_index == 1)
            tft180_show_string(100, 48, "<-");

        tft180_show_string(0, 64, param_select_index == 2 ? "->KD:" : "  KD:");
        tft180_show_float(40, 64, gyro_kd, 2, 3);
        if (edit_mode && param_select_index == 2)
            tft180_show_string(100, 64, "<-");

        tft180_show_string(0, 80, param_select_index == 3 ? "->Save" : "  Save");
        tft180_show_string(0, 96, param_select_index == 4 ? "->Exit" : "  Exit");
        break;

    case MENU_SPEED_PID_PAGE: // 速度环PID页
        tft180_clear();
        tft180_show_string(0, 0, "Speed PID Setup");

        if (edit_mode)
        {
            tft180_show_string(0, 16, "EDIT MODE");
        }

        tft180_show_string(0, 32, param_select_index == 0 ? "->KP:" : "  KP:");
        tft180_show_float(40, 32, speed_kp, 2, 3);
        if (edit_mode && param_select_index == 0)
            tft180_show_string(100, 32, "<-");

        tft180_show_string(0, 48, param_select_index == 1 ? "->KI:" : "  KI:");
        tft180_show_float(40, 48, speed_ki, 2, 3);
        if (edit_mode && param_select_index == 1)
            tft180_show_string(100, 48, "<-");

        tft180_show_string(0, 64, param_select_index == 2 ? "->KD:" : "  KD:");
        tft180_show_float(40, 64, speed_kd, 2, 3);
        if (edit_mode && param_select_index == 2)
            tft180_show_string(100, 64, "<-");

        tft180_show_string(0, 80, param_select_index == 3 ? "->Save" : "  Save");
        tft180_show_string(0, 96, param_select_index == 4 ? "->Exit" : "  Exit");
        break;

    case MENU_EXPECT_SPEED_PAGE: // 期望速度页
        tft180_clear();
        tft180_show_string(0, 0, "Expect Speed");

        if (edit_mode)
        {
            tft180_show_string(0, 16, "EDIT MODE");
        }

        tft180_show_string(0, 32, param_select_index == 0 ? "->Normal:" : "  Normal:");
        tft180_show_uint(75, 32, normal_speed, 4);
        if (edit_mode && param_select_index == 0)
            tft180_show_string(100, 32, "<-");

        tft180_show_string(0, 48, param_select_index == 1 ? "->Str:" : "  Str:");
        tft180_show_uint(75, 48, straight_speed, 4);
        if (edit_mode && param_select_index == 1)
            tft180_show_string(110, 48, "<-");

        tft180_show_string(0, 64, param_select_index == 2 ? "->Curve:" : "  Curve:");
        tft180_show_uint(75, 64, curve_speed, 4);
        if (edit_mode && param_select_index == 2)
            tft180_show_string(90, 64, "<-");

        tft180_show_string(0, 80, param_select_index == 3 ? "->Circle:" : "  Circle:");
        tft180_show_uint(75, 80, circle_speed, 4);
        if (edit_mode && param_select_index == 3)
            tft180_show_string(95, 80, "<-");

        tft180_show_string(0, 96, param_select_index == 4 ? "->Break:" : "  Break:");
        tft180_show_uint(75, 96, break_speed, 4);
        if (edit_mode && param_select_index == 4)
            tft180_show_string(90, 96, "<-");
        tft180_show_string(0, 112, param_select_index == 5 ? "->Save" : "  Save");
        tft180_show_string(60, 112, param_select_index == 6 ? "->Exit" : "  Exit");
        break;

    case MENU_MULTI_BREAK_SELECT_PAGE: // 多段断路选择页
        tft180_clear();
        
        // 根据多段断路模式显示不同的菜单
        if (multi_break_select_mode == 0)
        {
            // 断路选择模式
            tft180_show_string(0, 0, "Multi Break");
            tft180_show_string(0, 20, "===========");
            
            // 显示断路状态（仿照调参菜单的显示方式）
            tft180_show_string(0, 35, param_select_index == 0 ? "->B1" : "  B1");
            tft180_show_string(40, 35, multi_break_is_recorded(1) ? "[*]" : "[ ]");
            
            tft180_show_string(0, 50, param_select_index == 1 ? "->B2" : "  B2");
            tft180_show_string(40, 50, multi_break_is_recorded(2) ? "[*]" : "[ ]");
            
            tft180_show_string(0, 65, param_select_index == 2 ? "->B3" : "  B3");
            tft180_show_string(40, 65, multi_break_is_recorded(3) ? "[*]" : "[ ]");
            
            tft180_show_string(0, 80, param_select_index == 3 ? "->B4" : "  B4");
            tft180_show_string(40, 80, multi_break_is_recorded(4) ? "[*]" : "[ ]");
            
            tft180_show_string(0, 95, param_select_index == 4 ? "->Seq Set" : "  Seq Set");
            
            tft180_show_string(0, 112, "SET:OK RIGHT:Back");
        }
        else {
            // 断路操作模式
            tft180_show_string(0, 0, "B");
            tft180_show_uint(10, 0, selected_break_id, 1);
            tft180_show_string(20, 0, " Operations");
            tft180_show_string(0, 20, "=============");
            
            tft180_show_string(0, 35, param_select_index == BREAK_OP_RECORD ? "->Record" : "  Record");
            tft180_show_string(0, 50, param_select_index == BREAK_OP_VIEW ? "->View" : "  View");
            tft180_show_string(0, 65, param_select_index == BREAK_OP_CLEAR ? "->Clear" : "  Clear");
            tft180_show_string(0, 80, param_select_index == BREAK_OP_MANUAL ? "->Manual" : "  Manual");
            tft180_show_string(0, 95, param_select_index == BREAK_OP_PRE_RECORD ? "->Pre-Rec" : "  Pre-Rec");
            tft180_show_string(0, 110, param_select_index == BREAK_OP_BACK ? "->Back" : "  Back");
            
            // 显示当前断路段状态
            tft180_show_string(90, 35, "Rec:");
            tft180_show_string(115, 35, multi_break_is_recorded(selected_break_id) ? "Y" : "N");
        }
        break;

    case MENU_BREAK_SEQUENCE_PAGE: // 断路执行序列设置页
        tft180_clear();
        tft180_show_string(0, 0, "Break Sequence");
        tft180_show_string(0, 20, "==============");
        
        // 显示当前执行序列（仿照调参菜单显示方式）
        tft180_show_string(0, 35, param_select_index == 0 ? "->1st: B" : "  1st: B");
        tft180_show_uint(65, 35, MultiBreak.execution_sequence[0], 1);
        
        tft180_show_string(0, 50, param_select_index == 1 ? "->2nd: B" : "  2nd: B");
        tft180_show_uint(65, 50, MultiBreak.execution_sequence[1], 1);
        
        tft180_show_string(0, 65, param_select_index == 2 ? "->3rd: B" : "  3rd: B");
        tft180_show_uint(65, 65, MultiBreak.execution_sequence[2], 1);
        
        tft180_show_string(0, 80, param_select_index == 3 ? "->4th: B" : "  4th: B");
        tft180_show_uint(65, 80, MultiBreak.execution_sequence[3], 1);
        
        // 显示提示信息
        tft180_show_string(0, 100, "SET:Change RIGHT:Back");
        
        // 显示已启用段数
        tft180_show_string(0, 115, "En:");
        tft180_show_uint(20, 115, MultiBreak.total_segments_enabled, 1);
        tft180_show_string(30, 115, " segs");
        break;

    case MENU_BREAK_RECORD_PAGE: // 断路记录页
        tft180_clear();
        char record_title[32];
        sprintf(record_title, "Break %d Record", MultiBreak.current_active_segment);
        tft180_show_string(0, 0, record_title);
        tft180_show_string(0, 20, "=================");

        // 检查记录条件
        uint8 can_record = (system_running == 0 && current_page == MENU_BREAK_RECORD_PAGE);

        // 显示当前状态
        if (Break.have == 1 && can_record)
        {
            tft180_show_string(0, 40, "Status: Re");
            tft180_show_string(0, 60, "Distance:");
            tft180_show_float(60, 60, break_dis, 3, 1); // 显示当前距离（cm）
        }
        else
        {
            if (!can_record)
            {
                tft180_show_string(0, 40, "Status: Dis");
                
            }
            else
            {
                tft180_show_string(0, 40, "Status: Re");
            }

            // 显示上次记录结果
            if (MultiBreak.current_active_segment > 0)
            {
                BREAK_ROAD_SEGMENT* active_segment = &MultiBreak.segments[MultiBreak.current_active_segment - 1];
                if (active_segment->yaw_record_count > 0)
                {
                    tft180_show_string(0, 80, "Last: Suc");
                    tft180_show_float(80, 80, active_segment->last_total_distance, 3, 1);
                }
                else if (can_record)
                {
                    tft180_show_string(0, 80, "No record");
                }
            }
            else if (can_record)
            {
                tft180_show_string(0, 80, "No");
            }
        }


        break;

    case MENU_IMAGE_DISPLAY_PAGE: // 图像显示页
        // 图像显示模式下只清屏，不显示文字信息
        // 实际的图像显示由主循环中的tft180_show_gray_image()处理
        // 这样避免文字和图像的显示冲突
        tft180_clear();

        break;

    default:
        tft180_clear();
        tft180_show_string(0, 0, "Unknown Page");
        break;
    }
}

// 保存参数到Flash
void save_params_to_flash(void)
{
    Write_Flash_Flag = 1; // 设置写入标志，让主循环去处理实际的写入操作

    // 显示保存中的提示
    tft180_show_string(0, 80, "Saving...");
    system_delay_ms(100); // 短暂延时，让用户看到提示

    // 标记需要刷新菜单
    show_menu_flag = 1;
}

// 超简单的按键检测 - 用于图像模式下检测是否有按键按下
uint8 simple_key_check(void)
{
    static uint8 last_check_state[4] = {1, 1, 1, 1};
    static uint8 key_pressed_flag = 0;

    uint8 current_state[4];
    current_state[0] = gpio_get_level(KEY_UP);
    current_state[1] = gpio_get_level(KEY_DOWN);
    current_state[2] = gpio_get_level(KEY_RIGHT);
    current_state[3] = gpio_get_level(KEY_SET);

    // 检测任何按键的下降沿（从1变为0）
    for (int i = 0; i < 4; i++)
    {
        if (last_check_state[i] == 1 && current_state[i] == 0)
        {
            key_pressed_flag = 1;
        }
        last_check_state[i] = current_state[i];
    }

    // 如果检测到按键按下，返回1并清除标志
    if (key_pressed_flag)
    {
        key_pressed_flag = 0;
        return 1;
    }

    return 0; // 没有按键按下
}

// 更直接的按键检测函数 - 只要检测到任何按键为低电平就返回
uint8 direct_key_check(void)
{
    // 直接检测任何按键是否为低电平（按下状态）
    if (gpio_get_level(KEY_UP) == 0 ||
        gpio_get_level(KEY_DOWN) == 0 ||
        gpio_get_level(KEY_RIGHT) == 0 ||
        gpio_get_level(KEY_SET) == 0)
    {
        return 1; // 有按键按下
    }
    return 0; // 没有按键按下
}

// // 调试函数：显示按键状态
// void show_key_status_debug(void)
// {
//     static uint8 debug_counter = 0;

//     // 每10次调用显示一次，减少闪烁
//     if (++debug_counter >= 10)
//     {
//         debug_counter = 0;

//         tft180_show_string(0, 125, "DBG:");
//         tft180_show_uint(30, 125, gpio_get_level(KEY_UP), 1);
//         tft180_show_uint(40, 125, gpio_get_level(KEY_DOWN), 1);
//         tft180_show_uint(50, 125, gpio_get_level(KEY_RIGHT), 1);
//         tft180_show_uint(60, 125, gpio_get_level(KEY_SET), 1);
//         tft180_show_uint(70, 125, system_running, 1);
//     }
// }

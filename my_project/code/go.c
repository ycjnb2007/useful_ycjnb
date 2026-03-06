#include "go.h"
/******************************************************************************
* 函数名称     : go
* 描述         : 小车go！！！
* 进入参数     : void
* 返回参数     : void
******************************************************************************/
void go(void)
{
    // 【解释】从摄像头缓冲区拷贝灰度图。MT9V03X_IMAGE_SIZE 是逐飞库定义的宏。
    memcpy(imgGray[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    // 【解释】大津法获取当前帧的最佳二值化阈值。
    nowThreshold = getOSTUThreshold();/*大津法获取阈值*/

    // 【解释】根据阈值生成二值化图像 imgOSTU，赛道为白，背景为黑。
    Get_imgOSTU(); /*获取二值化图像imgOSTU*/

    // 【导师提醒】21届疯狂电路组已经明确取消断路，干得好，这里被注释掉了！
    // Dashedline_Makeup(); 规则不需要断路处理了

    Stop_line = 0; //动态前瞻初始化

    IF_L = IF;

    // 【解释】默认先把整幅图的中线 mid_line 放在屏幕正中央 (XM/2)。
    memset(mid_line, XM/2, sizeof(mid_line));
    
    // ==========================================================
    // 盲转强控阶段 (Yaw_Plus 切角核心)
    // ==========================================================
    // 如果处于盲转状态，优先跳过图像处理逻辑，保持原中线并让出控制权
    // (在此可交由陀螺仪偏角规划控制中线，当 blind_turning 为 1 时不执行图像处理)
    if(is_blind_turning)
    {
        // 既然已经进入盲转状态，那么这帧图像的数据就没用了，随便给个默认值就行
        for(uint8 i = 0; i <= Deal_Top; i++)
        {
            mid_line[i] = XM / 2;
            imgOSTU[i][(uint8)mid_line[i]] = Control_line;
        }
        return; // 直接返回，把控制交接给底盘
    }

    // ==========================================================
    // 正常图像处理与状态机阶段
    // ==========================================================
    if(get_start_point()) //如果找到起点
    {
        // 【解释】从底部开始向上利用八邻域爬线，如果在某一行遇到宽度突变（Trigger），
        // 爬线函数内部会改变 cur_state 的值并记录触发点 Y_trigger。
        search_l_r(start_point_l,start_center_y,start_point_r,start_center_y); //八邻域搜索边界
        
        // =========================================================================
        // 节点甄别状态及分支处理
        // 响应 Trigger 2.0 在 search_l_r 中触发的中断
        // =========================================================================
        if (cur_state == STATE_CHECK_NODE)
        {
            // 【解释】在 Y_trigger 上方画 15cm 的动态框，判断是真岔路还是假干扰。
            uint8 is_true_node = Check_Node_Box(Y_trigger);
            if (is_true_node) 
            {
                cur_state = STATE_SMOOTH_OFFSET; // 真节点，切入偏置状态
            } 
            else 
            {
                cur_state = STATE_FALSE_IGNORE;  // 假干扰，如电阻等，进入强制拉直
            }
        }

        // --- 状态机转移执行 ---
        if (cur_state == STATE_FALSE_IGNORE)
        {
            // 应对假干扰，直接将 trigger 行到 Deal_Top 的所有行边缘覆盖
            // 用这种方式强行改写边界，防止跟着锯齿跑偏
            // 为了简单教学，我们直接拉直处理
            for (int y = Y_trigger; y <= Deal_Top; y++) {
                imgOSTU[y][Deal_Left] = Left_line;
                imgOSTU[y][Deal_Right] = Right_line;
            }
            // 拉直后复位 NORMAL 状态，等待下一帧处理
            cur_state = STATE_NORMAL; 
        }
        else if (cur_state == STATE_SMOOTH_OFFSET)
        {
            // 应对真节点：查看路径规划数组，进行动态偏置引导
            uint8 target_dir = Path_Array[node_index]; // 0左转 1右转 2直行
            if (target_dir == 0) {
                // 左转：极强左偏引导车头偏向真节点岔路
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = Deal_Left + 5; 
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
            } else if (target_dir == 1) {
                // 右转：右偏引导
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = Deal_Right - 5; 
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
            } else {
                // 直行：取中
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = XM / 2;
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
            }
            
            // 为防止直接切入陀螺仪盲转前视角不确认（极偏可能导致提前撞击电阻等干扰），
            // 所以我们暂定为偏置一帧（或几帧）后再切盲转
            if (target_dir == 0) { Yaw_Target = 90.0f; } // 左转
            else if (target_dir == 1) { Yaw_Target = -90.0f; } // 右转
            else { Yaw_Target = 0.0f; } // 直行

            is_blind_turning = 1; // 开启盲转锁
            cur_state = STATE_BLIND_TURN_YAW;
            return; // 退出本帧图像处理
        }

        if(cur_state == STATE_NORMAL)  //常态巡线执行
        {
            Get_lost_tip(2); //获取边缘丢失特征

            Get_start_center(); //获取中线起点
            if(b_lost_num > 0 && t_lost_num) //上下直线
            {
               Stop_line = Get_top_straightline();
            }
            else if(b_lost_num > 0 && l_lost_num && !t_lost_num && !r_lost_num)
            {
                Stop_line = Left_curve_line(); //左曲线
            }
            else if(b_lost_num > 0 && r_lost_num && !t_lost_num && !l_lost_num)
            {
                Stop_line = Right_curve_line(); //右曲线
            }
            else
            {
                Stop_line = 0;
                IF = curve;
            }
        }
    }
}

#include "run.h"
#include "image_deal_best.h"
#include "imu660.h"
#include "motor.h"
#include "pid.h"
#include "filter.h"

/******************************************************************************
* 函数名称     : go
* 描述         : 图像处理主函数，每帧调用一次
*                负责：图像采集 -> 二值化 -> 八邻域 -> 状态机分发 -> 补线
* 进入参数     : void
* 返回参数     : void
******************************************************************************/
void go(void)
{
    /* ========== 第一步：图像采集与预处理 ========== */
    memcpy(imgGray[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    nowThreshold = getOSTUThreshold();
    Get_imgOSTU();

    
    IF_L = IF;
    memset(mid_line, XM/2, sizeof(mid_line));

    /* ========== 第二步：盲转强制阶段 ========== */
    /* 如果陀螺仪盲转正在进行，图像不做任何决策，直接返回 */
    if (is_blind_turning)
    {
        for (uint8 i = 0; i <= Deal_Top; i++)
        {
            mid_line[i] = XM / 2;
            imgOSTU[i][(uint8)mid_line[i]] = Control_line;
        }
        return;
    }

    /* ========== 第三步：寻找起点并八邻域爬线 ========== */
    if (get_start_point())
    {
        /* 八邻域爬线（内部含 Trigger 2.0 打断逻辑） */
        search_l_r(start_point_l, start_center_y, start_point_r, start_center_y);

        /* ========== 第四步：状态机分发 ========== */
        if (cur_state == STATE_CHECK_NODE)
        {
            uint8 box_result = Check_Node_Box(Y_trigger);

            if (box_result == 0) {
                cur_state = STATE_FALSE_IGNORE;   /* 假干扰 */
            }
            else if (box_result == 1) {
                cur_state = STATE_SMOOTH_OFFSET;  /* 真节点 */
            }
            else if (box_result == 2) {
                cur_state = STATE_L_CORNER;       /* 纯左直角弯 */
            }
            else if (box_result == 3) {
                cur_state = STATE_R_CORNER;       /* 纯右直角弯 */
            }
        }

        /* ===== 各状态执行 ===== */
        if (cur_state == STATE_FALSE_IGNORE)
        {
            /* 假干扰：从触发行到顶部拉一条预测直线穿过去 */
            float k = (float)((XM/2) - start_center_x) / (float)(Deal_Top - Y_trigger + 1);
            for (int y = Y_trigger; y <= Deal_Top; y++) {
                mid_line[y] = (uint8)(start_center_x + k * (y - Y_trigger));
                imgOSTU[y][mid_line[y]] = Control_line;
            }
            cur_state = STATE_NORMAL;
        }
        else if (cur_state == STATE_SMOOTH_OFFSET)
        {
            /* 真节点：查路径数组，偏置引导线，启动盲转 */
            uint8 target_dir = Path_Array[node_index]; /* 0=左转 1=右转 2=直行 */

            if (target_dir == 0) {
                /* 左转：强制偏向左侧 */
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = 0;
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
                Yaw_Target = Yaw_Start + 90.0f;
            }
            else if (target_dir == 1) {
                /* 右转：强制偏向右侧 */
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = XM;
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
                Yaw_Target = Yaw_Start - 90.0f;
            }
            else {
                /* 直行：中间拉线 */
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = XM / 2;
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
                Yaw_Target = Yaw_Start;
            }

            is_blind_turning = 1;
            node_index++;
            cur_state = STATE_BLIND_TURN_YAW;
            return; /* 盲转接管，本帧不再做图像处理 */
        }
        else if (cur_state == STATE_L_CORNER || cur_state == STATE_R_CORNER)
        {
            /* 纯直角弯：用斜线预测漂移，不消耗 node_index */
            int direction_offset = (cur_state == STATE_L_CORNER) ? -40 : 40;
            float k = (float)(direction_offset) / (float)(Deal_Top - Y_trigger + 1);
            for (int y = Y_trigger; y <= Deal_Top; y++) {
                int calc_x = start_center_x + (int)(k * (y - Y_trigger));
                if (calc_x < 0) calc_x = 0;
                if (calc_x > XX) calc_x = XX;
                mid_line[y] = (uint8)calc_x;
                imgOSTU[y][mid_line[y]] = Control_line;
            }
            cur_state = STATE_NORMAL;
        }

        /* ========== 第五步：正常巡线补线 ========== */
        if (cur_state == STATE_NORMAL)
        {
            Get_lost_tip(30);
            Get_start_center();

            if (t_lost_num >= 1 && l_lost_num == 0 && r_lost_num == 0)
            {
                /* 只有上方丢线：绘制顶端直线 */
                Get_top_straightline();
            }
            else if (l_lost_num >= 1 && r_lost_num == 0)
            {
                /* 左边丢线多：左弯补线 */
                Left_curve_line();
            }
            else if (r_lost_num >= 1 && l_lost_num == 0)
            {
                /* 右边丢线多：右弯补线 */
                Right_curve_line();
            }
            else
            {
                /* 正常直道 */
                Get_top_straightline();
            }
        }
    }
    else
    {
        /* 未找到起点（全黑或全白），保持中线不变 */
        for (uint8 i = 0; i <= Deal_Top; i++)
        {
            imgOSTU[i][XM/2] = Control_line;
        }
    }
}

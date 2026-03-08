#include "run.h"

// 新增：用于维持盲转期间的斜线偏置轨迹
float blind_k = 0;           // 盲转维持斜率
int blind_bottom_x = XM / 2; // 盲转底部起始X坐标

/******************************************************************************
 * 函数名称     : go
 * 描述         : 图像处理主函数，每帧调用一次
 *                负责：图像采集 -> 二值化 -> 八邻域 -> 状态机分发 -> 补线
 * 进入参数     : void
 * 返回参数     : void
 ******************************************************************************/
void go(void) {
  /* ========== 第一步：图像采集与预处理 ========== */
  memcpy(imgGray[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
  nowThreshold = getOSTUThreshold();
  Get_imgOSTU();

  IF_L = IF;
  memset(mid_line, XM / 2, sizeof(mid_line));

  /* ========== 第二步：盲转强制阶段 ========== */
  /* 如果正在盲转，维持触发时计算好的斜线轨迹，完成换道平移 */
  if (is_blind_turning) {
    for (int y = Deal_Bottom; y <= Deal_Top; y++) {
      int calc_x = blind_bottom_x + (int)(blind_k * (y - Deal_Bottom));
      if (calc_x < 0)
        calc_x = 0;
      if (calc_x > XX)
        calc_x = XX;
      mid_line[y] = (uint8)calc_x;
      imgOSTU[y][mid_line[y]] = Control_line;
    }
    return;
  }

  /* ========== 第三步：寻找起点并八邻域爬线 ========== */
  if (get_start_point()) {
    /* 八邻域爬线（内部含 Trigger 2.0 打断逻辑） */
    search_l_r(start_point_l, start_center_y, start_point_r, start_center_y);

    /* ========== 第四步：状态机分发 ========== */
    if (cur_state == STATE_CAPACITY_CHECK) {
      int expected_box_top = Y_trigger + Step_15cm[Y_trigger];
      if (expected_box_top > Deal_Top)
        expected_box_top = Deal_Top;
      int expected_anchor_y = expected_box_top + 5; // 再往上 5 行找锚点

      // 视野容量判定
      if (expected_anchor_y >= Deal_Top - 2) {
        // 视野不够！路口太远了，框不全！
        cur_state = STATE_WAIT_NODE; // 盲开一小段，等它沉下来
      } else {
        // 视野充足，立刻起框甄别
        cur_state = STATE_CHECK_NODE;
      }
    }

    if (cur_state == STATE_WAIT_NODE) {
      static uint32 wait_distance_start = 0;
      if (wait_distance_start == 0)
        wait_distance_start = Distance_Integral;

      // 强行用一个较低的锚点查框，看路口有没有沉下来
      uint8 temp_box_result = Check_Node_Box(Deal_Bottom + 20);

      if (temp_box_result != 0) {
        cur_state = STATE_CHECK_NODE; // 沉下来了，重新甄别
        wait_distance_start = 0;
      } else if (Distance_Integral - wait_distance_start > 1000) {
        // 走过了10cm左右还没看到框，判定为假路口或误触发
        cur_state = STATE_NORMAL; // 强制弃用，防止死锁！
        wait_distance_start = 0;
      }
    }

    if (cur_state == STATE_CHECK_NODE) {
      uint8 box_result = Check_Node_Box(Y_trigger);

      if (box_result == 0) {
        cur_state = STATE_FALSE_IGNORE; /* 假干扰 */
      } else if (box_result == 1) {
        cur_state = STATE_SMOOTH_OFFSET; /* 真节点 */
      } else if (box_result == 2) {
        cur_state = STATE_L_CORNER; /* 纯左直角弯 */
      } else if (box_result == 3) {
        cur_state = STATE_R_CORNER; /* 纯右直角弯 */
      }
    }

    /* ===== 各状态执行 ===== */
    if (cur_state == STATE_FALSE_IGNORE ||
        (cur_state == STATE_SMOOTH_OFFSET && Path_Array[node_index] == 2)) {
      /* 假干扰 或 真节点直行：计算动态上下锚点，画斜线诱导换道 */
      int top_scan_y = Y_trigger + Step_15cm[Y_trigger];
      top_scan_y += 2; // 稍微越过干扰区顶部往外找
      if (top_scan_y > Deal_Top - 2)
        top_scan_y = Deal_Top - 5; // 防越界

      int valid_cnt = 0;
      int valid_x_sum = 0;
      // 向上扫 5 行，寻找真实的白色出口中点作为 Top_Anchor
      for (int y = top_scan_y; y <= top_scan_y + 4; y++) {
        int left_edge = Deal_Left, right_edge = Deal_Right;
        for (int x = start_center_x; x > Deal_Left; x--)
          if (imgOSTU[y][x] == Black) {
            left_edge = x;
            break;
          }
        for (int x = start_center_x; x < Deal_Right; x++)
          if (imgOSTU[y][x] == Black) {
            right_edge = x;
            break;
          }
        if (right_edge > left_edge) {
          valid_x_sum += (left_edge + right_edge) / 2;
          valid_cnt++;
        }
      }

      int top_anchor_x = (valid_cnt > 0) ? (valid_x_sum / valid_cnt) : (XM / 2);
      int bottom_anchor_x = start_center_x;
      float k = (float)(top_anchor_x - bottom_anchor_x) /
                (float)(top_scan_y - Y_trigger + 1);

      // 划定单帧斜线
      for (int y = Y_trigger; y <= Deal_Top; y++) {
        int calc_x = bottom_anchor_x + (int)(k * (y - Y_trigger));
        if (calc_x < 0)
          calc_x = 0;
        if (calc_x > XX)
          calc_x = XX;
        mid_line[y] = (uint8)calc_x;
        imgOSTU[y][mid_line[y]] = Control_line;
      }

      if (cur_state == STATE_FALSE_IGNORE) {
        cur_state = STATE_NORMAL; // 假干扰直接恢复
      } else {
        /* 真节点直行：保存轨迹，切入盲转 */
        blind_k = k;
        blind_bottom_x = mid_line[Deal_Bottom];
        Yaw_Target = Yaw_Start; // 陀螺仪不偏航，保持直走姿态
        is_blind_turning = 1;   // 交给盲转维持视觉斜线
        node_index++;
        cur_state = STATE_BLIND_TURN_YAW;
      }
    } else if (cur_state == STATE_SMOOTH_OFFSET &&
               Path_Array[node_index] != 2) {
      /* 真节点转弯：0=左转 1=右转 (保留原逻辑，暴力切角) */
      uint8 target_dir = Path_Array[node_index];
      if (target_dir == 0) {
        blind_bottom_x = 0;
        blind_k = 0;
        Yaw_Target = Yaw_Start + 90.0f;
      } else if (target_dir == 1) {
        blind_bottom_x = XM;
        blind_k = 0;
        Yaw_Target = Yaw_Start - 90.0f;
      }
      is_blind_turning = 1;
      node_index++;
      cur_state = STATE_BLIND_TURN_YAW;
    } else if (cur_state == STATE_L_CORNER || cur_state == STATE_R_CORNER) {
      /* 纯直角弯：用斜线预测漂移，不消耗 node_index */
      int direction_offset = (cur_state == STATE_L_CORNER) ? -40 : 40;
      float k = (float)(direction_offset) / (float)(Deal_Top - Y_trigger + 1);
      for (int y = Y_trigger; y <= Deal_Top; y++) {
        int calc_x = start_center_x + (int)(k * (y - Y_trigger));
        if (calc_x < 0)
          calc_x = 0;
        if (calc_x > XX)
          calc_x = XX;
        mid_line[y] = (uint8)calc_x;
        imgOSTU[y][mid_line[y]] = Control_line;
      }
      cur_state = STATE_NORMAL;
    }

    /* ========== 第五步：正常巡线补线 ========== */
    if (cur_state == STATE_NORMAL) {
      Get_lost_tip(30);
      Get_start_center();

      if (t_lost_num >= 1 && l_lost_num == 0 && r_lost_num == 0) {
        /* 只有上方丢线：绘制顶端直线 */
        Get_top_straightline();
      } else if (l_lost_num >= 1 && r_lost_num == 0) {
        /* 左边丢线多：左弯补线 */
        Left_curve_line();
      } else if (r_lost_num >= 1 && l_lost_num == 0) {
        /* 右边丢线多：右弯补线 */
        Right_curve_line();
      } else {
        /* 正常直道 */
        Get_top_straightline();
      }
    }
  } else {
    /* 未找到起点（全黑或全白），保持中线不变 */
    for (uint8 i = 0; i <= Deal_Top; i++) {
      imgOSTU[i][XM / 2] = Control_line;
    }
  }
}

// 强制 IDE 刷新时间戳：8d8ad4f6

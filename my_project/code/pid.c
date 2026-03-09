#include "pid.h"
#include "filter.h"
#include "image_deal_best.h"
#include "imu660.h"
#include "motor.h"

int16_t speed_straight_l = 250;
int16_t speed_straight_s = 180;
int16_t speed_curve = 120;

float turn_kp_base = 1.50f;
float turn_kp_var = 0.05f;
float turn_kd = 0.80f;

float gyro_kp = 1.20f;
float gyro_kd = 0.10f;
float blind_turn_kp = 4.5f;
float blind_turn_kd = 0.3f;

float speed_kp = 18.0f;
float speed_ki = 1.50f;

float speed_kff = 15.0f;
float planned_speed = 0.0f;

System_Control_State_t ctrl_state;
Positional_PD_t pid_turn_outer;
Positional_PD_t pid_gyro_middle;
Incremental_PI_t pid_speed_L;
Incremental_PI_t pid_speed_R;

void PID_Init(void) {
  float max_pwm = 9500.0f;
  pid_speed_L.out_max = max_pwm;
  pid_speed_L.out_min = -max_pwm;
  pid_speed_R.out_max = max_pwm;
  pid_speed_R.out_min = -max_pwm;
}

float Low_Pass_Filter(float current_val, float last_val, float alpha) {
  return alpha * current_val + (1.0f - alpha) * last_val;
}

float Calc_Incremental_PI(Incremental_PI_t *pid, float target, float current) {
  float increment;
  pid->error = target - current;
  increment = pid->Kp * (pid->error - pid->last_error) + pid->Ki * pid->error;
  pid->output += increment;
  if (pid->output > pid->out_max)
    pid->output = pid->out_max;
  if (pid->output < pid->out_min)
    pid->output = pid->out_min;
  pid->last_error = pid->error;
  return pid->output;
}

float Outer_Loop_Camera(void) {
  float current_v = (Actual_Speed[0] + Actual_Speed[1]) / 2.0f;
  uint8_t dynamic_lookahead = 10 + (uint8_t)(current_v * 0.05f);
  uint8_t aim_row = Deal_Bottom + dynamic_lookahead;

  switch (cur_state) {
  case STATE_FALSE_IGNORE:
  case STATE_WAIT_NODE:
    aim_row = Deal_Top - 5;
    break;
  case STATE_SMOOTH_OFFSET:
  case STATE_L_CORNER:
  case STATE_R_CORNER:
  case STATE_BLEND:
    aim_row = Deal_Bottom + 10;
    break;
  default:
    if (IF == straightlineL)
      aim_row += 10;
    else if (IF == curve)
      aim_row -= 5;
    break;
  }

  if (aim_row > Deal_Top - 5)
    aim_row = Deal_Top - 5;
  if (aim_row < Deal_Bottom + 5)
    aim_row = Deal_Bottom + 5;

  float sum_y = 0, sum_x = 0, sum_yx = 0, sum_y2 = 0;
  int fit_points = 10;
  int start_y = aim_row - fit_points / 2;
  for (int i = 0; i < fit_points; i++) {
    int y_val = start_y + i;
    float x_val = (float)mid_line[y_val];
    sum_y += y_val;
    sum_x += x_val;
    sum_yx += y_val * x_val;
    sum_y2 += y_val * y_val;
  }
  float mean_y = sum_y / fit_points;
  float mean_x = sum_x / fit_points;

  float denominator = sum_y2 - fit_points * mean_y * mean_y;
  float slope_k = 0;
  if (denominator > 0.1f) {
    slope_k = (sum_yx - fit_points * mean_y * mean_x) / denominator;
  }
  float intercept_b = mean_x - slope_k * mean_y;

  // 【最小二乘法 - 航向角与横向偏移融合计算】
  // 说明：offset_x 表示在当前前瞻行横向车体的偏移量；slope_k
  // 表示连线的斜率，反映车头偏角（航向角）。 调参建议： K1
  // 是截距权重，控制车子必须在线中心。K1 过大容易画蛇画龙（震荡）。 K2
  // 是斜率(航向角)权重，主要用于消除车头歪的情况。如果直线跑得稳但是入弯反应极度迟钝，适当增加
  // K2！
  float offset_x = (slope_k * aim_row + intercept_b) - (XM / 2.0f);

  float K1 = 1.0f;  // 横向偏差系数
  float K2 = 15.0f; // 航向角偏差系数 (前瞻视野中的斜率放大系数)
  ctrl_state.camera_error = -(K1 * offset_x + K2 * slope_k);

  pid_turn_outer.error = ctrl_state.camera_error;

  float current_kp = turn_kp_base;
  float current_kd = turn_kd;

  switch (cur_state) {
  case STATE_NORMAL:
  case STATE_CAPACITY_CHECK:
  case STATE_CHECK_NODE:
    current_kp = turn_kp_base + turn_kp_var * fabsf(pid_turn_outer.error);
    current_kd = turn_kd;
    break;
  case STATE_FALSE_IGNORE:
  case STATE_WAIT_NODE:
    current_kp = turn_kp_base * 0.4f;
    current_kd = turn_kd * 2.5f;
    break;
  case STATE_SMOOTH_OFFSET:
  case STATE_L_CORNER:
  case STATE_R_CORNER:
    current_kp = turn_kp_base * 1.5f;
    current_kd = turn_kd * 0.3f;
    break;
  case STATE_BLIND_TURN_YAW:
    current_kp = 0;
    current_kd = 0;
    break;
  case STATE_BLEND:
    current_kp = turn_kp_base;
    current_kd = turn_kd;
    break;
  }

  pid_turn_outer.Kp = current_kp;
  pid_turn_outer.Kd = current_kd;
  pid_turn_outer.output =
      pid_turn_outer.Kp * pid_turn_outer.error +
      pid_turn_outer.Kd * (pid_turn_outer.error - pid_turn_outer.last_error);
  pid_turn_outer.last_error = pid_turn_outer.error;

  if (pid_turn_outer.output > 300.0f)
    pid_turn_outer.output = 300.0f;
  if (pid_turn_outer.output < -300.0f)
    pid_turn_outer.output = -300.0f;

  return pid_turn_outer.output;
}

uint8_t blind_turn_finished = 0;

void Control_Loop(void) {
  pid_gyro_middle.Kp = gyro_kp;
  pid_gyro_middle.Kd = gyro_kd;
  pid_speed_L.Kp = speed_kp;
  pid_speed_L.Ki = speed_ki;
  pid_speed_R.Kp = speed_kp;
  pid_speed_R.Ki = speed_ki;

  // ================= 速度分配重构 (严格状态分发) =================
  if (cur_state == STATE_BLIND_TURN_YAW) {
    // 盲转段，图像无参考价值，细分直行过路口 vs 转弯盲转
    // 注意：run.c 在触发盲转前已执行 node_index++，此处用前一个状态
    if (Path_Array[node_index - 1] == 2) {
      ctrl_state.base_speed = speed_straight_s; // 直行过路口的假节点，减速去
    } else {
      ctrl_state.base_speed = speed_curve; // 弯转盲转期，硬拐进弯
    }
  } else if (cur_state == STATE_CHECK_NODE ||
             cur_state == STATE_CAPACITY_CHECK) {
    ctrl_state.base_speed =
        speed_curve * 0.8f; // 检框阶段，减速防飞 MCU 等帧时间
  } else if (cur_state == STATE_L_CORNER || cur_state == STATE_R_CORNER ||
             cur_state == STATE_SMOOTH_OFFSET) {
    ctrl_state.base_speed = speed_curve; // 直角弯和节点引导期，侧倾高危
  } else if (cur_state == STATE_FALSE_IGNORE || cur_state == STATE_WAIT_NODE) {
    ctrl_state.base_speed =
        speed_straight_l; // 假干扰确认完，全速冲段、拥有赛道
  } else {
    // 正常巡线阶段 (STATE_NORMAL / STATE_BLEND)，交给底层 IF 标志
    switch (IF) {
    case straightlineL:
      ctrl_state.base_speed = speed_straight_l; // 双边可见的长直道
      break;
    case straightlineS:
      ctrl_state.base_speed = speed_straight_s; // 短或半截直道比较行驶
      break;
    case curve:
      ctrl_state.base_speed = speed_curve; // 弯道段，底层判定为固定弯角
      break;
    default:
      ctrl_state.base_speed = speed_straight_s;
      break;
    }
  }
  // ======================================================================

  ctrl_state.angular_rate_target = Outer_Loop_Camera();

  // 【盲转过渡态 (STATE_BLEND) 离合器】
  // 说明：盲转结束时交还控制权瞬间，利用下降的 alpha
  // 将陀螺仪权重(0)逐渐淡出，镜头权重(1.0-alpha)逐渐上升。 调参建议：0.05f 表示
  // 1/20 个周期（如 100ms内平滑切入）。如果交接还是抽搐，减小 0.05f（如
  // 0.02f），延长过渡时间。
  if (cur_state == STATE_BLEND) {
    static float blend_alpha = 1.0f; // 逐渐减小
    blend_alpha -= 0.05f;
    if (blend_alpha <= 0.0f) {
      blend_alpha = 0.0f;
      cur_state = STATE_NORMAL;
      blend_alpha = 1.0f;
    }
    // 陀螺仪目标角速度在此刻可近似认为是 0 或者是维持直线。此处与 0.0 融合。
    ctrl_state.angular_rate_target =
        (1.0f - blend_alpha) * ctrl_state.angular_rate_target +
        blend_alpha * 0.0f;
  }

  float raw_gyro = gyro_param.gyro_z;

  if (!velocity_filter.initialized) {
    LPF_InitByAlpha(&velocity_filter, 0.6f);
  }

  ctrl_state.angular_rate_current = LPF_Update(&velocity_filter, raw_gyro);

  static float Start_Dist = 0;

  if (is_blind_turning == 1) {
    float yaw_error = Yaw_Target - (yaw_plus - Yaw_Start);

    if (abs((int)Yaw_Target) < 1) {
      ctrl_state.angular_rate_target = 0;
      ctrl_state.base_speed = speed_straight_s;

      if ((Distance_Integral - Start_Dist) > 1500.0f) {
        is_blind_turning = 0;
      }
    } else {
      if (abs((int)yaw_error) < 5 || blind_turn_finished) {
        blind_turn_finished = 1;
        ctrl_state.angular_rate_target = gyro_param.gyro_z * 0.5f;
        is_blind_turning = 0;
        cur_state = STATE_BLEND;
      } else {
        ctrl_state.angular_rate_target =
            blind_turn_kp * yaw_error - blind_turn_kd * gyro_param.gyro_z;
        if (ctrl_state.angular_rate_target > 250.0f)
          ctrl_state.angular_rate_target = 250.0f;
        if (ctrl_state.angular_rate_target < -250.0f)
          ctrl_state.angular_rate_target = -250.0f;
        ctrl_state.base_speed = speed_curve;
      }
    }
  } else {
    blind_turn_finished = 0;
    Yaw_Start = yaw_plus;
    Start_Dist = Distance_Integral;
  }

  pid_gyro_middle.error =
      ctrl_state.angular_rate_target - ctrl_state.angular_rate_current;
  pid_gyro_middle.output =
      pid_gyro_middle.Kp * pid_gyro_middle.error +
      pid_gyro_middle.Kd * (pid_gyro_middle.error - pid_gyro_middle.last_error);
  pid_gyro_middle.last_error = pid_gyro_middle.error;
  ctrl_state.speed_diff = pid_gyro_middle.output;

  // 【速度控制斜坡防滑处理】
  // 说明：直接阶跃会引发轮胎空转或抽搐打滑。
  // target_step 指每次控制周期的最大速度增量。
  // 调参建议：如果加速打滑，请把 target_step 调小（如 1.5f
  // 或更低）；如果提速不够猛，适当增加。
  float target_step = 3.0f;
  if (planned_speed < ctrl_state.base_speed - target_step) {
    planned_speed += target_step;
  } else if (planned_speed > ctrl_state.base_speed + target_step) {
    planned_speed -= target_step;
  } else {
    planned_speed = ctrl_state.base_speed;
  }

  ctrl_state.target_left_speed = planned_speed - ctrl_state.speed_diff;
  ctrl_state.target_right_speed = planned_speed + ctrl_state.speed_diff;

  ctrl_state.current_left_speed = Actual_Speed[0];
  ctrl_state.current_right_speed = Actual_Speed[1];

  // 【电机前馈控制】
  // 说明：(K_ff * target_speed) 直接给电机兜底基础 PWM 输出，PI
  // 只做微小修正。极大改善响应速度！ 调参建议：先将 K_ff 置
  // 0，把电机调到能跑，再加 K_ff 使设定速度和响应速度贴合。
  float l_total =
      (speed_kff * ctrl_state.target_left_speed) +
      Calc_Incremental_PI(&pid_speed_L, ctrl_state.target_left_speed,
                          ctrl_state.current_left_speed);
  float r_total =
      (speed_kff * ctrl_state.target_right_speed) +
      Calc_Incremental_PI(&pid_speed_R, ctrl_state.target_right_speed,
                          ctrl_state.current_right_speed);

  if (l_total > 9500.0f)
    l_total = 9500.0f;
  else if (l_total < -9500.0f)
    l_total = -9500.0f;
  if (r_total > 9500.0f)
    r_total = 9500.0f;
  else if (r_total < -9500.0f)
    r_total = -9500.0f;

  ctrl_state.output_left_pwm = (int16_t)l_total;
  ctrl_state.output_right_pwm = (int16_t)r_total;

  if (system_running == 1) {
    Motor_Control(ctrl_state.output_left_pwm, ctrl_state.output_right_pwm);
  } else {
    Motor_Control(0, 0);
  }
}

// 强制 IDE 刷新时间戳：f82c8131

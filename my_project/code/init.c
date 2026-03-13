#include "init.h"

/******************************************************************************
 * 函数名称     : init
 * 描述         : 系统全量初始化，统一在此调用，保持 main 函数整洁
 * 进入参数     : void
 * 返回参数     : void
 ******************************************************************************/
void init(void) {
  /* 1. 摄像头初始化 */
  mt9v03x_init();

  /* 2. TFT 屏幕初始化（横屏） */
  ips200_set_dir(IPS200_CROSSWISE_180);
  ips200_init(IPS200_TYPE_SPI);

  /* 3. 电机与编码器初始化 */
  Motor_Init();
  Encoder_Init();

  /* 4. IMU660RB 初始化 + 零偏校准（至少等待 500ms，等陀螺仪稳定） */
  imu660rb_init();
  system_delay_ms(500);
  gyro_zero_param_init();
  acc_zero_param_init();

  /* 5. PID 结构体初始化（设置输出限幅） */
  PID_Init();

  /* 6. 速度低通滤波器初始化（10Hz 截止频率，5ms 控制周期） */
  LPF_InitByFrequency(&velocity_filter, 10.0f, 0.005f);

  /* 7. 图像状态机复位 */
  standard();
  cur_state = STATE_NORMAL;
  is_blind_turning = 0;
  node_index = 0;

  /* 8. UI 菜单初始化 */
  UI_Menu_Init();

  /* 9. 开启定时器中断：5ms 一次触发 cc60_pit_ch0_isr（控制循环） */
  pit_ms_init(CCU60_CH0, 5);
}

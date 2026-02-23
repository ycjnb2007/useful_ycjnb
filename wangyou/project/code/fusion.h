/*
 徐州工程学院
 */
#ifndef FUSION_H_
#define FUSION_H_


#include "zf_common_headfile.h"
#include "all_date.h"


extern   _st_AngE eulerAngle;
extern  _st_Mpu pMpu;
extern float  Now_yaw ;
extern float angular_rate_current; // 新增全局变量
void imuComputer(void);

// 添加到函数声明部分
void start_circle_angle_tracking(void);
void update_circle_angle_tracking(void);
extern float cha_yaw;
#endif /* FUSION_H_ */

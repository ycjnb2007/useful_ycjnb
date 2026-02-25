#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_
#include "zf_common_headfile.h"


//void calibrate_gyro(void) ;

void update_angle(void);

extern float filtered_omega_z;

extern float theta_total;

#endif
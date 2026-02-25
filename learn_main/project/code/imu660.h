/*
 徐州工程学院
 */
#ifndef IMU660_H_
#define IMU660_H_

#include "zf_common_headfile.h"

void IMU660RAReadGyro(short *gyroData);
void IMU660RAReadAcc(short *accData);
void imu_init(void);
void imu_data_calibration(short *gx, short *gy, short *gz, short *ax, short *ay, short *az);

#endif 

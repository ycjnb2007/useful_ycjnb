#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "zf_common_typedef.h"

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float roll;
    float pitch;
    float yaw;
    float gx_bias;
    float gy_bias;
    float gz_bias;
    uint8_t calibrated;
    uint16_t calibration_samples;
} Quaternion;

void quaternion_init(void);
void quaternion_calibrate(void);
void quaternion_update(int16 gx, int16 gy, int16 gz, float dt);
Quaternion* get_euler_angles(void);
void quaternion_reset(void);





#endif
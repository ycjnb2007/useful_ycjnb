/*
 ??????
 */
#ifndef IMU_H_
#define IMU_H_
#include "all_date.h"
#include "zf_common_headfile.h"



//extern float GetNormAccz(void);
extern void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt);
extern void imu_rest(void);

extern void imuUpdate(const _st_Mpu *pMpu, _st_AngE *pAngE , float dt);

#endif /* IMU_H_ */

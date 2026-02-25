#ifndef _gyro_h_
#define _gyro_h_

#define pi (3.14f)

typedef struct
{
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_zero_paramTypedef; // 零飘参数；

typedef struct
{
    float Xdata;
    float Ydata;
    float Zdata;
} acc_zero_paramTypedef; // 零飘参数；

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} gyro_paramTypedef; // 角加速度

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
} acc_paramTypedef; // 加速度计

extern volatile gyro_paramTypedef gyro_param; // 角加速度
extern volatile acc_paramTypedef acc_param;   // 加速度计
extern uint8_t gyro_zero_flag;                // 陀螺仪零飘校准完成标志；
extern uint8_t acc_zero_flag;                 // 陀螺仪零飘校准完成标志；
extern float yaw_memery[FLASH_PAGE_LENGTH];   // 记录的航偏角
extern float yaw_store[FLASH_PAGE_LENGTH];    // 打点储存的历史航偏角
extern volatile float yaw, yaw_plus;          // 实时航偏角
extern volatile float acc_y_speed;

void acc_y_integral(void);
void gyro_yaw_integral(void);    // 中值积分
void gyro_transform_value(void); // 转为实际物理值
void gyro_zero_param_init(void); // 标定零飘参数
void acc_transform_value(void);  // 转为实际物理值
void acc_zero_param_init(void);  // 标定零飘参数
void reset_yaw_integral(void);  //重置偏航角        

#endif
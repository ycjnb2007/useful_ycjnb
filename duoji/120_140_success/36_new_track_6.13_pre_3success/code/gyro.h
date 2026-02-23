#ifndef _gyro_h_
#define _gyro_h_

// 定义PI宏（部分编译器需要）
#define M_PI 3.14159265358979323846f
#define YAW_NUM      15


extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;

    // 四元数
typedef struct 
    {
        float q0;  // 实部
        float q1;  // 虚部i
        float q2;  // 虚部j
        float q3;  // 虚部k
    } Quaternion;

// 欧拉角结构体
typedef struct 
{
    float roll;   // 横滚角（度）
    float pitch;  // 俯仰角（度）
    float yaw;    // 偏航角（度）
} EulerAngles;
extern EulerAngles angles;

// 四元数姿态解算器结构体
typedef struct 
{

    Quaternion quaternion;
    // 控制参数
    float Kp;       // 比例增益
    float Ki;       // 积分增益
    float dt;       // 采样时间间隔(s)
    
    // 误差积分
    struct 
    {
        float ex;
        float ey;
        float ez;
    } integral_error;
    
} AttitudeEstimator;
extern AttitudeEstimator estimator;


extern float final_yaw[YAW_NUM];
extern uint8 final_yaw_num;

//零偏计数
void calibrate_gyro_bias(float* bias_x, float* bias_y, float* bias_z);

// 初始化函数
void attitude_estimator_init(AttitudeEstimator* estimator, 
                            float Kp, float Ki, 
                            float dt);

// 姿态更新函数
void attitude_update(AttitudeEstimator* estimator);

// 获取欧拉角
EulerAngles get_euler_angles(const AttitudeEstimator* estimator);

void reset_yaw_to_zero(AttitudeEstimator* estimator);

#endif
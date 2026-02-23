#ifndef _function_h_
#define _function_h_

uint8 Limit_uint8(uint8 min, uint8 input, uint8 max);
uint16 Limit_uint16(uint16 min, uint16 input, uint16 max);
uint32 Limit_uint32(uint32 min, uint32 input, uint32 max);
int32 Limit_int32(int32 min, int32 input, int32 max);
float Limit_float(float min, float input, float max);
double Limit_double(double min, double input, double max);


#define MAX_SENSOR_NUM 9   //使用滤波时的传感器数量
#define MAX_DATA_NUM 9     //最大采样点数量，即采样窗口长度
#define WINDOW_DATA_NUM 5  //滤波窗口长度
//去除采样窗口内最大最小值的数量，这里去除两个最大和两个最小
#define REMOVE_MAXMIN_NUM ((MAX_DATA_NUM - WINDOW_DATA_NUM)/2)
// 去除采样窗口内最大最小值的数量，这里去除两个最大和两个最小
#define M_PI 3.14f







/* 滤波器结构体（用户无需关心内部成员） */
typedef struct
{
    float alpha;         // 滤波系数（0.0-1.0）
    float prev_output;   // 上一次滤波输出
    uint8_t initialized; // 初始化标志位
} LowPassFilter;

    /**
     * @brief 初始化低通滤波器
     * @param filter    滤波器对象指针
     * @param alpha     滤波系数（直接指定）
     */
    void LPF_InitByAlpha(LowPassFilter *filter, float alpha);

    /**
     * @brief 通过截止频率和采样时间初始化滤波器
     * @param filter        滤波器对象指针
     * @param cutoff_freq   截止频率（Hz）
     * @param sample_time   采样时间（秒）
     */
    void LPF_InitByFrequency(LowPassFilter *filter, float cutoff_freq, float sample_time);

    /**
     * @brief 执行低通滤波计算
     * @param filter    滤波器对象指针
     * @param input     当前输入值
     * @return float    滤波后的输出值
     */
    float LPF_Update(LowPassFilter *filter, float input);

 extern  LowPassFilter velocity_filter;

    // 声明定义的函数
    int16 Filter_SlidingWindowAvg(int index, int16 data);


#endif
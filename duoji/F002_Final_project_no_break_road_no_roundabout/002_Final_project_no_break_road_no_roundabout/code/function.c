#include "zf_common_headfile.h"


/***********************************限幅函数*************************************/
uint8 Limit_uint8(uint8 min, uint8 input, uint8 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

uint16 Limit_uint16(uint16 min, uint16 input, uint16 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

uint32 Limit_uint32(uint32 min, uint32 input, uint32 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

int32 Limit_int32(int32 min, int32 input, int32 max)
{
    return (input > max) ? max : (input < min) ? min : input;
}

float Limit_float(float min, float input, float max)
{
  return (input > max) ? max : (input < min) ? min : input;
}

double Limit_double(double min, double input, double max)
{
    return (input > max) ? max : (input < min) ? min : input;
}


LowPassFilter velocity_filter={0};




void LPF_InitByAlpha(LowPassFilter *filter, float alpha)
{
    filter->alpha = alpha;
    filter->prev_output = 0.0f;
    filter->initialized = 0;
}

void LPF_InitByFrequency(LowPassFilter *filter, float cutoff_freq, float sample_time)
{
    // 计算RC时间常数
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    // 计算滤波系数
    filter->alpha = sample_time / (rc + sample_time);
    filter->prev_output = 0.0f;
    filter->initialized = 0;
}

float LPF_Update(LowPassFilter *filter, float input)
{
    // 首次调用时直接初始化输出值
    if (!filter->initialized)
    {
        filter->prev_output = input;
        filter->initialized = 1;
        return input;
    }

    // 一阶低通滤波公式
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;

    // 更新状态
    filter->prev_output = output;

    return output;
}
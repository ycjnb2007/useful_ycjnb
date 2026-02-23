#ifndef _FUNCATION_H_
#define _FUNCATION_H_

#include "zf_common_headfile.h"
extern float turn_diff;
// 限幅函数
/**
 * @brief 整型数值限幅函数
 * @param min 最小值
 * @param val 待限幅的值
 * @param max 最大值
 * @return 限幅后的值
 */
int Limit_int(int min, int val, int max);

/**
 * @brief 浮点型数值限幅函数
 * @param min 最小值
 * @param val 待限幅的值
 * @param max 最大值
 * @return 限幅后的值
 */
float Limit_float(float min, float val, float max);

// 绝对值函数
/**
 * @brief 浮点型绝对值函数
 * @param f_number 输入浮点数
 * @return 绝对值结果
 */
float my_abs(float f_number);

/**
 * @brief 整型绝对值函数
 * @param num 输入整数
 * @return 绝对值结果
 */
int ads_num(int num);

unsigned int sqrt_16(unsigned long M);
void Lianxian(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end);

#endif // _FUNCATION_H_
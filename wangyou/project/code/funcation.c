#include "funcation.h"
float turn_diff;

/**
 * @brief 中线连线函数
 * @details 使用Bresenham直线算法在两点间生成中线点序列
 * @param x_start 起始点X坐标
 * @param y_start 起始点Y坐标
 * @param x_end 终点X坐标
 * @param y_end 终点Y坐标
 * @note 算法特点：
 *       - 基于Bresenham算法，保证直线连续性
 *       - 自动边界检查，防止数组越界
 *       - 最多生成130个中线点，防止溢出
 * @author 尘烟
 */
void Lianxian(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end)
{
    x_start = (x_start < LCDW) ? x_start : LCDW - 1; // 边界检查
    y_start = (y_start < LCDH) ? y_start : LCDH - 1;
    x_end = (x_end < LCDW) ? x_end : LCDW - 1;
    y_end = (y_end < LCDH) ? y_end : LCDH - 1;

    int16 dx = abs(x_end - x_start);
    int16 dy = abs(y_end - y_start);
    int16 sx = (x_start < x_end) ? 1 : -1;
    int16 sy = (y_start < y_end) ? 1 : -1;
    int16 err = dx - dy;
    int16 current_err;

    Mid_line_count = 0; // 重置中线数组索引

    while (Mid_line_count < 130) // 防止数组溢出
    {
        Mid_line[Mid_line_count].col = x_start; // 将当前点加入数组
        Mid_line[Mid_line_count].row = y_start;
        Mid_line_count++;

        if (x_start == x_end && y_start == y_end) // 到达终点则退出循环
        {
            break;
        }

        current_err = 2 * err;

        if (current_err > -dy) // 更新误差和坐标
        {
            err -= dy;
            x_start += sx;
        }
        if (current_err < dx)
        {
            err += dx;
            y_start += sy;
        }
    }
}

/***************限幅***************** */
int Limit_int(int min, int val, int max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

float Limit_float(float min, float val, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

/********************绝对值*************** */
float my_abs(float f_number) // 求绝对值
{
    if (f_number > 0)
    {
        return f_number;
    }
    else
    {
        return -f_number;
    }
}
int ads_num(int num)
{
    if (num > 0)
    {
        return num;
    }
    else
    {
        return -num;
    }
}

unsigned int sqrt_16(unsigned long M)
{
    if (M <= 0)
    {
        M = -M;
    }
    unsigned int N, i;
    unsigned long tmp, ttp; // 结果、循环计数
    if (M == 0)             // 被开方数，开方结果也为0
        return 0;

    N = 0;

    tmp = (M >> 30); // 获取最高位：B[m-1]
    M <<= 2;
    if (tmp > 1) // 最高位为1
    {
        N++; // 结果当前位为1，否则为默认的0
        tmp -= N;
    }

    for (i = 15; i > 0; i--) // 求剩余的15位
    {
        N <<= 1; // 左移一位

        tmp <<= 2;
        tmp += (M >> 30); // 假设

        ttp = N;
        ttp = (ttp << 1) + 1;

        M <<= 2;
        if (tmp >= ttp) // 假设成立
        {
            tmp -= ttp;
            N++;
        }
    }

    return N;
}

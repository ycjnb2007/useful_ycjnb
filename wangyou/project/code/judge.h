#ifndef _JUDGE_H_
#define _JUDGE_H_

#include "zf_common_headfile.h"

typedef struct JUDGE
{
    uint8 Left_Flag; // 左直角标志位

    uint8 Right_Flag; // 右直角标志位

    uint8 left_area_true; // 图像左侧区域

    uint8 right_area_true; // 图像右侧区域

    uint8 top_area_true; // 图像上侧区域
    uint8 bottom_area_true;

    uint8 up_0_1_num; // 0_1_num代表黑白跳变数量，1_0_num代表白黑跳变数量，up为图像顶部元素行

    uint8 up_1_0_num;

    uint8 down_0_1_num; // down为图像底部的元素行

    uint8 down_1_0_num;

    uint8 left_0_1_num; // left为图像左侧的元素列

    uint8 left_1_0_num;

    uint8 right_0_1_num; // right为图像右侧的元素列

    uint8 right_1_0_num;
} JUDGE;

struct ANGLE
{
    uint8 row;
    uint8 col;
};

extern JUDGE judge;
extern float angleErr;
void judge_rowline(void);
void judge_road(void);
void judge_angle(void);
void clear_flags(void);
void Angle_Lianxian(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end);
void Handle_Left_Angle();
void Handle_Right_Angle();

void Handle_Cross();

#endif
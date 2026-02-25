/*
 * go.c
 *
 *  Created on: 2025年12月26日
 *      Author: ycj01
 */

#include "go.h"
/******************************************************************************
* 函数名称     : go
* 描述        : 小车go！！！
* 进入参数     : voidֵ
* 返回参数     : void
******************************************************************************/
void go(void)
{
    memcpy(imgGray[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    nowThreshold = getOSTUThreshold();/*大津法获取阈值*/

    Get_imgOSTU(); /*获取二值化图像imgOSTU*/

    Dashedline_Makeup(); //断路处理范围扩展

    Stop_line = 0; //动态前瞻初始化

    IF_L = IF;
    memset(mid_line, XM/2, sizeof(mid_line));
    if(disappear_flag == 0 && get_start_point()) //如果找到起点
    {
        search_l_r(start_point_l,start_center_y,start_point_r,start_center_y); //八邻域搜索边界
        Get_lost_tip(2); //获取边缘丢失特征



        if(true)  //十字处理
        {
            Stop_line = Deal_crossroads();
        }

        if(Stop_line == 0 )  //常规巡线
        {
            Get_start_center(); //获取中线起点
            if(b_lost_num > 0 && t_lost_num) //上下直线
            {
               Stop_line = Get_top_straightline();
            }
            else if(b_lost_num > 0 && l_lost_num && !t_lost_num && !r_lost_num)
            {
                Stop_line = Left_curve_line(); //左曲线
            }
            else if(b_lost_num > 0 && r_lost_num && !t_lost_num && !l_lost_num)
            {
                Stop_line = Right_curve_line(); //右曲线
            }
            else
            {
                Stop_line = 0;
                IF = curve;
            }
        }

    }




}




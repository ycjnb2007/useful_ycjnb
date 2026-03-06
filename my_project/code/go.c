/*
 * go.c
 *
 *  Created on: 2025��12��26��
 *      Author: ycj01
 */

#include "go.h"
/******************************************************************************
* ��������     : go
* ����        : С��go������
* �������     : void?
* ���ز���     : void
******************************************************************************/
void go(void)
{
    memcpy(imgGray[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    nowThreshold = getOSTUThreshold();/*��򷨻�ȡ��ֵ*/

    Get_imgOSTU(); /*��ȡ��ֵ��ͼ��imgOSTU*/

    // Dashedline_Makeup(); ���첻Ҫ�������� 

    Stop_line = 0; //��̬ǰհ��ʼ��

    IF_L = IF;
    memset(mid_line, XM/2, sizeof(mid_line));
    
    // �������ä��״̬�����Ȳ���ͼ�����߼�������ԭ������ȴ�����������
    // (�ڴ˿ɽ��ɸ�������ƻ����������ߣ��� blind_turning Ϊ 1 ʱ��ִ��ͼ�����)
    if(is_blind_turning)
    {
        // ��Ȼ�Ѿ����������ǣ�������ͼ�����������ô��������Ҫ�ˣ�����Ĭ��ֵ���Խ��
        for(uint8 i = 0; i <= Deal_Top; i++)
        {
            mid_line[i] = XM / 2;
            imgOSTU[i][(uint8)mid_line[i]] = Control_line;
        }
        return; // ֱ�ӷ��أ��ѳ��ӽ���������
    }

    if(get_start_point()) //����ҵ����
    {
        search_l_r(start_point_l,start_center_y,start_point_r,start_center_y); //�����������߽�
        
        // =========================================================================
        // ��������״̬���ַ��� 
        // Ӧ�� Trigger 2.0 �� search_l_r �д���������
        // =========================================================================
        if (cur_state == STATE_CHECK_NODE)
        {
            uint8 is_true_node = Check_Node_Box(Y_trigger);
            if (is_true_node) 
            {
                cur_state = STATE_SMOOTH_OFFSET; // ��ڵ㣬����ƫ������
            } 
            else 
            {
                cur_state = STATE_FALSE_IGNORE;  // �ٸ��ţ���еȣ�������ǿ����ֱ
            }
        }

        // --- ״̬��ת���� ---
        if (cur_state == STATE_FALSE_IGNORE)
        {
            // Ӧ�Լٸ��ţ�ֱ�ӽ� trigger ���� Deal_Top ������ı�Ե����
            // ������ʽ����ǿ��д��߽磬��ֹ�������ƫ
            // Ϊ�˼򵥽�ѧ�����ǽ�����ֱ������
            for (int y = Y_trigger; y <= Deal_Top; y++) {
                imgOSTU[y][Deal_Left] = Left_line;
                imgOSTU[y][Deal_Right] = Right_line;
            }
            // ��ֱ��ָ� NORMAL ��������������
            cur_state = STATE_NORMAL; 
        }
        else if (cur_state == STATE_SMOOTH_OFFSET)
        {
            // Ӧ����ڵ㣺�鿴·�߹滮������ж�̬ƫ������
            uint8 target_dir = Path_Array[node_index]; // 0��ת 1��ת 2ֱ��
            if (target_dir == 0) {
                // ��ת������ǿ�п����ó�ͷƫ�����ڵ�����
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = Deal_Left + 5; 
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
            } else if (target_dir == 1) {
                // ��ת�����߿���
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = Deal_Right - 5; 
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
            } else {
                // ֱ�У�ȡ��
                for (int y = Y_trigger; y <= Deal_Top; y++) {
                    mid_line[y] = XM / 2;
                    imgOSTU[y][(uint8)mid_line[y]] = Control_line;
                }
            }
            
            // Ϊ���Ͻ���������äתǰ�ɼ���ȷ�ϣ���ƫ���ܵ�����ײ����У�
            // �������Ǵ��Զ�Ϊƫ��һ֡���߼�֡�����äת
            // Ϊ���Ͻ���������äתǰ�ɼ���ȷ��
            if (target_dir == 0) { Yaw_Target = 90.0f; } // ��ת
            else if (target_dir == 1) { Yaw_Target = -90.0f; } // ��ת
            else { Yaw_Target = 0.0f; } // ֱ��
            
            is_blind_turning = 1;
            cur_state = STATE_BLIND_TURN_YAW;
            return; // ������֡��������
        }

        if(cur_state == STATE_NORMAL)  //��̬Ѳ�����е�
        {
            Get_lost_tip(2); //��ȡ��Ե��ʧ����
            
            Get_start_center(); //��ȡ�������
            if(b_lost_num > 0 && t_lost_num) //����ֱ��
            {
               Stop_line = Get_top_straightline();
            }
            else if(b_lost_num > 0 && l_lost_num && !t_lost_num && !r_lost_num)
            {
                Stop_line = Left_curve_line(); //������
            }
            else if(b_lost_num > 0 && r_lost_num && !t_lost_num && !l_lost_num)
            {
                Stop_line = Right_curve_line(); //������
            }
            else
            {
                Stop_line = 0;
                IF = curve;
            }
        }




}




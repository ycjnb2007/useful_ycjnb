#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_headfile.h"

#define KEY1_PIN    P20_3 //上		//页切换
#define KEY2_PIN    P20_2 //下		//选泽
#define KEY3_PIN    P20_1 //加		//加
#define KEY4_PIN    P20_0 //减		//减

void mykey_init();
uint8 scanf_key();

extern uint8 clear_flag;

#endif
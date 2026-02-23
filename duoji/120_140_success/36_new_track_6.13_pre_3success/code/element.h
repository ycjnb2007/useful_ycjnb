#ifndef _element_h
#define _element_h


#define Buzzer          (P23_7)

#define col_start_line    2
#define col_last_line      MT9V03X_W - 3
#define row_start_line    2
#define row_last_line      CUT_HEIGHT - 2
#define many_count        12            //最多找？行/列
#define many_count1       5
#define point_count     11              //每行最多跳变点个数
#define MAX_DISTANCE    20              //跳变点的最大间距

typedef struct {
    uint8_t x;
    uint8_t y;
} Point;


extern Point line_points[MT9V03X_W + CUT_HEIGHT];       //储存拟合出直线的点
extern Point roundabout_line_points[MT9V03X_W + CUT_HEIGHT];

extern uint16 num_points;
extern uint16 roundabout_num_points;

extern Point up_row_count[point_count];
extern Point down_row_count[point_count];
extern Point left_col_count[point_count];
extern Point right_col_count[point_count];

extern uint8 left_col_num;
extern uint8 right_col_num;
extern uint8 up_row_num;
extern uint8 down_row_num;

extern uint8 left_col_num1;
extern uint8 down_row_num1;
extern uint8 up_row_num1;
extern uint8 right_col_num1;

////环岛处理
extern uint8 X_roundabout_start;
extern uint8 Y_roundabout_start;   
extern uint8 X_roundabout_end;   
extern uint8 Y_roundabout_end;  




//连接起始点和中点
uint16 Bresenham_line(Point *points, uint8 x0, uint8 y0, uint8 x1, uint8 y1);
//左侧列
uint8 Element_col_left();
//右侧列
uint8 Element_col_right();
//上方行
uint8 Element_row_up();
//下方行
uint8 Element_row_down(); 

uint8 Element_col_left1();
uint8 Element_row_down1(); 
uint8 Element_col_right1();
uint8 Element_row_up1();


//元素行和元素列
void row_col_element_find();
//动态权重数值
void generate_dynamic_weights(uint16 num_points, float out_weights[]);
//动态中点
float calculate_weighted_midpoint(Point line_points[], uint16 num_points);
#endif
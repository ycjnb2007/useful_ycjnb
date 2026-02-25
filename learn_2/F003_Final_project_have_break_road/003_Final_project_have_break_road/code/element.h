#ifndef _element_h
#define _element_h


#define Buzzer          (P23_7)

/*********************************修改图像范围*********************************/
#define col_start_line          LEFT_COL_LIMIT
#define col_last_line           RIGHT_COL_LIMIT
#define row_start_line          UP_ROW_LIMIT
#define row_last_line           DOWN_ROW_LIMIT
/*********************************修改图像范围*********************************/


/*********************************修改图寻找范围*********************************/
#define MAX_LINE_NUM            500             //拟合直线最大点数
#define many_count              13              //最多找多少行/列（带坐标点）
#define many_count1             13               //最多找多少行/列（单纯数跳变点）
#define point_count             13              //每行最多跳变点个数
#define MAX_DISTANCE            20              //跳变点的最大间距
/*********************************修改图寻找范围*********************************/



/*********************************拟合直线存储点数组声明*********************************/
extern float Mid_point;                                  //拟合直线权重中点
#define PEAK_CENTER             (0.4f)                  //画面权重值（范围0.2f到1.0f）
#define PEAK_CENTER_ROUNDABOUT  (0.4f)                  //环岛处权重值                        
extern float peak_center_use;
typedef struct 
{
    uint8_t x;
    uint8_t y;
} Point;

extern Point dotted_line_points[MAX_LINE_NUM];          //虚线情况下拟合直线
extern Point line_points[MAX_LINE_NUM];                 //普通寻迹下拟合直线
extern Point roundabout_line_points[MAX_LINE_NUM];      //圆环状态下拟合直线


extern uint16 dotted_num_points;                        //虚线情况下拟合直线点的个数
extern uint16 num_points;                               //普通寻迹下拟合直线点的个数
extern uint16 roundabout_num_points;                    //圆环状态下拟合直线点的个数

/*********************************拟合直线存储点数组声明*********************************/


/*********************************元素行和元素列坐标点及个数*********************************/

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
/*********************************元素行和元素列坐标点及个数*********************************/


/*********************************函数声明*********************************/

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
/*********************************函数声明*********************************/


#endif
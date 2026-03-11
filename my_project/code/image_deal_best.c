#include "image_deal_best.h"
// ==================== 全局变量定义 ====================
enum ImgFlag IF = straightlineS;   // 当前图像标志，初始为短直道
enum ImgFlag IF_L = straightlineS; // 上一帧图像标志
// ==================== 21届状态机与控制特有变量 ====================
RunState cur_state = STATE_NORMAL;
float Yaw_Start = 0;
float Yaw_Target = 0;
uint8 Path_Array[20] = {1, 0, 1, 2, 0, 0};
uint8 node_index = 0;
uint8 is_blind_turning = 0; // 核心状态机：当前工作状态
uint8 Y_trigger = 0;
// 丢失边界统计变量
uint8 t_lost_num = 0, l_lost_num = 0, r_lost_num = 0;
uint8 b_lost_num = 0;
uint8 t_lost_tip = 0, l_lost_tip = 0, r_lost_tip = 0, b_lost_tip = 0;
uint8 dis_Solidline = 0;
uint32 t_center[2] = {0};
uint32 b_center[5] = {0};
uint8 l_center[5] = {0};
uint8 r_center[5] = {0};
// 触发动态检测框的起始行
// 真节点路线规划数组：0表示左转，1表示右转，2表示直行
// （需要用户在外部主控制逻辑中根据发车点选择不同的数组，此处置为示范）
// ===============================================================
volatile RUN_Dir run_dir = Right; // 初始运行方向为右转
// 图像处理状态
// 图像阈值处理
uint8_t nowThreshold = 0;   // 当前阈值
int minGrayscale = 40;      // 最小灰度限制
int maxGrayscale = 160;     // 最大灰度限制
int minGray = 0;            // 图像最小灰度值
int maxGray = 0;            // 图像最大灰度值
uint8_t minThreshold = 70;  // 最小阈值限制
uint8_t maxThreshold = 160; // 最大阈值限制
uint8 img_threshold_group[3];
uint16_t histogram[256];   // 灰度直方图
uint8 close_Threshold = 0; // 近景阈值补偿 5-15
uint8 mid_Threshold = 0;   // 中景阈值补偿 5-15
uint8 far_Threshold = 0;   // 远景阈值补偿 10-25
// 图像存储数组
uint8 imgGray[IMG_H][IMG_W];   // 原始灰度图像
uint8 imgTran[YM][XM];         // 临时过渡图像
uint8 imgOSTU[YM][XM];         // 二值化处理图像
uint8 white_num_row[YM];       // 每行白点数量统计
uint8 white_num_col[XM];       // 每列白点数量统计
uint8 mid_line[YM] = {XM / 2}; // 中线数组，初始为中间
// 起点终点坐标
uint8 start_point_l = 0;       // 左起点
uint8 start_point_r = XX;      // 右起点
uint8 start_center_y = 0;      // 起点行坐标
uint8 start_center_x = XM / 2; // 起点列坐标
uint8 end_center_x = XM / 2;   // 终点列坐标
uint8 end_center_y = Deal_Top; // 终点行坐标
// 边界丢失统计
// 处理区域尺寸
int Deal_W = XX; // 处理宽度
// 时间统计
// 行驶长度
// 几何校正表（像素到实际距离转换）
// =========================================================================================
// 【新手测宽必读: white_width 的测量方法】
// 这组数组代表：在摄像头的第 i
// 行，赛道的一侧边界到另一侧边界总共占了几个“图片像素点”。 怎么测：
// 1. 把车模放在直道正中央，打开你的 TFT180 屏幕，让它显示 imgOSTU
// (二值化图像)。
// 2. 取一根直尺。不要动车！
// 3.
// 从屏幕最底下（第0行）开始看，数一下整个白色区域（赛道宽度）占据了多宽的点数。填入
// white_width[0]。
// 4. 往上每走大概 5 行或者 10
// 行，重新数一下赛道宽度像素占多少，填入对应的数组元素中。
// 5. 注意因为透视原理，越往上，赛道在屏幕里的像素宽度就越窄！
// 6.
// 这个数组极其重要！它决定了打断条件(Trigger 2.0)是否能精准捕捉到扩宽的路口！！！
// =========================================================================================
float white_width[YM] = {17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 15,
                         15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13,
                         13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10,
                         10, 10, 10, 10, 10, 10, 10, 9,  9,  9,  9,  9,  9,  8,
                         8,  8,  8,  8,  7,  7,  7,  7,  6,  6,  6,  6,  5,  5};
// =========================================================================================
// 【新手测距必读: Length_5cm 的测量方法】
// 这组数组代表：在摄像头的第 i 行往前看，真实世界里的 "5 厘米"
// 究竟能在屏幕里向上延伸几个“像素行”。 怎么测：
// 1. 车身放平稳，车头前方地上用黑胶带每隔 5cm 贴一条横线。
// 2. 看屏幕：车头保险杠所对应的第一条线是在第几行？记录为起始 Y0。
// 3. 往上看第二条 5cm 线，记录在屏幕的第几行 Y1。
// 4. 那么对于屏幕的这段区域（Y0到Y1），真实 5cm 占据了 (Y1 - Y0)
// 行。把这个差值填进数组这段里。
// 5. 由近及远（由屏幕底下到上面）继续测量。越往远看，5cm 在屏幕里占的行数越少！
// 6. 填好后，Check_Node_Box 的 15cm 向前看距离才会是准的！
// =========================================================================================
float Length_5cm[YM] = {30, 29, 28, 28, 27, 27, 26, 26, 25, 25, 25, 25, 25, 25,
                        24, 24, 24, 23, 23, 23, 22, 22, 22, 22, 21, 21, 21, 20,
                        20, 20, 19, 19, 19, 18, 18, 18, 17, 17, 16, 16, 15, 15,
                        15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 11, 11, 11, 10,
                        10, 10, 9,  9,  9,  8,  8,  8,  8,  7,  7,  7,  7,  7};
const uint8_t Step_15cm[YM] = {
    61, 60, 59, 58, 57, 57, 56, 55, 54, 54, 54, 53, 53, 52, 51, 50, 49, 48,
    48, 48, 47, 46, 45, 45, 44, 43, 43, 42, 41, 40, 39, 38, 38, 37, 37, 37,
    36, 35, 34, 33, 32, 32, 32, 31, 30, 30, 29, 22, 21, 20, 19, 19, 19, 19,
    18, 17, 17, 17, 16, 16, 9,  8,  8,  8,  8,  7,  7,  7,  7,  0};

float k1[YM] = {25 / 17, 25 / 17, 25 / 17, 25 / 17, 25 / 17, 25 / 16, 25 / 16,
                25 / 16, 25 / 16, 25 / 16, 25 / 16, 25 / 16, 25 / 16, 25 / 15,
                25 / 15, 25 / 15, 25 / 15, 25 / 14, 25 / 14, 25 / 14, 25 / 14,
                25 / 14, 25 / 14, 25 / 13, 25 / 13, 25 / 13, 25 / 13, 25 / 13,
                25 / 13, 25 / 13, 25 / 12, 25 / 12, 25 / 12, 25 / 12, 25 / 12,
                25 / 11, 25 / 11, 25 / 11, 25 / 11, 25 / 10, 25 / 10, 25 / 10,
                25 / 10, 25 / 10, 25 / 10, 25 / 10, 25 / 10, 25 / 10, 25 / 10,
                25 / 9,  25 / 9,  25 / 9,  25 / 9,  25 / 9,  25 / 9,  25 / 8,
                25 / 8,  25 / 8,  25 / 8,  25 / 8,  25 / 7,  25 / 7,  25 / 7,
                25 / 7,  25 / 6,  25 / 6,  25 / 6,  25 / 6,  25 / 5,  25 / 5};
float k2[YM] = {0.00,  0.28,  0.57,  0.85,  1.14,  1.45,  1.71,  1.99,  2.28,
                2.56,  2.85,  3.13,  3.42,  3.70,  3.99,  4.27,  4.56,  4.84,
                5.13,  5.41,  5.70,  5.98,  6.27,  6.55,  6.84,  7.12,  7.41,
                7.69,  7.98,  8.26,  8.55,  8.83,  9.12,  9.40,  9.69,  9.97,
                10.26, 10.54, 10.83, 11.11, 11.28, 11.57, 11.85, 12.14, 12.43,
                12.71, 13.00, 13.28, 13.57, 13.86, 14.14, 14.43, 14.71, 15.00,
                15.29, 15.57, 15.86, 16.14, 16.43, 16.72, 17.00, 17.29, 17.57,
                17.86, 18.15, 18.43, 18.72, 19.01, 19.29, 20.00};
// ==================== 八邻域变量区域 ====================
int points_l[USE_num][2] = {{0}}; // 左边界点坐标数组
int points_r[USE_num][2] = {{0}}; // 右边界点坐标数组
float dir_r[USE_num] = {0};       // 右边生长方向
float dir_l[USE_num] = {0};       // 左边生长方向
uint32 l_data_statics = 0;        // 左边界点统计
uint32 r_data_statics = 0;        // 右边界点统计
uint8 hightest = 0;               // 巡线最高点
uint16 leftmost[2] = {0};         // 最左点记录
uint16 rightmost[2] = {0};        // 最右点记录
uint16 topmost[2] = {0};          // 最上点记录
// 左边八邻域变量
int center_point_l[2] = {0};        // 八邻域中心坐标
uint8 search_filds_l[8][2] = {{0}}; // 八邻域坐标
uint8 index_l = 0;                  // 索引下标
uint8 temp_l[8][2] = {{0}};         // 临时存储
int8_t seeds_l[8][2] = {{0, -1}, {-1, -1}, {-1, 0}, {-1, 1},
                        {0, 1},  {1, 1},   {1, 0},  {1, -1}}; // 左边生长种子
// 右边八邻域变量
int center_point_r[2] = {0};        // 八邻域中心坐标
uint8 search_filds_r[8][2] = {{0}}; // 八邻域坐标
uint8 index_r = 0;                  // 索引下标
uint8 temp_r[8][2] = {{0}};         // 临时存储
int8_t seeds_r[8][2] = {{0, -1}, {1, -1}, {1, 0},  {1, 1},
                        {0, 1},  {-1, 1}, {-1, 0}, {-1, -1}}; // 右边生长种子
// ==================== 函数实现 ====================
/******************************************************************************
 * 函数名称     : standard
 * 描述         : 图像处理初始化标准化
 * 进入参数     : void
 * 返回参数     : void
 ******************************************************************************/
void standard(void) {
  Deal_W = XX;
  // disappear_flag = 0;
  // disappear_num = 0;
  // disappear_total = 3;//待定
  // elements_index = 0;
}
/******************************************************************************
 * 函数名称     : image_copy
 * 描述         : 图像复制
 * 进入参数     : void
 * 返回参数     : void
 ******************************************************************************/
void image_copy() {
  memset(imgGray, 0, sizeof(imgGray));
  for (int i = 0; i < MT9V03X_H; i++) {
    for (int j = 0; j < MT9V03X_W; j++) {
      imgGray[i][j] = mt9v03x_image[i][j];
    }
  }
}
/******************************************************************************
 * 函数名称     : my_abs
 * 描述         : 取绝对值函数
 * 进入参数     : value
 * 返回参数     : 绝对值
 ******************************************************************************/
int my_abs(int value) {
  if (value >= 0)
    return value;
  else
    return -value;
}

/******************************************************************************
 * 函数名称     : getOSTUThreshold
 * 描述         : 大津法阈值计算
 * 进入参数     : void
 * 返回参数     : nowThreshold 大津法阈值
 ******************************************************************************/
/******************************************************************************
 * 函数名称     : getGrayscaleHistogram
 * 描述         : 灰度直方图统计（只扫ROI中心XM列）
 * 进入参数     : void
 * 返回参数     : void
 ******************************************************************************/
void getGrayscaleHistogram(void) {
  memset(histogram, 0, sizeof(histogram));
  minGray = 255;
  maxGray = 0;

  int start_y = IMG_H - YM;
  int end_y = IMG_H - 1;
  int start_x = (IMG_W - XM) / 2;
  int end_x = start_x + XM - 1;

  // 老老实实按行、按列双重遍历，跳过无用的 Margin 区域
  for (int y = start_y; y <= end_y; y++) {
    for (int x = start_x; x <= end_x; x++) {
      uint8_t pixel_val = imgGray[y][x];

      if (pixel_val < minGray)
        minGray = pixel_val;
      if (pixel_val > maxGray)
        maxGray = pixel_val;

      histogram[pixel_val]++;
    }
  }
}

uint8 getOSTUThreshold(void) {
  getGrayscaleHistogram();
  uint32_t sum = 0, valueSum = 0;
  uint64_t sigma = 0, maxSigma = 0;
  float w1 = 0, w2 = 0;
  int32_t u1 = 0, u2 = 0;
  uint8 min = 0, max = 255;
  min = minGray;
  max = maxGray;
  if (max < minThreshold)
    return minThreshold;
  if (min > maxThreshold)
    return maxThreshold;
  min = min < minGrayscale ? minGrayscale : min;
  max = max > maxGrayscale ? maxGrayscale : max;
  uint32_t lowSum[256] = {0};
  uint32_t lowValueSum[256] = {0};
  for (uint16_t i = min; i <= max; ++i) {
    sum += histogram[i];
    valueSum += histogram[i] * i;
    lowSum[i] = sum;
    lowValueSum[i] = valueSum;
  }
  for (uint16_t i = min; i < max + 1; ++i) {
    w1 = (float)lowSum[i] / sum;
    w2 = 1 - w1;
    u1 = (int32_t)(lowValueSum[i] / w1);
    u2 = (int32_t)((valueSum - lowValueSum[i]) / w2);
    sigma = (uint64_t)(w1 * w2 * (u1 - u2) * (u1 - u2));
    if (sigma >= maxSigma) {
      maxSigma = sigma;
      nowThreshold = i;
    } else {
      break;
    }
  }
  static uint8_t last_Threshold = 120; // 记录上一帧阈值
  nowThreshold = nowThreshold < minThreshold ? minThreshold : nowThreshold;
  nowThreshold = nowThreshold > maxThreshold ? maxThreshold : nowThreshold;
  nowThreshold = (uint8_t)(0.7f * nowThreshold + 0.3f * last_Threshold);
  last_Threshold = nowThreshold;
  return nowThreshold;
}
/******************************************************************************
 * 函数名称     : Get_imgOSTU
 * 描述         : 大津法二值化处理图像
 * 进入参数     : void
 * 返回参数     : void
 ******************************************************************************/
void Get_imgOSTU(void) {
  memset(imgOSTU, Black, sizeof(imgOSTU));
  memset(white_num_row, 0, sizeof(white_num_row));
  memset(white_num_col, 0, sizeof(white_num_col));
  img_threshold_group[0] = nowThreshold + close_Threshold; // 近景
  img_threshold_group[1] = nowThreshold - mid_Threshold;   // 中景
  img_threshold_group[2] = nowThreshold - far_Threshold;   // 远景
  int k = 0;
  int Loop[9][2] = {{0, 1},  {1, 1},   {1, 0},  {1, -1},
                    {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
  for (int i = 0; i <= YY; i++) // 腐蚀去噪
  {
    if (i == YY / 3) {
      k = 1;
    } else if (i == YY * 2 / 3) {
      k = 2;
    }
    for (int j = 0; j <= XX; j++) {
      if (imgGray[IMG_H - 1 - i][(IMG_W - XM) / 2 + j] >=
          img_threshold_group[k]) {
        int White_num = 0;
        int Black_num = 0;
        for (int m = 0; m <= 8; m++) {
          int x = j + Loop[m][0];
          int y = i + Loop[m][1];
          if (x < 0 || x > XX || y < 0 || y > YY) {
            White_num++;
          } else if (imgOSTU[y][x] == White) {
            White_num = 8;
            break;
          } else if (imgGray[IMG_H - 1 - y][(IMG_W - XM) / 2 + x] >=
                     nowThreshold) {
            White_num++;
          } else {
            Black_num++;
          }
          if (Black_num >= 2) {
            break;
          }
        }
        if (White_num >= 8) {
          imgOSTU[i][j] = White;
          white_num_row[i]++;
          white_num_col[j]++;
          for (int m = 0; m <= 8; m++) {
            int x = j + Loop[m][0];
            int y = i + Loop[m][1];
            if (x < 0 || x > XX || y < 0 || y > YY) {
              continue;
            } else if (imgOSTU[y][x] != White) {
              imgOSTU[y][x] = White;
              white_num_row[y]++;
              white_num_col[x]++;
            }
          }
        }
      }
    }
  }
  // [反光暴力消除]
  // 检查两侧纵向列是否有大面积反光发白带，若有则强制涂黑其以及周围外扩区域，防止爬线时被干扰跑偏
  uint8 white_row_max[2] = {0};
  uint8 white_col_max[2] = {0};
  for (uint8 i = Deal_Left; i <= Deal_Right; i++) {
    if (white_num_col[i] > white_col_max[0]) {
      white_col_max[0] = white_num_col[i];
    }
  }
  // 如果发白列异常多，且并不是全部屏幕白茫茫，说明是侧边大面积反光膜
  if (white_col_max[0] >= YY && white_row_max[0] < white_width[0]) {
    // 绘制左右及上下四周黑框隔离区
    for (uint8 x = 0; x <= XX; x++) {
      if (imgOSTU[Deal_Bottom - 1][x] == White ||
          imgOSTU[Deal_Bottom - 2][x] == White)
        imgOSTU[Deal_Bottom][x] = White;
      imgOSTU[Deal_Bottom - 1][x] = Black;
      imgOSTU[Deal_Bottom - 2][x] = Black;
      if (imgOSTU[Deal_Top + 1][x] == White ||
          imgOSTU[Deal_Top + 2][x] == White)
        imgOSTU[Deal_Top][x] = White;
      imgOSTU[Deal_Top + 1][x] = Black;
      imgOSTU[Deal_Top + 2][x] = Black;
    }
    for (uint8 y = 0; y <= Deal_Top; y++) {
      if (imgOSTU[y][Deal_Left - 1] == White ||
          imgOSTU[y][Deal_Left - 2] == White)
        imgOSTU[y][Deal_Left] = White;
      imgOSTU[y][Deal_Left - 1] = Black;
      imgOSTU[y][Deal_Left - 2] = Black;
      if (imgOSTU[y][Deal_Right + 1] == White ||
          imgOSTU[y][Deal_Right + 2] == White)
        imgOSTU[y][Deal_Right] = White;
      imgOSTU[y][Deal_Right + 1] = Black;
      imgOSTU[y][Deal_Right + 2] = Black;
    }
  }
}
/******************************************************************************
 * 函数名称     : get_start_point
 * 描述         : 寻找左右边界起点
 * 进入参数     : void
 * 返回参数     : 1找到左右边界，0未找到左右边界
 ******************************************************************************/
uint8 get_start_point(void) {
  b_lost_tip = 0;
  start_center_y = 0;
  uint8 l_found = 0, r_found = 0;
  uint32 l_point = 0, r_point = 0;
  for (uint8 i = Deal_Bottom; i <= Deal_Bottom + 30; i++) {
    if (white_num_row[i] >= 5 && white_num_row[i + 1] >= 5) {
      start_center_y = i;
      // 从左向右找黑白跳变点
      for (uint8 j = 0; j < XX - 1; j++) {
        if ((imgOSTU[start_center_y][j + 1] == Make_up ||
             imgOSTU[start_center_y][j + 1] == White) &&
            imgOSTU[start_center_y][j] == Black) {
          l_point = j;
          l_found = 1;
          break;
        }
      }
      // 从右向左找黑白跳变点
      for (uint8 j = XX; j > 0 + 1; j--) {
        if ((imgOSTU[start_center_y][j - 1] == Make_up ||
             imgOSTU[start_center_y][j - 1] == White) &&
            imgOSTU[start_center_y][j] == Black) {
          r_point = j;
          r_found = 1;
          break;
        }
      }
      if (r_found && l_found) {
        break;
      }
    }
  }
  if (l_found && r_found) {
    // 记录b_lost_tip
    for (uint8 i = l_point; i <= r_point; i++) {
      if (imgOSTU[start_center_y][i] == White) {
        b_lost_tip++;
        imgOSTU[Lost_Bottom][i] = Lost_line;
      } else if (imgOSTU[start_center_y + 1][i] == White) {
        b_lost_tip++;
        imgOSTU[Lost_Bottom][i] = Lost_line;
      } else if (imgOSTU[Deal_Bottom][i] == White) {
        b_lost_tip++;
        imgOSTU[Lost_Bottom][i] = Lost_line;
      }
    }
    // 起点位置记录
    start_center_x = (l_point + r_point) / 2;
    start_point_l = l_point;
    start_point_r = r_point;
    return 1;
  } else {
    return 0;
  }
}
/******************************************************************************
 * 函数名称     : search_l_r
 * 描述         : 八邻域探索边界线
 * 进入参数     : start_l_x,start_l_y 左八邻域开始坐标
 * start_r_x,start_r_y 右八邻域开始坐标
 * 返回参数     : void
 ******************************************************************************/
uint8 search_l_r(uint8 start_l_x, uint8 start_l_y, uint8 start_r_x,
                 uint8 start_r_y) {
  hightest = 0;
  leftmost[0] = Deal_Right;
  rightmost[0] = Deal_Left;
  topmost[1] = 0;
  l_lost_tip = 0;
  r_lost_tip = 0;
  t_lost_tip = 0;
  l_data_statics = 0;
  r_data_statics = 0;
  center_point_l[0] = start_l_x;
  center_point_l[1] = start_l_y;
  center_point_r[0] = start_r_x;
  center_point_r[1] = start_r_y;
  points_l[l_data_statics][0] = center_point_l[0];
  points_l[l_data_statics][1] = center_point_l[1];
  points_r[r_data_statics][0] = center_point_r[0];
  points_r[r_data_statics][1] = center_point_r[1];
  uint32 break_flag = USE_num;
  while (break_flag--) {
    // 左边8邻域初始化
    for (uint8 i = 0; i < 8; i++) {
      search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
      search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
    }
    // 右边8邻域初始化
    for (uint8 i = 0; i < 8; i++) {
      search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
      search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
    }
    // 左边生长判断
    l_data_statics++;
    index_l = 0;
    for (uint8 i = 0; i < 8; i++) {
      temp_l[i][0] = 0;
      temp_l[i][1] = 0;
    }
    for (uint8 i = 0; i < 8; i++) {
      if ((imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Black ||
           imgOSTU[search_filds_l[i][1]][search_filds_l[i][0]] == Lost_line) &&
          (imgOSTU[search_filds_l[(i + 1) & 7][1]]
                  [search_filds_l[(i + 1) & 7][0]] != Black &&
           imgOSTU[search_filds_l[(i + 1) & 7][1]]
                  [search_filds_l[(i + 1) & 7][0]] != Lost_line)) {
        temp_l[index_l][0] = search_filds_l[(i)][0];
        temp_l[index_l][1] = search_filds_l[(i)][1];
        index_l++;
        dir_l[l_data_statics - 1] = i;
      }
      if (index_l) {
        center_point_l[0] = temp_l[0][0];
        center_point_l[1] = temp_l[0][1];
        for (uint8 j = 0; j < index_l; j++) {
          if (center_point_l[1] < temp_l[j][1]) {
            center_point_l[0] = temp_l[j][0];
            center_point_l[1] = temp_l[j][1];
            dir_l[l_data_statics - 1] = i;
          }
        }
      }
      // 丢失点记录
      if (center_point_l[0] == Deal_Left - 1 &&
          imgOSTU[center_point_l[1]][Deal_Left] == White &&
          imgOSTU[center_point_l[1]][Deal_Left - 2] != Lost_line) {
        l_lost_tip++;
        imgOSTU[center_point_l[1]][Lost_Left] = Lost_line;
      } else if (center_point_l[0] == Deal_Right + 1 &&
                 imgOSTU[center_point_l[1]][Deal_Right] == White &&
                 imgOSTU[center_point_l[1]][Lost_Right] != Lost_line) {
        r_lost_tip++;
        imgOSTU[center_point_l[1]][Lost_Right] = Lost_line;
      } else if (center_point_l[1] == Deal_Top + 1 &&
                 imgOSTU[Deal_Top][center_point_l[0]] == White &&
                 imgOSTU[Deal_Top + 2][center_point_l[0]] != Lost_line) {
        t_lost_tip++;
        imgOSTU[Lost_Top][center_point_l[0]] = Lost_line;
      }
      // 更新最高点
      if (center_point_l[1] > hightest) {
        hightest = center_point_l[1];
        topmost[0] = center_point_l[0];
        topmost[1] = center_point_l[1];
      }
      // 更新最左点
      if (center_point_l[0] < leftmost[0]) {
        leftmost[0] = center_point_l[0];
        leftmost[1] = center_point_l[1];
      }
    }
    // 右边生长判断
    r_data_statics++;
    index_r = 0;
    for (uint8 i = 0; i < 8; i++) {
      temp_r[i][0] = 0;
      temp_r[i][1] = 0;
    }
    for (uint8 i = 0; i < 8; i++) {
      if ((imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Black ||
           imgOSTU[search_filds_r[i][1]][search_filds_r[i][0]] == Lost_line) &&
          (imgOSTU[search_filds_r[(i + 1) & 7][1]]
                  [search_filds_r[(i + 1) & 7][0]] != Black &&
           imgOSTU[search_filds_r[(i + 1) & 7][1]]
                  [search_filds_r[(i + 1) & 7][0]] != Lost_line)) {
        temp_r[index_r][0] = search_filds_r[(i)][0];
        temp_r[index_r][1] = search_filds_r[(i)][1];
        index_r++;
        dir_r[r_data_statics - 1] = i;
      }
      if (index_r) {
        center_point_r[0] = temp_r[0][0];
        center_point_r[1] = temp_r[0][1];
        for (uint8 j = 0; j < index_r; j++) {
          if (center_point_r[1] < temp_r[j][1]) {
            center_point_r[0] = temp_r[j][0];
            center_point_r[1] = temp_r[j][1];
            dir_r[r_data_statics - 1] = i;
          }
        }
      }
      // 丢失点记录
      if (center_point_r[0] == Deal_Right + 1 &&
          imgOSTU[center_point_r[1]][Deal_Right] == White &&
          imgOSTU[center_point_r[1]][Lost_Right] != Lost_line) {
        r_lost_tip++;
        imgOSTU[center_point_r[1]][Lost_Right] = Lost_line;
      } else if (center_point_r[0] == Deal_Left - 1 &&
                 imgOSTU[center_point_r[1]][Deal_Left] == White &&
                 imgOSTU[center_point_r[1]][Lost_Left] != Lost_line) {
        l_lost_tip++;
        imgOSTU[center_point_r[1]][Lost_Left] = Lost_line;
      } else if (center_point_r[1] == Deal_Top + 1 &&
                 imgOSTU[Deal_Top][center_point_r[0]] == White &&
                 imgOSTU[Lost_Top][center_point_r[0]] != Lost_line) {
        t_lost_tip++;
        imgOSTU[Lost_Top][center_point_r[0]] = Lost_line;
      }
      // 更新最高点
      if (center_point_r[1] > hightest) {
        hightest = center_point_r[1];
        topmost[0] = center_point_r[0];
        topmost[1] = center_point_r[1];
      }
      // 更新最右点
      if (center_point_r[0] > rightmost[0]) {
        rightmost[0] = center_point_r[0];
        rightmost[1] = center_point_r[1];
      }
    }
    // 清除顶部丢失线
    if (hightest == Deal_Top - 1) {
      for (uint8 i = Deal_Left; i <= Deal_Right; i++) {
        imgOSTU[Lost_Top][i] = Black;
        imgOSTU[Lost_Top - 1][i] = Black;
      }
    }
    // 储存边界数组
    points_l[l_data_statics][0] = center_point_l[0];
    points_l[l_data_statics][1] = center_point_l[1];
    points_r[r_data_statics][0] = center_point_r[0];
    points_r[r_data_statics][1] = center_point_r[1];
    // 退出条件判断
    if ((points_r[r_data_statics][0] == points_r[r_data_statics - 1][0] &&
         points_r[r_data_statics][0] == points_r[r_data_statics - 2][0] &&
         points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] &&
         points_r[r_data_statics][1] == points_r[r_data_statics - 2][1]) ||
        (points_l[l_data_statics - 1][0] == points_l[l_data_statics - 2][0] &&
         points_l[l_data_statics - 1][0] == points_l[l_data_statics - 3][0] &&
         points_l[l_data_statics - 1][1] == points_l[l_data_statics - 2][1] &&
         points_l[l_data_statics - 1][1] == points_l[l_data_statics - 3][1])) {
      break;
    }
    if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics][0]) <=
            1 &&
        my_abs(points_r[r_data_statics][1] - points_l[l_data_statics][1]) <=
            1) {
      break;
    }
    if ((l_data_statics > YY / 2 &&
         my_abs(start_l_x - points_l[l_data_statics][0]) <= 1 &&
         my_abs(start_l_y - points_l[l_data_statics][1]) <= 1) ||
        (r_data_statics > YY / 2 &&
         my_abs(start_r_x - points_r[r_data_statics][0]) <= 1 &&
         my_abs(start_r_y - points_r[r_data_statics][1]) <= 1)) {
      break; // 绕了一圈回来了
    }
    // =========================================================================
    // 【新增：Trigger 2.5 动态防抖与发散验证】
    // =========================================================================
    uint8 cur_y = hightest; // 用当前爬到的最高点作为基准行
    int current_width =
        points_r[r_data_statics][0] - points_l[l_data_statics][0];

    static int width_jump_cnt = 0; // 改成有符号整型，方便倒扣分

    if (current_width > (white_width[cur_y] * 1.5)) {
      // 斜率发散校验：左边界向左生长，右边界向右生长
      int left_diff =
          points_l[l_data_statics][0] - points_l[l_data_statics - 1][0];
      int right_diff =
          points_r[r_data_statics][0] - points_r[r_data_statics - 1][0];

      if (left_diff < 0 && right_diff > 0) {
        width_jump_cnt += 2; // 真实路口，双向发散，权重增加
      } else {
        width_jump_cnt += 1; // 单侧反光噪点或单侧岔路
      }
    } else {
      // 连续性惩罚：如果突然变窄，不直接清零，而是倒扣分，过滤高频噪点！
      width_jump_cnt -= 2;
      if (width_jump_cnt < 0)
        width_jump_cnt = 0;
    }

    // 触发条件 2：物理断崖式死胡同
    uint8 trigger_broken = (l_data_statics == r_data_statics &&
                            imgOSTU[cur_y + 1][XM / 2] == Black)
                               ? 1
                               : 0;

    // 阈值提高到 4
    if ((width_jump_cnt >= 4 || trigger_broken) && cur_y > Deal_Bottom + 5) {
      Y_trigger =
          (width_jump_cnt >= 4) ? (cur_y - 2) : cur_y; // 回退到真正突变的那一行
      cur_state = STATE_CAPACITY_CHECK;
      width_jump_cnt = 0;
      break;
    }
  }

  // 绘制左右边线
  for (int i = l_data_statics; i > 0; i--) {
    imgOSTU[points_l[i][1]][points_l[i][0]] = Left_line;
  }
  for (int i = r_data_statics; i > 0; i--) {
    imgOSTU[points_r[i][1]][points_r[i][0]] = Right_line;
  }
  return 0;
}

/******************************************************************************
 * 函数名称     : Get_start_center
 * 描述         : 获取中线起点
 ******************************************************************************/
void Get_start_center(void) {
  if (b_lost_tip >= white_width[Deal_Bottom] + 5 && b_lost_num == 1) {
    uint32 max = 0, min = Deal_Top, mid = Deal_Top / 2;
    uint8 white_heigh[XM] = {0};
    for (uint8 i = start_point_l; i <= start_point_r; i++) {
      if (imgOSTU[Lost_Bottom][i] == Lost_line) {
        for (uint8 j = Deal_Bottom; j <= Deal_Top + 1; j++) {
          if (imgOSTU[j][i] == Black || imgOSTU[j][i] == Left_line ||
              imgOSTU[j][i] == Right_line) {
            white_heigh[i] = j;
            max = max > j ? max : j;
            min = min < j ? min : j;
            break;
          }
        }
      }
    }
    if (b_lost_tip >= 2 * white_width[Deal_Bottom]) {
      mid = (max - min) * 2 / 3 + min;
    } else {
      mid = (max + min) / 2;
    }
    for (uint8 i = start_point_l; i <= start_point_r; i++) {
      if (imgOSTU[Lost_Bottom][i] == Lost_line && white_heigh[i] < mid) {
        b_lost_tip--;
        imgOSTU[Lost_Bottom][i] = Black;
      }
    }
  } else if (b_lost_num == 2) {
    uint8 l_point = 0, r_point = XM;
    float l_num = 0, r_num = 0;
    float l_sum = 0, r_sum = 0;
    float l_average = 0, r_average = 0;
    for (uint8 i = start_point_l; i <= start_point_r; i++) {
      if (imgOSTU[Lost_Bottom][i] == Lost_line) {
        l_point = i;
        for (uint8 j = l_point; j <= Deal_Right; j++) {
          if (imgOSTU[Lost_Bottom][j] == Lost_line) {
            for (uint8 k = Deal_Bottom + 1; k <= Deal_Top + 1; k++) {
              if (imgOSTU[k][j] == Left_line || imgOSTU[k][j] == Right_line) {
                l_num++;
                l_sum = l_sum + k;
                break;
              }
            }
          } else if (imgOSTU[Lost_Bottom][j] == Black) {
            break;
          }
        }
        break;
      }
    }
    for (uint8 i = start_point_r; i >= start_point_l; i--) {
      if (imgOSTU[Lost_Bottom][i] == Lost_line) {
        r_point = i;
        for (uint8 j = i; j >= Deal_Left; j--) {
          if (imgOSTU[Lost_Bottom][j] == Lost_line) {
            for (uint8 k = Deal_Bottom; k <= Deal_Top + 1; k++) {
              if (imgOSTU[k][j] == Black || imgOSTU[k][j] == Left_line ||
                  imgOSTU[k][j] == Right_line) {
                r_num++;
                r_sum = r_sum + k;
                break;
              }
            }
          } else if (imgOSTU[Lost_Bottom][j] == Black) {
            break;
          }
        }
        break;
      }
    }
    l_average = l_sum / l_num;
    r_average = r_sum / r_num;
    if (l_average >= r_average) {
      for (uint8 i = r_point; i >= l_point; i--) {
        if (imgOSTU[Lost_Bottom][i] == Lost_line) {
          imgOSTU[Lost_Bottom][i] = Black;
        } else {
          break;
        }
      }
    } else {
      for (uint8 i = l_point; i <= r_point; i++) {
        if (imgOSTU[Lost_Bottom][i] == Lost_line) {
          imgOSTU[Lost_Bottom][i] = Black;
        } else {
          break;
        }
      }
    }
    b_lost_num = 1;
  }
  uint32 l_point = 0, r_point = 0;
  for (uint8 i = start_point_l; i < start_point_r; i++) {
    if (imgOSTU[Lost_Bottom][i] == Lost_line) {
      l_point = i;
      break;
    }
  }
  for (uint8 i = start_point_r; i > start_point_l; i--) {
    if (imgOSTU[Lost_Bottom][i] == Lost_line) {
      r_point = i;
      break;
    }
  }
  start_center_x = (l_point + r_point) / 2;
}
/******************************************************************************
 * 函数名称     : Get_top_straightline
 * 描述         : 绘制顶端直线
 * 返回参数     : 终点行坐标
 ******************************************************************************/
uint8 Get_top_straightline(void) {
  uint8 Black_num = 0;
  if (t_lost_num == 1) {
    if ((t_lost_tip >= white_width[Deal_Top] + 5 && t_lost_num == 1) ||
        (t_lost_tip >= 3 * white_width[Deal_Top])) {
      uint32 max = 0, min = Deal_Top, mid = Deal_Top / 2;
      uint8 white_low[XM] = {0};
      for (uint8 i = Deal_Left; i <= Deal_Right; i++) {
        if (imgOSTU[Lost_Top][i] == Lost_line) {
          for (uint8 j = Deal_Top; j >= 0; j--) {
            if (imgOSTU[j][i] != White) {
              white_low[i] = j;
              max = max > j ? max : j;
              min = min < j ? min : j;
              break;
            }
          }
        }
      }
      mid = (max + min) / 2;
      for (uint8 i = Deal_Left; i <= Deal_Right; i++) {
        if (imgOSTU[Lost_Top][i] == Lost_line && white_low[i] > mid) {
          t_lost_tip--;
          imgOSTU[Deal_Top + 2][i] = Black;
        }
      }
      uint32 l_point = 0, r_point = 0;
      end_center_y = Deal_Top;
      for (uint8 i = Deal_Left; i <= Deal_Right; i++) {
        if (imgOSTU[Lost_Top][i] == Lost_line) {
          l_point = i;
          break;
        }
      }
      for (uint8 i = Deal_Right; i >= Deal_Left; i--) {
        if (imgOSTU[Lost_Top][i] == Lost_line) {
          r_point = i;
          break;
        }
      }
      end_center_x = (l_point + r_point) / 2;
    } else {
      end_center_y = Deal_Top;
      end_center_x = t_center[0];
    }
    if (end_center_y <= start_center_y) {
      return 0;
    }
    float k = ((float)end_center_x - (float)start_center_x) /
              ((float)end_center_y - (float)start_center_y);
    for (uint8 i = start_center_y; i <= end_center_y; ++i) {
      mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
      if (imgOSTU[i][(uint8)mid_line[i]] != White) {
        Black_num++;
      }
      imgOSTU[i][(uint8)mid_line[i]] = Control_line;
    }
  } else if (t_lost_num == 2) {
    end_center_y = Deal_Top;
    uint32 t_center_l = t_center[0], t_center_r = t_center[1];
    uint8 left_line[YM] = {XM / 2};
    uint8 right_line[YM] = {XM / 2};
    uint8 Black_num_l = 0, Black_num_r = 0;
    float kl = ((float)t_center_l - (float)start_center_x) /
               ((float)end_center_y - (float)start_center_y);
    for (uint8 i = start_center_y; i <= end_center_y; ++i) {
      left_line[i] = (uint8)(start_center_x + kl * (i - start_center_y));
      if (imgOSTU[i][left_line[i]] != White) {
        Black_num_l++;
      } else {
        imgOSTU[i][left_line[i]] = Judge_line;
      }
    }
    float kr = (float)((float)t_center_r - (float)start_center_x) /
               ((float)end_center_y - (float)start_center_y);
    for (uint8 i = start_center_y; i <= end_center_y; ++i) {
      right_line[i] = (uint8)(start_center_x + kr * (i - start_center_y));
      if (imgOSTU[i][right_line[i]] != White) {
        Black_num_r++;
      } else {
        imgOSTU[i][right_line[i]] = Judge_line;
      }
    }
    if (Black_num_l > Black_num_r) {
      end_center_x = t_center_r;
      Black_num = Black_num_r;
      for (uint8 i = start_center_y; i <= end_center_y; ++i) {
        mid_line[i] = right_line[i];
        imgOSTU[i][mid_line[i]] = Control_line;
      }
    } else {
      end_center_x = t_center_l;
      Black_num = Black_num_l;
      for (uint8 i = start_center_y; i <= end_center_y; ++i) {
        mid_line[i] = left_line[i];
        imgOSTU[i][mid_line[i]] = Control_line;
      }
    }
  }
  if (start_center_y == Deal_Bottom && Black_num < 3) {
    IF = straightlineL;
  } else if (start_center_y == Deal_Bottom) {
    IF = straightlineS;
  }
  return end_center_y;
}
/******************************************************************************
 * 函数名称     : Left_curve_line
 * 描述         : 绘制左边界曲线
 * 返回参数     : 终点行坐标
 ******************************************************************************/
uint8 Left_curve_line(void) {
  end_center_x = Deal_Left;
  end_center_y = l_center[l_lost_num - 1];
  uint8 Black_num = 0;
  if (end_center_y <= start_center_y) {
    return 0;
  }
  float k = ((float)end_center_x - (float)start_center_x) /
            ((float)end_center_y - (float)start_center_y);
  for (uint8 i = start_center_y; i <= end_center_y; ++i) {
    mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
    if (imgOSTU[i][(uint8)mid_line[i]] != White) {
      Black_num++;
    }
    imgOSTU[i][(uint8)mid_line[i]] = Control_line;
  }
  IF = curve;
  return end_center_y;
}
/******************************************************************************
 * 函数名称     : Right_curve_line
 * 描述         : 绘制右边界曲线
 * 返回参数     : 终点行坐标
 ******************************************************************************/
uint8 Right_curve_line(void) {
  end_center_x = Deal_Right;
  end_center_y = r_center[r_lost_num - 1];
  uint8 Black_num = 0;
  if (end_center_y <= start_center_y) {
    return 0;
  }
  float k = ((float)end_center_x - (float)start_center_x) /
            ((float)end_center_y - (float)start_center_y);
  for (uint8 i = start_center_y; i <= end_center_y; ++i) {
    mid_line[i] = (float)(start_center_x + k * (i - start_center_y));
    if (imgOSTU[i][(uint8)mid_line[i]] != White) {
      Black_num++;
    }
    imgOSTU[i][(uint8)mid_line[i]] = Control_line;
  }
  IF = curve;
  return end_center_y;
}
// ==========================================================
// 函数：Check_Node_Box(uint8 trigger_y)
// 作用：利用 15cm 动态框扫描边缘出口，执行纯拓扑分类
// 返回：0=假干扰直行，1=真节点(需查数组)，2=纯左L弯，3=纯右L弯
// ==========================================================
uint8 Check_Node_Box(uint8 trigger_y) {
  // 1. 不对称定框：[-5cm, +15cm]
  int box_bottom = trigger_y - Length_5cm[trigger_y];
  if (box_bottom < Deal_Bottom)
    box_bottom = Deal_Bottom;

  int box_top = trigger_y + Step_15cm[trigger_y];
  if (box_top > Deal_Top)
    box_top = Deal_Top;

  // 2. 扫三条边的有效出口 (必须在 box_bottom 到 box_top 的全区间扫侧边)
  uint8 exit_top = 0;
  uint8 exit_left = 0;
  uint8 exit_right = 0;
  int count_w = 0;
  // 检查顶边
  for (int x = Deal_Left; x <= Deal_Right; x++) {
    if (imgOSTU[box_top][x] == White || imgOSTU[box_top - 1][x] == White) {
      count_w++;
      if (count_w > 5) {
        exit_top = 1;
        break;
      }
    } else
      count_w = 0;
  }
  // 检查左边
  count_w = 0;
  for (int y = box_bottom; y <= box_top; y++) {
    if (imgOSTU[y][Deal_Left + 5] == White ||
        imgOSTU[y][Deal_Left + 6] == White) {
      count_w++;
      if (count_w > 5) {
        exit_left = 1;
        break;
      }
    } else
      count_w = 0;
  }
  // 检查右边
  count_w = 0;
  for (int y = box_bottom; y <= box_top; y++) {
    if (imgOSTU[y][Deal_Right - 5] == White ||
        imgOSTU[y][Deal_Right - 6] == White) {
      count_w++;
      if (count_w > 5) {
        exit_right = 1;
        break;
      }
    } else
      count_w = 0;
  }
  // ================= 3. 终极真值表判定 =================
  // 【A. 真节点】：至少两个有效出口 (十字、T字、Y字岔路)
  if ((exit_left && exit_right) || (exit_left && exit_top) ||
      (exit_right && exit_top)) {
    return 1; // 真岔路！只有这里允许交由决策层查数组！
  }
  // 【B. 单路拐弯】：只有一个侧向出口 (纯直角弯/急弯)
  else if (exit_left && !exit_top && !exit_right)
    return 2; // 纯左拐点
  else if (exit_right && !exit_top && !exit_left)
    return 3; // 纯右拐点
  // 【C. 假干扰】：只有上方有路，或者全视野丢失
  return 0;
}

// 强制 IDE 刷新时间戳：144a4aa7

/******************************************************************************
 * 函数名称     : Get_lost_tip
 * 描述         : [21届重构] 获取四方位边缘特征 (求取断点)
 * 进入参数     : length (有效断点的最小宽度，内联忽略)
 * 返回参数     : void
 * 架构师备注   : 绝对禁止在此处写任何十字/拐角等具体路况的判断逻辑
 ******************************************************************************/
void Get_lost_tip(uint8 length) {
    // 1. 全局状态初始化 (每帧都需要重置)
    dis_Solidline = 0;
    b_lost_num = 0; t_lost_num = 0; l_lost_num = 0; r_lost_num = 0;
    memset(b_center, 0, sizeof(b_center));
    memset(l_center, 0, sizeof(l_center));
    memset(r_center, 0, sizeof(r_center));
    memset(t_center, 0, sizeof(t_center));

    uint8 in_segment = 0; // 缺口状态标志
    uint8 seg_start = 0;  // 缺口起始坐标

    // ==========================================================
    // 2. 扫描顶部 (Lost_Top) - 沿X轴扫描
    // ==========================================================
    if (t_lost_tip > 0) { // 如果检测到顶部有丢失
        in_segment = 0;
        for (int x = Deal_Left; x <= Deal_Right; x++) {
            // 判断是否为有效丢失边线或超白区
            if (imgOSTU[Lost_Top][x] == Lost_line || imgOSTU[Deal_Top][x] == White) {
                if (!in_segment) {
                    in_segment = 1;
                    seg_start = x; // 捕捉到缺口起始
                }
            } else {
                if (in_segment) {
                    in_segment = 0; // 捕捉到缺口结束
                    // 过滤小于 length 的噪点
                    if (x - seg_start >= length && t_lost_num < 2) {
                        t_center[t_lost_num++] = (seg_start + x - 1) / 2; // 求中点
                    }
                }
            }
        }
        // 扫尾边界处未闭合的缺口
        if (in_segment && (Deal_Right - seg_start + 1 >= length) && t_lost_num < 2) {
            t_center[t_lost_num++] = (seg_start + Deal_Right) / 2;
        }
    }

    // ==========================================================
    // 3. 扫描底部 (Lost_Bottom) - 沿X轴扫描
    // ==========================================================
    if (b_lost_tip > 0) {
        in_segment = 0;
        for (int x = Deal_Left; x <= Deal_Right; x++) {
            if (imgOSTU[Lost_Bottom][x] == Lost_line) {
                if (!in_segment) { in_segment = 1; seg_start = x; }
            } else {
                if (in_segment) {
                    in_segment = 0;
                    if (x - seg_start >= length && b_lost_num < 2) {
                        b_center[b_lost_num++] = (seg_start + x - 1) / 2;
                    }
                }
            }
        }
        if (in_segment && (Deal_Right - seg_start + 1 >= length) && b_lost_num < 2) {
            b_center[b_lost_num++] = (seg_start + Deal_Right) / 2;
        }
    }

    // ==========================================================
    // 4. 扫描左部 (Lost_Left) - 沿Y轴扫描
    // ==========================================================
    if (l_lost_tip > 0) {
        in_segment = 0;
        for (int y = Deal_Bottom; y <= Deal_Top; y++) {
            if (imgOSTU[y][Lost_Left] == Lost_line || imgOSTU[y][Deal_Left] == White) {
                if (!in_segment) { in_segment = 1; seg_start = y; }
            } else {
                if (in_segment) {
                    in_segment = 0;
                    if (y - seg_start >= length && l_lost_num < 5) {
                        l_center[l_lost_num++] = (seg_start + y - 1) / 2;
                    }
                }
            }
        }
        if (in_segment && (Deal_Top - seg_start + 1 >= length) && l_lost_num < 5) {
            l_center[l_lost_num++] = (seg_start + Deal_Top) / 2;
        }
    }

    // ==========================================================
    // 5. 扫描右部 (Lost_Right) - 沿Y轴扫描
    // ==========================================================
    if (r_lost_tip > 0) {
        in_segment = 0;
        for (int y = Deal_Bottom; y <= Deal_Top; y++) {
            if (imgOSTU[y][Lost_Right] == Lost_line || imgOSTU[y][Deal_Right] == White) {
                if (!in_segment) { in_segment = 1; seg_start = y; }
            } else {
                if (in_segment) {
                    in_segment = 0;
                    if (y - seg_start >= length && r_lost_num < 5) {
                        r_center[r_lost_num++] = (seg_start + y - 1) / 2;
                    }
                }
            }
        }
        if (in_segment && (Deal_Top - seg_start + 1 >= length) && r_lost_num < 5) {
            r_center[r_lost_num++] = (seg_start + Deal_Top) / 2;
        }
    }
}


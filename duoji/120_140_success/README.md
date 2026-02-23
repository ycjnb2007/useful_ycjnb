# 提醒

该项目成功稳定实现了编码器速度120-140之间的稳定寻迹

## 主要参数

```c
PID Servo_pid = PID_CREATE(0.8, 0, 4.9, 0.8);//3参数：KP、KI、KD、滤波系数
float differential_k = 0.7;       //差速系数
const float peak_center = 0.30f;  // 权重峰值位置（模拟线的权重为0.3）
#define MT9V03X_FPS_DEF    ( 120  ) // 图像帧率设置   摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
//并且开启自动曝光和图像增益的最大值
```

```c
//#define ENABLE_BREAK     // 断路检测
//#define ENABLE_CROSS
#define ENABLE_PRE_LOOP
//注释掉断路和十字的检测、开启入环前的检测
```

## 操作提醒

对应此工程来说，速度越大，那么摄像头的前瞻就要看得越远

再说明一点，此次重大修改在于发现速度越大，那么就应该设置更远的前瞻和更低的权重
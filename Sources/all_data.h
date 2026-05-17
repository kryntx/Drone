#ifndef __ALL_DATA_H
#define __ALL_DATA_H

#include "stdint.h"

typedef struct
{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} _st_Mpu;

typedef struct
{
    int16_t magX;
    int16_t magY;
    int16_t magZ;
} _st_Mag;

typedef struct
{
    float rate;
    float height;
} High;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} _st_AngE;

typedef struct
{
    uint16_t roll;      // 横滚通道值 - 控制无人机左右倾斜的角度，通常范围1000-2000，中位1500
    uint16_t pitch;     // 俯仰通道值 - 控制无人机前后倾斜的角度，通常范围1000-2000，中位1500
    uint16_t thr;       // 油门通道值 - 控制无人机电机转速/升力大小，通常范围1000-2000，最低1000
    uint16_t yaw;       // 偏航通道值 - 控制无人机绕垂直轴旋转（左转/右转），通常范围1000-2000，中位1500
    uint16_t AUX1;      // 辅助通道1 - 自定义功能开关，如飞行模式切换、自动返航等，通常范围1000-2000
    uint16_t AUX2;      // 辅助通道2 - 自定义功能开关，如GPS开关、定高模式等，通常范围1000-2000
    uint16_t AUX3;      // 辅助通道3 - 自定义功能开关，预留扩展功能，通常范围1000-2000
    uint16_t AUX4;      // 辅助通道4 - 自定义功能开关，预留扩展功能，通常范围1000-2000
} _st_Remote;

typedef volatile struct
{
    float desired;        // 期望值/设定值 - 系统希望达到的目标值（如期望速度、位置、角度）
    float offset;         // 偏移量 - 用于校准或调整系统的静态偏差
    float prevError;      // 上一次误差值 - 记录上一个控制周期的误差，用于计算微分项
    float integ;          // 积分累积值 - 误差的累积和，用于消除稳态误差
    float kp;             // 比例增益 - 决定对当前误差的响应强度，值越大响应越快但过大会导致振荡
    float ki;             // 积分增益 - 决定对累积误差的响应强度，用于消除稳态误差但过大会引起超调
    float kd;             // 微分增益 - 决定对误差变化率的响应强度，用于预测误差趋势并增加系统阻尼
    float IntegLimitHigh; // 积分上限 - 防止积分饱和，限制积分项的最大值
    float IntegLimitLow;  // 积分下限 - 限制积分项的最小值
    float measured;       // 测量值/实际值 - 从传感器读取的实际测量值（如当前实际速度、位置）
    float out;            // 最终输出值 - PID 计算后的控制输出，用于驱动电机、舵机等执行机构
    float OutLimitHigh;   // 输出上限 - 限制 PID 控制器的最大输出值，保护执行机构
    float OutLimitLow;    // 输出下限 - 限制 PID 控制器的最小输出值
} PidObject;

typedef volatile struct
{
    uint8_t unlock;
    uint8_t sleep_timeout_s; // 超时休眠时长 (秒), 0 为关闭
    uint8_t paired;          // 配对标志：1 为已配对，0 为未配对
    uint8_t mpu_calibrated;  // MPU6050 校准标志：1 为已校准，0 为未校准
} _st_ALL_flag;

extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_Mag AK8975;
extern _st_AngE Angle;

extern _st_ALL_flag ALL_flag;

extern PidObject pidRateX;
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern PidObject pidPitch;
extern PidObject pidRoll;
extern PidObject pidYaw;

extern PidObject pidHeightRate;
extern PidObject pidHeightHigh;

void pid_param_Init(void);

#endif

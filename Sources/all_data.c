#include "all_data.h"

volatile uint32_t SysTick_count; // 系统时间计数

_st_Mpu MPU6050;   // MPU6050 传感器原始数据 - 包含三轴加速度和三轴陀螺仪的 ADC 采样值
_st_Mag AK8975;    // AK8975 磁力计数据 - 提供三轴地磁场强度测量值，用于电子罗盘和航向解算
_st_AngE Angle;    // 当前角度姿态值 - 经过卡尔曼或互补滤波融合后的飞行器姿态角（俯仰、横滚、偏航）
_st_Remote Remote; // 遥控通道值 - 接收遥控器各通道输入值（油门、俯仰、横滚、偏航等）

_st_ALL_flag ALL_flag; // 系统标志位 - 包含解锁状态、飞行模式、传感器状态等系统级标志位

PidObject pidRateX; // X 轴角速度 PID - 内环控制器，控制 X 轴（俯仰）的角速度响应
PidObject pidRateY; // Y 轴角速度 PID - 内环控制器，控制 Y 轴（横滚）的角速度响应
PidObject pidRateZ; // Z 轴角速度 PID - 内环控制器，控制 Z 轴（偏航）的角速度响应

PidObject pidPitch; // 俯仰角 PID - 外环位置控制器，控制飞行器的俯仰角度
PidObject pidRoll;  // 横滚角 PID - 外环位置控制器，控制飞行器的横滚角度
PidObject pidYaw;   // 偏航角 PID - 外环位置控制器，控制飞行器的偏航角度

PidObject pidHeightRate; // 高度变化率 PID - 内环控制器，控制垂直方向的速度
PidObject pidHeightHigh; // 高度 PID - 外环控制器，控制飞行器的目标高度

void pid_param_Init(void)
{
    pidRateX.kp = 2.0f;
    pidRateY.kp = 2.0f;
    pidRateZ.kp = 4.0f;

    pidRateX.ki = 1.0f;
    pidRateY.ki = 2.0f;
    pidRateZ.ki = 1.0f;

    pidRateX.kd = 0.1f;
    pidRateY.kd = 0.1f;
    pidRateZ.kd = 0.5f;

    pidPitch.kp = 6.0f;
    pidRoll.kp = 6.0f;
    pidYaw.kp = 7.0f;

    pidPitch.ki = 0.0f;
    pidRoll.ki = 0.0f;
    pidYaw.ki = 0.0f;

    pidPitch.kd = 0.0f;
    pidRoll.kd = 0.0f;
    pidYaw.kd = 0.0f;

    //---------------------

    // pidRateX.kp = 0.0f;
    // pidRateY.kp = 0.0f;
    // pidRateZ.kp = 0.0f;

    // pidRateX.ki = 0.0f;
    // pidRateY.ki = 0.0f;
    // pidRateZ.ki = 0.0f;

    // pidRateX.kd = 0.0f;
    // pidRateY.kd = 0.0f;
    // pidRateZ.kd = 0.0f;

    // pidPitch.kp = 0.0f;
    // pidRoll.kp = 0.0f;
    // pidYaw.kp = 0.0f;

    // pidPitch.ki = 0.0f;
    // pidRoll.ki = 0.0f;
    // pidYaw.ki = 0.0f;

    // pidPitch.kd = 0.0f;
    // pidRoll.kd = 0.0f;
    // pidYaw.kd = 0.0f;
}

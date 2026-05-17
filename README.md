# Drone - 基于 STM32F401CCU6 的小四轴无人机

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-STM32-blue)](https://www.st.com/)
[![Toolchain](https://img.shields.io/badge/toolchain-Keil%20MDK-green)](https://www.keil.com/)

## 简介

本项目是一个微型四轴无人机飞控固件，基于 STM32F401CCU6 主控芯片，使用 STM32CubeMX 生成外设初始化代码，在 Keil MDK 环境下开发。固件实现了姿态解算、电机控制、无线遥控、气压定高、光流悬停、超声波测距、卫星定位等功能。

## 硬件平台

| 组件          | 型号               | 说明                         |
| ------------- | ------------------ | ---------------------------- |
| 主控          | STM32F401CCU6      | Cortex-M4，84MHz，256KB Flash |
| 姿态传感器    | MPU6050            | 6轴（加速度+陀螺仪）           |
| 气压计        | SPL06-001          | 高精度气压传感器，用于定高     |
| 光流传感器    | GL-9306            | 光流，用于水平位置修正和悬停   |
| 无线通信      | NRF24L01+          | 2.4G，与遥控器通信             |
| 卫星定位模块  | AIR530             | 北斗/GPS双模定位，默认波特率9600 |
| 超声波模块    | HC-SR04            | 近距测距辅助，提升低空稳定性   |
| 电机驱动      | 集成MOS管（4路）   | 控制空心杯电机                 |
| 电源          | 1S锂电池（3.7V-4.2V） | 稳压至3.3V给芯片供电      |

## 软件架构

- 裸机框架，主循环 + 定时器中断
- 传感器驱动：I2C（MPU6050, SPL06-001）、SPI（NRF24L01）、UART（GL-9306 光流、AIR530定位）、定时器（HC-SR04超声波）
- 姿态解算：Mahony互补滤波 / 四元数
- PID控制器：角度环 → 角速度环 → 电机PWM输出
- 遥控协议：自定义NRF24L01数据包（油门、横滚、俯仰、偏航）
- 定高算法：气压计 + 超声波 + 光流数据融合

### 工程目录结构

```
Drone/
├── Core/           # 核心代码（CubeMX生成）
│   ├── Inc/        # 头文件
│   └── Src/        # 源文件（main.c, 中断等）
├── Drivers/        # 硬件驱动（STM32 HAL库）
├── Sources/            # 应用层（姿态解算、PID、遥控解析、传感器驱动、电机、NRF等）
├── MDK-ARM/        # Keil工程文件
└── README.md
```

## 编译与烧录

### 开发环境

- IDE：Keil MDK v5.31 或更高版本
- 工具链：ARM Compiler v6
- 固件包：Keil.STM32F4xx_DFP.2.x.x
- 调试器：ST-Link / J-Link / DAP-Link

### 步骤

1. 克隆仓库：
   ```bash
   git clone https://github.com/kryntx/Drone.git
   cd Drone
   ```

2. 打开 Keil 工程：进入 `MDK-ARM/` 目录，双击 `Drone.uvprojx`

3. 编译：按 `F7` 或点击 `Build`

4. 烧录：连接 ST-Link 到飞控板的 SWD 接口，按 `F8` 或点击 `Download`

## 飞控板 STM32F401CCU6 资源分配表

| 外设     | CubeMX引脚配置  | IO 口 | 功能     | 备注                                                         |
| -------- | --------------- | ----- | -------- | ------------------------------------------------------------ |
| SW调试   | SWDIO           | PA13  | 数据     | SW 程序下载与调试接口，数据                                  |
|          | SWCLK           | PA14  | 时钟     | SW 程序下载与调试接口，时钟                                  |
| 串口1    | USART1_TXD      | PA9   | 发送     | 上位机调试用串口，输出姿态数据                               |
|          | USART1_RXD      | PA10  | 接收     |                                                              |
| 串口2    | USART2_TXD      | PA2   | 发送     | 光流模块 GL-9306，波特率 19200                               |
|          | USART2_RXD      | PA3   | 接收     |                                                              |
| 串口6    | USART6_TXD      | PA11  | 发送     | 卫星定位 AIR530，默认波特率 9600                            |
|          | USART6_RXD      | PA12  | 接收     |                                                              |
| 定时器3  | TIM3_CH1        | PB4   | 通道1    | PWM 输出，用于电机1速度控制，频率 8kHz                       |
|          | TIM3_CH2        | PB5   | 通道2    | PWM 输出，用于电机2速度控制，频率同上                        |
|          | TIM3_CH3        | PB0   | 通道3    | PWM 输出，用于电机3速度控制，频率同上                        |
|          | TIM3_CH4        | PB1   | 通道4    | PWM 输出，用于电机4速度控制，频率同上                        |
| 定时器2  | TIM2_CH1        | PA15  | 通道1    | PWM 输出，用于输出触发信号给超声模块 HC-SR04，频率 5Hz，周期 200ms，高电平 10us，占空比 1/20000 |
|          | TIM2_CH2        | PB3   | 通道2    | 输入捕捉，用于测量超声模块反馈信号的高电平时间，从而计算距离 |
| I2C1     | I2C1_SCL        | PB6   | 时钟     | 接气压芯片 SPL06-001，姿态感知芯片 MPU6050                   |
|          | I2C1_SDA        | PB7   | 数据     |                                                              |
| SPI1     | GPIO_Output     | PA4   | 使能 CSN | 无线通信 nRF24L01                                            |
|          | SPI1_SCK        | PA5   | 时钟     |                                                              |
|          | SPI1_MISO       | PA6   | 主收从发 |                                                              |
|          | SPI1_MOSI       | PA7   | 主发从收 |                                                              |
|          | GPIO_Output     | PB2   | CE       |                                                              |
|          | GPIO_EXTI10     | PB10  | 中断 IRQ |                                                              |
| SPI2     | SPI1_NSS        | PB12  | 使能     | 备用                                                         |
|          | SPI2_SCK        | PB13  | 时钟     |                                                              |
|          | SPI2_MISO       | PB14  | 主收从发 |                                                              |
|          | SPI2_MOSI       | PB15  | 主发从收 |                                                              |
|          | GPIO_Output     | PC15  | CE2      |                                                              |

## 使用说明

1. 上电自检：LED闪烁表示传感器初始化成功。
2. 对频：打开NRF24L01遥控器，等待接收机灯常亮。
3. 解锁：遥控器油门最低 + 方向最右，电机怠速转动。
4. 飞行：推油门起飞，通过摇杆控制姿态，低空可依赖超声波辅助定高。
5. 锁定：油门最低 + 方向最左，电机停转。

## 待完善

- 姿态解算需要进一步校准陀螺仪零偏
- 气压计温度补偿未完全实现，定高有漂移
- GL-9306 光流数据解析需根据实际协议调整
- 遥控器数据包格式需与发射端严格匹配
- 超声波与气压计数据融合算法待优化
- 卫星定位模块数据解析与悬停控制未实现

## 贡献

欢迎提交 Issue 或 Pull Request。

## 许可证

MIT 许可证，详见 [LICENSE](LICENSE) 文件。

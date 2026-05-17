```markdown
# Drone - 基于 STM32F401CCU6 的小四轴无人机

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-STM32-blue)](https://www.st.com/)
[![Toolchain](https://img.shields.io/badge/toolchain-Keil%20MDK-green)](https://www.keil.com/)

## 简介

本项目是一个微型四轴无人机飞控固件，基于 STM32F401CCU6 主控芯片，使用 STM32CubeMX 生成外设初始化代码，在 Keil MDK 环境下开发。固件实现了姿态解算、电机控制、无线遥控、气压定高、光流悬停等功能。

## 硬件平台

| 组件          | 型号               | 说明                         |
| ------------- | ------------------ | ---------------------------- |
| 主控          | STM32F401CCU6      | Cortex-M4，84MHz，256KB Flash |
| 姿态传感器    | MPU6050            | 6轴（加速度+陀螺仪）           |
| 气压计        | SPL06-001          | 高精度气压传感器，用于定高     |
| 光流传感器    | GL9306             | 光流，用于水平位置修正和悬停   |
| 无线通信      | NRF24L01+          | 2.4G，与遥控器通信             |
| 电机驱动      | 集成MOS管（4路）   | 控制空心杯电机                 |
| 电源          | 1S锂电池（3.7V-4.2V） | 升压至5V给部分传感器供电      |

## 软件架构

- 裸机框架，主循环 + 定时器中断
- 传感器驱动：I2C（MPU6050, SPL06-001）、SPI（NRF24L01）、UART（GL9306 光流）
- 姿态解算：Mahony互补滤波 / 四元数
- PID控制器：角度环 → 角速度环 → 电机PWM输出
- 遥控协议：自定义NRF24L01数据包（油门、横滚、俯仰、偏航）
- 定高算法：气压计 + 光流数据融合

### 工程目录结构

```
Drone/
├── Core/           # 核心代码（CubeMX生成）
│   ├── Inc/        # 头文件
│   └── Src/        # 源文件（main.c, 中断等）
├── Drivers/        # 硬件驱动（STM32 HAL库）
├── Middlewares/    # 中间件
├── BSP/            # 板级支持包（传感器驱动、电机、NRF等）
├── App/            # 应用层（姿态解算、PID、遥控解析）
├── MDK-ARM/        # Keil工程文件
├── Docs/           # 文档（原理图、引脚分配表等）
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

### 引脚分配示例

| 外设       | 引脚                    | 备注               |
| ---------- | ----------------------- | ------------------ |
| MPU6050    | PB8 (SCL), PB9 (SDA)    | I2C1               |
| SPL06-001  | PB8 (SCL), PB9 (SDA)    | 与MPU6050共用I2C   |
| NRF24L01   | PA4 (CSN), PA5 (SCK), PA6 (MISO), PA7 (MOSI) | SPI1，CE/IRQ用普通GPIO |
| GL9306     | PA2 (TX), PA3 (RX)      | UART2，接收光流数据 |
| 电机1 (M1) | PA0 (TIM2_CH1)          | PWM，空心杯        |
| 电机2 (M2) | PA1 (TIM2_CH2)          | PWM                |
| 电机3 (M3) | PB0 (TIM3_CH3)          | PWM                |
| 电机4 (M4) | PB1 (TIM3_CH4)          | PWM                |

## 使用说明

1. 上电自检：LED闪烁表示传感器初始化成功。
2. 对频：打开NRF24L01遥控器，等待接收机灯常亮。
3. 解锁：遥控器油门最低 + 方向最右，电机怠速转动。
4. 飞行：推油门起飞，通过摇杆控制姿态。
5. 锁定：油门最低 + 方向最左，电机停转。

## 待完善

- 姿态解算需要进一步校准陀螺仪零偏
- 气压计温度补偿未完全实现，定高有漂移
- GL9306 光流数据解析需根据实际协议调整
- 遥控器数据包格式需与发射端严格匹配

## 贡献

欢迎提交 Issue 或 Pull Request。

## 许可证

MIT 许可证，详见 [LICENSE](LICENSE) 文件。
```

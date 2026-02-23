# FOC Self-Balancing Robot / FOC 自平衡小车

[English](#english) | [中文](#chinese)

---

<a name="english"></a>
## 🤖 English

### Project Overview

This is a **dual-motor self-balancing robot** based on **Field-Oriented Control (FOC)** technology, running on the STM32F407 microcontroller. The project implements a complete FOC control system from scratch, including current loop control, balance control, and velocity control, suitable for learning embedded systems, motor control, and robotics.

### ✨ Key Features

- **🎯 FOC Motor Control**: Custom SimpleFOC library implementation
  - Clarke/Park transformations using ARM CMSIS-DSP
  - SVPWM (Space Vector PWM) modulation
  - Current loop closed-loop control (id/iq tracking)
  - Torque mode control

- **⚖️ Self-Balancing Control**: Cascaded PID control architecture
  - **Inner Loop**: Vertical PD controller (angle + gyro feedback)
  - **Outer Loop**: Velocity PI controller (optional)
  - Real-time IMU data fusion (JY60 6-axis sensor)

- **🔧 Hardware Abstraction Layer (HAL)**
  - AS5600 magnetic encoder (I2C + DMA)
  - Dual ADC current sensing (DMA + interrupt)
  - Multi-timer PWM generation
  - UART debugging interface

- **📊 Debugging & Visualization**
  - VOFA+ protocol support for real-time data plotting
  - LCD display for parameter monitoring
  - Serial command interface

### 🛠️ Hardware Requirements

| Component | Model/Spec | Quantity |
|-----------|------------|----------|
| **MCU** | STM32F407VET6 (168MHz, Cortex-M4F) | 1 |
| **Motor** | BLDC Motor (7 pole pairs) | 2 |
| **Encoder** | AS5600 Magnetic Encoder (12-bit) | 2 |
| **IMU** | JY60 6-Axis Gyroscope (I2C) | 1 |
| **Current Sensor** | Inline current sensor (0.01Ω shunt, 50x gain) | 6 |
| **Display** | ST7735 LCD (SPI) | 1 |
| **Power Supply** | 12V Battery | 1 |

### 📁 Project Structure

```
foc-text/
├── Core/                   # STM32 HAL initialization code
│   ├── Src/
│   │   ├── main.c         # Main program entry
│   │   ├── stm32f4xx_it.c # Interrupt handlers
│   │   └── ...
│   └── Inc/               # Header files
├── simplefoc/             # FOC control library
│   ├── simplefoc.c        # FOC core algorithms
│   └── simplefoc.h        # FOC API definitions
├── hardware/              # Hardware driver layer
│   ├── as5600.c/h         # AS5600 encoder driver
│   ├── adc_measure.c/h    # ADC current sampling
│   ├── jy60.c/h           # JY60 IMU driver
│   ├── lcd.c/h            # LCD display driver
│   └── ...
├── software/              # Application layer
│   ├── blance.c/h         # Balance control algorithm
│   ├── vofa.c/h           # VOFA+ debugging protocol
│   └── Allfile.h          # Global header file
├── Drivers/               # STM32 HAL drivers
├── Middlewares/           # ARM DSP library
└── foc-text.ioc           # STM32CubeMX configuration file
```

### 🚀 Quick Start

#### 1. Environment Setup

- **IDE**: Keil MDK-ARM v5 or STM32CubeIDE
- **Toolchain**: ARM GCC or ARM Compiler 6
- **Debugger**: ST-Link V2/V3
- **Dependencies**:
  - STM32F4 HAL Library
  - ARM CMSIS-DSP Library

#### 2. Build & Flash

**Using Keil MDK:**
```bash
1. Open MDK-ARM/foc-text.uvprojx
2. Build Project (F7)
3. Download to Flash (F8)
```

**Using STM32CubeIDE:**
```bash
1. Import project as "Existing STM32CubeMX Configuration File"
2. Build Project (Ctrl+B)
3. Run/Debug (F11)
```

#### 3. Hardware Connections

| Peripheral | STM32 Pin | Description |
|------------|-----------|-------------|
| **Motor1 PWM** | TIM2_CH1/2/3 | Phase A/B/C |
| **Motor2 PWM** | TIM4_CH1/2/3 | Phase A/B/C |
| **Encoder1** | I2C1 (PB6/PB7) | AS5600 Left |
| **Encoder2** | I2C3 (PA8/PC9) | AS5600 Right |
| **IMU** | UART4 (PA0/PA1) | JY60 Gyroscope |
| **Current Sense** | ADC2/ADC3 | 6-channel current |
| **Debug UART** | USART6 (PC6/PC7) | Serial output |

#### 4. Initialization & Calibration

The initialization follows a strict 5-stage sequence to ensure proper startup:

```c
// Stage 1: Basic peripheral initialization
JY60_Init();                    // IMU initialization
AS5600_Init(&as5600_l, ...);    // Encoder initialization
HAL_TIM_Base_Start_IT(&htim3);  // Start encoder scheduler

// Stage 2: FOC motor object initialization
FOC_Init(&motor1, 7);           // Initialize motor structure
FOC_AttachDefaultHAL(&motor1);  // Bind hardware interfaces
FOC_SetVoltageLimit(&motor1, 12.0f);
FOC_SetCurrentLimit(&motor1, 2.0f);
FOC_SetMode(&motor1, FOC_MODE_TORQUE);
FOC_SetTarget(&motor1, 0.0f);   // Initial target = 0

// Stage 3: Motor calibration
FOC_CalibrateDirection(&motor1);  // Motor direction calibration
FOC_CalibrateZeroOffset(&motor1); // Encoder zero-point calibration

// Stage 4: Balance control initialization
balance_init(&vpid, ...);       // Vertical PD controller
speed_init(&spid, ...);         // Speed PI controller
HAL_TIM_Base_Start_IT(&htim5);  // Start speed loop timer

// Stage 5: ADC and current loop startup (LAST!)
ADC_Measure_Init();             // Initialize PWM and timers (no interrupt yet)
HAL_Delay(100);                 // Wait for ADC stabilization
ADC_Calibrate_Current_Sensors(); // Calibrate current sensor zero-point
ADC_Start_Interrupt();          // Start ADC interrupt, begin FOC control
```

**⚠️ Important**: 
- Keep the robot stationary during calibration!
- ADC interrupt is started LAST to avoid triggering FOC during initialization

#### 5. Parameter Tuning

Edit `software/blance.c` to adjust PID parameters:

```c
// Vertical PD controller
balance_init(&vpid,
    0.07f,   // kp: Proportional gain
    0.008f,  // kd: Derivative gain
    -28.0f   // target: Target angle (degrees)
);

// Current limit
FOC_SetCurrentLimit(&motor1, 1.8f); // Max 1.8A
```

### 📊 Control Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│              Cascaded Control Loop Architecture                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌────────────┐      ┌────────────┐      ┌────────────┐        │
│  │  Velocity  │ 500Hz│  Balance   │ 1kHz │  Current   │ 20kHz  │
│  │   Loop     │─────▶│   Loop     │─────▶│   Loop     │───────▶│
│  │   (PI)     │      │   (PD)     │      │   (FOC)    │  Motor │
│  └────────────┘      └────────────┘      └────────────┘        │
│       ▲                    ▲                    ▲                │
│       │                    │                    │                │
│   Encoder              IMU (JY60)          ADC Current           │
│  (AS5600)           Roll + Gyro           6-Channel              │
│  Velocity              Angle                Sensing              │
│                                                                   │
│  TIM5 (500Hz)     ────▶ Speed Control (Outer Loop)              │
│  TIM3 (1kHz)      ────▶ Balance Control (Middle Loop)           │
│  ADC Interrupt    ────▶ FOC Current Control (Inner Loop)        │
│  (20kHz, HW Trig)       Triggered by TIM2/TIM4 TRGO             │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

### 🔬 FOC Algorithm Details

The FOC implementation follows the standard field-oriented control pipeline:

**Hardware-Triggered ADC Sampling:**
- TIM2/TIM4 generate 20kHz center-aligned PWM
- CH4 output (TRGO) triggers ADC injected conversion at PWM center point
- ADC samples at minimum current ripple for best accuracy
- ADC interrupt fires upon conversion completion

**FOC Control Loop (in ADC ISR):**
1. **Current Sensing**: Read 3-phase currents via ADC (Ia, Ib, Ic)
2. **Clarke Transform**: Convert to α-β stationary frame
3. **Park Transform**: Convert to d-q rotating frame (aligned with rotor flux)
4. **PID Control**:
   - `id_target = 0` (minimize flux current)
   - `iq_target = torque_command` (control torque)
5. **Inverse Park**: Convert d-q voltages back to α-β
6. **SVPWM**: Generate 3-phase PWM signals
7. **Update PWM**: Apply new duty cycles to TIM2/TIM4

**Key Advantages:**
- Zero delay between sampling and control (executed in same ISR)
- Synchronized with PWM for optimal current measurement
- No separate timer needed for FOC loop

### 📈 Performance Metrics

- **Current Loop (Inner)**: 20 kHz (ADC interrupt, hardware-triggered) - FOC control
- **Balance Loop (Middle)**: 1 kHz (TIM3 interrupt) - PD control
- **Velocity Loop (Outer)**: 500 Hz (TIM5 interrupt) - PI control
- **Encoder Update Rate**: 1 kHz (I2C DMA)
- **PWM Frequency**: 20 kHz (TIM2/TIM4, center-aligned mode)
- **ADC Sampling**: Synchronized with PWM center point (minimum current ripple)
- **Current Loop Bandwidth**: ~5 kHz
- **Balance Recovery Time**: < 0.5s
- **Max Tilt Angle**: ±30°

### 🐛 Debugging Tools

#### VOFA+ Real-time Plotting

Connect via UART6 (115200 baud) and use VOFA+ software:

```c
// In main loop
printf("angle:%.3f,%.3f,%.3f,%.3f\n",
       imu.roll, imu.gx, vel_left, vel_right);
```

#### Serial Commands

```bash
# View encoder data
vofa_as5600_show();

# View current loop status
vofa_currentLoop();

# Open-loop motor test
FOC_OpenLoopTest(&motor1, 2.0f, 1.0f); // 2V, 1rad/s
```

### 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

### 🙏 Acknowledgments

- [SimpleFOC](https://simplefoc.com/) - FOC algorithm reference
- STMicroelectronics - HAL library
- ARM - CMSIS-DSP library

### 📧 Contact

For questions or collaboration, please open an issue on GitHub.

---

<a name="chinese"></a>
## 🤖 中文

### 项目简介

这是一个基于 **磁场定向控制（FOC）** 技术的 **双电机自平衡小车** 项目，运行在 STM32F407 微控制器上。项目从零实现了完整的 FOC 控制系统，包括电流环控制、平衡控制和速度控制，适合学习嵌入式系统、电机控制和机器人技术。

### ✨ 核心特性

- **🎯 FOC 电机控制**：自研 SimpleFOC 库实现
  - 使用 ARM CMSIS-DSP 的 Clarke/Park 变换
  - SVPWM（空间矢量脉宽调制）
  - 电流环闭环控制（id/iq 追踪）
  - 转矩模式控制

- **⚖️ 自平衡控制**：串级 PID 控制架构
  - **内环**：直立 PD 控制器（角度 + 陀螺仪反馈）
  - **外环**：速度 PI 控制器（可选）
  - 实时 IMU 数据融合（JY60 六轴传感器）

- **🔧 硬件抽象层（HAL）**
  - AS5600 磁编码器（I2C + DMA）
  - 双 ADC 电流采样（DMA + 中断）
  - 多定时器 PWM 生成
  - UART 调试接口

- **📊 调试与可视化**
  - 支持 VOFA+ 协议实时数据绘图
  - LCD 显示屏参数监控
  - 串口命令接口

### 🛠️ 硬件需求

| 组件 | 型号/规格 | 数量 |
|------|----------|------|
| **主控** | STM32F407VET6 (168MHz, Cortex-M4F) | 1 |
| **电机** | 无刷直流电机（7 对极） | 2 |
| **编码器** | AS5600 磁编码器（12 位） | 2 |
| **IMU** | JY60 六轴陀螺仪（I2C） | 1 |
| **电流传感器** | 在线电流传感器（0.01Ω 采样电阻，50 倍增益） | 6 |
| **显示屏** | ST7735 LCD（SPI） | 1 |
| **电源** | 12V 电池 | 1 |

### 📁 项目结构

```
foc-text/
├── Core/                   # STM32 HAL 初始化代码
│   ├── Src/
│   │   ├── main.c         # 主程序入口
│   │   ├── stm32f4xx_it.c # 中断处理函数
│   │   └── ...
│   └── Inc/               # 头文件
├── simplefoc/             # FOC 控制库
│   ├── simplefoc.c        # FOC 核心算法
│   └── simplefoc.h        # FOC API 定义
├── hardware/              # 硬件驱动层
│   ├── as5600.c/h         # AS5600 编码器驱动
│   ├── adc_measure.c/h    # ADC 电流采样
│   ├── jy60.c/h           # JY60 IMU 驱动
│   ├── lcd.c/h            # LCD 显示驱动
│   └── ...
├── software/              # 应用层
│   ├── blance.c/h         # 平衡控制算法
│   ├── vofa.c/h           # VOFA+ 调试协议
│   └── Allfile.h          # 全局头文件
├── Drivers/               # STM32 HAL 驱动
├── Middlewares/           # ARM DSP 库
└── foc-text.ioc           # STM32CubeMX 配置文件
```

### 🚀 快速开始

#### 1. 环境配置

- **IDE**：Keil MDK-ARM v5 或 STM32CubeIDE
- **工具链**：ARM GCC 或 ARM Compiler 6
- **调试器**：ST-Link V2/V3
- **依赖库**：
  - STM32F4 HAL 库
  - ARM CMSIS-DSP 库

#### 2. 编译与烧录

**使用 Keil MDK：**
```bash
1. 打开 MDK-ARM/foc-text.uvprojx
2. 编译项目（F7）
3. 下载到 Flash（F8）
```

**使用 STM32CubeIDE：**
```bash
1. 导入项目为"现有 STM32CubeMX 配置文件"
2. 编译项目（Ctrl+B）
3. 运行/调试（F11）
```

#### 3. 硬件连接

| 外设 | STM32 引脚 | 说明 |
|------|-----------|------|
| **电机1 PWM** | TIM2_CH1/2/3 | A/B/C 相 |
| **电机2 PWM** | TIM4_CH1/2/3 | A/B/C 相 |
| **编码器1** | I2C1 (PB6/PB7) | AS5600 左轮 |
| **编码器2** | I2C3 (PA8/PC9) | AS5600 右轮 |
| **IMU** | UART4 (PA0/PA1) | JY60 陀螺仪 |
| **电流采样** | ADC2/ADC3 | 6 路电流 |
| **调试串口** | USART6 (PC6/PC7) | 串口输出 |

#### 4. 初始化与校准

初始化遵循严格的 5 阶段顺序以确保正确启动：

```c
// 阶段 1：基础外设初始化
JY60_Init();                    // IMU 初始化
AS5600_Init(&as5600_l, ...);    // 编码器初始化
HAL_TIM_Base_Start_IT(&htim3);  // 启动编码器调度定时器

// 阶段 2：FOC 电机对象初始化
FOC_Init(&motor1, 7);           // 初始化电机结构体
FOC_AttachDefaultHAL(&motor1);  // 绑定硬件接口
FOC_SetVoltageLimit(&motor1, 12.0f);
FOC_SetCurrentLimit(&motor1, 2.0f);
FOC_SetMode(&motor1, FOC_MODE_TORQUE);
FOC_SetTarget(&motor1, 0.0f);   // 初始目标 = 0

// 阶段 3：电机校准
FOC_CalibrateDirection(&motor1);  // 电机方向校准
FOC_CalibrateZeroOffset(&motor1); // 编码器零点校准

// 阶段 4：平衡控制初始化
balance_init(&vpid, ...);       // 直立 PD 控制器
speed_init(&spid, ...);         // 速度 PI 控制器
HAL_TIM_Base_Start_IT(&htim5);  // 启动速度环定时器

// 阶段 5：ADC 和电流环启动（最后！）
ADC_Measure_Init();             // 初始化 PWM 和定时器（尚未启动中断）
HAL_Delay(100);                 // 等待 ADC 稳定
ADC_Calibrate_Current_Sensors(); // 校准电流传感器零点
ADC_Start_Interrupt();          // 启动 ADC 中断，开始 FOC 控制
```

**⚠️ 重要**：
- 校准期间请保持小车静止！
- ADC 中断最后启动，避免初始化期间触发 FOC 控制

#### 5. 参数调整

编辑 `software/blance.c` 调整 PID 参数：

```c
// 直立 PD 控制器
balance_init(&vpid,
    0.07f,   // kp：比例增益
    0.008f,  // kd：微分增益
    -28.0f   // target：目标角度（度）
);

// 电流限制
FOC_SetCurrentLimit(&motor1, 1.8f); // 最大 1.8A
```

### 📊 控制架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    串级控制回路架构                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌────────────┐      ┌────────────┐      ┌────────────┐         │
│  │   速度环   │ 500Hz│   直立环   │ 1kHz │   电流环   │ 20kHz     │
│  │   (PI)     │─────▶│   (PD)     │─────▶│   (FOC)    │───────▶│
│  │            │      │            │      │            │  电机    │
│  └────────────┘      └────────────┘      └────────────┘         │
│       ▲                    ▲                    ▲               │
│       │                    │                    │               │
│    编码器              IMU (JY60)          ADC 电流              │
│  (AS5600)           横滚角 + 角速度        6 路采样               │
│   速度反馈              姿态反馈              电流反馈            │
│                                                                 │
│  TIM5 (500Hz)     ────▶ 速度控制（外环）                         │
│  TIM3 (1kHz)      ────▶ 平衡控制（中环）                         │
│  ADC 中断         ────▶ FOC 电流控制（内环）                     │
│  (20kHz, 硬件触发)      由 TIM2/TIM4 TRGO 触发                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 🔬 FOC 算法详解

FOC 实现遵循标准的磁场定向控制流程：

**硬件触发 ADC 采样：**
- TIM2/TIM4 生成 20kHz 中心对齐 PWM
- CH4 输出（TRGO）在 PWM 中心点触发 ADC 注入转换
- ADC 在电流纹波最小处采样，精度最高
- ADC 转换完成后触发中断

**FOC 控制回路（在 ADC 中断中）：**
1. **电流采样**：通过 ADC 读取三相电流（Ia, Ib, Ic）
2. **Clarke 变换**：转换到 α-β 静止坐标系
3. **Park 变换**：转换到 d-q 旋转坐标系（与转子磁场对齐）
4. **PID 控制**：
   - `id_target = 0`（最小化磁通电流）
   - `iq_target = torque_command`（控制转矩）
5. **反 Park 变换**：将 d-q 电压转换回 α-β
6. **SVPWM**：生成三相 PWM 信号
7. **更新 PWM**：将新的占空比应用到 TIM2/TIM4

**关键优势：**
- 采样和控制零延迟（在同一中断中执行）
- 与 PWM 同步，电流测量最优
- 无需单独的定时器用于 FOC 回路

### 📈 性能指标

- **电流环（内环）**：20 kHz（ADC 中断，硬件触发）- FOC 控制
- **平衡环（中环）**：1 kHz（TIM3 中断）- PD 控制
- **速度环（外环）**：500 Hz（TIM5 中断）- PI 控制
- **编码器更新率**：1 kHz（I2C DMA）
- **PWM 频率**：20 kHz（TIM2/TIM4，中心对齐模式）
- **ADC 采样**：与 PWM 中心点同步（电流纹波最小）
- **电流环带宽**：~5 kHz
- **平衡恢复时间**：< 0.5s
- **最大倾角**：±30°

### 🐛 调试工具

#### VOFA+ 实时绘图

通过 UART6（115200 波特率）连接并使用 VOFA+ 软件：

```c
// 在主循环中
printf("angle:%.3f,%.3f,%.3f,%.3f\n",
       imu.roll, imu.gx, vel_left, vel_right);
```

#### 串口命令

```bash
# 查看编码器数据
vofa_as5600_show();

# 查看电流环状态
vofa_currentLoop();

# 开环电机测试
FOC_OpenLoopTest(&motor1, 2.0f, 1.0f); // 2V, 1rad/s
```

### 📝 开源协议

本项目采用 MIT 协议开源 - 详见 LICENSE 文件。

### 🙏 致谢

- [SimpleFOC](https://simplefoc.com/) - FOC 算法参考
- STMicroelectronics - HAL 库
- ARM - CMSIS-DSP 库

### 📧 联系方式

如有问题或合作意向，请在 GitHub 上提交 Issue。

---

## 🌟 Star History

如果这个项目对你有帮助，请给个 ⭐ Star！

If this project helps you, please give it a ⭐ Star!

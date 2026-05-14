# SRY_Slave_FreeRTOS — 灵巧手从机驱动板 (FreeRTOS)

FreeRTOS 实时操作系统驱动的灵巧手手指从机固件。运行于 STM32F407VETx (Cortex-M4F)，通过 Modbus RTU over RS485 接收主板指令，独立完成 4 路电机的闭环控制。

## 硬件平台

| 项 | 规格 |
|---|---|
| MCU | STM32F407VETx (Cortex-M4F, 168MHz) |
| Flash | 512 KB |
| RAM | 128 KB + 64 KB CCMRAM |
| RTOS | FreeRTOS V10.3.1 (CMSIS-OS V2 API, heap_4) |
| 通信 | RS485 (USART1, 115200) + Modbus RTU |
| 调试 | USART2, 115200 (printf) |
| 驱动电机数 | 4 路 (食指/中指/无名指/小指) |

## 目录结构

```
SRY_Slave_FreeRTOS/
├── Core/                       # 应用层源码 (用户代码 + CubeMX 生成)
│   ├── Inc/                    # 头文件
│   │   ├── Motor.h             #   电机控制结构体 & API 声明
│   │   ├── rs485.h             #   Modbus 协议栈 & RS485 宏
│   │   ├── rs485_crc.h         #   Modbus CRC16 查表
│   │   ├── FreeRTOSConfig.h    #   FreeRTOS 内核配置
│   │   └── *.h                 #   外设初始化头 (CubeMX 生成)
│   └── Src/                    # 源文件
│       ├── main.c              #   入口: 硬件初始化 → RTOS 调度器启动
│       ├── freertos.c          #   4 个 RTOS 任务体 + 互斥锁创建
│       ├── Motor.c             #   级联 PID + PWM 输出 + 编码器读取
│       ├── rs485.c             #   Modbus 帧解析 (DMA+IDLE 中断)
│       ├── rs485_crc.c         #   CRC16 计算
│       └── *.c                 #   外设初始化 (CubeMX 生成)
├── Drivers/                    # STM32 HAL 库 + CMSIS
│   ├── STM32F4xx_HAL_Driver/   #   HAL 驱动
│   └── CMSIS/                  #   Cortex-M4 内核接口
├── Middlewares/                # 第三方中间件
│   └── Third_Party/FreeRTOS/   #   FreeRTOS 内核源码
├── cmake/                      # CMake 工具链 & 构建脚本
│   ├── gcc-arm-none-eabi.cmake #   GCC ARM 交叉编译工具链
│   └── stm32cubemx/            #   CubeMX 生成的 CMake 子工程
├── build/                      # 构建输出 (CMake 生成, gitignore)
├── FR_Hand_s.ioc               # CubeMX 工程文件 (引脚配置 & 时钟树)
├── CMakeLists.txt              # 顶层 CMake 构建定义
├── CMakePresets.json           # CMake 预设配置
├── STM32F407XX_FLASH.ld        # 链接脚本
├── startup_stm32f407xx.s       # 启动文件 (中断向量表 + Reset_Handler)
├── PID.py                      # PID 参数整定辅助脚本
└── Figure_1/2.png              # 调试截图
```

## 构建

```bash
cd SRY_Slave_FreeRTOS
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -G Ninja
cmake --build build/Debug
```

输出: `build/Debug/FR_Hand_s.elf`

依赖: `arm-none-eabi-gcc` (GCC Arm Embedded), `ninja`, `cmake` ≥ 3.22

注意: 此项目**不再使用** Keil MDK-ARM 构建。原始裸机工程 (`SRY_Slave_Keil/`) 仍保留 Keil 工程作为历史参考。

## RTOS 任务架构

```
优先级: Realtime > High > Normal > Low
         ↓         ↓      ↓        ↓
    vMotorControl  vModbus  vSensor  vSystemMonitor
    (10ms PID)    (Modbus) (20ms ADC)  (20ms misc)
```

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| `vMotorControlTask` | `osPriorityRealtime` | 10ms 绝对周期 | 4 路电机级联 PID (位置环→角度环→速度环) + PWM 更新 |
| `vModbusCommTask` | `osPriorityHigh` | 事件驱动 (DMA+IDLE) | Modbus RTU 帧解析, 寄存器读写 (功能码 03/06/16) |
| `vSensorProcessTask` | `osPriorityNormal` | 20ms | ADC 4 通道滑动平均滤波 (20 样本/通道) |
| `vSystemMonitorTask` | `osPriorityLow` | 20ms 绝对周期 | Modbus 帧超时, 心跳, debug printf |

**同步机制**: `xMotorDataMutex` 保护共享的 `motor[]` 状态数组和 `Reg[]` 寄存器数组。控制任务持锁执行完整 PID 周期；通信任务以 10ms 超时尝试获取锁。

## 电机控制模式

模式由 `Reg[4]` 高 8 位 (`Reg[4] >> 8`) 选择:

| 模式 | 说明 |
|---|---|
| 1 | 正向出轴角度控制 (`Reg[i] >> 8` = 目标圈数, 软限位 50 圈) |
| 3 | 反向出轴角度控制 |
| 4 | 紧急停止 + 编码器清零 |
| 5 | 关节角度插值控制 (目标物理角度 → ADC 标定表映射, 40 圈安全限位) |

`Reg[i] & 0xFF` 为速度百分比 (仅模式 1/3 生效)。

## 关键文件

| 文件 | 内容 |
|---|---|
| `Core/Inc/FreeRTOSConfig.h` | `configENABLE_FPU=1` (必须!), tick=1kHz, max priority=56, heap_4 |
| `Core/Inc/Motor.h` | `Motor_Struct`, `PID_Increment_Struct`, `Motor_HW_Config` 定义 |
| `Core/Inc/rs485.h` | `MODBUS` 结构体, `Reg[100]` 寄存器数组, RS485 方向控制宏 |
| `Core/Src/freertos.c` | 4 个 RTOS 任务函数体 + `xMotorDataMutex` 创建 |
| `Core/Src/Motor.c` | `Motor_Control_Loop()` (级联 PID), `Set_Motor()` (PWM 输出), `Motor_Init()` |
| `Core/Src/rs485.c` | `Modbus_Event()`, `Modbus_Func3/6/16()`, UART DMA+IDLE 回调 |
| `Core/Src/main.c` | 硬件初始化顺序, RTOS 启动 |

## 与其他项目的关系

```
SRY_Master_FreeRTOS (主板)  ──RS485──>  FR_Hand_s (本机)   ← 手指从机
     │
     ├── 一个主板管理 2 块从机板:
     │    slave 0x01 = 拇指板 (ban1)
     │    slave 0x02 = 其余四指板 (ban2)
     │    每块从机板有 4 路电机 (FR_Hand_s)
     │
     └── SRY_Slave_Keil/
         og_s/   ← 旧版裸机从机 (保留作为历史参考)
```

本仓库内各分支关系:

| 分支 | 角色 | 状态 |
|---|---|---|
| `SRY_Slave_FreeRTOS` | FreeRTOS 从机驱动板 | **当前主要开发目标** |
| `SRY_Master_Keil` | FreeRTOS 主板 (Modbus/CAN/IR) | 开发中 |
| `SRY_Slave_Keil` | 原始裸机从机 (裸机) | 历史参考 |
| `SRY_Master_Keil` | 上一代主板 (裸机 + CAN) | 历史参考 |
| `CZZ_Slave1` / `CZZ_Master1` | 更早版本 | 归档 |

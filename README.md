# FR_Hand_m — 灵巧手主板 (FreeRTOS)

FreeRTOS 实时操作系统驱动的灵巧手主板固件。运行于 STM32F407VETx (Cortex-M4F)，通过 CAN 总线 / 红外遥控 / Modbus RTU (RS485) 三种方式接收控制指令，经 RS485 总线分发给手指从机板，同时直接控制两个手腕推杆电机。

## 硬件平台

| 项 | 规格 |
|---|---|
| MCU | STM32F407VETx (Cortex-M4F, 168MHz) |
| Flash | 512 KB |
| RAM | 128 KB + 64 KB CCMRAM |
| RTOS | FreeRTOS V10.3.1 (CMSIS-OS V2 API, heap_4) |
| CAN | CAN1 (PA11/PA12), 1 Mbps, ID=0x020(右手) 或 0x021(左手) |
| 上位机通信 | RS485 (USART2, 115200) — Modbus RTU 从机, 地址 0x10 |
| 手指板通信 | RS485 (USART1, 115200) — Modbus RTU 主机 (slave 0x01/0x02) |
| 手腕电机 | USART3 + USART6 (921600) — 推杆电机直驱 |
| 红外遥控 | TIM3 CH3 (PC8) — NEC 协议 |
| 调试输出 | UART4 (115200) — printf |

## 目录结构

```
FR_Hand_m/
├── Core/
│   ├── Inc/
│   │   ├── app_comm_tasks.h     # RTOS 任务 & 动作分发 API
│   │   ├── FreeRTOSConfig.h     # FreeRTOS 内核配置
│   │   └── *.h                  # 外设初始化头 (CubeMX 生成)
│   └── Src/
│       ├── main.c               # 入口: 硬件初始化 → RTOS 调度器启动
│       ├── freertos.c           # 调用 App_Comm_Init() 创建任务
│       ├── app_comm_tasks.c     # 全部 RTOS 任务体 + 动作分发 + CAN 协议
│       └── *.c                  # 外设初始化 (CubeMX 生成)
├── Hardware/                    # 硬件抽象层 (自定义驱动)
│   ├── rs485.c/h                #   Modbus RTU 主从双模 (USART1/2)
│   ├── wrist.c/h                #   手腕推杆电机控制 & S曲线/梯形插补
│   ├── Infrared1838.c/h         #   NEC 红外遥控解码 (TIM3 输入捕获)
│   ├── MyCan.c/h                #   CAN 总线驱动封装
│   ├── motion.c/h               #   手指预置动作数据表 & 播放函数
│   └── rs485_crc.c/h            #   Modbus CRC16 查表
├── Drivers/                     # STM32 HAL 库 + CMSIS
├── Middlewares/                  # FreeRTOS 内核源码
├── cmake/                       # CMake 工具链 & 构建脚本
├── build/                       # 构建输出 (gitignore)
├── FR_Hand_m.ioc                # CubeMX 工程文件
├── CMakeLists.txt               # 顶层 CMake
├── STM32F407XX_FLASH.ld         # 链接脚本
├── startup_stm32f407xx.s        # 启动文件
├── CAN_Protocol.md              # CAN 总线通信协议文档
├── STM32_Startup_Process.md     # STM32 启动流程说明
└── FR_Hand_m_Porting_Status.md  # 裸机→RTOS 移植记录
```

## 构建

```bash
cd FR_Hand_m
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -G Ninja
cmake --build build/Debug
```

输出: `build/Debug/FR_Hand_m.elf`

**左右手切换**：修改 `Core/Src/app_comm_tasks.c` 中的 `CAN_HAND_ID` 宏（`0x020` 右手 / `0x021` 左手）。

## RTOS 任务架构

```
优先级: AboveNormal > Normal > BelowNormal
              ↓           ↓            ↓
         WristMotion   CAN / IR    SlaveDispatch
          (40ms)      ModbusPC     (事件驱动)
```

| 任务 | 优先级 | 周期 | 触发方式 | 职责 |
|---|---|---|---|---|
| `Task_WristMotion` | AboveNormal | 40ms (队列超时) | `Queue_WristCmd` | 手腕摆动/旋转 S 曲线运动 |
| `Task_ModbusPC` | Normal | 事件驱动 | `Sem_Uart2_RxIdle` | 解析 PC Modbus 指令, 更新 `Reg[]` |
| `Task_CAN_Process` | Normal | 事件驱动 | `Queue_CAN_Rx` | CAN 协议解析, 心跳, 直接硬件指令 |
| `Task_IR_Process` | Normal | 事件驱动 | `Sem_IR_Ready` | 红外遥控按键解析 |
| `Task_SlaveDispatch` | BelowNormal | 事件驱动 | `Sem_NewSlaveCmd` | 向手指从板分发 485 指令 |

### 消息流

```
上位机 (CAN/IR/Modbus)
        │
        ▼
CAN → Task_CAN_Process ──→ CAN_Msg_MapToAction() ──→ Action_Dispatch()
IR  → Task_IR_Process  ──→ IR_Key_MapToAction()  ──→        │
Modbus → Task_ModbusPC ─────────────────────────────────────┘
                                                            │
                                          ┌─────────────────┤
                                          ▼                 ▼
                                  WristMotion_Start/Stop   finger_single_action
                                  (Queue_WristCmd)         finger_motion
                                          │
                                          ▼
                                  Task_WristMotion
                                  (updateSmartMotionControl)
```

## CAN 总线协议

详见 [`CAN_Protocol.md`](CAN_Protocol.md)。指令总览：

| data[0] | 功能 | 处理方式 |
|---|---|---|
| `0x00` | 心跳请求/应答 | 直接 I/O |
| `0x10` | 手腕摆动控制 (16位子命令) | Map → Dispatch |
| `0x11` | 手腕旋转控制 | Map → Dispatch |
| `0x12` | 手指连续动作 | Map → Dispatch |
| `0x13/0x14` | 手指单个动作 | Map → Dispatch |
| `0x15/0x16` | 全电机正转/反转 (7字节圈数) | 直接硬件 |
| `0x17` | 编码器清零 | 直接硬件 |
| `0x18` | 推杆电机定位 | 直接硬件 |

## 手腕运动

两个推杆电机 (USART3 + USART6, 921600 bps) 支持两种运动模式：

- **摆动模式 (Pendulum)**: S 曲线速度剖面 (`v = Vmax·sin(π·t/T)`)，左右往复
- **旋转模式 (Rotation)**: 四象限梯形速度曲线 (DOWN→LEFT→UP→RIGHT)

运动周期可通过 CAN `0x10-0x05` / `0x11-0x05` 或 Modbus `Reg[15]` 设置。

## 与其他项目的关系

```
FR_Hand_m (本机)  ──RS485──→  FR_Hand_s × 2 (手指从机)
     │                              ├── slave 0x01 = 拇指板 (3 电机)
     │                              └── slave 0x02 = 四指板 (4 电机)
     │
     ├── CAN  ──→  上位机 / 手柄
     ├── IR   ←──  遥控器 (NEC)
     └── Modbus ←─ PC 调试工具
```

| 目录 | 角色 | 状态 |
|---|---|---|
| `FR_Hand_m/` | FreeRTOS 主板 (CAN/Modbus/IR) | **当前主要开发目标** |
| `FR_Hand_s/` | FreeRTOS 从机驱动板 | 运行中 |
| `WristMaster_can/` | 上一代主板 (裸机 + CAN) | 历史参考 |
| `PalmSalver_V5_lefthand_ban2_newboard/` | 原始裸机从机 (Keil) | 历史参考 |
| `og_s/` / `og_m/` | 更早版本 | 归档 |

## 关键设计决策

- **手腕运动采用消息队列驱动** (`Queue_WristCmd`)：`osMessageQueueGet(40ms 超时)` 同时充当命令通道和 40ms 心跳定时，消除了 `wrist_enabled` 全局变量
- **`finger_motion` 打断用信号量** (`Sem_NewMotionCmd`)：替代了原来只响应红外遥控的 `rcvFlag` 全局变量，IR/CAN/Modbus 任意新指令都能打断正在播放的手指动画
- **CAN 消息分层**：非动作消息（心跳、直接硬件指令）在 `Task_CAN_Process_Entry` 中直接处理；动作消息经 `CAN_Msg_MapToAction()`(纯映射) → `Action_Dispatch()`(纯分发)
- **`pending_slave_dispatch` 已消除**：用 `Sem_NewSlaveCmd` 本身充当分发信号，不再需要额外的全局标志位

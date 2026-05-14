# FR_Hand_m 主板移植记录
claude --resume 1cd79543-3762-4230-9610-554cb018609a
## 项目概述

将裸机前后台架构的 `WristMaster_can` 移植为基于 FreeRTOS 的多任务并发架构 `FR_Hand_m`。主板负责：接收 PC 指令 (Modbus RTU) / 红外遥控 / CAN 总线，向手指驱动板分发 485 指令，控制手腕推杆电机。

**MCU**: STM32F407VETx (Cortex-M4F, 512KB Flash, 128KB RAM)  
**RTOS**: FreeRTOS V10.3.1 + CMSIS-OS V2 API  
**构建**: CMake + Ninja + GCC Arm None Eabi

---

## 工作任务总览

### 1. 硬件驱动层迁移 (Hardware/)

| 模块 | 状态 | 说明 |
|---|---|---|
| `rs485.c/h` | ✅ 已优化 | 消除阻塞 `HAL_Delay(50/100)`，纯事件驱动，批量 `HAL_UART_Transmit` 替代单字节循环 |
| `wrist.c/h` | ✅ 已重写 | 废弃 `HAL_GetTick()` 开环时序，改用步进式插补 (微积分推导 Vmax)，引入 MIN_SPEED 消撞击 |
| `Infrared1838.c/h` | ✅ 已优化 | 砍掉 TIM 溢出中断，利用 uint16 自然溢出去除 upcount；去双重 Buffer；`DumpFrame` 批量拼接 |
| `MyCan.c/h` | ✅ 已精简 | 移除复杂回调绑定，仅保留底层收发和过滤器配置 |
| `rs485_crc.c/h` | ✅ 不变 | Modbus CRC16 查表法，无改动 |
| `motion.c/h` | ✅ 不变 | 手指动作数据表，无改动 |

### 2. RTOS 任务架构 (Core/Src/app_comm_tasks.c)

| 任务 | 优先级 | 触发方式 | 职责 |
|---|---|---|---|
| `Task_WristMotion` | AboveNormal | `Queue_WristCmd` (超时 40ms 兼作心跳) | 手腕摆动/旋转 S 曲线运动 (详见 [手腕运动控制](#手腕运动控制)) |
| `Task_ModbusPC` | Normal | `Sem_Uart2_RxIdle` (UART2 DMA+IDLE 中断释放) | 解析 PC Modbus 指令 |
| `Task_CAN_Process` | Normal | `Queue_CAN_Rx` (CAN RX 中断推入) | 解析 CAN 总线消息 |
| `Task_IR_Process` | Normal | `Sem_IR_Ready` (TIM3 输入捕获中断释放) | 解析红外遥控按键 |
| `Task_SlaveDispatch` | BelowNormal | `Sem_NewSlaveCmd` (ModbusPC 释放) | 向手指板分发 485 指令 (slave 0x01/0x02, 间隔 15ms) |

同步机制：3 个 binary semaphore + 2 个 message queue，中断与任务间无缝对接。

### 3. 输入解耦与统一指令分发

#### 问题
旧代码中 CAN 消息处理通过模拟红外信号实现 (`rcvFlag=1, key_num=IR_*`)，IR 和 CAN 耦合在一起，扩展困难。

#### 方案
定义统一的 `ActionID` 枚举（14 种动作），IR 和 CAN 各自独立映射为 `ActionCmd_t`，汇入 `Action_Dispatch()` 分发：

```
IR 遥控  ──→  IR_Key_MapToAction()   ──┐
                                        ├──→  Action_Dispatch()  ──→  手腕 / 手指 / 系统
CAN 总线 ──→  CAN_Msg_MapToAction()  ──┘
```

- **`IR_Key_MapToAction()`**: NEC 键值 → ActionID 映射 (`app_comm_tasks.c:114`)
- **`CAN_Msg_MapToAction()`**: CAN data[0..3] → ActionID 映射 (`app_comm_tasks.c:197`)
- **`Action_Dispatch()`**: 统一路由 (`app_comm_tasks.c:32`) — 手腕 case 已实现，手指 case 为 TODO 框架

### 4. 中断 → 任务衔接

| 中断源 | ISR | 唤醒方式 | 目标任务 |
|---|---|---|---|
| USART2 (PC Modbus) | `HAL_UARTEx_RxEventCallback` | `Sem_Uart2_RxIdle` | Task_ModbusPC |
| CAN1 RX FIFO0 | `HAL_CAN_RxFifo0MsgPendingCallback` | `Queue_CAN_Rx` | Task_CAN_Process |
| TIM3 CH3 (IR) | `HAL_TIM_IC_CaptureCallback` (Infrared1838.c) | `Sem_IR_Ready` | Task_IR_Process |

### 5. main.c 清理

- ✅ `while(1)` 为空 — 裸机轮询代码全部移除
- ✅ `Infrared_Signal_Processing_Function()` 迁移到 RTOS 任务
- ✅ `MyCan_RxHandler()` 迁移到 RTOS 任务
- ✅ `HAL_UARTEx_ReceiveToIdle_DMA` 移至 `App_Comm_Init()` 信号量创建之后
- ✅ TIM2 ISR 中的手腕运动逻辑迁移到 `Task_WristMotion`（TIM2 不再承载业务代码）

---

## 手腕运动控制

### 设计思路

旧架构中手腕运动依赖 TIM2 硬件中断每 40ms 直接调用 `updateSmartMotionControl()`，启停通过全局变量 `wrist_enabled` / `wrist_enabled_constand` 控制。RTOS 移植过程中经历了三个迭代：

#### 迭代 1：信号量 + 全局标志位轮询

`Sem_WristRun` 信号量控制启停，任务在 `while(wrist_enabled || wrist_enabled_constand)` 内循环，外部通过设置/清除这两个全局变量来控制。

**问题**：`wrist_enabled` / `wrist_enabled_constand` 仍是跨文件的全局变量，Modbus、CAN、IR 三个输入源各自直接操作它们，状态分散、难以追踪。

#### 迭代 2（当前方案）：消息队列驱动

用 `Queue_WristCmd` 消息队列替代信号量和全局标志位。`osMessageQueueGet(..., 40)` 的超时机制同时承担两个职能：

1. **命令通道** — `WristMotion_Start(mode)` / `WristMotion_Stop()` 向队列投递 `WristCmd_t` 消息
2. **心跳定时器** — 40ms 超时本身就是运动节拍，不再需要 `osDelayUntil`

启停状态 `run_mode` 是任务栈上的局部变量，不再暴露为全局变量。

```
PC (Modbus Reg[14]) ──→  modbus2_Event()     ──┐
CAN (data[0]=0x10/11)──→  Action_Dispatch()   ──┼──→  WristMotion_Start(mode)
IR  (按键 1/2)       ──→  Action_Dispatch()   ──┘        │
                                                  osMessageQueuePut(Queue_WristCmd, msg)
                                                          │
                                                          ▼
                                         Task_WristMotion: osMessageQueueGet(..., 40)
                                                          │
                                              ┌─ timeout (40ms) → if running → updateSmartMotionControl()
                                              │
                                              └─ got message → update run_mode / stop / reset state
```

### 使用方法

#### 启动手腕运动

```c
// 摆动模式 (pendulum: 左右往复 S 曲线)
WristMotion_Start(MODE_PENDULUM);

// 旋转模式 (rotation: 四象限梯形速度曲线)
WristMotion_Start(MODE_ROTATION);
```

**各输入源的调用路径**：

| 输入源 | 触发条件 | 调用链 |
|---|---|---|
| PC Modbus | `Reg[14]/256 == 1` | `modbus2_Event()` → `WristMotion_Start(MODE_PENDULUM)` |
| CAN | `data[0]==0x10 && data[2]==0x01` | `CAN_Msg_MapToAction()` → `Action_Dispatch()` → `WristMotion_Start(MODE_PENDULUM)` |
| CAN | `data[0]==0x11 && data[2]==0x01` | `CAN_Msg_MapToAction()` → `Action_Dispatch()` → `WristMotion_Start(MODE_ROTATION)` |
| IR | 按键 1 | `IR_Key_MapToAction()` → `Action_Dispatch()` → `WristMotion_Start(MODE_PENDULUM)` |
| IR | 按键 2 | `IR_Key_MapToAction()` → `Action_Dispatch()` → `WristMotion_Start(MODE_ROTATION)` |

#### 停止手腕运动

```c
WristMotion_Stop();  // 发送 WRIST_CMD_STOP 消息，任务回到 idle 状态
```

所有 stop 路径（IR_8、CAN data[2]=0x00、`ACTION_ALL_STOP` 等）都汇入此函数。重复停止是幂等的，不会产生副作用。

#### 预设位置（一次性，非周期性运动）

CAN `data[2]=0x02/0x03/0x04` 和 `ACTION_WRIST_POS_PRESET1/2/3` 直接调用 `set_wrist_motion_Can()`，这是一次性电机定位指令，不走 40ms 循环。

#### 调整运动周期

PC 上位机通过 Modbus 修改 `Reg[15]`（单位 ms），在 `modbus2_Event()` 中写入 `cycle_time` 变量。下一次 `WristMotion_Start` 会携带最新的 `cycle_time` 值发送给任务。

### 运动学算法 (wrist.c)

#### 摆动模式 (Pendulum)

- S 曲线速度剖面：`v(t) = V_max · sin(π · t / T)`
- 速度从两端向中间递增，端点速度降至 MIN_SPEED (100)，消除换向机械撞击
- 运动学推导：$V_{max} = \frac{S · π}{2T}$（保证位移 S=2000 steps 与时间 T 的精确匹配）
- 单程步数 = `half_cycle_ms / 40`，每步递增 `motion_step`

#### 旋转模式 (Rotation)

- 四状态循环：DOWN → LEFT → UP → RIGHT
- 梯形速度曲线：前 20% 加速，中间 60% 匀速，后 20% 减速
- 1.2× 速度补偿系数抵消加减速段的位移损失
- 每个状态耗时 = `cycle_time / 4`，状态切换时 `motion_step` 回零

#### 关键常量

| 宏 | 值 | 含义 |
|---|---|---|
| `TIM2_PERIOD_MS` | 40 | 运动步进周期 (ms) |
| `MAX_POSITION` | 0x07D0 (2000) | 推杆最大行程 (对应 16mm) |
| `MAX_SPEED` | 0x0FA0 (4000) | 最大速度 (对应 32mm/s) |
| `MIN_SPEED` | 0x0064 (100) | 最小速度阈值，防止死区 |

### 文件清单

| 文件 | 职责 |
|---|---|
| `Core/Inc/app_comm_tasks.h:51-60` | `WristCmdType` / `WristCmd_t` 定义 |
| `Core/Src/app_comm_tasks.c:255-325` | `WristMotion_Start/Stop` + `Task_WristMotion_Entry` |
| `Hardware/wrist.c` | `updateSmartMotionControl()` + S 曲线/梯形插补算法 + `set_wrist_motion_Can` |
| `Hardware/rs485.c:108-116` | `modbus2_Event` 中通过 `WristMotion_Start/Stop` 响应 Reg[14] |

---

## CAN 总线通信协议

### 心跳检测

专用 CAN ID `0x001`，用于上位机检测主板在线状态：

```
上位机 → 手:  ID=0x001, data[0]=0x00 (请求)
手 → 上位机:  ID=0x001, data[0]=0x01 (应答)
```

实现位置：`Task_CAN_Process_Entry` 在进入动作分发前拦截心跳消息，不进入 `Action_Dispatch`。

### 手腕/手指控制帧 (ID = 0x020)

所有运动控制指令使用同一 CAN ID `0x020`，由 `data[0]` 区分功能大类。

#### 0x10 — 手腕摆动控制 (pendulum)

| data[1] | 含义 | 其他字段 |
|---|---|---|
| `0x00` | 停止连续摆动 | — |
| `0x01` | 启动连续摆动 (S 曲线往复) | — |
| `0x02` | 单次动作：最左侧 | — |
| `0x03` | 单次动作：最右侧 | — |
| `0x04` | 单次动作：回中 | — |
| `0x05` | 设置摆动周期 | `data[2..3]` = `cycle_ms` (uint16 大端，单位 ms) |

#### 0x11 — 手腕旋转控制 (rotation)

| data[1] | 含义 | 其他字段 |
|---|---|---|
| `0x00` | 停止连续旋转 | — |
| `0x01` | 启动连续旋转 (四象限梯形速度) | — |
| `0x02` | 预设位置 1 | — |
| `0x03` | 预设位置 2 | — |
| `0x04` | 预设位置 3 | — |
| `0x05` | 设置旋转周期 | `data[2..3]` = `cycle_ms` (uint16 大端，单位 ms) |

**注意**：`data[1]` 为子命令，与旧协议 (`data[2]` 为子命令) 不同。

#### 0x12 — 手指连续动作

| data[2] | 含义 | data[3] |
|---|---|---|
| `0x00` | 全部停止 | — |
| `0x01` | 连续弯折 | 循环次数 |
| `0x02` | 数数字 (0-9) | 循环次数 |
| `0x03` | 连续握拳 | 循环次数 |

#### 0x13 — 手指单个动作

`data[2]` = 手势编号 (0x00-0x0B)，直接调用 `finger_single_action()`。

#### 0x14 — 全部停止

停止手腕 + 停止手指，无子命令。

### 调用链

```
CAN 帧 (ID=0x020)
  │
  ├── ID=0x001 (心跳) → 直接回复, 不进 Action_Dispatch
  │
  └── ID=0x020 (控制)
        │
        CAN_Msg_MapToAction()   ← data[0..3] → ActionCmd_t
        │
        Action_Dispatch()
          ├── WristMotion_Start/Stop()    (0x10-0x01/0x00)
          ├── set_wrist_motion_Can()      (0x10-0x02/0x03/0x04)
          └── cycle_time / cycle_time_R   (0x10/0x11-0x05)
```

---

## 待完成工作

### [ ] 填充 Action_Dispatch() 手指业务逻辑
`app_comm_tasks.c:62-80`，手指连续动作和单个动作的 case 为 TODO 框架，具体实现已用注释标注：
- `ACTION_FINGER_COUNT/BEND/GESTURE/FIST` → `finger_motion()`
- `ACTION_FINGER_SINGLE` → `finger_single_action()`

### [ ] ACTION_ALL_STOP 手指停止
当前仅调用了 `WristMotion_Stop()`，手指停止的 `Host_write16_slave` 逻辑仍为注释。

### [ ] IR 方向键逻辑
IR_UP/Down/Left/Right 已预留 case，用于切换单手势编号或微调手腕位置，具体逻辑待定。

### [ ] IR_Xing 单手势编号来源
`IR_Key_MapToAction` 中 IR_Xing 的 `cmd->param1` 需要从全局状态获取（配合方向键选择或上次 CAN 指令）。

### [ ] 上机联调
- PC → 主板 → 手指从板 485 链路测试
- 手腕插补运动顺滑度验证
- 并发测试：手腕运动 + 高频 485 指令同时下发
- CAN 总线指令测试 (ID=0x020 全部 5 种 func)
- 红外遥控按键测试

---

## 编译

```bash
cd FR_Hand_m
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -G Ninja
cmake --build build/Debug
```

---

**最后更新**: 2026-05-07

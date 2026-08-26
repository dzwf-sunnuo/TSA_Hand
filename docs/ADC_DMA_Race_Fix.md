# ADC DMA 竞态修复与采样优化记录

> 日期: 2026-07-21
> 问题: `Print_Motor_Status(0, 3)` 打印的 ADC 原始值在 ~1243 ↔ ~1494 之间剧烈跳动，但示波器观测模拟波形稳定。

---

## 问题诊断

### 数据流路径

```
ADC硬件 → DMA(循环模式) → ADC_NativeValue[60] (SRAM)
    → ISR (HAL_ADC_ConvCpltCallback) 拷贝 → ADC_Snapshot[60] (CCM)
    → vSensorProcessTask 拷贝 → local[60] → 15样本均值 → ADC_HallValue[4]
    → vMotorControlTask → motor[i].CurrentPosition = ADC_HallValue[i]
    → vSystemMonitorTask (每100ms) → Print_Motor_Status(0, 3) → printf
```

### 根因分析

#### 问题 1 (P0): DMA 循环模式下的 ISR 竞态条件

**涉及文件**: `Core/Src/adc.c`, `Core/Src/freertos.c`

**原因**:

`adc.c` 中 ADC1 配置了 `DMA_CIRCULAR` + `ContinuousConvMode = ENABLE` + `DMAContinuousRequests = ENABLE`。DMA 每 60 次传输（约 43µs）触发一次 TC（传输完成）中断，但**硬件立即回绕到缓冲区头部继续写入**，DMA 永不停止。

ISR `HAL_ADC_ConvCpltCallback` 在 TC 中断中执行：

```c
// 竞态: DMA 已在回绕写入, ISR 同时读取 —— 两个操作在同一个缓冲区上并发
for (int i = 0; i < 60; i++) {
    ADC_Snapshot[i] = ADC_NativeValue[i];  // 读到一半新一半旧的数据
}
```

**后果**: `ADC_Snapshot` 包含新旧两个 DMA 周期的混合数据。由于 4 通道交错排列（CH0, CH1, CH2, CH3 循环），当中断延迟导致偏移量 mod 4 ≠ 0 时，**通道数据发生错位**——CH0 的样本中混入了 CH1/CH2/CH3 的数据。

**这解释了 ~1243 ↔ ~1494 的交替跳动**: 这两个值分别是不同关节霍尔传感器的真实读数，ISR 中断延迟的抖动导致通道时而错位、时而不错位。

#### 问题 2 (P1): ADC 采样时间不足

**涉及文件**: `Core/Src/adc.c`

**原因**: 4 个 ADC 通道均配置为 `ADC_SAMPLETIME_3CYCLES`（约 143ns @ 21MHz ADC 时钟）。对于霍尔传感器（经分压网络后输出阻抗可达 10-50kΩ），采样保持电容（约 8pF）的 RC 建立时间约 0.08-0.4µs/τ，需要 8-10τ 才能达到 12 位精度。

3 个周期仅提供 ~143ns 的采样窗口，对高阻抗信号源严重不足，导致采样值存在额外噪声。

#### 其他发现

- **缺少 ADC 自校准**: 全项目未调用 `HAL_ADCEx_Calibration_Start()`，STM32F4 ADC 上电后存在固有偏移
- **任务未正确停止 DMA**: `vSensorProcessTask` 在正常路径中从不调用 `HAL_ADC_Stop_DMA`，仅在超时时停止。DMA 循环运行中重复调用 `HAL_ADC_Start_DMA`，HAL 内部状态不可预测

---

## 修复方案

### 修复 1: DMA 正常模式 + 按需启停

将 DMA 从循环模式改为正常模式（单次传输后自动停止），消除 ISR 与 DMA 的竞态。

| 参数 | 修改前 | 修改后 | 说明 |
|------|--------|--------|------|
| `DMAContinuousRequests` | `ENABLE` | **`DISABLE`** | DMA 正常模式，传完即停 |
| `DMA Mode` | `DMA_CIRCULAR` | **`DMA_NORMAL`** | 静态初始化保持一致 |
| `ContinuousConvMode` | `ENABLE` | `ENABLE` (保持) | ADC 持续扫描 4 通道，由 DMA 控制传输次数 |
| 任务 Stop_DMA | 无 | **新增** | 数据处理后调用，为下一轮清状态 |

**新的数据流**（无竞态）:

```
vSensorProcessTask (每 10ms)
  │
  ├── HAL_ADC_Start_DMA    ← 启动单次采集 (ADC 持续扫描, DMA 传 60 次)
  │
  ├── osSemaphoreAcquire   ← 阻塞等待 DMA 完成 (~274µs)
  │       │
  │       └── HAL_ADC_ConvCpltCallback (ISR)
  │             DMA 已自动停止, ADC_NativeValue[] 稳定
  │             ADC_Snapshot ← ADC_NativeValue  (安全拷贝, 无并发写入)
  │             osSemaphoreRelease(Sem_ADC_Done)
  │
  ├── 均值滤波              ← 处理快照数据
  │
  ├── HAL_ADC_Stop_DMA     ← 停止 ADC, 清状态, 为下一轮准备
  │
  └── osDelay(10)          ← 10ms 后下一轮
```

**文件改动**:

| 文件 | 行号 | 改动 |
|------|------|------|
| `Core/Src/adc.c` | 57 | `DMAContinuousRequests = DISABLE` |
| `Core/Src/adc.c` | 186 | `DMA_CIRCULAR` → `DMA_NORMAL` |
| `Core/Src/freertos.c` | 313 | 新增 `HAL_ADC_Stop_DMA(&hadc1)` |

### 修复 2: 增加 ADC 采样时间

| 参数 | 修改前 | 修改后 | 提升 |
|------|--------|--------|------|
| `SamplingTime` (4通道) | `ADC_SAMPLETIME_3CYCLES` (0.14µs) | **`ADC_SAMPLETIME_84CYCLES`** (4.0µs) | **28×** |

84 个 ADC 时钟周期（@21MHz = 4µs）对输出阻抗高达 ~50kΩ 的信号源仍能提供约 10τ 的建立时间，满足 12 位精度要求。

**性能影响**: 单次 DMA 采集时间从 ~43µs 增加到 ~274µs，仍在 10ms 任务周期的 3% 以内，零实际性能代价。

**文件改动**:

| 文件 | 行号 | 改动 |
|------|------|------|
| `Core/Src/adc.c` | 68 (×4) | `ADC_SAMPLETIME_3CYCLES` → `ADC_SAMPLETIME_84CYCLES` |

### 附带改动: 调试输出改为关节角度

`Print_Motor_Status(0, 3)` 之前直接打印 ADC 原始值，不直观。改为通过标定表 `ADValue[4][2]` 线性换算为 0-90° 关节角度：

```
换算公式: 角度 = (ADC值 - ADC_0°) / (ADC_90° - ADC_0°) × 90°

输出格式: Motor 1 Target: 45.0 deg, Actual: 32.5 deg (ADC: 1835)
```

自动适配正反方向（关节 2-4 的 ADC 值随角度增大而减小，分母为负自动处理）。括号内保留 ADC 原始值便于验证采集稳定性。

| 文件 | 函数 | 改动 |
|------|------|------|
| `Core/Src/Motor.c` | `Print_Motor_Status` param=3 | ADC 值 → 0-90° 角度显示 |

---

## 完整文件改动清单

| 文件 | 改动项 | 类型 |
|------|--------|------|
| `Core/Src/adc.c:51` | `ContinuousConvMode` 注释说明 | 注释 |
| `Core/Src/adc.c:57` | `DMAContinuousRequests = DISABLE` | **修复 P0** |
| `Core/Src/adc.c:68` (4处) | `ADC_SAMPLETIME_84CYCLES` | **修复 P1** |
| `Core/Src/adc.c:186` | `DMA_NORMAL` | **修复 P0** |
| `Core/Src/freertos.c:270-275` | 任务注释更新 | 注释 |
| `Core/Src/freertos.c:313` | 新增 `HAL_ADC_Stop_DMA(&hadc1)` | **修复 P0** |
| `Core/Src/freertos.c:378-384` | 回调注释更新 | 注释 |
| `Core/Src/Motor.c:511-520` | `Print_Motor_Status` param=3 显示角度 | 调试辅助 |

---

## 后续建议

1. **ADC 自校准**: 在 `MX_ADC1_Init()` 之后添加 `HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)` 以消除固有偏移
2. **采样时间可进一步加大**: 如果仍有轻微跳动，可尝试 `ADC_SAMPLETIME_144CYCLES` (6.9µs)
3. **裁剪均值滤波**: 如果 PWM 耦合噪声仍然存在，可将当前简单均值改为裁剪均值（去掉最大/最小若干个样本），参见 `docs/ADC_Filter_Optimization.md`
4. **PWM 同步触发**: 使用定时器触发 ADC 在 PWM 开关沿之间采样，从根源上避免 EMI 干扰

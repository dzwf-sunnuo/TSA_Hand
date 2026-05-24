# ADC 采集与滤波优化记录

## 原始方案

### 数据流

```
ADC1 DMA 循环模式 (80 半字, 4ch×20samples)
        │
        ▼
ADC_NativeValue[80]  ← DMA 持续写入 (无完成通知)
        │
        │  vSensorProcessTask 每 20ms 固定周期读取 (osDelay)
        ▼
  简单算术平均: sum(20) / 20 → ADC_HallValue[4]
```

### 原始参数

| 项 | 值 | 问题 |
|---|---|---|
| ADC 采样时间 | `ADC_SAMPLETIME_3CYCLES` (0.14μs) | 对外部霍尔传感器偏短, 采样电容未充分充电 |
| 缓冲区 | 单缓冲 `ADC_NativeValue[80]`, 无快照 | DMA 写与任务读存在竞态 |
| 滤波算法 | 20 样本简单算术平均 | 对 PWM 脉冲干扰无免疫力, 一个尖峰拉偏平均值 |
| 任务触发 | `osDelay(20)` 固定周期 | 不与 DMA 节奏同步, 样本数不确定 |

### 关键问题

1. **DMA 竞态**: 任务在任意时刻读取缓冲区，DMA 同时在循环写入，读到一半被覆盖 → 数据不一致
2. **脉冲敏感**: 电机 PWM 开关噪声走长线耦合到霍尔传感器，单个 4095 的脉冲就能把平均值拉偏 200 个 LSB
3. **采样时间**: 3 个 ADC 时钟周期远低于外部高阻抗信号源的建立时间，采样值不准确

---

## 优化方案

### 1. ADC 采样时间: 3 → 28 周期

```c
// adc.c:67 — 仅改一个宏, 4 通道继承同一配置
sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;  // was: 3CYCLES
```

采样时间从 ~0.14μs 提升到 ~1.3μs。对 4 通道扫描模式，总扫描时间从 ~4.6μs 增加到 ~6.8μs，远小于 10ms PID 周期，零性能代价。

### 2. DMA 单次模式: 任务按需启停

DMA 改为**单次模式** (非循环)。任务每 10ms 启动 DMA → 等待完成 → 处理 → 停止。
DMA 仅在采集期间运行 (~152μs), 其余 99.85% 时间 ADC/DMA 空闲。

```
vSensorProcessTask (每 10ms)
  │
  ├── HAL_ADC_Start_DMA    ← 启动单次采集
  │
  ├── osSemaphoreAcquire   ← 阻塞等待 DMA 完成 (~152μs)
  │       │
  │       └── HAL_ADC_ConvCpltCallback
  │             快照 ADC_Snapshot ← ADC_NativeValue
  │             osSemaphoreRelease(Sem_ADC_Done)
  │
  ├── 裁剪均值滤波          ← 处理快照数据
  │
  └── osDelay(10)          ← 等 10ms 后下一轮
```

**对比**:

| | 循环模式 | 单次按需模式 |
|---|---|---|
| ADC 运行时间 | 100% (持续) | ~1.5% (152μs / 10ms) |
| DMA 中断频率 | 6.5 kHz | 100 Hz |
| 信号量释放频率 | — | 100 Hz (合理) |
| ADC 功耗 | 持续 | 降低 ~98% |

源文件改动: `adc.c` 设 `DMAContinuousRequests = DISABLE`; `main.c` 移除 `HAL_ADC_Start_DMA`; `freertos.c` 任务改为启动→等待→处理→延时。

### 3. 裁剪均值滤波 (Trimmed Mean)

**算法**: 每通道 20 个样本排序 → 去掉最小 4 个 + 最大 4 个 → 中间 12 个取平均

```c
// 每 DMA 周期 (80 samples / 4ch)
for (ch = 0; ch < 4; ch++) {
    sorted[0..19] = ADC_Snapshot[ch], ADC_Snapshot[ch+4], ...
    sort(sorted);                    // 冒泡 20 元素
    ADC_HallValue[ch] = mean(sorted[4..15]);  // 裁剪 20% 首尾 → 均值
}
```

**抗噪声原理**: 

- 脉冲干扰（PWM 耦合）通常表现为 1~2 个极大值 → 落入被丢弃的 top 4
- 地弹/共模噪声可能产生极小值 → 落入被丢弃的 bottom 4  
- 中间 60% (12/20) 样本代表真实的物理信号

**计算量**: 20 元素冒泡排序 ~190 次比较，4 通道 ~760 次比较，在 168MHz Cortex-M4F 上 < 50μs。

与简单平均对比:

| | 简单平均 | 裁剪均值 |
|---|---|---|
| 单脉冲尖峰 (4095) 的影响 | +205 LSB (不可接受) | 0 (尖峰在 top-4 被丢弃) |
| 连续 2 个尖峰 | +410 LSB | 0 |
| 连续 5 个尖峰 | +1024 LSB | 第 5 个进入 middle-12, +34 LSB |
| 无噪声时精度 | 与样本数平方根成正比 | 略低于简单平均 (样本数少 40%) |

---

## 文件改动清单

| 文件 | 改动 |
|---|---|
| `Core/Src/adc.c:56` | `DMAContinuousRequests = DISABLE` (单次模式) |
| `Core/Src/adc.c:67` | `ADC_SAMPLETIME_3CYCLES` → `ADC_SAMPLETIME_28CYCLES` |
| `Core/Src/main.c:118` | 移除 `HAL_ADC_Start_DMA` (改由任务启动) |
| `Core/Src/freertos.c` | 新增 `ADC_Snapshot[80]`, `Sem_ADC_Done`, `HAL_ADC_ConvCpltCallback` |
| `Core/Src/freertos.c` `vSensorProcessTask` | 任务启动 DMA → 等信号量 → 裁剪均值 → osDelay(10) |

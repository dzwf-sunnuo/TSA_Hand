# 二号板（MODBUS 0x02） CCMRAM数据分配说明

## 本次调整

沿用已有`__CCM_RAM_DATA`宏和`.ccmram`段，仅迁移CPU访问的数据；控制算法、MODBUS寄存器、CAN帧和采集周期保持原配置。

| 文件 | 新增迁入CCMRAM的数据 |
|---|---|
| Core/Src/sensor_feedback.c | 完整反馈缓存、物理传感器句柄数组、重试计时/标志、ADC批次及完成状态 |
| Core/Src/can_telemetry.c | CAN统计、任务句柄、待发邮箱掩码/时间、初始化时刻、启动/Bus-Off/序号/当前频率 |
| Core/Src/Tactile_Sensor.c | 已注册传感器指针表及数量 |
| Core/Src/Motor.c | ADC滤波结果、堵转计时及目标记录 |
| Core/Src/rs485.c | PID/导纳重启/实验事件标志、LED计数；原有Reg数组统一使用既有宏 |
| Core/Src/power_loss.c | 掉电状态与消抖计数，非零上电初值1保留 |
| Core/Src/freertos.c | 应用任务/同步对象句柄、采集超期计数、空闲和软件定时器任务的静态存储 |

保持中指、无名指、小指原有独立导纳映射。

已有CCMRAM中的电机/PID/导纳数组、标定表、ADC_Snapshot、控制任务栈及CAN任务栈保留原分配。只读常量和映射表保留Flash位置，不迁入CCMRAM浪费空间。外设HAL句柄与第三方源码保持原样。

## 空闲与定时器任务

在freertos.c的USER CODE区域实现同名应用层强定义，覆盖CMSIS_RTOS_V2/cmsis_os2.c中的弱定义，不修改中间件：

- vApplicationGetIdleTaskMemory：静态控制块与configMINIMAL_STACK_SIZE个StackType_t栈元素放入CCMRAM。
- vApplicationGetTimerTaskMemory：静态控制块与configTIMER_TASK_STACK_DEPTH个StackType_t栈元素放入CCMRAM。

当前分别为128和256个32位栈元素，合计1536字节（1.5 KiB），另有两个控制块。栈显式按portBYTE_ALIGNMENT=8对齐，深度、回调接口和任务优先级均不变。这是源码声明的栈容量，不等同于本次构建已验证的SRAM净节省数。

应用层动态创建的通信、采集、监控任务继续使用普通SRAM动态堆；本次不改configTOTAL_HEAP_SIZE=15360，也不将整个堆迁入CCMRAM。对象句柄迁移不意味着其动态分配的对象本体也随之迁移。

## DMA边界

STM32F407的CCMRAM只能由CPU访问，不在DMA可访问的总线矩阵内，依据[ST RM0090参考手册](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。

必须保留在普通SRAM的数据：

- freertos.c的ADC_NativeValue[60]：HAL_ADC_Start_DMA的实际目的地址。
- rs485.c的整个modbus结构体：其rcbuf成员是HAL_UARTEx_ReceiveToIdle_DMA的接收地址，不能给整个结构体添加CCMRAM属性。
- 普通动态堆、通信任务栈，以及其他可能交给DMA的外设缓冲。

SensorFeedback的成品快照、滤波结果和ADC完成标志由CPU复制或中断更新；DMA硬件不读写这些变量。传感器句柄虽包含SPI/GPIO指针，其本身仅由HAL和CPU访问。当前SPI收发使用阻塞轮询，不使用SPI DMA。CAN HAL由CPU写邮箱寄存器，CAN状态统计可放CCMRAM。

以后若改为SPI DMA或让定时器回调启动DMA，必须重新检查传入缓冲的实际地址，不能直接把CCMRAM变量或CCMRAM任务栈上的数组交给DMA。

## 初始化和容量边界

链接脚本已有64 KiB CCMRAM区域：起址0x10000000。`.ccmram`仍为`>CCMRAM AT>FLASH`，startup_stm32f407xx.s在进入main前按_siccmram、_sccmram、_eccmram复制初值。因此新增静态变量的零初值及上电状态1、CAN默认频率20等非零初值沿用启动初始化流程。

本次不新增NOLOAD段，也不修改链接脚本或启动汇编。现有方式会为CCMRAM变量保留Flash初值镜像，零初值静态栈也可能增加镜像大小；这是用现有机制迁移数据的代价，不宣称同时节省Flash。

CCMRAM为数据RAM，不能用于执行STM32F407指令；main.h中保留旧TEXT宏以兼容旧接口，并补充禁止用于函数的说明。本项目实际迁移只使用DATA宏。

## 核对状态

- 已阅读两块板的声明、DMA传参、链接段、启动复制流程和CMSIS弱回调定义，并进行差异/格式静态检查。
- 已核对搬移对象不包含ADC_NativeValue或modbus，不改变两板各自的电机映射、传感器数量和CAN布局。
- 按用户项目要求，本次不编译、不运行测试、不烧录；旧build目录及map仅为历史产物，不能作为本次地址分配或占用量证明。
- 后续编译时，以新map/ELF确认搬移符号位于0x10000000～0x1000FFFF，ADC_NativeValue和modbus位于普通SRAM，并检查CCMRAM占用未超64 KiB。

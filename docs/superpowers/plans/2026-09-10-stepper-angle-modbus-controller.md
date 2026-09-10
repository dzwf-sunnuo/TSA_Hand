# 步进电机角度跟随上位机 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建一个 Windows Tkinter 上位机，通过串口跟随或手动输入角度，以 Modbus RTU 定点位置模式驱动步进电机。

**Architecture:** 单个 Python 程序包含纯协议函数、单线程串口控制器和 Tkinter 界面。所有串口操作在一个后台线程内串行执行，界面通过命令队列下发动作、通过事件队列接收状态，从而保证 Modbus 同时只有一个未完成请求且 Tkinter 只在主线程更新。

**Tech Stack:** Python 3.10+、Tkinter、pyserial、pytest、Modbus RTU CRC16。

**Spec:** `docs/superpowers/specs/2026-09-10-stepper-angle-modbus-controller-design.md`

## Global Constraints

- 两个串口默认均为 `115200 bit/s、8N1`，并且不得选择同一个 COM 口。
- 串口输入仅接受去除外围空白后符合 `数字.两位小数` 且位于 `0.00～90.00` 的有限值。
- `0.00°～90.00°` 端点精确映射到 `500～90000` 步，采用四舍五入。
- 位置请求只写 `0x0010～0x0014` 五个寄存器，速度为 `2000`，加速时间为 `250`，不写 `0x0015`。
- 异常输入不得产生 Modbus 请求；急停后必须停止串口跟随。
- 所有函数、方法和类上方必须使用项目规定的中文说明格式，关键逻辑添加中文注释，代码标识符保持英文。
- 不修改当前 STM32 固件文件和已有未提交内容。

---

### Task 1: 角度解析、映射和 Modbus 请求帧

**Files:**
- Create: `tools/stepper_angle_controller.py`
- Create: `tests/test_stepper_angle_controller.py`

**Interfaces:**
- Consumes: 原始串口字节、手动输入字符串、角度浮点数、Modbus 从站地址。
- Produces: `parse_stream_angle(raw: bytes) -> float | None`、`parse_manual_angle(text: str) -> float`、`angle_to_step(angle: float) -> int`、`calculate_modbus_crc(data: bytes) -> bytes`、`build_position_request(slave_address: int, target_step: int) -> bytes`、`build_emergency_stop_request(slave_address: int) -> bytes`、`format_hex(data: bytes) -> str`。

- [ ] **Step 1: 写角度解析和映射失败测试**

```python
import math

import pytest

from tools.stepper_angle_controller import (
    angle_to_step,
    parse_manual_angle,
    parse_stream_angle,
)


@pytest.mark.parametrize(
    ("raw", "expected"),
    [(b"0.00\n", 0.0), (b"45.25\r\n", 45.25), (b"90.00 \n", 90.0)],
)
# test_parse_stream_angle_accepts_valid_lines：验证串口两位小数角度解析
# 参数：raw - 原始串口字节；expected - 期望角度
# 返回值：无
def test_parse_stream_angle_accepts_valid_lines(raw: bytes, expected: float) -> None:
    assert parse_stream_angle(raw) == expected


@pytest.mark.parametrize(
    "raw",
    [b"", b"45\n", b"45.2\n", b"45.250\n", b"NaN\n", b"inf\n", b"-1.00\n", b"90.01\n", b"abc\n", b"\xff\n"],
)
# test_parse_stream_angle_rejects_invalid_lines：验证异常串口行被忽略
# 参数：raw - 非法串口字节
# 返回值：无
def test_parse_stream_angle_rejects_invalid_lines(raw: bytes) -> None:
    assert parse_stream_angle(raw) is None


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 500), (45.0, 45250), (90.0, 90000)],
)
# test_angle_to_step_maps_endpoints：验证角度线性映射
# 参数：angle - 输入角度；expected - 期望步数
# 返回值：无
def test_angle_to_step_maps_endpoints(angle: float, expected: int) -> None:
    assert angle_to_step(angle) == expected


# test_manual_angle_validation：验证手动输入范围
# 参数：无
# 返回值：无
def test_manual_angle_validation() -> None:
    assert parse_manual_angle("45.5") == 45.5
    with pytest.raises(ValueError, match="0.00～90.00"):
        parse_manual_angle(str(math.nan))
```

- [ ] **Step 2: 运行角度测试并确认因模块不存在而失败**

Run: `python -m pytest tests/test_stepper_angle_controller.py -v`

Expected: FAIL，错误包含 `ModuleNotFoundError` 或缺少待实现函数。

- [ ] **Step 3: 实现角度解析和端点精确映射**

在 `tools/stepper_angle_controller.py` 中加入常量和纯函数：

```python
import math
import re
from decimal import Decimal, ROUND_HALF_UP


MIN_ANGLE = Decimal("0.00")
MAX_ANGLE = Decimal("90.00")
MIN_STEP = 500
MAX_STEP = 90000
STREAM_ANGLE_PATTERN = re.compile(r"^\d{1,2}\.\d{2}$")


# parse_stream_angle：解析串口发送的两位小数角度
# 参数：raw - 包含一行角度的原始字节
# 返回值：float或None - 合法角度，非法时返回None
def parse_stream_angle(raw: bytes) -> float | None:
    try:
        text = raw.decode("ascii").strip()
    except UnicodeDecodeError:
        return None
    if not STREAM_ANGLE_PATTERN.fullmatch(text):
        return None
    angle = float(text)
    if not math.isfinite(angle) or not 0.0 <= angle <= 90.0:
        return None
    return angle


# parse_manual_angle：校验界面手动输入的角度
# 参数：text - 用户输入文本
# 返回值：float - 合法角度
def parse_manual_angle(text: str) -> float:
    try:
        angle = float(text.strip())
    except ValueError as error:
        raise ValueError("角度必须是0.00～90.00之间的数字") from error
    if not math.isfinite(angle) or not 0.0 <= angle <= 90.0:
        raise ValueError("角度必须位于0.00～90.00")
    return angle


# angle_to_step：把关节角度线性映射为目标步数
# 参数：angle - 0～90度角度
# 返回值：int - 500～90000目标步数
def angle_to_step(angle: float) -> int:
    if not math.isfinite(angle) or not 0.0 <= angle <= 90.0:
        raise ValueError("角度必须位于0.00～90.00")
    mapped = Decimal(MIN_STEP) + Decimal(str(angle)) / MAX_ANGLE * Decimal(MAX_STEP - MIN_STEP)
    target = int(mapped.quantize(Decimal("1"), rounding=ROUND_HALF_UP))
    return min(MAX_STEP, max(MIN_STEP, target))
```

- [ ] **Step 4: 写 CRC 和请求帧失败测试**

```python
from tools.stepper_angle_controller import (
    build_emergency_stop_request,
    build_position_request,
    calculate_modbus_crc,
    format_hex,
)


# test_crc_known_vector：用手册状态读取帧验证CRC顺序
# 参数：无
# 返回值：无
def test_crc_known_vector() -> None:
    assert calculate_modbus_crc(bytes.fromhex("01 03 00 00 00 01")) == bytes.fromhex("84 0A")


@pytest.mark.parametrize(
    ("target_step", "expected_hex"),
    [
        (500, "01 10 00 10 00 05 0A 00 00 01 F4 00 00 07 D0 00 FA E4 E5"),
        (90000, "01 10 00 10 00 05 0A 00 01 5F 90 00 00 07 D0 00 FA 48 03"),
    ],
)
# test_build_position_request：验证五寄存器位置请求
# 参数：target_step - 目标步数；expected_hex - 期望报文
# 返回值：无
def test_build_position_request(target_step: int, expected_hex: str) -> None:
    assert format_hex(build_position_request(1, target_step)) == expected_hex


# test_build_emergency_stop_request：验证急停请求
# 参数：无
# 返回值：无
def test_build_emergency_stop_request() -> None:
    assert format_hex(build_emergency_stop_request(1)) == "01 06 00 04 00 01 09 CB"
```

- [ ] **Step 5: 实现 CRC、位置请求和急停请求**

实现时使用 `struct.pack(">BBHHBHHHHH", ...)` 组织功能码 `0x10` 的大端字段，并在末尾追加标准 Modbus CRC 低字节、高字节。验证从站地址为 `1～254`、目标步数为 `500～90000`。位置请求的寄存器数量固定为 `5`、字节数固定为 `10`；急停使用功能码 `0x06` 向 `0x0004` 写 `1`。

- [ ] **Step 6: 运行 Task 1 测试并提交**

Run: `python -m pytest tests/test_stepper_angle_controller.py -v`

Expected: PASS。

```bash
git add tools/stepper_angle_controller.py tests/test_stepper_angle_controller.py
git commit -m "feat: add stepper modbus protocol core"
```

---

### Task 2: Modbus 响应校验和最新目标合并

**Files:**
- Modify: `tools/stepper_angle_controller.py`
- Modify: `tests/test_stepper_angle_controller.py`

**Interfaces:**
- Consumes: 驱动器响应字节、预期功能码、起始地址和寄存器数量；连续到达的角度目标。
- Produces: `validate_modbus_response(response: bytes, slave_address: int, function_code: int, start_address: int, quantity_or_value: int) -> None`、`LatestTargetBuffer.put(angle: float)`、`LatestTargetBuffer.take() -> float | None`、`LatestTargetBuffer.mark_sent(step: int)`、`LatestTargetBuffer.should_send(step: int) -> bool`。

- [ ] **Step 1: 写响应校验失败测试**

```python
from tools.stepper_angle_controller import ModbusProtocolError, validate_modbus_response


# test_validate_position_response：验证正确的位置响应
# 参数：无
# 返回值：无
def test_validate_position_response() -> None:
    validate_modbus_response(bytes.fromhex("01 10 00 10 00 05 01 CF"), 1, 0x10, 0x0010, 5)


@pytest.mark.parametrize(
    ("response_hex", "message"),
    [
        ("01 10 00 10 00 05 00 00", "CRC"),
        ("02 10 00 10 00 05 01 FC", "从站地址"),
        ("01 90 02 CD C1", "地址错误"),
    ],
)
# test_validate_response_rejects_errors：验证错误响应被拒绝
# 参数：response_hex - 响应报文；message - 期望错误提示
# 返回值：无
def test_validate_response_rejects_errors(response_hex: str, message: str) -> None:
    with pytest.raises(ModbusProtocolError, match=message):
        validate_modbus_response(bytes.fromhex(response_hex), 1, 0x10, 0x0010, 5)
```

- [ ] **Step 2: 实现响应校验**

实现 `ModbusProtocolError`，校验最短长度、CRC、从站地址、异常功能码和回显字段。异常码映射必须固定为 `{1: "功能码错误", 2: "地址错误", 3: "长度错误"}`，未知异常码显示十六进制值。正常 `0x10` 和 `0x06` 响应均要求恰好 8 字节。

- [ ] **Step 3: 写最新目标与重复抑制失败测试**

```python
from tools.stepper_angle_controller import LatestTargetBuffer


# test_latest_target_buffer_coalesces_values：验证高频目标只保留最新值
# 参数：无
# 返回值：无
def test_latest_target_buffer_coalesces_values() -> None:
    buffer = LatestTargetBuffer()
    buffer.put(10.0)
    buffer.put(20.0)
    buffer.put(30.0)
    assert buffer.take() == 30.0
    assert buffer.take() is None


# test_latest_target_buffer_suppresses_duplicate_steps：验证成功目标不重复发送
# 参数：无
# 返回值：无
def test_latest_target_buffer_suppresses_duplicate_steps() -> None:
    buffer = LatestTargetBuffer()
    assert buffer.should_send(1000)
    buffer.mark_sent(1000)
    assert not buffer.should_send(1000)
    assert buffer.should_send(1001)
```

- [ ] **Step 4: 实现线程安全的最新目标容器**

使用 `threading.Lock` 保护 `_latest_angle` 和 `_last_sent_step`。`put` 覆盖旧目标，`take` 取出后清空，只有收到合法且通过校验的 Modbus 响应后才调用 `mark_sent`。

- [ ] **Step 5: 运行 Task 2 测试并提交**

Run: `python -m pytest tests/test_stepper_angle_controller.py -v`

Expected: PASS。

```bash
git add tools/stepper_angle_controller.py tests/test_stepper_angle_controller.py
git commit -m "feat: validate modbus responses"
```

---

### Task 3: 后台双串口控制器

**Files:**
- Modify: `tools/stepper_angle_controller.py`
- Modify: `tests/test_stepper_angle_controller.py`

**Interfaces:**
- Consumes: `ControllerCommand` 命令队列、pyserial 串口对象、角度输入行。
- Produces: `ControllerEvent` 事件队列；`SerialController.start()`、`SerialController.submit(command)`、`SerialController.shutdown()`。

- [ ] **Step 1: 为请求收发写可替换串口测试桩**

```python
from tools.stepper_angle_controller import exchange_modbus


# FakeSerial：提供不访问硬件的串口收发测试桩
# 参数：response - 驱动器响应字节
# 返回值：FakeSerial - 可记录请求并返回预置响应的对象
class FakeSerial:
    # FakeSerial.__init__：创建预置响应的串口测试桩
    # 参数：response - 驱动器响应字节
    # 返回值：无
    def __init__(self, response: bytes) -> None:
        self.response = bytearray(response)
        self.written = []

    # FakeSerial.write：记录主站请求
    # 参数：data - 请求报文
    # 返回值：int - 写入长度
    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    # FakeSerial.read：返回指定数量的响应字节
    # 参数：size - 最大读取长度
    # 返回值：bytes - 本次读取结果
    def read(self, size: int) -> bytes:
        data = bytes(self.response[:size])
        del self.response[:size]
        return data

    # FakeSerial.reset_input_buffer：模拟清空接收缓存
    # 参数：无
    # 返回值：无
    def reset_input_buffer(self) -> None:
        return None


# test_exchange_modbus_writes_and_validates：验证请求写入和响应读取
# 参数：无
# 返回值：无
def test_exchange_modbus_writes_and_validates() -> None:
    port = FakeSerial(bytes.fromhex("01 10 00 10 00 05 01 CF"))
    request = build_position_request(1, 500)
    response = exchange_modbus(port, request, 1, 0x10, 0x0010, 5)
    assert port.written == [request]
    assert response == bytes.fromhex("01 10 00 10 00 05 01 CF")
```

- [ ] **Step 2: 实现固定长度 Modbus 收发**

`exchange_modbus` 在发送前清空驱动串口接收缓存，写入请求后先读 2 字节；若功能码带 `0x80` 再读 3 字节，否则再读 6 字节。读取不足时抛出包含“响应超时”的 `ModbusProtocolError`，随后调用 Task 2 的校验函数。

- [ ] **Step 3: 实现命令和事件数据结构**

使用 `dataclasses.dataclass` 定义连接参数、命令和事件：

```python
@dataclass(frozen=True)
# ConnectionSettings：保存两个串口和Modbus连接参数
# 参数：driver_port - 驱动串口；angle_port - 可选角度串口；baudrate - 波特率；slave_address - 从站地址
# 返回值：ConnectionSettings - 不可变连接配置
class ConnectionSettings:
    driver_port: str
    angle_port: str | None
    baudrate: int
    slave_address: int


@dataclass(frozen=True)
# ControllerCommand：描述界面发给后台线程的控制命令
# 参数：kind - 命令类型；payload - 命令数据
# 返回值：ControllerCommand - 不可变控制命令
class ControllerCommand:
    kind: str
    payload: object | None = None


@dataclass(frozen=True)
# ControllerEvent：描述后台线程发给界面的状态事件
# 参数：kind - 事件类型；message - 中文消息；payload - 附加数据
# 返回值：ControllerEvent - 不可变状态事件
class ControllerEvent:
    kind: str
    message: str
    payload: object | None = None
```

命令类型固定为 `connect`、`disconnect`、`start_follow`、`stop_follow`、`manual_angle`、`emergency_stop`、`shutdown`。事件类型固定为 `status`、`angle`、`tx`、`rx`、`warning`、`error`。

- [ ] **Step 4: 实现 SerialController 后台循环**

`SerialController` 持有命令队列、事件队列、`LatestTargetBuffer`、停止事件和一个守护线程。`connect` 命令的载荷为 `ConnectionSettings`，后台使用 `serial.Serial(..., bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=0.05)` 打开驱动串口；若配置中包含角度串口则同时打开。两个端口名称相同时拒绝连接。手动模式允许 `angle_port=None`，只打开驱动串口。

用户从手动模式切换到串口跟随模式后，`start_follow` 的载荷使用当前界面生成的 `ConnectionSettings`。如果角度串口尚未打开，后台线程按该配置打开角度串口；未选择角度串口、选择了驱动串口同名端口或打开失败时拒绝开始跟随，但保持驱动串口连接，用户仍可返回手动模式。

后台循环每轮先处理全部待处理控制命令，再在跟随启用时调用角度串口 `readline()`。合法角度写入最新目标并发布 `angle` 事件；非法非空行发布 `warning` 事件。若存在最新目标，则映射、构帧、发送、校验响应并发布 `tx` 和 `rx` 事件。通信失败时重试一次，第二次失败发布 `error`。

急停命令必须先把跟随状态设为假、清空待发送目标，再发送急停帧。断开和退出命令按“停止跟随、清空目标、关闭角度串口、关闭驱动串口”的顺序执行。

- [ ] **Step 5: 运行全部无硬件测试并提交**

Run: `python -m pytest tests/test_stepper_angle_controller.py -v`

Expected: PASS，测试不访问真实 COM 口。

```bash
git add tools/stepper_angle_controller.py tests/test_stepper_angle_controller.py
git commit -m "feat: add dual serial controller worker"
```

---

### Task 4: Tkinter 图形界面

**Files:**
- Modify: `tools/stepper_angle_controller.py`

**Interfaces:**
- Consumes: `SerialController` 命令与事件接口、`serial.tools.list_ports.comports()`。
- Produces: `StepperControllerApp` 窗口及命令行入口 `main() -> None`。

- [ ] **Step 1: 实现界面布局和状态变量**

创建 `StepperControllerApp(tk.Tk)`，分为以下 `ttk.LabelFrame`：

1. “串口配置”：刷新按钮、角度串口下拉框、驱动串口下拉框、波特率输入框、从站地址输入框、连接和断开按钮。
2. “角度来源”：串口跟随和手动调试两个单选按钮。
3. “实时状态”：原始数据、有效角度、映射步数、驱动状态、最近响应。
4. “手动调试”：角度输入框、0～90 滑块、`0°/30°/45°/60°/90°` 快捷按钮和“发送角度”按钮。
5. “运行控制”：开始跟随、停止跟随、电机急停。
6. “通信日志”：带垂直滚动条的只读 `tk.Text`。

初始状态为未连接、手动调试模式。驱动未连接时禁用发送角度、开始跟随、停止跟随和急停；串口模式下角度串口未连接时禁用开始跟随。

- [ ] **Step 2: 实现界面动作**

刷新串口时保留仍然存在的已选端口。点击连接时验证两个端口不相同、波特率为正整数、从站地址为 `1～254`，然后提交 `connect`。手动输入和滑块统一更新两位小数预览及步数，但只有“发送角度”提交 `manual_angle`。快捷按钮仅更新预览。

“开始跟随”“停止跟随”“电机急停”分别提交同名命令。急停按钮使用红色强调，并在提交前弹出 `messagebox.askyesno` 确认；确认后立即提交，不等待其他普通目标。

- [ ] **Step 3: 实现线程安全事件轮询**

使用 `root.after(50, poll_events)` 每 50 ms 清空事件队列。根据事件类型更新状态标签和按钮状态；日志格式固定为：

```text
[HH:MM:SS.mmm] 信息内容
```

日志最多保留 2000 行，超过后删除最早的 200 行。窗口关闭处理函数提交 `shutdown`、等待后台线程最多 1 秒，然后销毁窗口；关闭窗口不发送急停。

- [ ] **Step 4: 增加启动入口和依赖错误提示**

`main()` 创建控制器和 Tk 根窗口，设置标题“步进电机角度跟随控制器”、最小尺寸和 Windows 缩放友好的默认字体。若未安装 pyserial，在启动时显示中文错误并打印安装命令：

```text
python -m pip install pyserial
```

入口固定为：

```python
if __name__ == "__main__":
    main()
```

- [ ] **Step 5: 执行静态检查、测试和无串口启动检查**

Run: `python -m py_compile tools/stepper_angle_controller.py`

Expected: 无输出，退出码为 0。

Run: `python -m pytest tests/test_stepper_angle_controller.py -v`

Expected: PASS。

Run: `python tools/stepper_angle_controller.py`

Expected: 窗口正常打开；串口列表可刷新；手动模式控件可预览；未连接时不能发送。人工关闭窗口后进程退出。

- [ ] **Step 6: 提交图形界面**

```bash
git add tools/stepper_angle_controller.py
git commit -m "feat: add stepper controller desktop UI"
```

---

### Task 5: Modbus 位置模式使用指南与最终验证

**Files:**
- Create: `docs/Modbus_Stepper_Position_Mode_Guide.md`
- Modify: `tools/stepper_angle_controller.py`
- Modify: `tests/test_stepper_angle_controller.py`

**Interfaces:**
- Consumes: 已实现程序、已批准设计和驱动器手册中的通信说明、寄存器表及示例。
- Produces: 可独立阅读的安装、接线、协议、操作、安全和故障排查文档。

- [ ] **Step 1: 编写使用指南**

文档必须包含：

- 适用驱动板、RS485 A/B 接线、共地和上电前检查。
- 默认通信参数 `115200、8N1、地址1`，以及同一总线上地址不得重复。
- Modbus RTU 功能码 `0x03/0x06/0x10`、CRC 低字节先发和帧间隔概念。
- `0x0010～0x0015` 定点模式寄存器表，明确本程序只写到 `0x0014`。
- 角度映射公式、端点和 `45° -> 45250步` 示例。
- 500 步、90000 步、成功响应和急停完整帧。
- Python 3 和 pyserial 安装命令、启动命令 `python tools/stepper_angle_controller.py`。
- 串口跟随模式和手动调试模式的逐步操作。
- 日志中发送、响应、CRC 错误、超时和异常码的解释。
- 首次测试先脱开负载或保留足够行程、先测试中间角度、确认方向后再测试端点的安全提示。

- [ ] **Step 2: 检查文档中的所有报文 CRC**

添加一个参数化测试，把指南中的四个标准报文作为已知向量，分别验证去掉末尾两字节后的 CRC 计算结果。运行：

Run: `python -m pytest tests/test_stepper_angle_controller.py -v`

Expected: PASS。

- [ ] **Step 3: 执行最终质量检查**

Run: `python -m py_compile tools/stepper_angle_controller.py`

Expected: 退出码为 0。

Run: `python -m pytest tests/test_stepper_angle_controller.py -v`

Expected: 全部通过。

Run: `rg -n "TODO|TBD|pass$|NotImplemented" tools/stepper_angle_controller.py tests/test_stepper_angle_controller.py docs/Modbus_Stepper_Position_Mode_Guide.md`

Expected: 无未完成占位内容；测试桩中不使用空 `pass`。

Run: `git diff --check`

Expected: 本任务文件无空白错误；已有工作区修改保持不变。

- [ ] **Step 4: 提交文档和最终修正**

```bash
git add tools/stepper_angle_controller.py tests/test_stepper_angle_controller.py docs/Modbus_Stepper_Position_Mode_Guide.md
git commit -m "docs: add stepper position mode guide"
```

- [ ] **Step 5: 交付前人工硬件验证**

使用用户提供的两个实际 COM 口执行以下顺序：连接驱动串口、手动发送 `45.00°`、核对 `45250` 步响应、启动串口跟随、输入三条合法角度、输入一条非法角度、停止跟随、执行急停。若实际硬件不可用，在交付说明中明确自动测试已通过但硬件验证待用户执行。

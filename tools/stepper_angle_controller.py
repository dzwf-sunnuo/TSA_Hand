"""步进电机角度映射与 Modbus RTU 协议工具。"""

import math
import queue
import re
import struct
import threading
import tkinter as tk
from datetime import datetime
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
from tkinter import font as tkfont
from tkinter import messagebox, ttk
from typing import Any, Callable


MIN_ANGLE = Decimal("0.00")
MAX_ANGLE = Decimal("90.00")
MIN_STEP = 500
MAX_STEP = 90000
POSITION_START_ADDRESS = 0x0010
POSITION_REGISTER_COUNT = 5
POSITION_BYTE_COUNT = POSITION_REGISTER_COUNT * 2
POSITION_SPEED = 2000
POSITION_ACCELERATION_TIME = 250
EMERGENCY_STOP_ADDRESS = 0x0004
STREAM_ANGLE_PATTERN = re.compile(r"^\d{1,2}\.\d{2}$")
MODBUS_EXCEPTION_MESSAGES = {
    1: "功能码错误",
    2: "地址错误",
    3: "长度错误",
}


# ConnectionSettings：保存两个串口和Modbus连接参数
# 参数：driver_port - 驱动串口；angle_port - 可选角度串口；baudrate - 波特率；slave_address - 从站地址
# 返回值：ConnectionSettings - 不可变连接配置
@dataclass(frozen=True)
class ConnectionSettings:
    driver_port: str
    angle_port: str | None
    baudrate: int
    slave_address: int


# ControllerCommand：描述界面发给后台线程的控制命令
# 参数：kind - 命令类型；payload - 命令数据
# 返回值：ControllerCommand - 不可变控制命令
@dataclass(frozen=True)
class ControllerCommand:
    kind: str
    payload: object | None = None


# ControllerEvent：描述后台线程发给界面的状态事件
# 参数：kind - 事件类型；message - 中文消息；payload - 附加数据
# 返回值：ControllerEvent - 不可变状态事件
@dataclass(frozen=True)
class ControllerEvent:
    kind: str
    message: str
    payload: object | None = None


# ModbusProtocolError：表示Modbus响应违反协议或返回异常码
# 参数：message - 协议错误说明
# 返回值：ModbusProtocolError - 可由上层捕获的协议异常
class ModbusProtocolError(Exception):
    """Modbus响应校验失败。"""


# LatestTargetBuffer：线程安全地合并最新角度并记录成功发送的步数
# 参数：无
# 返回值：LatestTargetBuffer - 最新目标容器
class LatestTargetBuffer:
    # LatestTargetBuffer.__init__：初始化空的目标和成功发送记录
    # 参数：无
    # 返回值：无
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest_angle: float | None = None
        self._last_sent_step: int | None = None

    # LatestTargetBuffer.put：用新角度覆盖尚未处理的旧目标
    # 参数：angle - 最新的有效角度
    # 返回值：无
    def put(self, angle: float) -> None:
        with self._lock:
            self._latest_angle = angle

    # LatestTargetBuffer.take：取出并清空当前最新角度
    # 参数：无
    # 返回值：float或None - 待处理角度，无目标时返回None
    def take(self) -> float | None:
        with self._lock:
            angle = self._latest_angle
            self._latest_angle = None
            return angle

    # LatestTargetBuffer.mark_sent：记录已获得有效响应的目标步数
    # 参数：step - 成功发送的目标步数
    # 返回值：无
    def mark_sent(self, step: int) -> None:
        with self._lock:
            self._last_sent_step = step

    # LatestTargetBuffer.should_send：判断目标步数是否与最近成功值不同
    # 参数：step - 待检查的目标步数
    # 返回值：bool - 需要发送时返回True
    def should_send(self, step: int) -> bool:
        with self._lock:
            return step != self._last_sent_step

    # LatestTargetBuffer.clear：清空待处理目标并可选清除成功发送记录
    # 参数：reset_last_sent - 是否同时清除最近成功发送的步数
    # 返回值：无
    def clear(self, reset_last_sent: bool = False) -> None:
        with self._lock:
            self._latest_angle = None
            if reset_last_sent:
                self._last_sent_step = None


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
        raise ValueError("角度必须位于0.00～90.00") from error

    if not math.isfinite(angle) or not 0.0 <= angle <= 90.0:
        raise ValueError("角度必须位于0.00～90.00")
    return angle


# angle_to_step：把关节角度线性映射为目标步数
# 参数：angle - 0～90度角度
# 返回值：int - 500～90000目标步数
def angle_to_step(angle: float) -> int:
    if not math.isfinite(angle) or not 0.0 <= angle <= 90.0:
        raise ValueError("角度必须位于0.00～90.00")

    # 使用十进制定点计算和ROUND_HALF_UP，避免Python默认的银行家舍入。
    mapped = (
        Decimal(MIN_STEP)
        + Decimal(str(angle)) / MAX_ANGLE * Decimal(MAX_STEP - MIN_STEP)
    )
    target = int(mapped.quantize(Decimal("1"), rounding=ROUND_HALF_UP))
    return min(MAX_STEP, max(MIN_STEP, target))


# calculate_modbus_crc：计算Modbus RTU的CRC16并按低字节在前返回
# 参数：data - 不含CRC的报文字节
# 返回值：bytes - 低字节、高字节顺序的两字节CRC
def calculate_modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


# validate_modbus_response：校验写寄存器响应的CRC、从站、功能码和回显字段
# 参数：response - 完整响应；slave_address - 预期从站地址；function_code - 预期功能码；start_address - 预期起始地址；quantity_or_value - 预期寄存器数量或写入值
# 返回值：无，校验失败时抛出ModbusProtocolError
def validate_modbus_response(
    response: bytes,
    slave_address: int,
    function_code: int,
    start_address: int,
    quantity_or_value: int,
) -> None:
    if len(response) < 5:
        raise ModbusProtocolError(f"Modbus响应长度不足：{len(response)}字节")

    payload = response[:-2]
    received_crc = response[-2:]
    expected_crc = calculate_modbus_crc(payload)
    if received_crc != expected_crc:
        raise ModbusProtocolError("Modbus响应CRC校验失败")

    if response[0] != slave_address:
        raise ModbusProtocolError(
            f"Modbus从站地址不匹配：收到0x{response[0]:02X}，"
            f"期望0x{slave_address:02X}"
        )

    received_function = response[1]
    exception_function = function_code | 0x80
    if received_function == exception_function:
        if len(response) != 5:
            raise ModbusProtocolError(
                f"Modbus异常响应长度错误：收到{len(response)}字节，期望5字节"
            )
        exception_code = response[2]
        message = MODBUS_EXCEPTION_MESSAGES.get(
            exception_code, f"未知异常码0x{exception_code:02X}"
        )
        raise ModbusProtocolError(f"Modbus异常响应：{message}")

    if received_function != function_code:
        raise ModbusProtocolError(
            f"Modbus功能码不匹配：收到0x{received_function:02X}，"
            f"期望0x{function_code:02X}"
        )

    if function_code not in (0x06, 0x10):
        raise ModbusProtocolError(f"不支持校验功能码0x{function_code:02X}的响应")
    if len(response) != 8:
        raise ModbusProtocolError(
            f"Modbus正常响应长度错误：收到{len(response)}字节，期望8字节"
        )

    _, _, received_address, received_quantity_or_value = struct.unpack(
        ">BBHH", payload
    )
    if received_address != start_address:
        raise ModbusProtocolError(
            f"Modbus起始地址不匹配：收到0x{received_address:04X}，"
            f"期望0x{start_address:04X}"
        )
    if received_quantity_or_value != quantity_or_value:
        field_name = "寄存器数量" if function_code == 0x10 else "写入值"
        raise ModbusProtocolError(
            f"Modbus{field_name}不匹配：收到0x{received_quantity_or_value:04X}，"
            f"期望0x{quantity_or_value:04X}"
        )


# validate_slave_address：校验Modbus从站地址
# 参数：slave_address - 待校验的从站地址
# 返回值：无
def validate_slave_address(slave_address: int) -> None:
    if not isinstance(slave_address, int) or not 1 <= slave_address <= 254:
        raise ValueError("Modbus从站地址必须位于1～254")


# build_position_request：构造五寄存器定点位置模式请求
# 参数：slave_address - Modbus从站地址；target_step - 500～90000目标步数
# 返回值：bytes - 包含CRC的完整Modbus RTU请求
def build_position_request(slave_address: int, target_step: int) -> bytes:
    validate_slave_address(slave_address)
    if not isinstance(target_step, int) or not MIN_STEP <= target_step <= MAX_STEP:
        raise ValueError("目标步数必须位于500～90000")

    # 32位步数按高16位、低16位依次写入0x0010和0x0011。
    target_high = (target_step >> 16) & 0xFFFF
    target_low = target_step & 0xFFFF
    payload = struct.pack(
        ">BBHHBHHHHH",
        slave_address,
        0x10,
        POSITION_START_ADDRESS,
        POSITION_REGISTER_COUNT,
        POSITION_BYTE_COUNT,
        target_high,
        target_low,
        0,
        POSITION_SPEED,
        POSITION_ACCELERATION_TIME,
    )
    return payload + calculate_modbus_crc(payload)


# build_emergency_stop_request：构造写入急停寄存器的请求
# 参数：slave_address - Modbus从站地址
# 返回值：bytes - 包含CRC的完整Modbus RTU请求
def build_emergency_stop_request(slave_address: int) -> bytes:
    validate_slave_address(slave_address)
    payload = struct.pack(">BBHH", slave_address, 0x06, EMERGENCY_STOP_ADDRESS, 1)
    return payload + calculate_modbus_crc(payload)


# format_hex：把报文字节格式化为便于核对的十六进制文本
# 参数：data - 待格式化的报文字节
# 返回值：str - 大写且以空格分隔的十六进制文本
def format_hex(data: bytes) -> str:
    return data.hex(" ").upper()


# _read_exact：从串口读取指定数量字节并识别超时短帧
# 参数：port - 已打开的串口对象；size - 期望读取字节数
# 返回值：bytes - 长度恰好为size的响应片段
def _read_exact(port: Any, size: int) -> bytes:
    received = bytearray()
    while len(received) < size:
        chunk = port.read(size - len(received))
        if not chunk:
            break
        received.extend(chunk)
    if len(received) != size:
        raise ModbusProtocolError(
            f"Modbus响应超时：收到{len(received)}字节，期望{size}字节"
        )
    return bytes(received)


# exchange_modbus：在串口上发送一次Modbus请求并校验固定长度响应
# 参数：port - 驱动串口对象；request - 完整请求；slave_address - 从站地址；function_code - 功能码；start_address - 起始地址；quantity_or_value - 寄存器数量或写入值
# 返回值：bytes - 通过协议校验的完整响应
def exchange_modbus(
    port: Any,
    request: bytes,
    slave_address: int,
    function_code: int,
    start_address: int,
    quantity_or_value: int,
) -> bytes:
    port.reset_input_buffer()
    written = port.write(request)
    if written is not None and written != len(request):
        raise ModbusProtocolError(
            f"Modbus请求写入不完整：写入{written}字节，期望{len(request)}字节"
        )

    # 先读取地址和功能码，据异常标志确定剩余响应长度。
    header = _read_exact(port, 2)
    remaining_size = 3 if header[1] & 0x80 else 6
    response = header + _read_exact(port, remaining_size)
    validate_modbus_response(
        response,
        slave_address,
        function_code,
        start_address,
        quantity_or_value,
    )
    return response


# _default_serial_factory：使用pyserial创建一个配置为8N1的串口
# 参数：kwargs - 串口名称、波特率和超时等参数
# 返回值：object - 已打开的pyserial串口对象
def _default_serial_factory(**kwargs: object) -> object:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError(
            "未安装pyserial，请运行 python -m pip install pyserial"
        ) from error
    return serial.Serial(**kwargs)


# SerialController：在单个后台线程中管理角度串口与Modbus驱动串口
# 参数：serial_factory - 可替换的串口创建工厂，测试时用于注入假串口
# 返回值：SerialController - 双串口后台控制器
class SerialController:
    COMMAND_KINDS = {
        "connect",
        "disconnect",
        "start_follow",
        "stop_follow",
        "manual_angle",
        "emergency_stop",
        "shutdown",
    }

    # SerialController.__init__：初始化队列、状态和守护线程但不访问串口
    # 参数：serial_factory - 接收port、baudrate、timeout参数的串口工厂
    # 返回值：无
    def __init__(
        self, serial_factory: Callable[..., object] | None = None
    ) -> None:
        self.command_queue: queue.Queue[ControllerCommand] = queue.Queue()
        self.event_queue: queue.Queue[ControllerEvent] = queue.Queue()
        self.target_buffer = LatestTargetBuffer()
        self._serial_factory = serial_factory or _default_serial_factory
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="stepper-serial-controller",
            daemon=True,
        )
        self._started = False
        self._driver_serial: Any | None = None
        self._angle_serial: Any | None = None
        self._settings: ConnectionSettings | None = None
        self._angle_port_name: str | None = None
        self._following = False

    # SerialController.start：启动串口后台线程
    # 参数：无
    # 返回值：无
    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    # SerialController.submit：向后台线程提交一个合法控制命令
    # 参数：command - 待执行的控制命令
    # 返回值：无
    def submit(self, command: ControllerCommand) -> None:
        if command.kind not in self.COMMAND_KINDS:
            raise ValueError(f"未知控制命令：{command.kind}")
        if not self._started:
            self.start()
        self.command_queue.put(command)

    # SerialController.shutdown：请求后台关闭串口并等待线程退出
    # 参数：timeout - 最长等待秒数
    # 返回值：bool - 后台线程已经退出时返回True
    def shutdown(self, timeout: float = 1.0) -> bool:
        if not self._started:
            return True
        if self._thread.is_alive():
            self.command_queue.put(ControllerCommand("shutdown"))
            self._thread.join(timeout)
        return not self._thread.is_alive()

    # SerialController._emit：向界面事件队列发布状态或通信事件
    # 参数：kind - 事件类型；message - 中文消息；payload - 附加数据
    # 返回值：无
    def _emit(
        self, kind: str, message: str, payload: object | None = None
    ) -> None:
        self.event_queue.put(ControllerEvent(kind, message, payload))

    # SerialController._open_port：通过串口工厂在后台打开一个8N1串口
    # 参数：port_name - 串口名称；baudrate - 波特率
    # 返回值：object - 已打开串口对象
    def _open_port(self, port_name: str, baudrate: int) -> object:
        return self._serial_factory(
            port=port_name,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05,
        )

    # SerialController._validate_settings：校验串口名称、波特率、地址及端口冲突
    # 参数：settings - 待校验连接配置；require_angle - 是否必须配置角度串口
    # 返回值：无
    def _validate_settings(
        self, settings: ConnectionSettings, require_angle: bool = False
    ) -> None:
        if not settings.driver_port.strip():
            raise ValueError("必须选择驱动串口")
        if not isinstance(settings.baudrate, int) or settings.baudrate <= 0:
            raise ValueError("波特率必须是正整数")
        validate_slave_address(settings.slave_address)
        angle_port = settings.angle_port.strip() if settings.angle_port else None
        if require_angle and not angle_port:
            raise ValueError("开始跟随前必须选择角度串口")
        if angle_port and angle_port.casefold() == settings.driver_port.strip().casefold():
            raise ValueError("角度串口和驱动串口不能相同")

    # SerialController._close_port：安全关闭一个可能已失效的串口对象
    # 参数：port - 待关闭串口对象
    # 返回值：无
    def _close_port(self, port: Any | None) -> None:
        if port is None:
            return
        try:
            port.close()
        except Exception as error:
            self._emit("warning", f"关闭串口时出现异常：{error}")

    # SerialController._disconnect：按安全顺序停止跟随、清目标并关闭两个串口
    # 参数：emit_status - 是否发布断开状态事件
    # 返回值：无
    def _disconnect(self, emit_status: bool = True) -> None:
        self._following = False
        self.target_buffer.clear(reset_last_sent=True)
        self._close_port(self._angle_serial)
        self._angle_serial = None
        self._angle_port_name = None
        self._close_port(self._driver_serial)
        self._driver_serial = None
        self._settings = None
        if emit_status:
            self._emit(
                "status",
                "串口已断开",
                {"connected": False, "following": False},
            )

    # SerialController._connect：按照配置打开驱动串口及可选角度串口
    # 参数：payload - 应为ConnectionSettings连接配置
    # 返回值：无
    def _connect(self, payload: object | None) -> None:
        if not isinstance(payload, ConnectionSettings):
            self._emit("error", "连接命令缺少有效配置")
            return
        try:
            self._validate_settings(payload)
        except ValueError as error:
            self._emit("error", str(error))
            return

        self._disconnect(emit_status=False)
        try:
            self._driver_serial = self._open_port(
                payload.driver_port.strip(), payload.baudrate
            )
            if payload.angle_port and payload.angle_port.strip():
                self._angle_serial = self._open_port(
                    payload.angle_port.strip(), payload.baudrate
                )
                self._angle_port_name = payload.angle_port.strip()
        except Exception as error:
            self._disconnect(emit_status=False)
            self._emit("error", f"串口连接失败：{error}")
            return

        self._settings = payload
        self._emit(
            "status",
            "驱动串口连接成功",
            {
                "connected": True,
                "angle_connected": self._angle_serial is not None,
                "following": False,
            },
        )

    # SerialController._start_follow：必要时打开角度串口并开始读取角度
    # 参数：payload - 当前界面生成的ConnectionSettings配置
    # 返回值：无
    def _start_follow(self, payload: object | None) -> None:
        if self._driver_serial is None or self._settings is None:
            self._emit("error", "驱动串口未连接，无法开始跟随")
            return
        if not isinstance(payload, ConnectionSettings):
            self._emit("error", "开始跟随命令缺少有效配置")
            return
        try:
            self._validate_settings(payload, require_angle=True)
            if (
                payload.driver_port.strip().casefold()
                != self._settings.driver_port.strip().casefold()
            ):
                raise ValueError("跟随配置的驱动串口与当前连接不一致")
        except ValueError as error:
            self._emit("error", str(error))
            return

        requested_angle_port = payload.angle_port.strip() if payload.angle_port else ""
        if (
            self._angle_serial is not None
            and self._angle_port_name is not None
            and self._angle_port_name.casefold() != requested_angle_port.casefold()
        ):
            self._close_port(self._angle_serial)
            self._angle_serial = None
            self._angle_port_name = None
        if self._angle_serial is None:
            try:
                self._angle_serial = self._open_port(
                    requested_angle_port, payload.baudrate
                )
                self._angle_port_name = requested_angle_port
            except Exception as error:
                self._angle_serial = None
                self._angle_port_name = None
                self._emit("error", f"角度串口打开失败：{error}")
                return

        self._following = True
        self._emit(
            "status",
            "串口跟随已开始",
            {"connected": True, "angle_connected": True, "following": True},
        )

    # SerialController._queue_manual_angle：校验手动角度并保存为最新目标
    # 参数：payload - 浮点数或可解析的角度文本
    # 返回值：无
    def _queue_manual_angle(self, payload: object | None) -> None:
        if self._driver_serial is None:
            self._emit("error", "驱动串口未连接，无法发送角度")
            return
        try:
            if isinstance(payload, str):
                angle = parse_manual_angle(payload)
            elif isinstance(payload, (int, float)) and not isinstance(payload, bool):
                angle = float(payload)
                angle_to_step(angle)
            else:
                raise ValueError("角度必须位于0.00～90.00")
        except ValueError as error:
            self._emit("warning", str(error))
            return
        self.target_buffer.put(angle)

    # SerialController._exchange_position：发送位置命令，失败时最多重试一次
    # 参数：angle - 已校验的目标角度
    # 返回值：无
    def _exchange_position(self, angle: float) -> None:
        if self._driver_serial is None or self._settings is None:
            self._emit("error", "驱动串口未连接，目标未发送")
            return
        target_step = angle_to_step(angle)
        if not self.target_buffer.should_send(target_step):
            return
        request = build_position_request(self._settings.slave_address, target_step)
        for attempt in range(2):
            self._emit(
                "tx",
                f"发送位置命令：{format_hex(request)}",
                request,
            )
            try:
                response = exchange_modbus(
                    self._driver_serial,
                    request,
                    self._settings.slave_address,
                    0x10,
                    POSITION_START_ADDRESS,
                    POSITION_REGISTER_COUNT,
                )
            except Exception as error:
                if attempt == 0:
                    self._emit("warning", f"位置命令失败，正在重试：{error}")
                    continue
                self._emit("error", f"位置命令重试失败：{error}")
                return
            self.target_buffer.mark_sent(target_step)
            self._emit("rx", f"收到位置响应：{format_hex(response)}", response)
            return

    # SerialController._emergency_stop：先停止跟随并清空目标，再发送一次急停命令
    # 参数：无
    # 返回值：无
    def _emergency_stop(self) -> None:
        self._following = False
        self.target_buffer.clear()
        if self._driver_serial is None or self._settings is None:
            self._emit("error", "驱动串口未连接，无法发送急停")
            return
        request = build_emergency_stop_request(self._settings.slave_address)
        self._emit("tx", f"发送急停命令：{format_hex(request)}", request)
        try:
            response = exchange_modbus(
                self._driver_serial,
                request,
                self._settings.slave_address,
                0x06,
                EMERGENCY_STOP_ADDRESS,
                1,
            )
        except Exception as error:
            self._emit("error", f"急停命令失败：{error}")
            return
        self._emit("rx", f"收到急停响应：{format_hex(response)}", response)
        self._emit(
            "status",
            "急停已执行，串口跟随已停止",
            {"connected": True, "following": False},
        )

    # SerialController._handle_command：在后台处理一条界面控制命令
    # 参数：command - 待执行命令
    # 返回值：无
    def _handle_command(self, command: ControllerCommand) -> None:
        if command.kind == "connect":
            self._connect(command.payload)
        elif command.kind == "disconnect":
            self._disconnect()
        elif command.kind == "start_follow":
            self._start_follow(command.payload)
        elif command.kind == "stop_follow":
            self._following = False
            self.target_buffer.clear()
            self._emit(
                "status",
                "串口跟随已停止",
                {"connected": self._driver_serial is not None, "following": False},
            )
        elif command.kind == "manual_angle":
            self._queue_manual_angle(command.payload)
        elif command.kind == "emergency_stop":
            self._emergency_stop()
        elif command.kind == "shutdown":
            self._stop_event.set()

    # SerialController._drain_commands：清空当前全部命令并使急停优先于待发目标
    # 参数：first_command - 阻塞等待时已取得的首条命令
    # 返回值：无
    def _drain_commands(
        self, first_command: ControllerCommand | None = None
    ) -> None:
        commands: list[ControllerCommand] = []
        if first_command is not None:
            commands.append(first_command)
        while True:
            try:
                commands.append(self.command_queue.get_nowait())
            except queue.Empty:
                break

        # 所有命令先处理完毕，急停可在任何普通目标实际发送前清空目标。
        for command in commands:
            self._handle_command(command)
            if self._stop_event.is_set():
                break

    # SerialController._handle_angle_raw：校验一行角度并覆盖最新目标
    # 参数：raw - 角度串口返回的原始行
    # 返回值：无
    def _handle_angle_raw(self, raw: bytes) -> None:
        angle = parse_stream_angle(raw)
        if angle is None:
            self._emit("warning", f"忽略非法角度数据：{raw!r}", raw)
            return
        target_step = angle_to_step(angle)
        self.target_buffer.put(angle)
        self._emit(
            "angle",
            f"角度{angle:.2f}°映射为{target_step}步",
            {"raw": raw, "angle": angle, "step": target_step},
        )

    # SerialController._read_angle_lines：读取已到达的角度行并只保留最新目标
    # 参数：无
    # 返回值：无
    def _read_angle_lines(self) -> None:
        if not self._following or self._angle_serial is None:
            return
        try:
            raw = self._angle_serial.readline()
        except Exception as error:
            self._following = False
            self._close_port(self._angle_serial)
            self._angle_serial = None
            self._angle_port_name = None
            self._emit("error", f"角度串口读取失败，跟随已停止：{error}")
            return
        if not raw:
            return
        self._handle_angle_raw(raw)

        # 串口缓存已有多行时快速取尽，让未发送目标始终覆盖为最新角度。
        try:
            while getattr(self._angle_serial, "in_waiting", 0) > 0:
                raw = self._angle_serial.readline()
                if not raw:
                    break
                self._handle_angle_raw(raw)
        except Exception as error:
            self._following = False
            self._close_port(self._angle_serial)
            self._angle_serial = None
            self._angle_port_name = None
            self._emit("error", f"角度串口读取失败，跟随已停止：{error}")

    # SerialController._run：持续处理命令、角度输入和最新位置目标
    # 参数：无
    # 返回值：无
    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                first_command: ControllerCommand | None = None
                try:
                    first_command = self.command_queue.get(
                        timeout=0.001 if self._following else 0.02
                    )
                except queue.Empty:
                    pass
                self._drain_commands(first_command)
                if self._stop_event.is_set():
                    break
                self._read_angle_lines()
                angle = self.target_buffer.take()
                if angle is not None:
                    self._exchange_position(angle)
        finally:
            self._disconnect(emit_status=False)
            self._emit(
                "status",
                "串口后台任务已关闭",
                {"connected": False, "following": False, "shutdown": True},
            )


# StepperControllerApp：提供步进电机角度跟随与手动调试图形界面
# 参数：controller - 后台串口控制器；port_lister - 可替换的串口枚举函数
# 返回值：StepperControllerApp - Tkinter主窗口
class StepperControllerApp(tk.Tk):
    LOG_MAX_LINES = 2000
    LOG_TRIM_LINES = 200
    POLL_INTERVAL_MS = 50

    # StepperControllerApp.__init__：初始化窗口状态、控件和后台事件轮询
    # 参数：controller - 后台串口控制器；port_lister - 可替换的串口枚举函数
    # 返回值：无
    def __init__(
        self,
        controller: SerialController | None = None,
        port_lister: Callable[[], list[str]] | None = None,
    ) -> None:
        super().__init__()
        self.controller = controller or SerialController()
        self._port_lister = port_lister or self._list_serial_ports
        self._connected = False
        self._angle_connected = False
        self._following = False
        self._closing = False
        self._log_line_count = 0

        self.title("步进电机角度跟随控制器")
        self.minsize(820, 720)
        self.geometry("920x780")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self._create_variables()
        self._create_widgets()
        self._refresh_ports()
        self._update_manual_preview("45.00")
        self._update_control_states()
        self.after(self.POLL_INTERVAL_MS, self._poll_events)

    # StepperControllerApp._create_variables：创建界面使用的Tk状态变量
    # 参数：无
    # 返回值：无
    def _create_variables(self) -> None:
        self.driver_port_var = tk.StringVar()
        self.angle_port_var = tk.StringVar()
        self.baudrate_var = tk.StringVar(value="115200")
        self.slave_address_var = tk.StringVar(value="1")
        self.mode_var = tk.StringVar(value="manual")
        self.raw_data_var = tk.StringVar(value="--")
        self.valid_angle_var = tk.StringVar(value="--")
        self.target_step_var = tk.StringVar(value="--")
        self.driver_status_var = tk.StringVar(value="未连接")
        self.last_response_var = tk.StringVar(value="--")
        self.manual_angle_var = tk.StringVar(value="45.00")
        self.slider_angle_var = tk.DoubleVar(value=45.0)
        self.manual_error_var = tk.StringVar()

    # StepperControllerApp._create_widgets：按功能区域创建并排列全部控件
    # 参数：无
    # 返回值：无
    def _create_widgets(self) -> None:
        container = ttk.Frame(self, padding=10)
        container.grid(row=0, column=0, sticky="nsew")
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)
        container.rowconfigure(5, weight=1)

        self._create_connection_frame(container)
        self._create_mode_frame(container)
        self._create_status_frame(container)
        self._create_manual_frame(container)
        self._create_run_frame(container)
        self._create_log_frame(container)

    # StepperControllerApp._create_connection_frame：创建串口配置和连接操作区域
    # 参数：parent - 父级容器
    # 返回值：无
    def _create_connection_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="串口配置", padding=8)
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

        ttk.Label(frame, text="驱动串口").grid(row=0, column=0, padx=4, pady=4)
        self.driver_port_combo = ttk.Combobox(
            frame, textvariable=self.driver_port_var, state="readonly", width=18
        )
        self.driver_port_combo.grid(row=0, column=1, sticky="ew", padx=4, pady=4)
        ttk.Label(frame, text="角度串口").grid(row=0, column=2, padx=4, pady=4)
        self.angle_port_combo = ttk.Combobox(
            frame, textvariable=self.angle_port_var, state="readonly", width=18
        )
        self.angle_port_combo.grid(row=0, column=3, sticky="ew", padx=4, pady=4)
        self.refresh_button = ttk.Button(
            frame, text="刷新串口", command=self._refresh_ports
        )
        self.refresh_button.grid(row=0, column=4, padx=4, pady=4)

        ttk.Label(frame, text="波特率").grid(row=1, column=0, padx=4, pady=4)
        self.baudrate_entry = ttk.Entry(frame, textvariable=self.baudrate_var)
        self.baudrate_entry.grid(row=1, column=1, sticky="ew", padx=4, pady=4)
        ttk.Label(frame, text="从站地址").grid(row=1, column=2, padx=4, pady=4)
        self.slave_address_entry = ttk.Entry(
            frame, textvariable=self.slave_address_var
        )
        self.slave_address_entry.grid(
            row=1, column=3, sticky="ew", padx=4, pady=4
        )
        self.connect_button = ttk.Button(
            frame, text="连接", command=self._connect
        )
        self.connect_button.grid(row=1, column=4, padx=4, pady=4, sticky="ew")
        self.disconnect_button = ttk.Button(
            frame, text="断开", command=self._disconnect
        )
        self.disconnect_button.grid(row=1, column=5, padx=4, pady=4, sticky="ew")

    # StepperControllerApp._create_mode_frame：创建互斥的角度来源选择区域
    # 参数：parent - 父级容器
    # 返回值：无
    def _create_mode_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="角度来源", padding=8)
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        self.follow_mode_button = ttk.Radiobutton(
            frame,
            text="串口跟随模式",
            variable=self.mode_var,
            value="follow",
            command=self._on_mode_changed,
        )
        self.follow_mode_button.pack(side="left", padx=12)
        self.manual_mode_button = ttk.Radiobutton(
            frame,
            text="手动调试模式",
            variable=self.mode_var,
            value="manual",
            command=self._on_mode_changed,
        )
        self.manual_mode_button.pack(side="left", padx=12)

    # StepperControllerApp._create_status_frame：创建原始输入和运行状态显示区域
    # 参数：parent - 父级容器
    # 返回值：无
    def _create_status_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="实时状态", padding=8)
        frame.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)
        status_items = (
            ("原始数据", self.raw_data_var),
            ("有效角度", self.valid_angle_var),
            ("映射步数", self.target_step_var),
            ("驱动状态", self.driver_status_var),
            ("最近响应", self.last_response_var),
        )
        for row, (label, variable) in enumerate(status_items):
            ttk.Label(frame, text=label).grid(
                row=row, column=0, sticky="w", padx=4, pady=2
            )
            ttk.Label(frame, textvariable=variable).grid(
                row=row, column=1, sticky="w", padx=8, pady=2
            )

    # StepperControllerApp._create_manual_frame：创建手动角度预览与单次发送区域
    # 参数：parent - 父级容器
    # 返回值：无
    def _create_manual_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="手动调试", padding=8)
        frame.grid(row=3, column=0, sticky="ew", pady=(0, 8))
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="角度（°）").grid(row=0, column=0, padx=4, pady=4)
        self.manual_angle_entry = ttk.Entry(
            frame, textvariable=self.manual_angle_var, width=12
        )
        self.manual_angle_entry.grid(row=0, column=1, sticky="w", padx=4, pady=4)
        self.manual_angle_entry.bind("<KeyRelease>", self._on_manual_entry_changed)
        self.manual_error_label = ttk.Label(
            frame, textvariable=self.manual_error_var, foreground="#B00020"
        )
        self.manual_error_label.grid(row=0, column=2, sticky="w", padx=4, pady=4)

        self.angle_scale = ttk.Scale(
            frame,
            from_=0.0,
            to=90.0,
            variable=self.slider_angle_var,
            command=self._on_slider_changed,
        )
        self.angle_scale.grid(
            row=1, column=0, columnspan=3, sticky="ew", padx=4, pady=4
        )

        quick_frame = ttk.Frame(frame)
        quick_frame.grid(row=2, column=0, columnspan=3, sticky="w", pady=4)
        self.quick_buttons: list[ttk.Button] = []
        for angle in (0, 30, 45, 60, 90):
            button = ttk.Button(
                quick_frame,
                text=f"{angle}°",
                command=lambda value=angle: self._set_manual_angle(float(value)),
            )
            button.pack(side="left", padx=4)
            self.quick_buttons.append(button)
        self.send_button = ttk.Button(
            quick_frame, text="发送角度", command=self._send_manual_angle
        )
        self.send_button.pack(side="left", padx=(20, 4))

    # StepperControllerApp._create_run_frame：创建跟随、停止和急停控制区域
    # 参数：parent - 父级容器
    # 返回值：无
    def _create_run_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="运行控制", padding=8)
        frame.grid(row=4, column=0, sticky="ew", pady=(0, 8))
        self.start_follow_button = ttk.Button(
            frame, text="开始跟随", command=self._start_follow
        )
        self.start_follow_button.pack(side="left", padx=4)
        self.stop_follow_button = ttk.Button(
            frame, text="停止跟随", command=self._stop_follow
        )
        self.stop_follow_button.pack(side="left", padx=4)
        self.emergency_button = tk.Button(
            frame,
            text="电机急停",
            command=self._emergency_stop,
            background="#C62828",
            activebackground="#B71C1C",
            foreground="white",
            activeforeground="white",
            relief="raised",
            padx=12,
        )
        self.emergency_button.pack(side="right", padx=4)

    # StepperControllerApp._create_log_frame：创建带滚动条的只读通信日志区域
    # 参数：parent - 父级容器
    # 返回值：无
    def _create_log_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="通信日志", padding=8)
        frame.grid(row=5, column=0, sticky="nsew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(frame, height=12, wrap="none", state="disabled")
        scrollbar = ttk.Scrollbar(
            frame, orient="vertical", command=self.log_text.yview
        )
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

    # StepperControllerApp._list_serial_ports：通过pyserial枚举当前可用串口名称
    # 参数：无
    # 返回值：list[str] - 按名称排序的串口列表
    def _list_serial_ports(self) -> list[str]:
        try:
            from serial.tools import list_ports
        except ImportError as error:
            raise RuntimeError(
                "未安装pyserial，请运行 python -m pip install pyserial"
            ) from error
        return sorted(port.device for port in list_ports.comports())

    # StepperControllerApp._refresh_ports：刷新下拉列表并保留仍然存在的选择
    # 参数：无
    # 返回值：无
    def _refresh_ports(self) -> None:
        previous_driver = self.driver_port_var.get()
        previous_angle = self.angle_port_var.get()
        try:
            ports = list(self._port_lister())
        except Exception as error:
            self._append_log(f"刷新串口失败：{error}")
            messagebox.showerror("串口刷新失败", str(error), parent=self)
            return
        self.driver_port_combo["values"] = ports
        self.angle_port_combo["values"] = ports
        self.driver_port_var.set(previous_driver if previous_driver in ports else "")
        self.angle_port_var.set(previous_angle if previous_angle in ports else "")
        self._append_log(f"已刷新串口列表，共发现{len(ports)}个端口")
        self._update_control_states()

    # StepperControllerApp._read_settings：读取并校验当前界面连接参数
    # 参数：require_angle - 是否要求必须选择角度串口
    # 返回值：ConnectionSettings - 通过校验的连接配置
    def _read_settings(self, require_angle: bool = False) -> ConnectionSettings:
        driver_port = self.driver_port_var.get().strip()
        angle_port = self.angle_port_var.get().strip() or None
        if not driver_port:
            raise ValueError("必须选择驱动串口")
        if require_angle and not angle_port:
            raise ValueError("串口跟随模式必须选择角度串口")
        if angle_port and angle_port.casefold() == driver_port.casefold():
            raise ValueError("角度串口和驱动串口不能相同")
        try:
            baudrate = int(self.baudrate_var.get().strip())
        except ValueError as error:
            raise ValueError("波特率必须是正整数") from error
        if baudrate <= 0:
            raise ValueError("波特率必须是正整数")
        try:
            slave_address = int(self.slave_address_var.get().strip())
        except ValueError as error:
            raise ValueError("Modbus从站地址必须位于1～254") from error
        validate_slave_address(slave_address)
        return ConnectionSettings(
            driver_port=driver_port,
            angle_port=angle_port if require_angle else None,
            baudrate=baudrate,
            slave_address=slave_address,
        )

    # StepperControllerApp._show_input_error：记录并弹窗显示界面参数错误
    # 参数：message - 中文错误信息
    # 返回值：无
    def _show_input_error(self, message: str) -> None:
        self._append_log(message)
        messagebox.showerror("输入错误", message, parent=self)

    # StepperControllerApp._connect：校验配置并提交后台连接命令
    # 参数：无
    # 返回值：无
    def _connect(self) -> None:
        try:
            settings = self._read_settings(require_angle=self.mode_var.get() == "follow")
        except ValueError as error:
            self._show_input_error(str(error))
            return
        self.driver_status_var.set("正在连接…")
        self.connect_button.configure(state="disabled")
        self.controller.submit(ControllerCommand("connect", settings))

    # StepperControllerApp._disconnect：提交停止跟随并断开串口命令
    # 参数：无
    # 返回值：无
    def _disconnect(self) -> None:
        self.controller.submit(ControllerCommand("disconnect"))

    # StepperControllerApp._on_mode_changed：切换角度来源并停止当前串口跟随
    # 参数：无
    # 返回值：无
    def _on_mode_changed(self) -> None:
        if self._connected:
            # 无条件排入停止命令，可覆盖尚未处理完成的开始跟随操作。
            self.controller.submit(ControllerCommand("stop_follow"))
        self._following = False
        self._append_log(
            "已切换到串口跟随模式"
            if self.mode_var.get() == "follow"
            else "已切换到手动调试模式"
        )
        self._update_control_states()

    # StepperControllerApp._update_manual_preview：校验并显示手动角度对应步数
    # 参数：text - 手动角度文本
    # 返回值：float或None - 合法角度，非法时返回None
    def _update_manual_preview(self, text: str) -> float | None:
        try:
            angle = parse_manual_angle(text)
        except ValueError as error:
            self.manual_error_var.set(str(error))
            return None
        target_step = angle_to_step(angle)
        self.manual_error_var.set("")
        self.valid_angle_var.set(f"{angle:.2f}°")
        self.target_step_var.set(str(target_step))
        return angle

    # StepperControllerApp._on_manual_entry_changed：在手动输入变化时同步滑块和预览
    # 参数：event - Tkinter键盘事件
    # 返回值：无
    def _on_manual_entry_changed(self, event: tk.Event | None = None) -> None:
        del event
        angle = self._update_manual_preview(self.manual_angle_var.get())
        if angle is not None:
            self.slider_angle_var.set(angle)

    # StepperControllerApp._on_slider_changed：拖动滑块时更新两位小数输入和预览
    # 参数：value - 滑块当前值文本
    # 返回值：无
    def _on_slider_changed(self, value: str) -> None:
        angle = float(value)
        text = f"{angle:.2f}"
        self.manual_angle_var.set(text)
        self._update_manual_preview(text)

    # StepperControllerApp._set_manual_angle：由快捷按钮设置手动预览角度
    # 参数：angle - 快捷角度
    # 返回值：无
    def _set_manual_angle(self, angle: float) -> None:
        self.slider_angle_var.set(angle)
        text = f"{angle:.2f}"
        self.manual_angle_var.set(text)
        self._update_manual_preview(text)

    # StepperControllerApp._send_manual_angle：校验后提交一次手动位置命令
    # 参数：无
    # 返回值：无
    def _send_manual_angle(self) -> None:
        angle = self._update_manual_preview(self.manual_angle_var.get())
        if angle is None:
            self._append_log("手动角度无效，未发送位置命令")
            return
        self.controller.submit(ControllerCommand("manual_angle", angle))

    # StepperControllerApp._start_follow：校验串口配置并提交开始跟随命令
    # 参数：无
    # 返回值：无
    def _start_follow(self) -> None:
        try:
            settings = self._read_settings(require_angle=True)
        except ValueError as error:
            self._show_input_error(str(error))
            return
        self.controller.submit(ControllerCommand("start_follow", settings))

    # StepperControllerApp._stop_follow：提交停止读取角度和发送新目标命令
    # 参数：无
    # 返回值：无
    def _stop_follow(self) -> None:
        self.controller.submit(ControllerCommand("stop_follow"))

    # StepperControllerApp._emergency_stop：确认后提交急停命令并立即停止界面跟随态
    # 参数：无
    # 返回值：无
    def _emergency_stop(self) -> None:
        confirmed = messagebox.askyesno(
            "确认急停",
            "确定立即停止电机吗？急停后串口跟随将停止。",
            icon="warning",
            parent=self,
        )
        if not confirmed:
            return
        self._following = False
        self.driver_status_var.set("正在执行急停…")
        self._update_control_states()
        self.controller.submit(ControllerCommand("emergency_stop"))

    # StepperControllerApp._handle_event：把一条后台事件同步到状态区和日志
    # 参数：event - 后台控制器事件
    # 返回值：无
    def _handle_event(self, event: ControllerEvent) -> None:
        payload = event.payload if isinstance(event.payload, dict) else {}
        if event.kind == "status":
            if "connected" in payload:
                self._connected = bool(payload["connected"])
            if "angle_connected" in payload:
                self._angle_connected = bool(payload["angle_connected"])
            elif not self._connected:
                self._angle_connected = False
            if "following" in payload:
                self._following = bool(payload["following"])
            self.driver_status_var.set(event.message)
        elif event.kind == "angle":
            raw = payload.get("raw", b"")
            if isinstance(raw, bytes):
                self.raw_data_var.set(raw.decode("ascii", errors="replace").strip())
            self.valid_angle_var.set(f"{float(payload['angle']):.2f}°")
            self.target_step_var.set(str(payload["step"]))
        elif event.kind == "rx":
            raw_response = event.payload if isinstance(event.payload, bytes) else b""
            self.last_response_var.set(
                format_hex(raw_response) if raw_response else event.message
            )
        elif event.kind == "error":
            self.driver_status_var.set(event.message)
            if "跟随已停止" in event.message:
                self._following = False
            if "角度串口读取失败" in event.message:
                self._angle_connected = False
        self._append_log(event.message)
        self._update_control_states()

    # StepperControllerApp._poll_events：每50毫秒清空后台事件队列并刷新界面
    # 参数：无
    # 返回值：无
    def _poll_events(self) -> None:
        while True:
            try:
                event = self.controller.event_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_event(event)
        if not self._closing:
            self.after(self.POLL_INTERVAL_MS, self._poll_events)

    # StepperControllerApp._append_log：按固定时间格式追加日志并限制总行数
    # 参数：message - 待显示的日志内容
    # 返回值：无
    def _append_log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self._log_line_count += 1
        if self._log_line_count > self.LOG_MAX_LINES:
            self.log_text.delete("1.0", f"{self.LOG_TRIM_LINES + 1}.0")
            self._log_line_count -= self.LOG_TRIM_LINES
        self.log_text.configure(state="disabled")
        self.log_text.see("end")

    # StepperControllerApp._update_control_states：按连接、模式和跟随状态更新控件可用性
    # 参数：无
    # 返回值：无
    def _update_control_states(self) -> None:
        manual_mode = self.mode_var.get() == "manual"
        connected_state = "normal" if self._connected else "disabled"
        self.connect_button.configure(state="disabled" if self._connected else "normal")
        self.disconnect_button.configure(state=connected_state)
        self.emergency_button.configure(state=connected_state)
        self.send_button.configure(
            state="normal" if self._connected and manual_mode else "disabled"
        )
        for widget in [self.manual_angle_entry, self.angle_scale, *self.quick_buttons]:
            widget.configure(state="normal" if manual_mode else "disabled")

        can_start = (
            self._connected
            and not manual_mode
            and self._angle_connected
            and not self._following
        )
        self.start_follow_button.configure(state="normal" if can_start else "disabled")
        self.stop_follow_button.configure(
            state="normal" if self._connected and self._following else "disabled"
        )

        # 已连接期间锁定通信参数，防止界面显示与后台实际连接配置不一致。
        config_state = "disabled" if self._connected else "normal"
        combo_state = "disabled" if self._connected else "readonly"
        self.driver_port_combo.configure(state=combo_state)
        self.angle_port_combo.configure(state=combo_state)
        self.baudrate_entry.configure(state=config_state)
        self.slave_address_entry.configure(state=config_state)
        self.refresh_button.configure(state=config_state)

    # StepperControllerApp._on_close：关闭后台线程后销毁窗口且不隐式发送急停
    # 参数：无
    # 返回值：无
    def _on_close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.controller.shutdown(timeout=1.0)
        self.destroy()


# _configure_default_font：按Windows缩放设置适合中文显示的Tk默认字体
# 参数：root - 已创建的Tk根窗口
# 返回值：无
def _configure_default_font(root: tk.Tk) -> None:
    family = (
        "Microsoft YaHei UI"
        if root.tk.call("tk", "windowingsystem") == "win32"
        else "TkDefaultFont"
    )
    default_font = tkfont.nametofont("TkDefaultFont")
    default_font.configure(family=family, size=10)
    root.option_add("*Font", default_font)


# main：检查运行依赖并启动步进电机控制器图形界面
# 参数：无
# 返回值：无
def main() -> None:
    try:
        import serial  # noqa: F401
    except ImportError:
        root = tk.Tk()
        root.withdraw()
        message = "未安装pyserial，请运行：\npython -m pip install pyserial"
        print(message)
        messagebox.showerror("缺少依赖", message, parent=root)
        root.destroy()
        return

    app = StepperControllerApp()
    _configure_default_font(app)
    app.mainloop()


if __name__ == "__main__":
    main()

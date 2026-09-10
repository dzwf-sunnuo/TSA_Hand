"""步进电机角度映射与 Modbus RTU 协议工具。"""

import math
import re
import struct
import threading
from decimal import Decimal, ROUND_HALF_UP


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

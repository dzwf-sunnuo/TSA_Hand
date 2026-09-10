"""步进电机角度映射与 Modbus RTU 协议工具。"""

import math
import re
import struct
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

import math

import pytest

from tools.stepper_angle_controller import (
    LatestTargetBuffer,
    ModbusProtocolError,
    angle_to_step,
    build_emergency_stop_request,
    build_position_request,
    calculate_modbus_crc,
    format_hex,
    parse_manual_angle,
    parse_stream_angle,
    validate_modbus_response,
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
    [
        b"",
        b"45\n",
        b"45.2\n",
        b"45.250\n",
        b"NaN\n",
        b"inf\n",
        b"-1.00\n",
        b"90.01\n",
        b"abc\n",
        b"\xff\n",
    ],
)
# test_parse_stream_angle_rejects_invalid_lines：验证异常串口行被忽略
# 参数：raw - 非法串口字节
# 返回值：无
def test_parse_stream_angle_rejects_invalid_lines(raw: bytes) -> None:
    assert parse_stream_angle(raw) is None


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 500), (45.0, 45250), (90.0, 90000), (0.01, 510)],
)
# test_angle_to_step_maps_endpoints：验证角度线性映射和四舍五入
# 参数：angle - 输入角度；expected - 期望步数
# 返回值：无
def test_angle_to_step_maps_endpoints(angle: float, expected: int) -> None:
    assert angle_to_step(angle) == expected


# test_angle_to_step_uses_round_half_up：验证步数采用四舍五入而非银行家舍入
# 参数：无
# 返回值：无
def test_angle_to_step_uses_round_half_up() -> None:
    half_step_angle = 90.0 / (90000 - 500) * 0.5
    assert angle_to_step(half_step_angle) == 501


# test_manual_angle_validation：验证手动输入格式与范围
# 参数：无
# 返回值：无
def test_manual_angle_validation() -> None:
    assert parse_manual_angle("45.5") == 45.5
    assert parse_manual_angle(" 90.00 ") == 90.0
    with pytest.raises(ValueError, match="0.00～90.00"):
        parse_manual_angle(str(math.nan))
    with pytest.raises(ValueError, match="0.00～90.00"):
        parse_manual_angle("not-a-number")


# test_crc_known_vector：用手册状态读取帧验证CRC顺序
# 参数：无
# 返回值：无
def test_crc_known_vector() -> None:
    assert calculate_modbus_crc(bytes.fromhex("01 03 00 00 00 01")) == bytes.fromhex(
        "84 0A"
    )


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


@pytest.mark.parametrize("slave_address", [0, 255])
# test_requests_reject_invalid_slave_address：验证非法从站地址被拒绝
# 参数：slave_address - 非法从站地址
# 返回值：无
def test_requests_reject_invalid_slave_address(slave_address: int) -> None:
    with pytest.raises(ValueError, match="1～254"):
        build_position_request(slave_address, 500)
    with pytest.raises(ValueError, match="1～254"):
        build_emergency_stop_request(slave_address)


@pytest.mark.parametrize("target_step", [499, 90001])
# test_position_request_rejects_invalid_step：验证越界目标步数被拒绝
# 参数：target_step - 非法目标步数
# 返回值：无
def test_position_request_rejects_invalid_step(target_step: int) -> None:
    with pytest.raises(ValueError, match="500～90000"):
        build_position_request(1, target_step)


# build_response：构造带有效CRC的Modbus测试响应
# 参数：payload_hex - 不含CRC的十六进制报文
# 返回值：bytes - 包含CRC的完整响应
def build_response(payload_hex: str) -> bytes:
    payload = bytes.fromhex(payload_hex)
    return payload + calculate_modbus_crc(payload)


# test_validate_position_response：验证正确的位置响应
# 参数：无
# 返回值：无
def test_validate_position_response() -> None:
    validate_modbus_response(
        bytes.fromhex("01 10 00 10 00 05 01 CF"), 1, 0x10, 0x0010, 5
    )


# test_validate_write_single_register_response：验证正确的0x06回显响应
# 参数：无
# 返回值：无
def test_validate_write_single_register_response() -> None:
    validate_modbus_response(
        bytes.fromhex("01 06 00 04 00 01 09 CB"), 1, 0x06, 0x0004, 1
    )


@pytest.mark.parametrize(
    ("response", "message"),
    [
        (bytes.fromhex("01 10 00 10 00 05 00 00"), "CRC"),
        (build_response("02 10 00 10 00 05"), "从站地址"),
        (build_response("01 06 00 10 00 05"), "功能码"),
        (build_response("01 10 00 11 00 05"), "起始地址"),
        (build_response("01 10 00 10 00 06"), "寄存器数量"),
    ],
)
# test_validate_response_rejects_crc_address_and_field_errors：验证错误响应被拒绝
# 参数：response - 错误响应；message - 期望错误提示
# 返回值：无
def test_validate_response_rejects_crc_address_and_field_errors(
    response: bytes, message: str
) -> None:
    with pytest.raises(ModbusProtocolError, match=message):
        validate_modbus_response(response, 1, 0x10, 0x0010, 5)


@pytest.mark.parametrize(
    ("exception_code", "message"),
    [(1, "功能码错误"), (2, "地址错误"), (3, "长度错误"), (4, "0x04")],
)
# test_validate_exception_response：验证0x90异常响应的错误码解析
# 参数：exception_code - Modbus异常码；message - 期望错误提示
# 返回值：无
def test_validate_exception_response(exception_code: int, message: str) -> None:
    response = build_response(f"01 90 {exception_code:02X}")
    with pytest.raises(ModbusProtocolError, match=message):
        validate_modbus_response(response, 1, 0x10, 0x0010, 5)


@pytest.mark.parametrize("response", [b"", bytes.fromhex("01 10 00 10")])
# test_validate_response_rejects_short_frames：验证过短响应被拒绝
# 参数：response - 过短响应
# 返回值：无
def test_validate_response_rejects_short_frames(response: bytes) -> None:
    with pytest.raises(ModbusProtocolError, match="长度"):
        validate_modbus_response(response, 1, 0x10, 0x0010, 5)


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

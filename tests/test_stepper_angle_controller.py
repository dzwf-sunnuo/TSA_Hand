import math

import pytest

from tools.stepper_angle_controller import (
    angle_to_step,
    build_emergency_stop_request,
    build_position_request,
    calculate_modbus_crc,
    format_hex,
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

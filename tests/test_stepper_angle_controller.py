import math
import queue
import threading
import time

import pytest

from tools.stepper_angle_controller import (
    LatestTargetBuffer,
    ModbusProtocolError,
    ConnectionSettings,
    ControllerCommand,
    ControllerEvent,
    SerialController,
    angle_to_step,
    build_emergency_stop_request,
    build_position_request,
    calculate_modbus_crc,
    exchange_modbus,
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


# FakeSerial：提供不访问硬件的串口收发测试桩
# 参数：responses - 每次写入后可供读取的响应；lines - 角度串口待返回行
# 返回值：FakeSerial - 可记录请求并返回预置数据的对象
class FakeSerial:
    # FakeSerial.__init__：创建预置响应和角度行的串口测试桩
    # 参数：responses - 每次请求对应的响应；lines - 依次读取的角度行
    # 返回值：无
    def __init__(
        self,
        responses: list[bytes] | None = None,
        lines: list[bytes] | None = None,
    ) -> None:
        self.responses = list(responses or [])
        self.lines = queue.Queue()
        for line in lines or []:
            self.lines.put(line)
        self.response = bytearray()
        self.written: list[bytes] = []
        self.closed = False
        self.operation_thread_ids: list[int] = []

    # FakeSerial.in_waiting：返回尚未读取的角度数据估算字节数
    # 参数：无
    # 返回值：int - 有待处理行时返回正数，否则返回0
    @property
    def in_waiting(self) -> int:
        return self.lines.qsize()

    # FakeSerial.write：记录主站请求并装载本次预置响应
    # 参数：data - 请求报文
    # 返回值：int - 写入长度
    def write(self, data: bytes) -> int:
        self.operation_thread_ids.append(threading.get_ident())
        self.written.append(data)
        self.response = bytearray(self.responses.pop(0) if self.responses else b"")
        return len(data)

    # FakeSerial.read：返回指定数量的响应字节
    # 参数：size - 最大读取长度
    # 返回值：bytes - 本次读取结果
    def read(self, size: int) -> bytes:
        self.operation_thread_ids.append(threading.get_ident())
        data = bytes(self.response[:size])
        del self.response[:size]
        return data

    # FakeSerial.readline：返回一条预置角度数据或模拟超时
    # 参数：无
    # 返回值：bytes - 原始角度行，无数据时为空字节
    def readline(self) -> bytes:
        self.operation_thread_ids.append(threading.get_ident())
        try:
            return self.lines.get_nowait()
        except queue.Empty:
            time.sleep(0.002)
            return b""

    # FakeSerial.reset_input_buffer：模拟发送前清空接收缓存
    # 参数：无
    # 返回值：无
    def reset_input_buffer(self) -> None:
        self.operation_thread_ids.append(threading.get_ident())

    # FakeSerial.close：记录串口已由后台线程关闭
    # 参数：无
    # 返回值：无
    def close(self) -> None:
        self.operation_thread_ids.append(threading.get_ident())
        self.closed = True


# FakeSerialFactory：按端口名称返回预置假串口并记录创建参数
# 参数：ports - 端口名称到假串口对象的映射
# 返回值：FakeSerialFactory - 可调用串口工厂
class FakeSerialFactory:
    # FakeSerialFactory.__init__：保存端口映射和调用记录
    # 参数：ports - 端口名称到假串口对象的映射
    # 返回值：无
    def __init__(self, ports: dict[str, FakeSerial]) -> None:
        self.ports = ports
        self.calls: list[dict[str, object]] = []
        self.thread_ids: list[int] = []

    # FakeSerialFactory.__call__：模拟按指定参数打开串口
    # 参数：kwargs - 端口、波特率和超时
    # 返回值：FakeSerial - 指定名称的假串口
    def __call__(self, **kwargs: object) -> FakeSerial:
        self.calls.append(kwargs)
        self.thread_ids.append(threading.get_ident())
        port_name = str(kwargs["port"])
        if port_name not in self.ports:
            raise OSError(f"端口不存在：{port_name}")
        return self.ports[port_name]


# wait_for_event：等待控制器发布符合条件的事件
# 参数：controller - 后台控制器；kind - 事件类型；message_part - 消息包含文本；timeout - 最长等待秒数
# 返回值：ControllerEvent - 首个符合条件的事件
def wait_for_event(
    controller: SerialController,
    kind: str,
    message_part: str = "",
    timeout: float = 1.0,
) -> ControllerEvent:
    deadline = time.monotonic() + timeout
    seen: list[ControllerEvent] = []
    while time.monotonic() < deadline:
        try:
            event = controller.event_queue.get(timeout=0.02)
        except queue.Empty:
            continue
        seen.append(event)
        if event.kind == kind and message_part in event.message:
            return event
    raise AssertionError(f"未收到事件{kind}/{message_part}，已收到：{seen}")


# test_exchange_modbus_writes_and_validates：验证请求写入和响应读取
# 参数：无
# 返回值：无
def test_exchange_modbus_writes_and_validates() -> None:
    port = FakeSerial(responses=[bytes.fromhex("01 10 00 10 00 05 01 CF")])
    request = build_position_request(1, 500)
    response = exchange_modbus(port, request, 1, 0x10, 0x0010, 5)
    assert port.written == [request]
    assert response == bytes.fromhex("01 10 00 10 00 05 01 CF")


# test_exchange_modbus_reports_timeout：验证响应不足时报告超时
# 参数：无
# 返回值：无
def test_exchange_modbus_reports_timeout() -> None:
    port = FakeSerial(responses=[bytes.fromhex("01 10 00")])
    with pytest.raises(ModbusProtocolError, match="响应超时"):
        exchange_modbus(port, build_position_request(1, 500), 1, 0x10, 0x0010, 5)


# test_controller_manual_mode_opens_only_driver_and_sends：验证手动模式仅打开驱动串口并发送角度
# 参数：无
# 返回值：无
def test_controller_manual_mode_opens_only_driver_and_sends() -> None:
    driver = FakeSerial(responses=[bytes.fromhex("01 10 00 10 00 05 01 CF")])
    factory = FakeSerialFactory({"COM_DRIVER": driver})
    controller = SerialController(factory)
    settings = ConnectionSettings("COM_DRIVER", None, 115200, 1)
    controller.submit(ControllerCommand("connect", settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("manual_angle", 45.0))
    wait_for_event(controller, "rx", "位置响应")
    assert [call["port"] for call in factory.calls] == ["COM_DRIVER"]
    assert factory.calls[0] == {
        "port": "COM_DRIVER",
        "baudrate": 115200,
        "bytesize": 8,
        "parity": "N",
        "stopbits": 1,
        "timeout": 0.05,
    }
    assert driver.written == [build_position_request(1, 45250)]
    assert set(factory.thread_ids + driver.operation_thread_ids) == {
        controller._thread.ident
    }
    assert controller.shutdown()


# test_controller_start_follow_opens_angle_port_and_follows：验证手动连接后可按开始跟随配置打开角度串口
# 参数：无
# 返回值：无
def test_controller_start_follow_opens_angle_port_and_follows() -> None:
    driver = FakeSerial(responses=[bytes.fromhex("01 10 00 10 00 05 01 CF")])
    angle = FakeSerial(lines=[b"30.00\n"])
    factory = FakeSerialFactory({"COM_DRIVER": driver, "COM_ANGLE": angle})
    controller = SerialController(factory)
    manual_settings = ConnectionSettings("COM_DRIVER", None, 115200, 1)
    follow_settings = ConnectionSettings("COM_DRIVER", "COM_ANGLE", 115200, 1)
    controller.submit(ControllerCommand("connect", manual_settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("start_follow", follow_settings))
    event = wait_for_event(controller, "angle", "30.00°")
    wait_for_event(controller, "rx", "位置响应")
    assert event.payload == {"raw": b"30.00\n", "angle": 30.0, "step": 30333}
    assert [call["port"] for call in factory.calls] == ["COM_DRIVER", "COM_ANGLE"]
    assert driver.written == [build_position_request(1, 30333)]
    assert controller.shutdown()


# test_controller_follow_coalesces_buffered_angles：验证已缓存的多行角度只发送最新目标
# 参数：无
# 返回值：无
def test_controller_follow_coalesces_buffered_angles() -> None:
    driver = FakeSerial(responses=[bytes.fromhex("01 10 00 10 00 05 01 CF")])
    angle = FakeSerial(lines=[b"10.00\n", b"20.00\n", b"30.00\n"])
    controller = SerialController(
        FakeSerialFactory({"COM_DRIVER": driver, "COM_ANGLE": angle})
    )
    settings = ConnectionSettings("COM_DRIVER", "COM_ANGLE", 115200, 1)
    controller.submit(ControllerCommand("connect", settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("start_follow", settings))
    wait_for_event(controller, "rx", "位置响应")
    assert driver.written == [build_position_request(1, 30333)]
    assert controller.shutdown()


# test_start_follow_failure_keeps_driver_available：验证角度串口打开失败后仍可手动控制
# 参数：无
# 返回值：无
def test_start_follow_failure_keeps_driver_available() -> None:
    driver = FakeSerial(responses=[bytes.fromhex("01 10 00 10 00 05 01 CF")])
    controller = SerialController(FakeSerialFactory({"DRIVER": driver}))
    manual_settings = ConnectionSettings("DRIVER", None, 115200, 1)
    follow_settings = ConnectionSettings("DRIVER", "MISSING", 115200, 1)
    controller.submit(ControllerCommand("connect", manual_settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("start_follow", follow_settings))
    wait_for_event(controller, "error", "角度串口打开失败")
    controller.submit(ControllerCommand("manual_angle", 0.0))
    wait_for_event(controller, "rx", "位置响应")
    assert not driver.closed
    assert driver.written == [build_position_request(1, 500)]
    assert controller.shutdown()


# test_controller_warns_and_ignores_invalid_angle_line：验证非法非空角度行报警且不驱动电机
# 参数：无
# 返回值：无
def test_controller_warns_and_ignores_invalid_angle_line() -> None:
    driver = FakeSerial()
    angle = FakeSerial(lines=[b"45.2\n"])
    factory = FakeSerialFactory({"DRIVER": driver, "ANGLE": angle})
    controller = SerialController(factory)
    settings = ConnectionSettings("DRIVER", "ANGLE", 115200, 1)
    controller.submit(ControllerCommand("connect", settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("start_follow", settings))
    wait_for_event(controller, "warning", "非法角度数据")
    assert driver.written == []
    assert controller.shutdown()


# test_controller_retries_position_once_after_timeout：验证位置响应超时最多重试一次
# 参数：无
# 返回值：无
def test_controller_retries_position_once_after_timeout() -> None:
    success = bytes.fromhex("01 10 00 10 00 05 01 CF")
    driver = FakeSerial(responses=[b"", success])
    factory = FakeSerialFactory({"DRIVER": driver})
    controller = SerialController(factory)
    settings = ConnectionSettings("DRIVER", None, 115200, 1)
    controller.submit(ControllerCommand("connect", settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("manual_angle", 0.0))
    wait_for_event(controller, "warning", "正在重试")
    wait_for_event(controller, "rx", "位置响应")
    assert driver.written == [
        build_position_request(1, 500),
        build_position_request(1, 500),
    ]
    assert controller.shutdown()


# test_controller_stops_retrying_after_second_timeout：验证连续超时只发送两次请求
# 参数：无
# 返回值：无
def test_controller_stops_retrying_after_second_timeout() -> None:
    driver = FakeSerial(responses=[b"", b""])
    controller = SerialController(FakeSerialFactory({"DRIVER": driver}))
    settings = ConnectionSettings("DRIVER", None, 115200, 1)
    controller.submit(ControllerCommand("connect", settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("manual_angle", 0.0))
    wait_for_event(controller, "error", "重试失败")
    time.sleep(0.05)
    assert driver.written == [
        build_position_request(1, 500),
        build_position_request(1, 500),
    ]
    assert controller.shutdown()


# test_controller_suppresses_duplicate_successful_step：验证成功位置不会重复发送
# 参数：无
# 返回值：无
def test_controller_suppresses_duplicate_successful_step() -> None:
    response = bytes.fromhex("01 10 00 10 00 05 01 CF")
    driver = FakeSerial(responses=[response])
    controller = SerialController(FakeSerialFactory({"DRIVER": driver}))
    settings = ConnectionSettings("DRIVER", None, 115200, 1)
    controller.submit(ControllerCommand("connect", settings))
    wait_for_event(controller, "status", "连接成功")
    controller.submit(ControllerCommand("manual_angle", 45.0))
    wait_for_event(controller, "rx", "位置响应")
    controller.submit(ControllerCommand("manual_angle", 45.0))
    time.sleep(0.08)
    assert len(driver.written) == 1
    assert controller.shutdown()


# test_emergency_stop_clears_pending_target_and_stops_follow：验证急停优先清目标且停止跟随
# 参数：无
# 返回值：无
def test_emergency_stop_clears_pending_target_and_stops_follow() -> None:
    stop_response = bytes.fromhex("01 06 00 04 00 01 09 CB")
    driver = FakeSerial(responses=[stop_response])
    controller = SerialController(FakeSerialFactory({"DRIVER": driver}))
    settings = ConnectionSettings("DRIVER", None, 115200, 1)
    # 在线程启动前排入整批命令，验证急停在循环发送目标前清空待发目标。
    controller.command_queue.put(ControllerCommand("connect", settings))
    controller.command_queue.put(ControllerCommand("manual_angle", 90.0))
    controller.command_queue.put(ControllerCommand("emergency_stop"))
    controller.start()
    event = wait_for_event(controller, "status", "急停已执行")
    assert event.payload == {"connected": True, "following": False}
    assert driver.written == [build_emergency_stop_request(1)]
    assert controller.target_buffer.take() is None
    assert controller.shutdown()


# test_controller_closes_both_ports_on_shutdown：验证退出在后台关闭两个串口
# 参数：无
# 返回值：无
def test_controller_closes_both_ports_on_shutdown() -> None:
    driver = FakeSerial()
    angle = FakeSerial()
    factory = FakeSerialFactory({"DRIVER": driver, "ANGLE": angle})
    controller = SerialController(factory)
    settings = ConnectionSettings("DRIVER", "ANGLE", 115200, 1)
    controller.submit(ControllerCommand("connect", settings))
    wait_for_event(controller, "status", "连接成功")
    assert controller.shutdown()
    assert driver.closed
    assert angle.closed
    assert set(driver.operation_thread_ids + angle.operation_thread_ids) == {
        controller._thread.ident
    }

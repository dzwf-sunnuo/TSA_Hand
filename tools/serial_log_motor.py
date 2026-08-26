#!/usr/bin/env python3
"""
串口数据记录工具 — 解析 Motor 状态输出并自动保存为 TXT 文件

用法:
    python serial_log_motor.py [COM端口] [波特率]

示例:
    python serial_log_motor.py COM9 115200
    python serial_log_motor.py              (默认 COM9, 115200)

输出格式 (每行):
    时间戳  MotorX Target:xx.x deg  Actual:xx.x deg  ADC:xxxx

输出文件:
    motor_log_20260721_145030.txt          (自动以时间戳命名)
"""

import serial
import sys
import os
import re
from datetime import datetime


# ── 默认配置 ──────────────────────────────────────────────
DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 115200
LOG_DIR = "logs"  # 日志输出目录


def parse_line(line: str):
    """
    解析 printf 输出行, 提取结构化数据

    匹配格式:
        Motor 1 Target: 45.0 deg, Actual: 32.5 deg (ADC: 1835)

    返回:
        dict 或 None
    """
    # 正则匹配: Motor编号, Target角度, Actual角度, ADC原始值
    pattern = (
        r"Motor\s+(\d+)\s+"
        r"Target:\s+([-\d.]+)\s+deg,\s+"
        r"Actual:\s+([-\d.]+)\s+deg\s+"
        r"\(ADC:\s+(\d+)\)"
    )
    m = re.search(pattern, line)
    if not m:
        return None

    return {
        "motor": int(m.group(1)),
        "target_deg": float(m.group(2)),
        "actual_deg": float(m.group(3)),
        "adc_raw": int(m.group(4)),
    }


def format_log_entry(ts: datetime, data: dict) -> str:
    """将解析后的数据格式化为日志行"""
    return (
        f"{ts.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}  "
        f"Motor{data['motor']}  "
        f"Target:{data['target_deg']:6.1f} deg  "
        f"Actual:{data['actual_deg']:6.1f} deg  "
        f"ADC:{data['adc_raw']:5d}"
    )


def main():
    # ── 解析命令行参数 ────────────────────────────────────
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    # ── 创建日志目录 ──────────────────────────────────────
    os.makedirs(LOG_DIR, exist_ok=True)

    # ── 生成输出文件名 ────────────────────────────────────
    filename = os.path.join(
        LOG_DIR, f"motor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    )

    print(f"═" * 60)
    print(f"  串口数据记录工具")
    print(f"  端口: {port}  波特率: {baud}")
    print(f"  输出: {filename}")
    print(f"  按 Ctrl+C 停止记录")
    print(f"═" * 60)

    # ── 打开串口 ──────────────────────────────────────────
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"❌ 无法打开串口 {port}: {e}")
        sys.exit(1)

    # ── 打开输出文件并开始记录 ────────────────────────────
    line_count = 0
    motor_count = 0

    try:
        with open(filename, "w", encoding="utf-8") as f:
            # 文件头
            f.write(f"# 串口数据记录\n")
            f.write(f"# 端口: {port}  波特率: {baud}\n")
            f.write(f"# 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# 格式: 时间戳  Motor编号  Target:目标角度  Actual:实际角度  ADC:原始值\n")
            f.write(f"#\n")
            f.flush()

            print("✅ 开始记录 (Ctrl+C 停止)...\n")

            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException as e:
                    print(f"⚠️ 串口读取错误: {e}")
                    break

                if not raw:
                    continue

                try:
                    line = raw.decode("utf-8", errors="replace").strip()
                except UnicodeDecodeError:
                    continue

                if not line:
                    continue

                # 解析数据行
                data = parse_line(line)
                if data is None:
                    continue

                ts = datetime.now()
                log_line = format_log_entry(ts, data)

                # 终端回显
                print(log_line)

                # 写入文件
                f.write(log_line + "\n")

                line_count += 1
                motor_count += 1

                # 每 50 行刷新磁盘缓冲
                if line_count % 50 == 0:
                    f.flush()

    except KeyboardInterrupt:
        print(f"\n\n⏹️  用户中断")

    finally:
        ser.close()
        print(f"📁 已保存 {motor_count} 条记录 → {filename}")
        print(f"   文件大小: {os.path.getsize(filename):,} 字节")


if __name__ == "__main__":
    main()

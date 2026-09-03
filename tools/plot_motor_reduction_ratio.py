#!/usr/bin/env python3
"""读取减速比实验串口数据，实时绘图并保存CSV和PNG。"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial


DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 115200
DEFAULT_MOTOR = 1
MIN_ANGLE_SPAN_DEG = 1.0


# parse_args：解析串口采集命令行参数
# 参数：无
# 返回值：argparse.Namespace - 解析后的端口、波特率、电机编号和输出目录
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="实时绘制电机圈数与关节角度并计算减速比")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT, help="串口号，默认COM9")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD, help="波特率，默认115200")
    parser.add_argument("--motor", type=int, default=DEFAULT_MOTOR, choices=range(1, 5), help="采集的电机编号，默认1")
    parser.add_argument("--output-dir", default="logs", help="数据和图表输出目录，默认logs")
    return parser.parse_args()


# parse_motor_line：解析固件输出的三列CSV数据
# 参数：line - 串口接收到的一行文本
# 返回值：tuple或None - 电机编号、电机累计圈数、关节角度；格式错误时返回None
def parse_motor_line(line: str):
    fields = [field.strip() for field in line.split(",")]
    if len(fields) != 3:
        return None

    try:
        motor_number = int(fields[0])
        motor_turns = float(fields[1])
        joint_angle_deg = float(fields[2])
    except ValueError:
        return None

    if motor_number not in range(1, 5):
        return None
    if not np.isfinite(motor_turns) or not np.isfinite(joint_angle_deg):
        return None
    return motor_number, motor_turns, joint_angle_deg


# calculate_reduction_ratio：根据相对首点的数据拟合减速比
# 参数：motor_turns - 电机累计圈数序列；joint_angles - 关节角度序列
# 返回值：float或None - 电机圈数与关节圈数之比，运动范围不足时返回None
def calculate_reduction_ratio(motor_turns, joint_angles):
    if len(motor_turns) < 2:
        return None

    delta_turns = np.asarray(motor_turns, dtype=float) - motor_turns[0]
    delta_angles = np.asarray(joint_angles, dtype=float) - joint_angles[0]
    if np.ptp(delta_angles) < MIN_ANGLE_SPAN_DEG:
        return None

    slope_turns_per_degree, _ = np.polyfit(delta_angles, delta_turns, 1)
    return abs(float(slope_turns_per_degree) * 360.0)


# configure_plot：创建实时曲线和减速比拟合图
# 参数：motor_number - 要显示的电机编号
# 返回值：tuple - 图形、坐标轴和曲线对象
def configure_plot(motor_number: int):
    plt.ion()
    figure, (axis_time, axis_relation) = plt.subplots(2, 1, figsize=(10, 8))
    axis_angle = axis_time.twinx()

    turns_line, = axis_time.plot([], [], color="tab:blue", label="Motor turns")
    angle_line, = axis_angle.plot([], [], color="tab:orange", label="Joint angle")
    relation_points, = axis_relation.plot([], [], "o", markersize=3, color="tab:green")
    fit_line, = axis_relation.plot([], [], "-", color="tab:red", label="Linear fit")

    axis_time.set_xlabel("Time (s)")
    axis_time.set_ylabel("Motor turns", color="tab:blue")
    axis_angle.set_ylabel("Joint angle (deg)", color="tab:orange")
    axis_time.grid(True, alpha=0.3)
    axis_time.legend(handles=[turns_line, angle_line], loc="upper left")

    axis_relation.set_xlabel("Joint angle change (deg)")
    axis_relation.set_ylabel("Motor turn change")
    axis_relation.grid(True, alpha=0.3)
    axis_relation.legend(loc="upper left")
    figure.suptitle(f"Motor {motor_number} reduction-ratio experiment")
    figure.tight_layout()

    return figure, axis_time, axis_angle, axis_relation, turns_line, angle_line, relation_points, fit_line


# update_plot：使用最新数据刷新实时图表
# 参数：plot_items - configure_plot返回的绘图对象；times、motor_turns、joint_angles - 采样数据
# 返回值：float或None - 当前拟合得到的减速比
def update_plot(plot_items, times, motor_turns, joint_angles):
    figure, axis_time, axis_angle, axis_relation, turns_line, angle_line, relation_points, fit_line = plot_items
    turns_line.set_data(times, motor_turns)
    angle_line.set_data(times, joint_angles)
    axis_time.relim()
    axis_time.autoscale_view()
    axis_angle.relim()
    axis_angle.autoscale_view()

    delta_turns = np.asarray(motor_turns) - motor_turns[0]
    delta_angles = np.asarray(joint_angles) - joint_angles[0]
    relation_points.set_data(delta_angles, delta_turns)
    axis_relation.relim()
    axis_relation.autoscale_view()

    ratio = calculate_reduction_ratio(motor_turns, joint_angles)
    if ratio is not None:
        slope, intercept = np.polyfit(delta_angles, delta_turns, 1)
        fit_x = np.array([delta_angles.min(), delta_angles.max()])
        fit_line.set_data(fit_x, slope * fit_x + intercept)
        axis_relation.set_title(f"Estimated reduction ratio: {ratio:.2f}:1")
    else:
        fit_line.set_data([], [])
        axis_relation.set_title("Move the joint by at least 1 degree to estimate ratio")

    figure.canvas.draw_idle()
    figure.canvas.flush_events()
    plt.pause(0.001)
    return ratio


# main：打开串口、记录CSV、实时绘图并在结束时保存PNG
# 参数：无
# 返回值：无
def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"reduction_ratio_motor{args.motor}_{timestamp}.csv"
    png_path = output_dir / f"reduction_ratio_motor{args.motor}_{timestamp}.png"

    try:
        serial_port = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as error:
        print(f"无法打开串口 {args.port}: {error}", file=sys.stderr)
        raise SystemExit(1) from error

    plot_items = configure_plot(args.motor)
    sample_times = []
    motor_turns = []
    joint_angles = []
    start_time = time.monotonic()
    latest_ratio = None

    print(f"正在采集电机{args.motor}：{args.port} @ {args.baud}，按Ctrl+C停止")
    print(f"CSV文件：{csv_path}")

    try:
        with csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["elapsed_s", "motor", "motor_turns", "joint_angle_deg"])

            while plt.fignum_exists(plot_items[0].number):
                raw_line = serial_port.readline()
                if not raw_line:
                    plt.pause(0.01)
                    continue

                parsed = parse_motor_line(raw_line.decode("utf-8", errors="ignore").strip())
                if parsed is None or parsed[0] != args.motor:
                    continue

                elapsed = time.monotonic() - start_time
                _, turns, angle = parsed
                sample_times.append(elapsed)
                motor_turns.append(turns)
                joint_angles.append(angle)
                writer.writerow([f"{elapsed:.3f}", args.motor, f"{turns:.4f}", f"{angle:.2f}"])
                csv_file.flush()

                latest_ratio = update_plot(plot_items, sample_times, motor_turns, joint_angles)
                ratio_text = "等待足够运动" if latest_ratio is None else f"{latest_ratio:.2f}:1"
                print(f"t={elapsed:7.2f}s turns={turns:9.4f} angle={angle:7.2f}deg ratio={ratio_text}")
    except KeyboardInterrupt:
        print("\n采集已停止")
    except serial.SerialException as error:
        print(f"\n串口读取失败：{error}", file=sys.stderr)
    finally:
        serial_port.close()
        plot_items[0].savefig(png_path, dpi=160)
        print(f"已保存数据：{csv_path}")
        print(f"已保存图表：{png_path}")
        if latest_ratio is not None:
            print(f"最终估算减速比：{latest_ratio:.2f}:1")


if __name__ == "__main__":
    main()

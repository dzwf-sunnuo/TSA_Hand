#!/usr/bin/env python3
"""读取减速比实验串口数据，实时进行多项式拟合并保存结果。"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial


DEFAULT_PORT = "COM8"
DEFAULT_BAUD = 115200
DEFAULT_MOTOR = 1
DEFAULT_DEGREE = 3
MIN_ANGLE_SPAN_DEG = 1.0


# parse_args：解析串口采集和多项式拟合参数
# 参数：无
# 返回值：argparse.Namespace - 解析后的串口、拟合阶数和输出配置
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="实时拟合电机圈数与关节角度的非线性关系")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT, help="串口号，默认COM8")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD, help="波特率，默认115200")
    parser.add_argument("--motor", type=int, default=DEFAULT_MOTOR, choices=range(1, 5), help="采集的电机编号，默认1")
    parser.add_argument("--degree", type=int, default=DEFAULT_DEGREE, choices=range(1, 6), help="多项式阶数，默认3")
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


# fit_polynomial：拟合关节角度变化到电机圈数变化的多项式
# 参数：motor_turns - 电机累计圈数序列；joint_angles - 关节角度序列；degree - 多项式阶数
# 返回值：dict或None - 拟合系数、R²、角度范围和局部减速比，数据不足时返回None
def fit_polynomial(motor_turns, joint_angles, degree):
    if len(motor_turns) < degree + 1:
        return None

    delta_turns = np.asarray(motor_turns, dtype=float) - motor_turns[0]
    delta_angles = np.asarray(joint_angles, dtype=float) - joint_angles[0]
    if np.ptp(delta_angles) < MIN_ANGLE_SPAN_DEG:
        return None

    coefficients = np.polyfit(delta_angles, delta_turns, degree)
    fitted_turns = np.polyval(coefficients, delta_angles)
    residual_sum = float(np.sum((delta_turns - fitted_turns) ** 2))
    total_sum = float(np.sum((delta_turns - np.mean(delta_turns)) ** 2))
    r_squared = 1.0 - residual_sum / total_sum if total_sum > 0.0 else 1.0

    derivative = np.polyder(coefficients)
    current_angle = float(delta_angles[-1])
    local_ratio = abs(float(np.polyval(derivative, current_angle)) * 360.0)
    return {
        "coefficients": coefficients,
        "r_squared": r_squared,
        "angle_min": float(delta_angles.min()),
        "angle_max": float(delta_angles.max()),
        "current_angle": current_angle,
        "local_ratio": local_ratio,
    }


# format_polynomial：将多项式系数格式化为便于记录的公式
# 参数：coefficients - 按最高次幂到常数项排列的系数
# 返回值：str - 以x表示关节角度变化量的公式
def format_polynomial(coefficients) -> str:
    degree = len(coefficients) - 1
    terms = []
    for index, coefficient in enumerate(coefficients):
        power = degree - index
        if power == 0:
            terms.append(f"{coefficient:+.8g}")
        elif power == 1:
            terms.append(f"{coefficient:+.8g}*x")
        else:
            terms.append(f"{coefficient:+.8g}*x^{power}")
    return " ".join(terms).lstrip("+")


# configure_plot：创建实时曲线和多项式拟合图
# 参数：motor_number - 电机编号；degree - 多项式阶数
# 返回值：tuple - 图形、坐标轴和曲线对象
def configure_plot(motor_number: int, degree: int):
    plt.ion()
    figure, (axis_time, axis_relation) = plt.subplots(2, 1, figsize=(10, 8))
    axis_angle = axis_time.twinx()

    turns_line, = axis_time.plot([], [], color="tab:blue", label="Motor turns")
    angle_line, = axis_angle.plot([], [], color="tab:orange", label="Joint angle")
    relation_points, = axis_relation.plot([], [], "o", markersize=3, color="tab:green", label="Samples")
    fit_line, = axis_relation.plot([], [], "-", color="tab:red", label=f"Degree {degree} fit")

    axis_time.set_xlabel("Time (s)")
    axis_time.set_ylabel("Motor turns", color="tab:blue")
    axis_angle.set_ylabel("Joint angle (deg)", color="tab:orange")
    axis_time.grid(True, alpha=0.3)
    axis_time.legend(handles=[turns_line, angle_line], loc="upper left")

    axis_relation.set_xlabel("Joint angle change (deg)")
    axis_relation.set_ylabel("Motor turn change")
    axis_relation.grid(True, alpha=0.3)
    axis_relation.legend(loc="upper left")
    figure.suptitle(f"Motor {motor_number} nonlinear transmission experiment")
    figure.tight_layout()

    return figure, axis_time, axis_angle, axis_relation, turns_line, angle_line, relation_points, fit_line


# update_plot：使用最新数据刷新实时图表和多项式拟合结果
# 参数：plot_items - 绘图对象；times、motor_turns、joint_angles - 采样数据；degree - 多项式阶数
# 返回值：dict或None - 当前多项式拟合结果
def update_plot(plot_items, times, motor_turns, joint_angles, degree):
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

    fit_result = fit_polynomial(motor_turns, joint_angles, degree)
    if fit_result is not None:
        fit_x = np.linspace(delta_angles.min(), delta_angles.max(), 300)
        fit_y = np.polyval(fit_result["coefficients"], fit_x)
        fit_line.set_data(fit_x, fit_y)
        axis_relation.set_title(
            f"Degree {degree} fit, R²={fit_result['r_squared']:.5f}, "
            f"local ratio={fit_result['local_ratio']:.2f}:1"
        )
    else:
        fit_line.set_data([], [])
        axis_relation.set_title(f"Need {degree + 1} samples and at least 1 degree of motion")

    figure.canvas.draw_idle()
    figure.canvas.flush_events()
    plt.pause(0.001)
    return fit_result


# save_fit_result：保存多项式、拟合优度和局部减速比
# 参数：result_path - 结果文件；motor_number - 电机编号；degree - 阶数；fit_result - 拟合结果
# 返回值：无
def save_fit_result(result_path: Path, motor_number: int, degree: int, fit_result) -> None:
    with result_path.open("w", encoding="utf-8") as result_file:
        result_file.write(f"电机编号: {motor_number}\n")
        result_file.write(f"多项式阶数: {degree}\n")
        if fit_result is None:
            result_file.write("拟合失败: 有效采样点或关节运动范围不足\n")
            return

        formula = format_polynomial(fit_result["coefficients"])
        result_file.write("定义: x=相对首个采样点的关节角度变化(度)\n")
        result_file.write("定义: y=相对首个采样点的电机圈数变化(圈)\n")
        result_file.write(f"拟合公式: y = {formula}\n")
        result_file.write(f"R²: {fit_result['r_squared']:.8f}\n")
        result_file.write(
            f"有效角度范围: {fit_result['angle_min']:.4f} 至 "
            f"{fit_result['angle_max']:.4f} 度\n"
        )
        result_file.write(f"末点局部减速比: {fit_result['local_ratio']:.4f}:1\n")


# main：打开串口、记录CSV、实时拟合并保存图表和结果
# 参数：无
# 返回值：无
def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"reduction_ratio_motor{args.motor}_{timestamp}.csv"
    png_path = output_dir / f"reduction_ratio_motor{args.motor}_{timestamp}.png"
    result_path = output_dir / f"reduction_ratio_motor{args.motor}_{timestamp}_fit.txt"

    try:
        serial_port = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as error:
        print(f"无法打开串口 {args.port}: {error}", file=sys.stderr)
        raise SystemExit(1) from error

    plot_items = configure_plot(args.motor, args.degree)
    sample_times = []
    motor_turns = []
    joint_angles = []
    start_time = time.monotonic()
    latest_fit = None

    print(f"正在采集电机{args.motor}：{args.port} @ {args.baud}，按Ctrl+C停止")
    print(f"多项式阶数：{args.degree}")
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

                latest_fit = update_plot(
                    plot_items, sample_times, motor_turns, joint_angles, args.degree
                )
                ratio_text = (
                    "等待足够运动"
                    if latest_fit is None
                    else f"{latest_fit['local_ratio']:.2f}:1"
                )
                print(f"t={elapsed:7.2f}s turns={turns:9.4f} angle={angle:7.2f}deg ratio={ratio_text}")
    except KeyboardInterrupt:
        print("\n采集已停止")
    except serial.SerialException as error:
        print(f"\n串口读取失败：{error}", file=sys.stderr)
    finally:
        serial_port.close()
        plot_items[0].savefig(png_path, dpi=160)
        save_fit_result(result_path, args.motor, args.degree, latest_fit)
        print(f"已保存数据：{csv_path}")
        print(f"已保存图表：{png_path}")
        print(f"已保存拟合结果：{result_path}")
        if latest_fit is not None:
            formula = format_polynomial(latest_fit["coefficients"])
            print(f"拟合公式：y = {formula}")
            print(f"R^2：{latest_fit['r_squared']:.6f}")
            print(f"末点局部减速比：{latest_fit['local_ratio']:.2f}:1")


if __name__ == "__main__":
    main()

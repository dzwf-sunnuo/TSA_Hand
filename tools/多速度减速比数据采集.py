#!/usr/bin/env python3
"""按固件实验标志采集多种输出限幅的数据并生成减速比对比结果。"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial

from plot_motor_reduction_ratio import fit_polynomial, format_polynomial


DEFAULT_PORT = "COM8"
DEFAULT_BAUD = 115200
DEFAULT_MOTOR = 1
DEFAULT_DEGREE = 3
DEFAULT_LIMITS = (20, 40, 60, 80, 100)


# parse_args：解析多速度实验的串口和分析参数
# 参数：无
# 返回值：argparse.Namespace - 串口、目标电机、拟合阶数和输出限幅配置
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="按实验标志采集多输出限幅减速比数据")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT, help="串口号，默认COM8")
    parser.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD, help="波特率，默认115200")
    parser.add_argument("--motor", type=int, default=DEFAULT_MOTOR, choices=range(1, 5), help="电机编号，默认1")
    parser.add_argument("--degree", type=int, default=DEFAULT_DEGREE, choices=range(1, 6), help="多项式阶数，默认3")
    parser.add_argument("--limits", default="20,40,60,80,100", help="需要记录的输出限幅百分比")
    parser.add_argument("--output-dir", default="logs", help="输出目录，默认logs")
    args = parser.parse_args()

    try:
        args.limits = tuple(int(value.strip()) for value in args.limits.split(","))
    except ValueError as error:
        parser.error(f"输出限幅必须是逗号分隔的整数：{error}")
    if not args.limits or any(value < 0 or value > 255 for value in args.limits):
        parser.error("输出限幅必须位于0～255")
    return args


# parse_event_line：解析实验开始或停止标志
# 参数：line - 串口文本行
# 返回值：tuple或None - 事件名称及开始事件的电机、限幅、模式
def parse_event_line(line: str):
    if line == "EXPERIMENT_STOP":
        return "stop", None, None, None

    fields = [field.strip() for field in line.split(",")]
    if len(fields) != 4 or fields[0] != "EXPERIMENT_START":
        return None
    try:
        return "start", int(fields[1]), int(fields[2]), int(fields[3])
    except ValueError:
        return None


# parse_data_line：解析电机编号、电机圈数和关节角度三列CSV
# 参数：line - 串口文本行
# 返回值：tuple或None - 三列有效数据，格式错误时返回None
def parse_data_line(line: str):
    fields = [field.strip() for field in line.split(",")]
    if len(fields) != 3:
        return None
    try:
        motor = int(fields[0])
        turns = float(fields[1])
        angle = float(fields[2])
    except ValueError:
        return None
    if motor not in range(1, 5) or not np.isfinite(turns) or not np.isfinite(angle):
        return None
    return motor, turns, angle


# start_run：根据开始标志创建一段新的实验记录
# 参数：run_id - 实验编号；motor - 电机编号；limit - 输出限幅；mode - 控制模式
# 返回值：dict - 新实验的数据容器
def start_run(run_id: int, motor: int, limit: int, mode: int):
    return {
        "run_id": run_id,
        "motor": motor,
        "limit": limit,
        "mode": mode,
        "start_time": time.monotonic(),
        "base_turns": None,
        "base_angle": None,
        "elapsed": [],
        "turns": [],
        "angles": [],
        "delta_turns": [],
        "delta_angles": [],
    }


# analyze_runs：对每段实验分别执行多项式拟合和线性度分析
# 参数：runs - 全部实验段；degree - 多项式阶数
# 返回值：list - 含有效拟合结果的实验段列表
def analyze_runs(runs, degree: int):
    analyzed = []
    for run in runs:
        fit_result = fit_polynomial(run["turns"], run["angles"], degree)
        if fit_result is None:
            continue
        run["fit"] = fit_result
        analyzed.append(run)
    return analyzed


# save_summary：保存每段实验的拟合和线性度对比摘要
# 参数：summary_path - 摘要CSV路径；runs - 有效实验段；degree - 多项式阶数
# 返回值：无
def save_summary(summary_path: Path, runs, degree: int) -> None:
    with summary_path.open("w", newline="", encoding="utf-8-sig") as summary_file:
        writer = csv.writer(summary_file)
        writer.writerow([
            "run_id", "output_limit_percent", "mode", "samples", "degree",
            "angle_span_deg", "polynomial_r_squared", "polynomial_rmse_turns",
            "linear_r_squared", "linear_rmse_turns", "max_linear_deviation_turns",
            "linearity_percent_fs", "end_local_ratio",
        ])
        for run in runs:
            fit_result = run["fit"]
            linearity = fit_result["linearity"]
            writer.writerow([
                run["run_id"], run["limit"], run["mode"], len(run["turns"]), degree,
                f"{np.ptp(run['angles']):.6f}",
                f"{fit_result['r_squared']:.8f}",
                f"{fit_result['polynomial_rmse']:.8f}",
                f"{linearity['r_squared']:.8f}",
                f"{linearity['rmse']:.8f}",
                f"{linearity['max_deviation']:.8f}",
                "" if linearity["linearity_percent_fs"] is None else f"{linearity['linearity_percent_fs']:.8f}",
                f"{fit_result['local_ratio']:.8f}",
            ])


# save_report：保存便于直接阅读的多速度实验报告
# 参数：report_path - 文本报告路径；runs - 有效实验段；degree - 多项式阶数
# 返回值：无
def save_report(report_path: Path, runs, degree: int) -> None:
    with report_path.open("w", encoding="utf-8") as report_file:
        report_file.write(f"多速度减速比对比，多项式阶数={degree}\n\n")
        for run in runs:
            fit_result = run["fit"]
            linearity = fit_result["linearity"]
            formula = format_polynomial(fit_result["coefficients"])
            report_file.write(
                f"实验{run['run_id']}：限幅={run['limit']}%，模式={run['mode']}，"
                f"样本={len(run['turns'])}\n"
            )
            report_file.write(f"多项式：y = {formula}\n")
            report_file.write(f"多项式R²：{fit_result['r_squared']:.8f}\n")
            report_file.write(f"多项式RMSE：{fit_result['polynomial_rmse']:.8f}圈\n")
            report_file.write(f"线性R²：{linearity['r_squared']:.8f}\n")
            report_file.write(f"线性RMSE：{linearity['rmse']:.8f}圈\n")
            if linearity["linearity_percent_fs"] is not None:
                report_file.write(f"线性度误差：{linearity['linearity_percent_fs']:.6f}%FS\n")
            report_file.write(f"末点局部减速比：{fit_result['local_ratio']:.4f}:1\n\n")


# plot_comparison：绘制不同输出限幅下的曲线和指标对比图
# 参数：figure_path - PNG路径；runs - 有效实验段；degree - 多项式阶数
# 返回值：无
def plot_comparison(figure_path: Path, runs, degree: int) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(13, 9))
    axis_curve, axis_ratio, axis_linearity, axis_rmse = axes.flat

    labels = []
    linearity_values = []
    linear_rmse_values = []
    polynomial_rmse_values = []

    for run in runs:
        fit_result = run["fit"]
        label = f"{run['limit']}% run{run['run_id']}"
        labels.append(label)
        angle_min = fit_result["angle_min"]
        angle_max = fit_result["angle_max"]
        fit_x = np.linspace(angle_min, angle_max, 300)
        fit_y = np.polyval(fit_result["coefficients"], fit_x)
        derivative = np.polyder(fit_result["coefficients"])
        local_ratio = np.abs(np.polyval(derivative, fit_x)) * 360.0

        axis_curve.plot(run["delta_angles"], run["delta_turns"], ".", alpha=0.35)
        axis_curve.plot(fit_x, fit_y, label=label)
        axis_ratio.plot(fit_x, local_ratio, label=label)

        linearity = fit_result["linearity"]
        linearity_values.append(
            np.nan if linearity["linearity_percent_fs"] is None
            else linearity["linearity_percent_fs"]
        )
        linear_rmse_values.append(linearity["rmse"])
        polynomial_rmse_values.append(fit_result["polynomial_rmse"])

    axis_curve.set_title(f"Degree {degree} transmission curves")
    axis_curve.set_xlabel("Joint angle change (deg)")
    axis_curve.set_ylabel("Motor turn change")
    axis_curve.grid(True, alpha=0.3)
    axis_curve.legend()

    axis_ratio.set_title("Local reduction ratio")
    axis_ratio.set_xlabel("Joint angle change (deg)")
    axis_ratio.set_ylabel("Ratio")
    axis_ratio.grid(True, alpha=0.3)
    axis_ratio.legend()

    positions = np.arange(len(labels))
    axis_linearity.bar(positions, linearity_values, color="tab:orange")
    axis_linearity.set_title("BFSL linearity error")
    axis_linearity.set_ylabel("%FS")
    axis_linearity.set_xticks(positions, labels, rotation=30, ha="right")
    axis_linearity.grid(True, axis="y", alpha=0.3)

    width = 0.38
    axis_rmse.bar(positions - width / 2, linear_rmse_values, width, label="Linear")
    axis_rmse.bar(positions + width / 2, polynomial_rmse_values, width, label="Polynomial")
    axis_rmse.set_title("Fit RMSE comparison")
    axis_rmse.set_ylabel("Motor turns")
    axis_rmse.set_xticks(positions, labels, rotation=30, ha="right")
    axis_rmse.grid(True, axis="y", alpha=0.3)
    axis_rmse.legend()

    figure.tight_layout()
    figure.savefig(figure_path, dpi=160)
    plt.close(figure)


# main：等待固件标志、分段记录数据并生成多速度对比结果
# 参数：无
# 返回值：无
def main() -> None:
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = output_dir / f"多速度减速比原始数据_{timestamp}.csv"
    summary_path = output_dir / f"多速度减速比汇总_{timestamp}.csv"
    report_path = output_dir / f"多速度减速比报告_{timestamp}.txt"
    figure_path = output_dir / f"多速度减速比对比_{timestamp}.png"

    try:
        serial_port = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as error:
        print(f"无法打开串口{args.port}：{error}", file=sys.stderr)
        raise SystemExit(1) from error

    runs = []
    current_run = None
    next_run_id = 1
    allowed_limits = set(args.limits)

    print(f"等待实验标志：{args.port} @ {args.baud}")
    print(f"记录限幅：{', '.join(str(value) + '%' for value in args.limits)}")
    print("收到EXPERIMENT_START前的数据会被自动丢弃，按Ctrl+C结束全部采集")

    try:
        with raw_path.open("w", newline="", encoding="utf-8-sig") as raw_file:
            writer = csv.writer(raw_file)
            writer.writerow([
                "run_id", "output_limit_percent", "mode", "elapsed_s", "motor",
                "motor_turns", "joint_angle_deg", "delta_motor_turns", "delta_joint_angle_deg",
            ])

            while True:
                raw_line = serial_port.readline()
                if not raw_line:
                    continue
                line = raw_line.decode("utf-8", errors="ignore").strip()

                event = parse_event_line(line)
                if event is not None:
                    event_name, motor, limit, mode = event
                    if event_name == "stop":
                        if current_run is not None:
                            print(f"实验{current_run['run_id']}停止，共{len(current_run['turns'])}点")
                        current_run = None
                    elif motor == args.motor and limit in allowed_limits:
                        current_run = start_run(next_run_id, motor, limit, mode)
                        runs.append(current_run)
                        print(f"实验{next_run_id}开始：限幅{limit}%，模式{mode}")
                        next_run_id += 1
                    else:
                        current_run = None
                    continue

                data = parse_data_line(line)
                if current_run is None or data is None or data[0] != args.motor:
                    continue

                motor, turns, angle = data
                if current_run["base_turns"] is None:
                    current_run["base_turns"] = turns
                    current_run["base_angle"] = angle
                    current_run["start_time"] = time.monotonic()

                elapsed = time.monotonic() - current_run["start_time"]
                delta_turns = turns - current_run["base_turns"]
                delta_angle = angle - current_run["base_angle"]
                current_run["elapsed"].append(elapsed)
                current_run["turns"].append(turns)
                current_run["angles"].append(angle)
                current_run["delta_turns"].append(delta_turns)
                current_run["delta_angles"].append(delta_angle)
                writer.writerow([
                    current_run["run_id"], current_run["limit"], current_run["mode"],
                    f"{elapsed:.3f}", motor, f"{turns:.4f}", f"{angle:.2f}",
                    f"{delta_turns:.4f}", f"{delta_angle:.2f}",
                ])
                raw_file.flush()
    except KeyboardInterrupt:
        print("\n采集结束，正在生成对比结果")
    except serial.SerialException as error:
        print(f"\n串口读取失败：{error}", file=sys.stderr)
    finally:
        serial_port.close()

    analyzed_runs = analyze_runs(runs, args.degree)
    if not analyzed_runs:
        print(f"原始数据已保存：{raw_path}")
        print("没有满足拟合条件的实验段，未生成对比图")
        return

    save_summary(summary_path, analyzed_runs, args.degree)
    save_report(report_path, analyzed_runs, args.degree)
    plot_comparison(figure_path, analyzed_runs, args.degree)
    print(f"原始数据：{raw_path}")
    print(f"指标汇总：{summary_path}")
    print(f"分析报告：{report_path}")
    print(f"对比图表：{figure_path}")


if __name__ == "__main__":
    main()

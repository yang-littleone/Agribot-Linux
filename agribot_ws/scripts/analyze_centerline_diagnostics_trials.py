#!/usr/bin/env python3
"""Analyze centerline detection diagnostics from repeated ROS 2 bag trials."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import matplotlib.pyplot as plt
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32MultiArray


DIAG_FIELDS = [
    "valid",
    "left_points",
    "right_points",
    "corridor_width_m",
    "safety_margin_m",
    "confidence",
    "row_yaw_rad",
    "center_offset_m",
    "left_slope",
    "left_intercept_m",
    "right_slope",
    "right_intercept_m",
    "line_lost_count",
    "path_points",
]


def bag_reader(bag_dir: Path):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    return reader, topic_types


def read_trial(trial_dir: Path) -> Dict[str, List[dict]]:
    bag_dir = trial_dir / "bag"
    reader, topic_types = bag_reader(bag_dir)
    type_map = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    data = {
        "diag": [],
        "cmd": [],
        "odom": [],
    }
    first_stamp = None
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if first_stamp is None:
            first_stamp = stamp
        t = (stamp - first_stamp) * 1e-9

        if topic == "/centerline_detection_diagnostics":
            msg: Float32MultiArray = deserialize_message(raw, type_map[topic])
            if len(msg.data) >= len(DIAG_FIELDS):
                row = {"time_s": t}
                row.update(dict(zip(DIAG_FIELDS, [float(v) for v in msg.data[: len(DIAG_FIELDS)]])))
                row["row_yaw_deg"] = math.degrees(row["row_yaw_rad"])
                data["diag"].append(row)
        elif topic == "/cmd_vel":
            msg: Twist = deserialize_message(raw, type_map[topic])
            data["cmd"].append(
                {
                    "time_s": t,
                    "linear_x": float(msg.linear.x),
                    "angular_z": float(msg.angular.z),
                }
            )
        elif topic == "/odom":
            msg: Odometry = deserialize_message(raw, type_map[topic])
            data["odom"].append(
                {
                    "time_s": t,
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "linear_x": float(msg.twist.twist.linear.x),
                    "angular_z": float(msg.twist.twist.angular.z),
                }
            )
    return data


def mean(values: Iterable[float]) -> float:
    vals = list(values)
    return statistics.mean(vals) if vals else float("nan")


def stdev(values: Iterable[float]) -> float:
    vals = list(values)
    return statistics.stdev(vals) if len(vals) > 1 else 0.0


def rms(values: Iterable[float]) -> float:
    vals = list(values)
    return math.sqrt(mean(v * v for v in vals)) if vals else float("nan")


def p95_abs(values: Iterable[float]) -> float:
    vals = sorted(abs(v) for v in values)
    if not vals:
        return float("nan")
    idx = min(len(vals) - 1, int(0.95 * (len(vals) - 1)))
    return vals[idx]


def summarize_trial(trial_name: str, data: Dict[str, List[dict]]) -> dict:
    diag = data["diag"]
    cmd = data["cmd"]
    odom = data["odom"]
    valid = [r["valid"] for r in diag]
    center = [r["center_offset_m"] for r in diag if r["valid"] >= 0.5]
    yaw = [r["row_yaw_deg"] for r in diag if r["valid"] >= 0.5]
    conf = [r["confidence"] for r in diag]
    margin = [r["safety_margin_m"] for r in diag]
    width = [r["corridor_width_m"] for r in diag]
    hold = [r["line_lost_count"] for r in diag]
    speeds = [r["linear_x"] for r in cmd]
    angular = [r["angular_z"] for r in cmd]

    distance = 0.0
    for a, b in zip(odom, odom[1:]):
        distance += math.hypot(b["x"] - a["x"], b["y"] - a["y"])

    return {
        "trial": trial_name,
        "duration_s": max([r["time_s"] for r in diag + cmd + odom], default=0.0),
        "diag_samples": len(diag),
        "cmd_samples": len(cmd),
        "odom_samples": len(odom),
        "valid_ratio_pct": 100.0 * mean(1.0 if v >= 0.5 else 0.0 for v in valid),
        "center_offset_mean_m": mean(center),
        "center_offset_rmse_m": rms(center),
        "center_offset_std_m": stdev(center),
        "center_offset_p95_abs_m": p95_abs(center),
        "row_yaw_mean_deg": mean(yaw),
        "row_yaw_std_deg": stdev(yaw),
        "confidence_mean": mean(conf),
        "confidence_low_ratio_pct": 100.0 * mean(1.0 if v < 0.45 else 0.0 for v in conf),
        "safety_margin_mean_m": mean(margin),
        "safety_margin_low_ratio_pct": 100.0 * mean(1.0 if v < 0.10 else 0.0 for v in margin),
        "corridor_width_mean_m": mean(width),
        "hold_ratio_pct": 100.0 * mean(1.0 if v > 0.0 else 0.0 for v in hold),
        "path_points_mean": mean(r["path_points"] for r in diag),
        "mean_cmd_linear_mps": mean(abs(v) for v in speeds),
        "zero_speed_ratio_pct": 100.0 * mean(1.0 if abs(v) < 1e-3 else 0.0 for v in speeds),
        "mean_cmd_angular_abs_rps": mean(abs(v) for v in angular),
        "distance_m": distance,
    }


def write_csv(path: Path, rows: List[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_timeseries_csv(path: Path, all_data: Dict[str, Dict[str, List[dict]]]) -> None:
    fields = ["trial", "time_s"] + DIAG_FIELDS + ["row_yaw_deg"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for trial, data in all_data.items():
            for row in data["diag"]:
                out = {"trial": trial}
                out.update(row)
                writer.writerow(out)


def grouped_stats(rows: List[dict], key: str) -> Tuple[float, float, float, float]:
    vals = [r[key] for r in rows]
    return mean(vals), stdev(vals), min(vals), max(vals)


def make_plots(fig_dir: Path, all_data: Dict[str, Dict[str, List[dict]]], rows: List[dict]) -> None:
    fig_dir.mkdir(parents=True, exist_ok=True)

    plot_specs = [
        ("center_offset_m", "Center offset (m)", "center_offset_timeseries.png"),
        ("row_yaw_deg", "Row yaw (deg)", "row_yaw_timeseries.png"),
        ("confidence", "Corridor confidence", "confidence_timeseries.png"),
        ("safety_margin_m", "Safety margin (m)", "safety_margin_timeseries.png"),
        ("corridor_width_m", "Corridor width (m)", "corridor_width_timeseries.png"),
    ]
    for key, ylabel, filename in plot_specs:
        plt.figure(figsize=(10, 5))
        for trial, data in all_data.items():
            diag = data["diag"]
            plt.plot([r["time_s"] for r in diag], [r[key] for r in diag], linewidth=1.0, label=trial)
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        plt.grid(True, alpha=0.3)
        plt.legend(ncol=2, fontsize=8)
        plt.tight_layout()
        plt.savefig(fig_dir / filename, dpi=200)
        plt.close()

    bar_keys = [
        ("valid_ratio_pct", "Valid ratio (%)"),
        ("center_offset_rmse_m", "Center offset RMSE (m)"),
        ("row_yaw_std_deg", "Row yaw std (deg)"),
        ("zero_speed_ratio_pct", "Zero speed ratio (%)"),
    ]
    plt.figure(figsize=(11, 5))
    names = [r["trial"] for r in rows]
    x = range(len(rows))
    width = 0.2
    for idx, (key, label) in enumerate(bar_keys):
        vals = [r[key] for r in rows]
        offset = (idx - 1.5) * width
        plt.bar([i + offset for i in x], vals, width=width, label=label)
    plt.xticks(list(x), names)
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(fig_dir / "summary_bars.png", dpi=200)
    plt.close()


def write_markdown(path: Path, fig_dir: Path, rows: List[dict], root: Path) -> None:
    lines: List[str] = []
    lines.append("# zhenshi4hang16m 五次中心线检测诊断汇总")
    lines.append("")
    lines.append(f"- 数据目录：`{root}`")
    lines.append("- 数据来源：`/centerline_detection_diagnostics`、`/cmd_vel`、`/odom`")
    lines.append("- 实验组：质量感知 PID + 中心线检测诊断")
    lines.append("")
    lines.append("## 单次实验结果")
    lines.append("")
    lines.append("| Trial | 时长/s | 诊断帧 | 有效率/% | 中心偏移RMSE/m | 中心偏移Std/m | 行向角Std/deg | 平均置信度 | 平均安全裕度/m | 零速比例/% | 行驶距离/m |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in rows:
        lines.append(
            f"| {r['trial']} | {r['duration_s']:.1f} | {r['diag_samples']:.0f} | "
            f"{r['valid_ratio_pct']:.2f} | {r['center_offset_rmse_m']:.4f} | "
            f"{r['center_offset_std_m']:.4f} | {r['row_yaw_std_deg']:.3f} | "
            f"{r['confidence_mean']:.3f} | {r['safety_margin_mean_m']:.3f} | "
            f"{r['zero_speed_ratio_pct']:.2f} | {r['distance_m']:.3f} |"
        )
    lines.append("")
    lines.append("## 组内统计")
    lines.append("")
    metrics = [
        ("valid_ratio_pct", "中心线有效率/%"),
        ("center_offset_rmse_m", "中心偏移RMSE/m"),
        ("center_offset_std_m", "中心偏移标准差/m"),
        ("center_offset_p95_abs_m", "中心偏移P95绝对值/m"),
        ("row_yaw_std_deg", "行向角标准差/deg"),
        ("confidence_mean", "平均置信度"),
        ("confidence_low_ratio_pct", "低置信比例/%"),
        ("safety_margin_mean_m", "平均安全裕度/m"),
        ("safety_margin_low_ratio_pct", "低安全裕度比例/%"),
        ("corridor_width_mean_m", "平均走廊宽度/m"),
        ("hold_ratio_pct", "历史模型保持比例/%"),
        ("zero_speed_ratio_pct", "零速比例/%"),
        ("distance_m", "行驶距离/m"),
    ]
    lines.append("| 指标 | 均值 | 标准差 | 最小值 | 最大值 |")
    lines.append("|---|---:|---:|---:|---:|")
    for key, label in metrics:
        m, s, mn, mx = grouped_stats(rows, key)
        lines.append(f"| {label} | {m:.4f} | {s:.4f} | {mn:.4f} | {mx:.4f} |")
    lines.append("")
    lines.append("## 曲线图")
    lines.append("")
    for title, name in [
        ("中心偏移曲线", "center_offset_timeseries.png"),
        ("行向角曲线", "row_yaw_timeseries.png"),
        ("走廊置信度曲线", "confidence_timeseries.png"),
        ("安全裕度曲线", "safety_margin_timeseries.png"),
        ("走廊宽度曲线", "corridor_width_timeseries.png"),
        ("关键指标柱状图", "summary_bars.png"),
    ]:
        lines.append(f"### {title}")
        lines.append("")
        lines.append(f"![{title}](figures/zhenshi4hang16m_detection_diagnostics/{name})")
        lines.append("")
    lines.append("## 结果说明")
    lines.append("")
    valid_mean, _, _, _ = grouped_stats(rows, "valid_ratio_pct")
    center_rmse, _, _, _ = grouped_stats(rows, "center_offset_rmse_m")
    yaw_std, _, _, _ = grouped_stats(rows, "row_yaw_std_deg")
    conf, _, _, _ = grouped_stats(rows, "confidence_mean")
    margin, _, _, _ = grouped_stats(rows, "safety_margin_mean_m")
    zero, _, _, _ = grouped_stats(rows, "zero_speed_ratio_pct")
    lines.append(f"- 五次实验中心线有效率均值为 `{valid_mean:.2f}%`，说明在该场景下中心线输出连续性较好。")
    lines.append(f"- 中心偏移 RMSE 均值为 `{center_rmse:.4f} m`，可作为中心线横向稳定性的量化指标。")
    lines.append(f"- 行向角标准差均值为 `{yaw_std:.3f} deg`，说明行方向估计存在一定短时波动，但整体处于较小角度范围。")
    lines.append(f"- 平均置信度为 `{conf:.3f}`，平均安全裕度为 `{margin:.3f} m`，表明检测得到的通行走廊质量较高。")
    lines.append(f"- 零速比例均值为 `{zero:.2f}%`，该指标反映控制器因低质量感知、路径缺失或速度调节导致的停顿程度。")
    lines.append("")
    lines.append("## 可写入论文的表述")
    lines.append("")
    lines.append(
        "在 `zhenshi4hang16m.world` 场景下进行五次重复实验，所提出的中心线检测诊断结果显示，"
        f"中心线有效率均值为 {valid_mean:.2f}%，中心偏移 RMSE 均值为 {center_rmse:.4f} m，"
        f"行方向角标准差均值为 {yaw_std:.3f} deg，平均走廊置信度为 {conf:.3f}，"
        f"平均安全裕度为 {margin:.3f} m。结果表明，该方法在多行玉米仿真场景中能够保持较高的中心线输出连续性和较稳定的行间走廊估计。"
    )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.home() / "agribot_test_logs/zhenshi4hang16m_quality_pid_diag")
    parser.add_argument("--out-doc", type=Path, default=Path("docs/zhenshi4hang16m_detection_diagnostics_summary.md"))
    parser.add_argument("--out-csv", type=Path, default=Path("docs/zhenshi4hang16m_detection_diagnostics_summary.csv"))
    parser.add_argument("--out-timeseries", type=Path, default=Path("docs/zhenshi4hang16m_detection_diagnostics_timeseries.csv"))
    parser.add_argument("--fig-dir", type=Path, default=Path("docs/figures/zhenshi4hang16m_detection_diagnostics"))
    args = parser.parse_args()

    trial_dirs = sorted(args.root.glob("trial_0*"))
    if not trial_dirs:
        raise SystemExit(f"No trials found under {args.root}")

    all_data = {}
    rows = []
    for trial_dir in trial_dirs:
        data = read_trial(trial_dir)
        all_data[trial_dir.name] = data
        rows.append(summarize_trial(trial_dir.name, data))

    args.out_doc.parent.mkdir(parents=True, exist_ok=True)
    args.fig_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out_csv, rows)
    write_timeseries_csv(args.out_timeseries, all_data)
    make_plots(args.fig_dir, all_data, rows)
    write_markdown(args.out_doc, args.fig_dir, rows, args.root)
    print(args.out_doc)
    print(args.out_csv)
    print(args.out_timeseries)
    print(args.fig_dir)


if __name__ == "__main__":
    main()

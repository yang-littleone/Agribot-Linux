#!/usr/bin/env python3
import argparse
import csv
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


PERCEPTION_PROFILES = {
    "towrow.world": {
        "min_cluster_size": 2,
        "min_section_points": 1,
        "min_innermost_seeds": 2,
        "desired_row_separation": 1.0,
        "min_row_separation": 0.2,
        "max_row_separation": 2.0,
        "use_simple_inner_row_mode": False,
    },
    "corn_leaf_world.world": {
        "min_cluster_size": 2,
        "min_section_points": 1,
        "min_innermost_seeds": 2,
        "desired_row_separation": 1.5,
        "min_row_separation": 0.4,
        "max_row_separation": 2.2,
        "y_min": -1.3,
        "y_max": 1.3,
        "use_simple_inner_row_mode": False,
    },
    "cornlinens.world": {
        "min_cluster_size": 2,
        "min_section_points": 1,
        "min_innermost_seeds": 2,
        "desired_row_separation": 1.0,
        "min_row_separation": 0.2,
        "max_row_separation": 2.0,
        "use_simple_inner_row_mode": False,
    },
    "cornlinens_angular2.world": {
        "min_cluster_size": 2,
        "min_section_points": 1,
        "min_innermost_seeds": 2,
        "desired_row_separation": 0.8,
        "min_row_separation": 0.2,
        "max_row_separation": 1.8,
        "use_simple_inner_row_mode": False,
    },
    "twoworld.world": {},
}


def bash_cmd(command: str) -> list[str]:
    return ["bash", "-lc", f"source /opt/ros/humble/setup.bash && source {ROOT}/install/setup.bash && {command}"]


def start_process(command: str, log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "w")
    return subprocess.Popen(
        bash_cmd(command),
        cwd=ROOT,
        stdout=log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )


def stop_process(proc: subprocess.Popen, timeout: float = 8.0):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=timeout)


def run_capture(command: str, timeout: float = 20.0) -> str:
    return subprocess.check_output(
        bash_cmd(command),
        cwd=ROOT,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        text=True,
    )


def wait_for_topic(topic: str, timeout: float = 45.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            output = run_capture("ros2 topic list", timeout=6.0)
            if topic in output.splitlines():
                return True
        except subprocess.CalledProcessError:
            pass
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


def params_to_args(params: dict[str, object]) -> str:
    if not params:
        return ""
    pieces = ["--ros-args"]
    for key, value in params.items():
        if isinstance(value, bool):
            value_text = "true" if value else "false"
        else:
            value_text = str(value)
        pieces.extend(["-p", f"{key}:={value_text}"])
    return " ".join(pieces)


def append_combined_summary(combined_path: Path, world: str, controller: str, summary_path: Path):
    with open(summary_path, newline="") as f:
        rows = list(csv.DictReader(f))
    write_header = not combined_path.exists()
    with open(combined_path, "a", newline="") as f:
        fieldnames = ["world", "controller"] + list(rows[0].keys())
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        for row in rows:
            writer.writerow({"world": world, "controller": controller, **row})


def write_comparison_report(root: Path, combined_path: Path, worlds: list[str], trials: int, gui: bool):
    rows = list(csv.DictReader(open(combined_path)))
    groups = {}
    for row in rows:
        key = (row["world"], row["controller"])
        groups.setdefault(key, []).append(row)

    metrics = [
        ("distance", "行驶距离 (m)"),
        ("final_x", "最终 x (m)"),
        ("mean_abs_y", "平均横向误差 (m)"),
        ("max_abs_y", "最大横向误差 (m)"),
        ("mean_confidence", "平均置信度"),
        ("mean_width", "平均走廊宽度 (m)"),
        ("mean_safety_margin", "平均安全裕度 (m)"),
        ("centerline_loss_ratio", "中心线丢失率"),
        ("mean_speed_cmd", "平均速度指令 (m/s)"),
    ]

    def mean_std(values):
        vals = [float(v) for v in values]
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        return mean, var ** 0.5

    report = root / "comparison_report.md"
    with open(report, "w") as f:
        f.write("# 不同世界与控制算法对比实验\n\n")
        f.write("本实验对比 Gazebo 世界和两种控制算法。由于 Livox 点云插件在 `gui:=false` 下不发布 `/mid360_PointCloud2` 实测数据，本轮采用 `gui:=true` 运行，以保证传感器数据有效。\n\n")
        f.write("对比组合如下：\n\n")
        f.write("- 世界：" + "、".join(f"`{world}`" for world in worlds) + "\n")
        f.write("- 控制算法：PID、纯追踪\n")
        f.write(f"- 每组重复次数：{trials} 次\n")
        f.write(f"- Gazebo UI：`{'true' if gui else 'false'}`\n\n")
        f.write("## 汇总结果\n\n")
        f.write("| 世界 | 控制器 | 行驶距离 m | 最终 x m | 平均横向误差 m | 最大横向误差 m | 平均置信度 | 平均宽度 m | 中心线丢失率 | 平均速度 m/s |\n")
        f.write("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for (world, controller), group_rows in sorted(groups.items()):
            values = {}
            for key, _ in metrics:
                values[key] = mean_std([r[key] for r in group_rows])
            f.write(
                f"| {world} | {controller} | "
                f"{values['distance'][0]:.3f}±{values['distance'][1]:.3f} | "
                f"{values['final_x'][0]:.3f}±{values['final_x'][1]:.3f} | "
                f"{values['mean_abs_y'][0]:.3f}±{values['mean_abs_y'][1]:.3f} | "
                f"{values['max_abs_y'][0]:.3f}±{values['max_abs_y'][1]:.3f} | "
                f"{values['mean_confidence'][0]:.3f}±{values['mean_confidence'][1]:.3f} | "
                f"{values['mean_width'][0]:.3f}±{values['mean_width'][1]:.3f} | "
                f"{values['centerline_loss_ratio'][0]:.3f}±{values['centerline_loss_ratio'][1]:.3f} | "
                f"{values['mean_speed_cmd'][0]:.3f}±{values['mean_speed_cmd'][1]:.3f} |\n"
            )
        f.write("\n## 输出文件\n\n")
        f.write(f"- 总汇总 CSV：`{combined_path.name}`\n")
        for (world, controller) in sorted(groups.keys()):
            f.write(f"- `{world}` + `{controller}`：`{world}_{controller}/`\n")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--max-duration", type=float, default=180.0)
    parser.add_argument("--stop-hold", type=float, default=2.0)
    parser.add_argument("--output-root", default=None)
    parser.add_argument("--worlds", nargs="+", default=["towrow.world", "corn_leaf_world.world"])
    parser.add_argument("--gui", choices=["true", "false"], default="true")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = Path(args.output_root or ROOT / "experiments" / f"world_controller_comparison_{stamp}")
    output_root.mkdir(parents=True, exist_ok=True)
    combined_summary = output_root / "combined_summary.csv"

    worlds = args.worlds
    controllers = ["pid", "pure_pursuit"]

    for world in worlds:
        for controller in controllers:
            name = f"{world}_{controller}"
            combo_dir = output_root / name
            log_dir = combo_dir / "logs"
            print(f"[INFO] Running {world} / {controller}", flush=True)
            procs = []
            try:
                procs.append(start_process(
                    f"ros2 launch diff_drive_robot robot.launch.py world:={world} gui:={args.gui}",
                    log_dir / "gazebo.log",
                ))
                if not wait_for_topic("/odom", timeout=60.0):
                    raise RuntimeError(f"/odom not available for {name}")
                if not wait_for_topic("/mid360_PointCloud2", timeout=60.0):
                    raise RuntimeError(f"/mid360_PointCloud2 not available for {name}")

                procs.append(start_process(
                    "ros2 run centerline_extraction corn_row_detector_projection "
                    f"{params_to_args(PERCEPTION_PROFILES.get(world, {}))}",
                    log_dir / "perception.log",
                ))
                if not wait_for_topic("/corn_row_center_line", timeout=60.0):
                    raise RuntimeError(f"/corn_row_center_line not available for {name}")

                procs.append(start_process(
                    f"ros2 run centerline_extraction cornfield_navigation_node --controller_type {controller}",
                    log_dir / "controller.log",
                ))
                time.sleep(2.0)

                run_capture(
                    "python3 scripts/run_sci_batch_experiment.py "
                    f"--trials {args.trials} "
                    f"--max-duration {args.max_duration} "
                    f"--stop-hold {args.stop_hold} "
                    f"--output-dir {combo_dir}",
                    timeout=args.trials * args.max_duration + 180.0,
                )
                append_combined_summary(combined_summary, world, controller, combo_dir / "summary.csv")
            finally:
                for proc in reversed(procs):
                    stop_process(proc)
                time.sleep(2.0)

    report = write_comparison_report(output_root, combined_summary, worlds, args.trials, args.gui == "true")
    print(f"[INFO] Report written: {report}", flush=True)


if __name__ == "__main__":
    main()

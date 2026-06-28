#!/usr/bin/env python3
"""Run a PID path tracking accuracy test with a known reference path.

The script launches Gazebo, publishes a synthetic path containing a straight
segment, a circular arc, and another straight segment, runs the existing
cornfield_navigation_node PID controller, and evaluates odometry tracking
accuracy against the known reference path.
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


@dataclass
class ReferencePath:
    points: List[Tuple[float, float]]
    cumulative_s: List[float]
    segment_breaks: Dict[str, Tuple[float, float]]


@dataclass
class OdomSample:
    t: float
    x: float
    y: float
    yaw: float
    v: float
    w: float


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def build_reference_path(step: float = 0.05) -> ReferencePath:
    points: List[Tuple[float, float]] = []
    segment_breaks: Dict[str, Tuple[float, float]] = {}

    def append_point(x: float, y: float) -> None:
        if not points or math.hypot(points[-1][0] - x, points[-1][1] - y) > 1e-6:
            points.append((x, y))

    # Segment 1: straight line along +x, length 4 m.
    straight_1_length = 4.0
    straight_1_steps = max(1, int(round(straight_1_length / step)))
    for i in range(straight_1_steps + 1):
        ratio = i / straight_1_steps
        append_point(straight_1_length * ratio, 0.0)

    cumulative_s = compute_cumulative_s(points)
    straight_1_start = 0.0
    straight_1_end = cumulative_s[-1]

    # Segment 2: quarter-circle left turn, radius 2 m.
    # Center is (4, 2); angle from -90 deg to 0 deg.
    # The final point is forced to exactly (6, 2) so the next straight
    # segment is geometrically continuous in RViz and in the error evaluator.
    radius = 2.0
    center_x, center_y = 4.0, 2.0
    arc_length = radius * math.pi / 2.0
    arc_steps = max(1, int(round(arc_length / step)))
    for i in range(arc_steps + 1):
        ratio = i / arc_steps
        theta = -math.pi / 2.0 + ratio * (math.pi / 2.0)
        append_point(center_x + radius * math.cos(theta), center_y + radius * math.sin(theta))

    cumulative_s = compute_cumulative_s(points)
    arc_start = straight_1_end
    arc_end = cumulative_s[-1]

    # Segment 3: straight line after the arc, heading +y, length 3 m.
    straight_2_length = 3.0
    straight_2_steps = max(1, int(round(straight_2_length / step)))
    for i in range(1, straight_2_steps + 1):
        ratio = i / straight_2_steps
        append_point(6.0, 2.0 + straight_2_length * ratio)

    cumulative_s = compute_cumulative_s(points)
    segment_breaks["straight_1"] = (straight_1_start, straight_1_end)
    segment_breaks["arc"] = (arc_start, arc_end)
    segment_breaks["straight_2"] = (arc_end, cumulative_s[-1])
    return ReferencePath(points=points, cumulative_s=cumulative_s, segment_breaks=segment_breaks)


def compute_cumulative_s(points: Sequence[Tuple[float, float]]) -> List[float]:
    cumulative = [0.0]
    for p0, p1 in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.hypot(p1[0] - p0[0], p1[1] - p0[1]))
    return cumulative


def nearest_path_error(
    x: float, y: float, ref: ReferencePath
) -> Tuple[float, float, float, float, float]:
    """Return signed error, absolute error, progress s, reference yaw, nearest x/y."""
    best_dist = float("inf")
    best_signed = 0.0
    best_s = 0.0
    best_yaw = 0.0
    best_xy = ref.points[0]

    for i, (p0, p1) in enumerate(zip(ref.points, ref.points[1:])):
        x0, y0 = p0
        x1, y1 = p1
        vx, vy = x1 - x0, y1 - y0
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq < 1e-12:
            continue
        t = ((x - x0) * vx + (y - y0) * vy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        nx = x0 + t * vx
        ny = y0 + t * vy
        dx, dy = x - nx, y - ny
        dist = math.hypot(dx, dy)
        if dist < best_dist:
            seg_len = math.sqrt(seg_len_sq)
            cross = vx * (y - y0) - vy * (x - x0)
            best_signed = dist if cross >= 0.0 else -dist
            best_dist = dist
            best_s = ref.cumulative_s[i] + t * seg_len
            best_yaw = math.atan2(vy, vx)
            best_xy = (nx, ny)

    return best_signed, abs(best_signed), best_s, best_yaw, best_xy[0], best_xy[1]


class PathPublisherAndRecorder(Node):
    def __init__(self, ref: ReferencePath, publish_rate_hz: float) -> None:
        super().__init__("pid_tracking_accuracy_test")
        self.ref = ref
        self.samples: List[OdomSample] = []
        self.start_time: Optional[float] = None
        self.path_pub = self.create_publisher(NavPath, "/corn_row_center_line", 10)
        self.marker_pub = self.create_publisher(Marker, "/pid_test_reference_path_marker", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.conf_pub = self.create_publisher(Float32, "/corridor_confidence", 10)
        self.margin_pub = self.create_publisher(Float32, "/corridor_safety_margin", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_path)

    def publish_path(self) -> None:
        msg = NavPath()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.ref.points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.08
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.publish_path_marker()
        conf = Float32()
        conf.data = 1.0
        margin = Float32()
        margin.data = 1.0
        self.conf_pub.publish(conf)
        self.margin_pub.publish(margin)

    def publish_path_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pid_test_reference_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1

        for x, y in self.ref.points:
            point = PoseStamped().pose.position
            point.x = x
            point.y = y
            point.z = 0.16
            marker.points.append(point)

        self.marker_pub.publish(marker)

    def odom_callback(self, msg: Odometry) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        self.samples.append(
            OdomSample(
                t=now - self.start_time,
                x=float(msg.pose.pose.position.x),
                y=float(msg.pose.pose.position.y),
                yaw=yaw_from_quaternion(msg.pose.pose.orientation),
                v=float(msg.twist.twist.linear.x),
                w=float(msg.twist.twist.angular.z),
            )
        )

    def publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())


def reached_reference_goal(
    ref: ReferencePath,
    sample: OdomSample,
    goal_tolerance: float,
    progress_tolerance: float,
) -> bool:
    _, _, progress, _, _, _ = nearest_path_error(sample.x, sample.y, ref)
    goal_x, goal_y = ref.points[-1]
    distance_to_goal = math.hypot(sample.x - goal_x, sample.y - goal_y)
    return (
        progress >= ref.cumulative_s[-1] - progress_tolerance
        and distance_to_goal <= goal_tolerance
    )


def run_process(command: str, log_file: Path, cwd: Path) -> subprocess.Popen:
    log_file.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_file, "w", encoding="utf-8")
    return subprocess.Popen(
        ["bash", "-lc", command],
        cwd=str(cwd),
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def stop_processes(processes: Sequence[subprocess.Popen]) -> None:
    for proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    time.sleep(3.0)
    for proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    time.sleep(1.0)
    for proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def cleanup_stale_processes() -> None:
    patterns = [
        "ros2 launch diff_drive_robot robot.launch.py",
        "gzserver .*agribot_simulation/world",
        "gzclient",
        "gazebo",
        "spawn_entity.py.*diff_drive_robot",
        "robot_state_publisher.*robot_state_publisher",
        "cornfield_navigation_node",
    ]
    for sig in ("-INT", "-TERM", "-KILL"):
        for pattern in patterns:
            subprocess.run(["pkill", sig, "-f", pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)


def summarize(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {"mean": float("nan"), "rmse": float("nan"), "max": float("nan"), "p95": float("nan")}
    sorted_values = sorted(values)
    p95_idx = min(len(sorted_values) - 1, int(math.ceil(0.95 * len(sorted_values))) - 1)
    return {
        "mean": sum(values) / len(values),
        "rmse": math.sqrt(sum(v * v for v in values) / len(values)),
        "max": max(values),
        "p95": sorted_values[p95_idx],
    }


def analyze_samples(ref: ReferencePath, samples: Sequence[OdomSample]) -> Tuple[List[Dict[str, float]], Dict[str, Dict[str, float]]]:
    rows: List[Dict[str, float]] = []
    for sample in samples:
        signed, abs_err, progress, ref_yaw, nearest_x, nearest_y = nearest_path_error(sample.x, sample.y, ref)
        rows.append(
            {
                "time_s": sample.t,
                "x_m": sample.x,
                "y_m": sample.y,
                "yaw_rad": sample.yaw,
                "linear_vel_mps": sample.v,
                "angular_vel_radps": sample.w,
                "signed_cross_track_error_m": signed,
                "abs_cross_track_error_m": abs_err,
                "progress_m": progress,
                "reference_yaw_rad": ref_yaw,
                "heading_error_rad": wrap_angle(sample.yaw - ref_yaw),
                "nearest_ref_x_m": nearest_x,
                "nearest_ref_y_m": nearest_y,
            }
        )

    metrics: Dict[str, Dict[str, float]] = {}
    errors = [row["abs_cross_track_error_m"] for row in rows]
    heading_errors = [abs(row["heading_error_rad"]) for row in rows]
    metrics["overall_cross_track_error_m"] = summarize(errors)
    metrics["overall_heading_error_deg"] = summarize([math.degrees(v) for v in heading_errors])

    for name, (s0, s1) in ref.segment_breaks.items():
        seg_errors = [row["abs_cross_track_error_m"] for row in rows if s0 <= row["progress_m"] <= s1]
        seg_heading = [abs(row["heading_error_rad"]) for row in rows if s0 <= row["progress_m"] <= s1]
        metrics[f"{name}_cross_track_error_m"] = summarize(seg_errors)
        metrics[f"{name}_heading_error_deg"] = summarize([math.degrees(v) for v in seg_heading])

    if rows:
        metrics["run"] = {
            "samples": float(len(rows)),
            "duration_s": rows[-1]["time_s"] - rows[0]["time_s"],
            "final_progress_m": rows[-1]["progress_m"],
            "reference_length_m": ref.cumulative_s[-1],
            "completion_pct": 100.0 * rows[-1]["progress_m"] / max(ref.cumulative_s[-1], 1e-6),
            "final_x_m": rows[-1]["x_m"],
            "final_y_m": rows[-1]["y_m"],
            "mean_linear_speed_mps": sum(abs(s.v) for s in samples) / len(samples),
            "mean_abs_angular_speed_radps": sum(abs(s.w) for s in samples) / len(samples),
        }
    return rows, metrics


def write_outputs(
    output_dir: Path,
    ref: ReferencePath,
    rows: Sequence[Dict[str, float]],
    metrics: Dict[str, Dict[str, float]],
    args: argparse.Namespace,
    stop_reason: str,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = output_dir / "tracking_samples.csv"
    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    metrics_path = output_dir / "tracking_metrics.csv"
    with open(metrics_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "mean", "rmse", "max", "p95", "samples_or_value"])
        for key, value in metrics.items():
            writer.writerow(
                [
                    key,
                    value.get("mean", ""),
                    value.get("rmse", ""),
                    value.get("max", ""),
                    value.get("p95", ""),
                    value.get("samples", value.get("duration_s", value.get("completion_pct", ""))),
                ]
            )

    plot_trajectory(output_dir / "tracking_trajectory.png", ref, rows)
    plot_error(output_dir / "cross_track_error.png", rows)
    plot_heading_error(output_dir / "heading_error.png", rows)

    md_path = output_dir / "tracking_accuracy_report.md"
    run = metrics.get("run", {})
    overall = metrics.get("overall_cross_track_error_m", {})
    heading = metrics.get("overall_heading_error_deg", {})
    md_path.write_text(
        "\n".join(
            [
                "# PID Path Tracking Accuracy Test",
                "",
                "## Test Setup",
                "",
                f"- World: `{args.world}`",
                f"- Duration: `{args.duration}` s",
                f"- Stop reason: `{stop_reason}`",
                f"- Reference path length: `{ref.cumulative_s[-1]:.3f}` m",
                "- Reference path: straight segment + circular arc + straight segment.",
                "- Published topic: `/corn_row_center_line`",
                "- Recorded topic: `/odom`",
                "",
                "## Overall Metrics",
                "",
                "| Metric | Value |",
                "|---|---:|",
                f"| Samples | {run.get('samples', float('nan')):.0f} |",
                f"| Duration | {run.get('duration_s', float('nan')):.3f} s |",
                f"| Final progress | {run.get('final_progress_m', float('nan')):.3f} m |",
                f"| Completion | {run.get('completion_pct', float('nan')):.1f}% |",
                f"| Cross-track MAE | {overall.get('mean', float('nan')):.4f} m |",
                f"| Cross-track RMSE | {overall.get('rmse', float('nan')):.4f} m |",
                f"| Cross-track max | {overall.get('max', float('nan')):.4f} m |",
                f"| Cross-track P95 | {overall.get('p95', float('nan')):.4f} m |",
                f"| Heading error MAE | {heading.get('mean', float('nan')):.3f} deg |",
                f"| Heading error RMSE | {heading.get('rmse', float('nan')):.3f} deg |",
                "",
                "## Segment Metrics",
                "",
                "| Segment | CTE mean/m | CTE RMSE/m | CTE max/m | Heading mean/deg |",
                "|---|---:|---:|---:|---:|",
                *[
                    (
                        f"| {name} | "
                        f"{metrics.get(f'{name}_cross_track_error_m', {}).get('mean', float('nan')):.4f} | "
                        f"{metrics.get(f'{name}_cross_track_error_m', {}).get('rmse', float('nan')):.4f} | "
                        f"{metrics.get(f'{name}_cross_track_error_m', {}).get('max', float('nan')):.4f} | "
                        f"{metrics.get(f'{name}_heading_error_deg', {}).get('mean', float('nan')):.3f} |"
                    )
                    for name in ("straight_1", "arc", "straight_2")
                ],
                "",
                "## Figures",
                "",
                "![Trajectory](tracking_trajectory.png)",
                "",
                "![Cross-track error](cross_track_error.png)",
                "",
                "![Heading error](heading_error.png)",
                "",
                "## Files",
                "",
                "- `tracking_samples.csv`: odom samples and nearest-reference errors.",
                "- `tracking_metrics.csv`: summary metrics.",
                "- `tracking_trajectory.png`: odom trajectory against reference path.",
                "- `cross_track_error.png`: cross-track error over time.",
                "- `heading_error.png`: heading error over time.",
                "",
            ]
        ),
        encoding="utf-8",
    )


def plot_trajectory(path: Path, ref: ReferencePath, rows: Sequence[Dict[str, float]]) -> None:
    fig, ax = plt.subplots(figsize=(8, 6))
    ref_x = [p[0] for p in ref.points]
    ref_y = [p[1] for p in ref.points]
    ax.plot(ref_x, ref_y, "k--", linewidth=2.0, label="Reference path")
    if rows:
        ax.plot([r["x_m"] for r in rows], [r["y_m"] for r in rows], "b-", linewidth=1.5, label="Robot odom")
        ax.scatter([rows[0]["x_m"]], [rows[0]["y_m"]], color="green", marker="o", label="Start")
        ax.scatter([rows[-1]["x_m"]], [rows[-1]["y_m"]], color="red", marker="x", label="End")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("PID tracking trajectory")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_error(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot([r["time_s"] for r in rows], [r["signed_cross_track_error_m"] for r in rows], linewidth=1.2)
    ax.axhline(0.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("signed cross-track error (m)")
    ax.set_title("Cross-track error over time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_heading_error(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot([r["time_s"] for r in rows], [math.degrees(r["heading_error_rad"]) for r in rows], linewidth=1.2)
    ax.axhline(0.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("heading error (deg)")
    ax.set_title("Heading error over time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=220)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PID path tracking accuracy test")
    parser.add_argument("--workspace", default="/home/xkai/agribot/agribot_ws")
    parser.add_argument("--world", default="empty.world")
    parser.add_argument("--output-root", default=str(Path.home() / "agribot_test_logs" / "pid_tracking_accuracy"))
    parser.add_argument("--trial-name", default=time.strftime("trial_%Y%m%d_%H%M%S"))
    parser.add_argument("--duration", type=float, default=55.0)
    parser.add_argument("--stop-when-complete", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--goal-tolerance", type=float, default=0.25)
    parser.add_argument("--progress-tolerance", type=float, default=0.20)
    parser.add_argument("--gazebo-wait", type=float, default=12.0)
    parser.add_argument("--nav-wait", type=float, default=3.0)
    parser.add_argument("--path-rate", type=float, default=10.0)
    parser.add_argument("--gui", default="true", choices=["true", "false"])
    parser.add_argument("--max-linear-speed", type=float, default=0.35)
    parser.add_argument("--target-distance", type=float, default=0.45)
    parser.add_argument("--use-quality-aware-control", default="false", choices=["true", "false"])
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace = Path(args.workspace)
    output_dir = Path(args.output_root) / args.trial_name
    output_dir.mkdir(parents=True, exist_ok=True)

    cleanup_stale_processes()
    processes: List[subprocess.Popen] = []

    setup = f"source {workspace / 'install' / 'setup.bash'}"
    gazebo_cmd = (
        f"{setup} && ros2 launch diff_drive_robot robot.launch.py "
        f"world:={args.world} gui:={args.gui}"
    )
    nav_cmd = (
        f"{setup} && ros2 run centerline_extraction cornfield_navigation_node --ros-args "
        f"-p use_sim_time:=true "
        f"-p use_quality_aware_control:={args.use_quality_aware_control} "
        f"-p max_linear_speed:={args.max_linear_speed} "
        f"-p target_distance:={args.target_distance}"
    )

    try:
        print(f"[INFO] Output: {output_dir}")
        print("[INFO] Starting Gazebo...")
        processes.append(run_process(gazebo_cmd, output_dir / "gazebo.log", workspace))
        time.sleep(args.gazebo_wait)

        print("[INFO] Starting PID navigation node...")
        processes.append(run_process(nav_cmd, output_dir / "navigation_pid.log", workspace))
        time.sleep(args.nav_wait)

        ref = build_reference_path()
        rclpy.init()
        node = PathPublisherAndRecorder(ref, args.path_rate)
        print(f"[INFO] Publishing reference path and recording odom for up to {args.duration:.1f}s...")
        print(
            "[INFO] Stop condition: "
            f"stop_when_complete={args.stop_when_complete}, "
            f"goal_tolerance={args.goal_tolerance:.2f}m, "
            f"progress_tolerance={args.progress_tolerance:.2f}m"
        )
        start = time.monotonic()
        stop_reason = "timeout"
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.stop_when_complete and node.samples:
                if reached_reference_goal(
                    ref,
                    node.samples[-1],
                    args.goal_tolerance,
                    args.progress_tolerance,
                ):
                    stop_reason = "reached_reference_goal"
                    node.publish_stop()
                    break
        samples = list(node.samples)
        for _ in range(5):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()

        rows, metrics = analyze_samples(ref, samples)
        write_outputs(output_dir, ref, rows, metrics, args, stop_reason)

        print("[INFO] Test finished.")
        print(f"[INFO] Stop reason: {stop_reason}")
        print(f"[INFO] Report: {output_dir / 'tracking_accuracy_report.md'}")
        print(f"[INFO] Cross-track RMSE: {metrics['overall_cross_track_error_m']['rmse']:.4f} m")
        print(f"[INFO] Cross-track max: {metrics['overall_cross_track_error_m']['max']:.4f} m")
        print(f"[INFO] Completion: {metrics['run']['completion_pct']:.1f}%")
        return 0
    finally:
        stop_processes(processes)
        cleanup_stale_processes()


if __name__ == "__main__":
    sys.exit(main())

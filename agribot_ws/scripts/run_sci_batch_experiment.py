#!/usr/bin/env python3
import argparse
import csv
import math
import os
import time
from dataclasses import dataclass, field
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty


@dataclass
class Sample:
    trial: int
    t_wall: float
    t_sim: float
    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float
    confidence: float
    width: float
    safety_margin: float
    obstacle: bool
    center_points: int


@dataclass
class TrialResult:
    trial: int
    duration_wall: float
    duration_sim: float
    distance: float
    final_x: float
    final_y: float
    mean_abs_y: float
    max_abs_y: float
    mean_confidence: float
    min_confidence: float
    mean_width: float
    min_width: float
    mean_safety_margin: float
    min_safety_margin: float
    centerline_loss_ratio: float
    obstacle_ratio: float
    mean_speed_cmd: float
    end_reason: str


class SciBatchExperiment(Node):
    def __init__(self, output_dir: str, trials: int, max_duration: float, stop_hold: float):
        super().__init__("sci_batch_experiment_logger")
        self.output_dir = output_dir
        self.trials = trials
        self.max_duration = max_duration
        self.stop_hold = stop_hold

        self.odom = None
        self.cmd = Twist()
        self.path = Path()
        self.confidence = 0.0
        self.width = 0.0
        self.safety_margin = 0.0
        self.obstacle = False

        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 20)
        self.create_subscription(Path, "/corn_row_center_line", self._path_cb, 10)
        self.create_subscription(Float32, "/corridor_confidence", self._confidence_cb, 10)
        self.create_subscription(Float32, "/corridor_width", self._width_cb, 10)
        self.create_subscription(Float32, "/corridor_safety_margin", self._safety_cb, 10)
        self.create_subscription(Bool, "/obstacle_detected", self._obstacle_cb, 10)
        self.reset_client = self.create_client(Empty, "/reset_world")

        self.samples: list[Sample] = []
        self.results: list[TrialResult] = []

    def _odom_cb(self, msg):
        self.odom = msg

    def _cmd_cb(self, msg):
        self.cmd = msg

    def _path_cb(self, msg):
        self.path = msg

    def _confidence_cb(self, msg):
        self.confidence = float(msg.data)

    def _width_cb(self, msg):
        self.width = float(msg.data)

    def _safety_cb(self, msg):
        self.safety_margin = float(msg.data)

    def _obstacle_cb(self, msg):
        self.obstacle = bool(msg.data)

    def wait_for_inputs(self, timeout=20.0):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None:
                return True
        return False

    def reset_world(self):
        if not self.reset_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/reset_world service is unavailable")
        future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError("/reset_world call timed out")
        self.path = Path()
        self.confidence = 0.0
        self.width = 0.0
        self.safety_margin = 0.0
        self.obstacle = False
        time.sleep(0.8)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

    def current_pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return p.x, p.y, math.atan2(siny, cosy)

    def current_sim_time(self):
        if self.odom is None:
            return 0.0
        stamp = self.odom.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def run_trial(self, trial: int):
        self.get_logger().info(f"Trial {trial}: reset world")
        self.reset_world()
        if not self.wait_for_inputs():
            raise RuntimeError("No /odom received after reset")

        start_wall = time.monotonic()
        start_sim = self.current_sim_time()
        last_sample_wall = 0.0
        stop_started = None
        moved_enough = False
        no_progress_started = None
        trial_samples: list[Sample] = []

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now_wall = time.monotonic()
            elapsed = now_wall - start_wall
            if elapsed - last_sample_wall >= 0.1 and self.odom is not None:
                last_sample_wall = elapsed
                x, y, yaw = self.current_pose()
                moved_enough = moved_enough or abs(x) > 0.6
                sample = Sample(
                    trial=trial,
                    t_wall=elapsed,
                    t_sim=self.current_sim_time() - start_sim,
                    x=x,
                    y=y,
                    yaw=yaw,
                    linear_x=float(self.cmd.linear.x),
                    angular_z=float(self.cmd.angular.z),
                    confidence=self.confidence,
                    width=self.width,
                    safety_margin=self.safety_margin,
                    obstacle=self.obstacle,
                    center_points=len(self.path.poses),
                )
                self.samples.append(sample)
                trial_samples.append(sample)

                stopped = abs(sample.linear_x) < 1e-4 and abs(sample.angular_z) < 1e-4
                center_lost = sample.center_points == 0 or sample.confidence <= 1e-5
                no_progress = (not moved_enough) and stopped and center_lost
                if no_progress:
                    if no_progress_started is None:
                        no_progress_started = elapsed
                    elif elapsed - no_progress_started >= 12.0:
                        return self.summarize_trial(trial, trial_samples, "no_valid_centerline_at_start")
                else:
                    no_progress_started = None

                if moved_enough and (stopped or center_lost):
                    if stop_started is None:
                        stop_started = elapsed
                    elif elapsed - stop_started >= self.stop_hold:
                        return self.summarize_trial(trial, trial_samples, "centerline_lost_or_stopped")
                else:
                    stop_started = None

            if elapsed >= self.max_duration:
                return self.summarize_trial(trial, trial_samples, "timeout")

    def summarize_trial(self, trial: int, samples: list[Sample], end_reason: str):
        if not samples:
            raise RuntimeError(f"Trial {trial} has no samples")
        xs = np.array([s.x for s in samples], dtype=float)
        ys = np.array([s.y for s in samples], dtype=float)
        conf = np.array([s.confidence for s in samples], dtype=float)
        widths = np.array([s.width for s in samples], dtype=float)
        margins = np.array([s.safety_margin for s in samples], dtype=float)
        speeds = np.array([s.linear_x for s in samples], dtype=float)
        center_points = np.array([s.center_points for s in samples], dtype=float)
        obstacles = np.array([1.0 if s.obstacle else 0.0 for s in samples], dtype=float)

        distance = float(np.sum(np.hypot(np.diff(xs), np.diff(ys)))) if len(xs) > 1 else 0.0
        valid_conf = conf[conf > 0.0]
        valid_widths = widths[widths > 0.0]
        valid_margins = margins[margins != 0.0]
        result = TrialResult(
            trial=trial,
            duration_wall=float(samples[-1].t_wall),
            duration_sim=float(samples[-1].t_sim),
            distance=distance,
            final_x=float(xs[-1]),
            final_y=float(ys[-1]),
            mean_abs_y=float(np.mean(np.abs(ys))),
            max_abs_y=float(np.max(np.abs(ys))),
            mean_confidence=float(np.mean(valid_conf)) if valid_conf.size else 0.0,
            min_confidence=float(np.min(valid_conf)) if valid_conf.size else 0.0,
            mean_width=float(np.mean(valid_widths)) if valid_widths.size else 0.0,
            min_width=float(np.min(valid_widths)) if valid_widths.size else 0.0,
            mean_safety_margin=float(np.mean(valid_margins)) if valid_margins.size else 0.0,
            min_safety_margin=float(np.min(valid_margins)) if valid_margins.size else 0.0,
            centerline_loss_ratio=float(np.mean(center_points <= 0.0)),
            obstacle_ratio=float(np.mean(obstacles)),
            mean_speed_cmd=float(np.mean(np.abs(speeds))),
            end_reason=end_reason,
        )
        self.results.append(result)
        self.get_logger().info(
            f"Trial {trial} done: distance={result.distance:.2f}m, "
            f"final_x={result.final_x:.2f}m, mean_conf={result.mean_confidence:.2f}, "
            f"reason={end_reason}"
        )
        return result

    def write_outputs(self):
        os.makedirs(self.output_dir, exist_ok=True)
        samples_path = os.path.join(self.output_dir, "samples.csv")
        summary_path = os.path.join(self.output_dir, "summary.csv")
        report_path = os.path.join(self.output_dir, "report.md")

        with open(samples_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(Sample.__dataclass_fields__.keys())
            for s in self.samples:
                writer.writerow([getattr(s, k) for k in Sample.__dataclass_fields__.keys()])

        with open(summary_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(TrialResult.__dataclass_fields__.keys())
            for r in self.results:
                writer.writerow([getattr(r, k) for k in TrialResult.__dataclass_fields__.keys()])

        self.plot_trajectory()
        self.plot_metrics()
        self.write_report(report_path, samples_path, summary_path)
        return report_path

    def plot_trajectory(self):
        plt.figure(figsize=(8, 4.5), dpi=160)
        for trial in sorted({s.trial for s in self.samples}):
            xs = [s.x for s in self.samples if s.trial == trial]
            ys = [s.y for s in self.samples if s.trial == trial]
            plt.plot(xs, ys, label=f"trial {trial}")
        plt.xlabel("x / m")
        plt.ylabel("y / m")
        plt.title("Robot trajectory in odom frame")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, "trajectory.png"))
        plt.close()

    def plot_metrics(self):
        fig, axes = plt.subplots(3, 1, figsize=(8, 8), dpi=160, sharex=True)
        for trial in sorted({s.trial for s in self.samples}):
            rows = [s for s in self.samples if s.trial == trial]
            t = [s.t_wall for s in rows]
            axes[0].plot(t, [s.confidence for s in rows], label=f"trial {trial}")
            axes[1].plot(t, [s.width for s in rows], label=f"trial {trial}")
            axes[2].plot(t, [s.linear_x for s in rows], label=f"trial {trial}")
        axes[0].set_ylabel("confidence")
        axes[1].set_ylabel("width / m")
        axes[2].set_ylabel("cmd vx / m/s")
        axes[2].set_xlabel("wall time / s")
        for ax in axes:
            ax.grid(True, alpha=0.3)
        axes[0].legend(ncol=2, fontsize=8)
        plt.tight_layout()
        plt.savefig(os.path.join(self.output_dir, "metrics_timeseries.png"))
        plt.close()

    def write_report(self, report_path, samples_path, summary_path):
        fields = TrialResult.__dataclass_fields__.keys()
        numeric = [
            "duration_wall", "distance", "final_x", "mean_abs_y", "max_abs_y",
            "mean_confidence", "min_confidence", "mean_width", "min_width",
            "mean_safety_margin", "centerline_loss_ratio", "obstacle_ratio",
            "mean_speed_cmd",
        ]
        aggregate = {}
        for key in numeric:
            vals = np.array([getattr(r, key) for r in self.results], dtype=float)
            aggregate[key] = (float(np.mean(vals)), float(np.std(vals))) if vals.size else (0.0, 0.0)

        with open(report_path, "w") as f:
            f.write("# SCI Batch Experiment Report\n\n")
            f.write(f"- Generated: {datetime.now().isoformat(timespec='seconds')}\n")
            f.write("- Launch: `ros2 launch diff_drive_robot robot.launch.py`\n")
            f.write("- Perception: `corn_row_detector_projection`\n")
            f.write("- Control: `cornfield_navigation_node` / PID controller\n")
            f.write("- World reset service: `/reset_world`\n\n")
            f.write("## Result Summary\n\n")
            f.write("| Metric | Mean | Std |\n")
            f.write("|---|---:|---:|\n")
            for key, (mean, std) in aggregate.items():
                f.write(f"| {key} | {mean:.4f} | {std:.4f} |\n")
            f.write("\n## Trial Table\n\n")
            f.write("| " + " | ".join(fields) + " |\n")
            f.write("|" + "|".join(["---"] * len(fields)) + "|\n")
            for r in self.results:
                values = []
                for key in fields:
                    value = getattr(r, key)
                    values.append(f"{value:.4f}" if isinstance(value, float) else str(value))
                f.write("| " + " | ".join(values) + " |\n")
            f.write("\n## Outputs\n\n")
            f.write(f"- Samples CSV: `{os.path.basename(samples_path)}`\n")
            f.write(f"- Summary CSV: `{os.path.basename(summary_path)}`\n")
            f.write("- Trajectory plot: `trajectory.png`\n")
            f.write("- Metric time series: `metrics_timeseries.png`\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--max-duration", type=float, default=90.0)
    parser.add_argument("--stop-hold", type=float, default=2.0)
    args = parser.parse_args()

    output_dir = args.output_dir
    if output_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = os.path.join("experiments", f"sci_batch_{stamp}")

    rclpy.init()
    node = SciBatchExperiment(output_dir, args.trials, args.max_duration, args.stop_hold)
    try:
        if not node.wait_for_inputs(timeout=30.0):
            raise RuntimeError("No /odom received. Start Gazebo before running the batch.")
        for trial in range(1, args.trials + 1):
            node.run_trial(trial)
        report = node.write_outputs()
        node.get_logger().info(f"Report written: {report}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

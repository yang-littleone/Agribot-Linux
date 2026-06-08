#!/usr/bin/env python3
import argparse
import csv
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from std_msgs.msg import Float32


ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Profile:
    name: str
    params: dict[str, object]


PROFILES = [
    Profile("default", {}),
    Profile(
        "sparse_two_row",
        {
            "min_cluster_size": 2,
            "min_section_points": 1,
            "min_innermost_seeds": 2,
            "desired_row_separation": 1.0,
            "min_row_separation": 0.2,
            "max_row_separation": 2.0,
            "use_simple_inner_row_mode": False,
        },
    ),
    Profile(
        "wide_two_row",
        {
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
    ),
    Profile(
        "known_good_relaxed",
        {
            "min_cluster_size": 5,
            "min_section_points": 2,
            "desired_row_separation": 0.8,
            "min_row_separation": 0.25,
            "max_row_separation": 1.5,
        },
    ),
]


class ViabilityProbe(Node):
    def __init__(self):
        super().__init__("world_viability_probe")
        self.path_points = 0
        self.confidence = 0.0
        self.width = 0.0
        self.path_seen = False
        self.conf_seen = False
        self.width_seen = False
        self.create_subscription(RosPath, "/corn_row_center_line", self._path_cb, 10)
        self.create_subscription(Float32, "/corridor_confidence", self._conf_cb, 10)
        self.create_subscription(Float32, "/corridor_width", self._width_cb, 10)

    def _path_cb(self, msg):
        self.path_seen = True
        self.path_points = len(msg.poses)

    def _conf_cb(self, msg):
        self.conf_seen = True
        self.confidence = float(msg.data)

    def _width_cb(self, msg):
        self.width_seen = True
        self.width = float(msg.data)


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
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
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


def probe_once(world: str, profile: Profile, output_root: Path, duration: float) -> dict[str, object]:
    run_name = f"{world}_{profile.name}"
    log_dir = output_root / run_name / "logs"
    procs: list[subprocess.Popen] = []
    result = {
        "world": world,
        "profile": profile.name,
        "runnable": "false",
        "path_seen": "false",
        "path_points": 0,
        "confidence": 0.0,
        "width": 0.0,
        "reason": "",
        "params": " ".join(f"{k}:={v}" for k, v in profile.params.items()),
    }
    try:
        procs.append(start_process(
            f"ros2 launch diff_drive_robot robot.launch.py world:={world} gui:=true",
            log_dir / "gazebo.log",
        ))
        if not wait_for_topic("/odom", timeout=60.0):
            result["reason"] = "no_odom"
            return result
        if not wait_for_topic("/mid360_PointCloud2", timeout=60.0):
            result["reason"] = "no_pointcloud_topic"
            return result

        procs.append(start_process(
            f"ros2 run centerline_extraction corn_row_detector_projection {params_to_args(profile.params)}",
            log_dir / "perception.log",
        ))
        if not wait_for_topic("/corn_row_center_line", timeout=30.0):
            result["reason"] = "no_centerline_topic"
            return result

        rclpy.init(args=None)
        node = ViabilityProbe()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.path_points > 0 and node.confidence > 0.0:
                break
        result.update({
            "runnable": "true" if node.path_points > 0 and node.confidence > 0.0 else "false",
            "path_seen": "true" if node.path_seen else "false",
            "path_points": node.path_points,
            "confidence": node.confidence,
            "width": node.width,
            "reason": "valid_centerline" if node.path_points > 0 and node.confidence > 0.0 else "empty_centerline",
        })
        node.destroy_node()
        rclpy.shutdown()
        return result
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        for proc in reversed(procs):
            stop_process(proc)
        time.sleep(2.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--worlds", nargs="+", default=[
        "towrow.world",
        "corn_leaf_world.world",
        "twoworld.world",
        "cornlinens.world",
        "cornlinens_angular2.world",
    ])
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--output-root", default=None)
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = Path(args.output_root or ROOT / "experiments" / f"world_viability_{stamp}")
    output_root.mkdir(parents=True, exist_ok=True)
    csv_path = output_root / "viability.csv"

    rows = []
    for world in args.worlds:
        for profile in PROFILES:
            print(f"[INFO] Probing {world} / {profile.name}", flush=True)
            row = probe_once(world, profile, output_root, args.duration)
            rows.append(row)
            print(
                f"[INFO] {world} / {profile.name}: {row['reason']}, "
                f"points={row['path_points']}, confidence={float(row['confidence']):.3f}, width={float(row['width']):.3f}",
                flush=True,
            )

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"[INFO] Viability CSV written: {csv_path}", flush=True)


if __name__ == "__main__":
    main()

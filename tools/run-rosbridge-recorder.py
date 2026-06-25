#!/usr/bin/env python3
"""Run rosbridge and record MapEverything topics into chunked ROS 2 bags.

Run this from a sourced ROS 2 workspace that includes `reconstructor_msgs`, or
pass `--setup ~/mapeverything_ws/install/setup.bash`.
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence


DEFAULT_TOPICS = [
    "/mapping/pose",
    "/mapping/camera/image/compressed",
    "/mapping/camera/camera_info",
    "/mapping/pointcloud",
    "/mapping/gps/fix",
    "/mapping/gps/metadata",
    "/mapping/satellite/image/compressed",
    "/mapping/satellite/tile_info",
    "/mapping/dem/tile",
]

OPTIONAL_TOPICS = [
    "/tf",
    "/mapping/odom",
    "/mapping/imu",
    "/mapping/map",
    "/mapping/mesh_snapshot",
    "/mapping/radio",
    "/mapping/indoor_localization",
    "/mapping/session",
    "/mapping/status",
]


class RecorderError(RuntimeError):
    """Raised when the recorder cannot be launched safely."""


class ManagedProcess:
    def __init__(self, name: str, command: list[str], setup: Path | None = None):
        self.name = name
        self.command = command
        self.setup = setup
        self.process: subprocess.Popen[bytes] | None = None

    def display_command(self) -> str:
        if self.setup:
            return f"source {shlex.quote(str(self.setup))} && {shlex.join(self.command)}"
        return shlex.join(self.command)

    def start(self) -> None:
        command = command_with_setup(self.command, self.setup)
        print(f"[mapeverything] starting {self.name}: {self.display_command()}")
        self.process = subprocess.Popen(command, start_new_session=True)

    def poll(self) -> int | None:
        if not self.process:
            return None
        return self.process.poll()

    def terminate(self, timeout: float) -> None:
        if not self.process or self.process.poll() is not None:
            return

        pid = self.process.pid
        print(f"[mapeverything] stopping {self.name}")
        send_signal(pid, signal.SIGINT)
        try:
            self.process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass

        send_signal(pid, signal.SIGTERM)
        try:
            self.process.wait(timeout=max(1.0, timeout / 2))
            return
        except subprocess.TimeoutExpired:
            pass

        send_signal(pid, signal.SIGKILL)
        self.process.wait(timeout=2)


def send_signal(pid: int, sig: signal.Signals) -> None:
    try:
        os.killpg(os.getpgid(pid), sig)
    except ProcessLookupError:
        return


def command_with_setup(command: Sequence[str], setup: Path | None) -> list[str]:
    if setup is None:
        return list(command)

    shell_command = f"set -e; source {shlex.quote(str(setup))}; exec {shlex.join(command)}"
    return ["bash", "-lc", shell_command]


def default_output_path() -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("bags") / f"mapeverything_{timestamp}"


def read_topics_file(path: Path) -> list[str]:
    topics: list[str] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        topic = line.split("#", 1)[0].strip()
        if not topic:
            continue
        if not topic.startswith("/"):
            raise RecorderError(f"{path}:{line_number} topic must start with '/': {topic}")
        topics.append(topic)
    return topics


def unique_topics(topics: Sequence[str]) -> list[str]:
    seen: set[str] = set()
    unique: list[str] = []
    for topic in topics:
        if topic in seen:
            continue
        seen.add(topic)
        unique.append(topic)
    return unique


def selected_topics(args: argparse.Namespace) -> list[str]:
    topics: list[str] = []
    topics.extend(DEFAULT_TOPICS)
    if args.include_optional:
        topics.extend(OPTIONAL_TOPICS)
    for topics_file in args.topics_file:
        topics.extend(read_topics_file(topics_file))
    topics.extend(args.topic)
    return unique_topics(topics)


def validate_environment(setup: Path | None) -> None:
    if setup is not None and not setup.exists():
        raise RecorderError(f"Setup file not found: {setup}")

    if setup is None and shutil.which("ros2") is None:
        raise RecorderError(
            "Unable to find `ros2` on PATH. Source your ROS 2 workspace first "
            "or pass --setup ~/mapeverything_ws/install/setup.bash."
        )


def rosbridge_command(args: argparse.Namespace) -> list[str]:
    command = [
        "ros2",
        "launch",
        "rosbridge_server",
        "rosbridge_websocket_launch.xml",
        f"address:={args.address}",
        f"port:={args.port}",
    ]
    command.extend(args.rosbridge_arg)
    return command


def bag_record_command(args: argparse.Namespace, topics: Sequence[str]) -> list[str]:
    chunk_bytes = args.chunk_size_mb * 1024 * 1024
    command = [
        "ros2",
        "bag",
        "record",
        "--storage",
        args.storage,
        "--output",
        str(args.output),
        "--max-bag-size",
        str(chunk_bytes),
    ]

    if args.max_bag_duration > 0:
        command.extend(["--max-bag-duration", str(args.max_bag_duration)])
    if args.include_hidden:
        command.append("--include-hidden-topics")

    if args.record_all:
        command.append("--all")
    else:
        command.extend(topics)

    return command


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Start rosbridge_server and ros2 bag record for MapEverything, "
            "rotating rosbag2 SQLite chunks by size."
        )
    )
    parser.add_argument(
        "--setup",
        type=Path,
        help="Optional ROS 2 setup.bash to source for both rosbridge and rosbag2.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output_path(),
        help="rosbag2 output directory. Default: bags/mapeverything_<timestamp>.",
    )
    parser.add_argument(
        "--chunk-size-mb",
        type=int,
        default=512,
        help="Rotate bag chunks after this many MB. Default: 512.",
    )
    parser.add_argument(
        "--max-bag-duration",
        type=int,
        default=0,
        help="Optional chunk duration in seconds. 0 disables duration-based rotation.",
    )
    parser.add_argument(
        "--storage",
        default="sqlite3",
        help="rosbag2 storage plugin. Default: sqlite3.",
    )
    parser.add_argument(
        "--address",
        default="0.0.0.0",
        help="rosbridge bind address. Default: 0.0.0.0.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="rosbridge websocket port. Default: 9090.",
    )
    parser.add_argument(
        "--bridge-startup-delay",
        type=float,
        default=2.0,
        help="Seconds to wait after starting rosbridge before starting rosbag2.",
    )
    parser.add_argument(
        "--include-optional",
        action="store_true",
        help="Also record optional MapEverything topics such as TF, IMU, mesh, radio, session, and status.",
    )
    parser.add_argument(
        "--record-all",
        action="store_true",
        help="Use `ros2 bag record --all` instead of the MapEverything topic list.",
    )
    parser.add_argument(
        "--include-hidden",
        action="store_true",
        help="Pass --include-hidden-topics to ros2 bag record.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Additional topic to record. May be passed multiple times.",
    )
    parser.add_argument(
        "--topics-file",
        action="append",
        type=Path,
        default=[],
        help="File containing additional topics, one per line. # comments are allowed.",
    )
    parser.add_argument(
        "--rosbridge-arg",
        action="append",
        default=[],
        help="Additional rosbridge launch argument, e.g. retry_startup_delay:=5.0.",
    )
    parser.add_argument(
        "--shutdown-timeout",
        type=float,
        default=8.0,
        help="Seconds to let each child process shut down cleanly before escalating.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the rosbridge and rosbag commands without running them.",
    )
    args = parser.parse_args(argv)

    if args.chunk_size_mb <= 0:
        parser.error("--chunk-size-mb must be positive")
    if args.port <= 0 or args.port > 65535:
        parser.error("--port must be between 1 and 65535")
    return args


def run(args: argparse.Namespace) -> int:
    topics = selected_topics(args)
    if not args.record_all and not topics:
        raise RecorderError("No topics selected for recording.")

    bridge = ManagedProcess("rosbridge", rosbridge_command(args), args.setup)
    recorder = ManagedProcess("rosbag2 recorder", bag_record_command(args, topics), args.setup)

    if args.dry_run:
        print(f"rosbridge: {bridge.display_command()}")
        print(f"rosbag2:    {recorder.display_command()}")
        if not args.record_all:
            print("topics:")
            for topic in topics:
                print(f"  {topic}")
        return 0

    validate_environment(args.setup)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        raise RecorderError(f"Output bag directory already exists: {args.output}")

    processes: list[ManagedProcess] = []
    try:
        bridge.start()
        processes.append(bridge)
        time.sleep(args.bridge_startup_delay)
        bridge_code = bridge.poll()
        if bridge_code is not None:
            raise RecorderError(f"rosbridge exited early with code {bridge_code}")

        recorder.start()
        processes.append(recorder)
        print("[mapeverything] recorder is running. Press Ctrl-C to stop and finalize the bag.")

        while True:
            for process in processes:
                code = process.poll()
                if code is not None:
                    print(f"[mapeverything] {process.name} exited with code {code}")
                    return code if code > 0 else 0
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[mapeverything] Ctrl-C received; finalizing recorder.")
        return 0
    finally:
        for process in reversed(processes):
            process.terminate(args.shutdown_timeout)


def main(argv: Sequence[str]) -> int:
    try:
        return run(parse_args(argv))
    except RecorderError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

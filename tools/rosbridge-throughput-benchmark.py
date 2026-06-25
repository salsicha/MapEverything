#!/usr/bin/env python3
"""Benchmark MapEverything rosbridge publish throughput at target field rates.

The script can run in dry-run mode to size payloads locally, or connect to a
rosbridge WebSocket endpoint when the optional `websockets` package is
installed.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import sys
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TopicProfile:
    name: str
    topic: str
    message_type: str
    rate_hz: float
    raw_payload_bytes: int


PROFILES = [
    TopicProfile(
        name="camera",
        topic="/mapping/camera/image/compressed",
        message_type="sensor_msgs/msg/CompressedImage",
        rate_hz=10,
        raw_payload_bytes=92_000,
    ),
    TopicProfile(
        name="point-cloud",
        topic="/mapping/pointcloud",
        message_type="sensor_msgs/msg/PointCloud2",
        rate_hz=5,
        raw_payload_bytes=220_000,
    ),
    TopicProfile(
        name="mesh",
        topic="/mapping/mesh_snapshot",
        message_type="reconstructor_msgs/msg/MeshSnapshot",
        rate_hz=0.5,
        raw_payload_bytes=512_000,
    ),
    TopicProfile(
        name="satellite",
        topic="/mapping/satellite/image/compressed",
        message_type="sensor_msgs/msg/CompressedImage",
        rate_hz=0.2,
        raw_payload_bytes=160_000,
    ),
    TopicProfile(
        name="dem",
        topic="/mapping/dem/tile",
        message_type="reconstructor_msgs/msg/GeoRasterTile",
        rate_hz=0.2,
        raw_payload_bytes=262_144,
    ),
]


def ros_stamp(sequence: int) -> dict[str, int]:
    now = time.time()
    return {
        "sec": int(now),
        "nanosec": int((now % 1) * 1_000_000_000) + sequence,
    }


def encoded_blob(size_bytes: int) -> str:
    return base64.b64encode(bytes(size_bytes)).decode("ascii")


def message_for(profile: TopicProfile, sequence: int) -> dict[str, Any]:
    header = {"stamp": ros_stamp(sequence), "frame_id": "map"}
    blob = encoded_blob(profile.raw_payload_bytes)

    if profile.name in {"camera", "satellite"}:
        return {
            "header": header,
            "format": "jpeg" if profile.name == "satellite" else "jpeg_q0.4",
            "data": blob,
        }

    if profile.name == "point-cloud":
        return {
            "header": header,
            "height": 1,
            "width": max(profile.raw_payload_bytes // 16, 1),
            "fields": [
                {"name": "x", "offset": 0, "datatype": 7, "count": 1},
                {"name": "y", "offset": 4, "datatype": 7, "count": 1},
                {"name": "z", "offset": 8, "datatype": 7, "count": 1},
                {"name": "rgb", "offset": 12, "datatype": 7, "count": 1},
            ],
            "is_bigendian": False,
            "point_step": 16,
            "row_step": profile.raw_payload_bytes,
            "data": blob,
            "is_dense": True,
        }

    if profile.name == "mesh":
        vertex_count = max(profile.raw_payload_bytes // 48, 3)
        return {
            "header": header,
            "snapshot_id": f"benchmark-{sequence}",
            "source": "throughput_benchmark",
            "frame_id": "map",
            "anchor_count": 1,
            "vertices": [{"x": 0.0, "y": 0.0, "z": 0.0}] * vertex_count,
            "triangle_indices": list(range(vertex_count)),
            "original_vertex_count": vertex_count,
            "original_triangle_count": vertex_count // 3,
            "is_truncated": False,
            "published_payload_bytes": profile.raw_payload_bytes,
            "metadata_json": "{}",
        }

    return {
        "header": header,
        "provider": "USGS 3DEP",
        "layer": "3DEPElevation",
        "kind": "dem",
        "crs": "EPSG:3857",
        "zoom": 12,
        "tile_x": 818,
        "tile_y": 1583,
        "bounds": "{}",
        "device_location": "{}",
        "format": "tiff",
        "mime_type": "image/tiff",
        "encoding": "usgs_3dep_float32_tiff",
        "source_url": "https://elevation.nationalmap.gov/",
        "attribution": "USGS 3D Elevation Program (3DEP) through The National Map",
        "license": "USGS public data",
        "source_policy": "{}",
        "is_cached": False,
        "data": blob,
    }


def publish_payload(profile: TopicProfile, sequence: int) -> str:
    payload = {
        "op": "publish",
        "topic": profile.topic,
        "msg": message_for(profile, sequence),
    }
    return json.dumps(payload, separators=(",", ":"))


async def run_profile(websocket: Any, profile: TopicProfile, duration_seconds: float) -> dict[str, Any]:
    start = time.perf_counter()
    next_send = start
    sent_messages = 0
    sent_bytes = 0
    target_interval = 1.0 / profile.rate_hz if profile.rate_hz > 0 else duration_seconds

    if websocket is not None:
        await websocket.send(
            json.dumps(
                {
                    "op": "advertise",
                    "topic": profile.topic,
                    "type": profile.message_type,
                },
                separators=(",", ":"),
            )
        )

    while time.perf_counter() - start < duration_seconds:
        encoded = publish_payload(profile, sent_messages)
        sent_bytes += len(encoded.encode("utf-8"))
        if websocket is not None:
            await websocket.send(encoded)
        sent_messages += 1
        next_send += target_interval
        sleep_until = min(next_send, start + duration_seconds)
        await asyncio.sleep(max(0.0, sleep_until - time.perf_counter()))

    elapsed = time.perf_counter() - start
    return {
        "profile": profile.name,
        "topic": profile.topic,
        "target_rate_hz": profile.rate_hz,
        "messages": sent_messages,
        "elapsed_seconds": round(elapsed, 3),
        "observed_rate_hz": round(sent_messages / elapsed, 3) if elapsed else 0,
        "json_bytes": sent_bytes,
        "json_mbps": round((sent_bytes * 8) / elapsed / 1_000_000, 3) if elapsed else 0,
    }


async def run(args: argparse.Namespace) -> list[dict[str, Any]]:
    selected = [profile for profile in PROFILES if profile.name in args.profiles]
    websocket = None

    if not args.dry_run:
        try:
            import websockets
        except ImportError:
            print(
                "Install the optional dependency with `python3 -m pip install websockets` "
                "or rerun with --dry-run.",
                file=sys.stderr,
            )
            return []
        websocket = await websockets.connect(args.url, max_size=None)

    try:
        results = []
        for profile in selected:
            results.append(await run_profile(websocket, profile, args.duration))
        return results
    finally:
        if websocket is not None:
            await websocket.close()


def print_markdown(results: list[dict[str, Any]]) -> None:
    print("| Profile | Topic | Target Hz | Observed Hz | Messages | JSON MB/s |")
    print("| :--- | :--- | ---: | ---: | ---: | ---: |")
    for result in results:
        print(
            f"| {result['profile']} | `{result['topic']}` | "
            f"{result['target_rate_hz']} | {result['observed_rate_hz']} | "
            f"{result['messages']} | {result['json_mbps']} |"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:9090", help="rosbridge WebSocket URL")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to run each profile")
    parser.add_argument(
        "--profiles",
        nargs="+",
        choices=[profile.name for profile in PROFILES],
        default=[profile.name for profile in PROFILES],
        help="topic profiles to benchmark",
    )
    parser.add_argument("--dry-run", action="store_true", help="size payloads without opening a WebSocket")
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = asyncio.run(run(args))
    if not results:
        return 2
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print_markdown(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

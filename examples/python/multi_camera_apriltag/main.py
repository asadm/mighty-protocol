#!/usr/bin/env python3
"""Two Mighty cameras showing AprilTag relocalization in one 3D space."""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List


THIS_FILE = Path(__file__).resolve()
SDK_PY_DIR = THIS_FILE.parents[3] / "python"
if str(SDK_PY_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_PY_DIR))

from mighty_sdk import MightyClient, MightyWebDevice  # noqa: E402

from bag_playback import run_bag_playback  # noqa: E402
from recording import RecorderController  # noqa: E402
from state import CameraPanelState  # noqa: E402
from stream import run_camera_lifecycle, wire_client_callbacks  # noqa: E402
from viewer import launch_gui  # noqa: E402


MIGHTY_RED = "#ef4444"
MIGHTY_BLUE = "#2563eb"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Multi-camera Mighty AprilTag relocalization demo",
    )
    parser.add_argument("--red-host", default="http://192.168.7.1", help="Red camera Mighty base URL")
    parser.add_argument("--blue-host", default="http://192.168.8.1", help="Blue camera Mighty base URL")
    parser.add_argument("--red-name", default="red", help="Red camera display name")
    parser.add_argument("--blue-name", default="blue", help="Blue camera display name")
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="SDK reconnect delay in seconds")
    parser.add_argument("--fps", type=int, default=20, help="GUI refresh FPS")
    parser.add_argument("--start-vio", action="store_true", help="Send start_vio to both cameras on launch")

    parser.add_argument("--bag", default="", help="Replay a previously recorded multi-camera AprilTag bag")
    parser.add_argument("--bag-rate", type=float, default=1.0, help="Bag replay speed; use 0 for as-fast-as-possible")
    parser.add_argument("--bag-loop", action="store_true", help="Loop bag playback")

    parser.add_argument("--record", action="store_true", help="Start ROS bag recording on launch")
    parser.add_argument("--record-dir", default="~", help="Directory for button-created bags")
    parser.add_argument("--record-out", default="", help="Exact .bag path for next recording")
    parser.add_argument("--record-jpeg-quality", type=int, default=90, help="JPEG quality for recorded preview images")

    parser.add_argument("--red-image-topic", default="/mighty/red/image_raw/compressed")
    parser.add_argument("--blue-image-topic", default="/mighty/blue/image_raw/compressed")
    parser.add_argument("--red-imu-topic", default="/mighty/red/imu")
    parser.add_argument("--blue-imu-topic", default="/mighty/blue/imu")
    parser.add_argument("--red-pose-topic", default="/mighty/red/pose")
    parser.add_argument("--blue-pose-topic", default="/mighty/blue/pose")
    parser.add_argument("--red-tags-topic", default="/mighty/red/apriltags")
    parser.add_argument("--blue-tags-topic", default="/mighty/blue/apriltags")
    parser.add_argument("--red-events-topic", default="/mighty/red/events")
    parser.add_argument("--blue-events-topic", default="/mighty/blue/events")
    return parser.parse_args()


def build_recorder(args: argparse.Namespace) -> RecorderController:
    return RecorderController(
        record_dir=Path(str(args.record_dir)),
        record_out=str(args.record_out),
        red_image_topic=str(args.red_image_topic),
        blue_image_topic=str(args.blue_image_topic),
        red_imu_topic=str(args.red_imu_topic),
        blue_imu_topic=str(args.blue_imu_topic),
        red_pose_topic=str(args.red_pose_topic),
        blue_pose_topic=str(args.blue_pose_topic),
        red_tags_topic=str(args.red_tags_topic),
        blue_tags_topic=str(args.blue_tags_topic),
        red_events_topic=str(args.red_events_topic),
        blue_events_topic=str(args.blue_events_topic),
        jpeg_quality=int(args.record_jpeg_quality),
    )


def command_both(clients: List[MightyClient], method: str) -> Dict[str, Any]:
    if not clients:
        return {"ok": False, "message": "live clients unavailable during bag replay"}
    results = []
    for client in clients:
        fn = getattr(client, method)
        results.append(fn())
    ok = all(bool(result.get("ok")) for result in results)
    messages = [str(result.get("message") or "ok") for result in results]
    return {"ok": ok, "message": " | ".join(messages)}


def main() -> None:
    args = parse_args()
    bag_path = Path(str(args.bag)).expanduser() if str(args.bag or "").strip() else None
    bag_mode = bag_path is not None

    red_state = CameraPanelState(
        name=str(args.red_name),
        base_url=str(args.red_host if not bag_mode else args.red_image_topic),
        color=MIGHTY_RED,
    )
    blue_state = CameraPanelState(
        name=str(args.blue_name),
        base_url=str(args.blue_host if not bag_mode else args.blue_image_topic),
        color=MIGHTY_BLUE,
    )

    recorder = build_recorder(args)
    if bool(args.record) and bag_mode:
        print("--record is ignored during --bag playback")
    elif bool(args.record):
        ok, message = recorder.start()
        if not ok:
            raise RuntimeError(f"Failed to start ROS bag recording: {message}")
        print(f"Recording ROS bag to {message}")

    stop_event = threading.Event()
    clients: List[MightyClient] = []
    workers: List[threading.Thread] = []

    if bag_mode:
        workers.append(
            threading.Thread(
                target=run_bag_playback,
                kwargs={
                    "bag_path": bag_path,
                    "red_state": red_state,
                    "blue_state": blue_state,
                    "stop_event": stop_event,
                    "red_image_topic": str(args.red_image_topic),
                    "blue_image_topic": str(args.blue_image_topic),
                    "red_imu_topic": str(args.red_imu_topic),
                    "blue_imu_topic": str(args.blue_imu_topic),
                    "red_pose_topic": str(args.red_pose_topic),
                    "blue_pose_topic": str(args.blue_pose_topic),
                    "red_tags_topic": str(args.red_tags_topic),
                    "blue_tags_topic": str(args.blue_tags_topic),
                    "red_events_topic": str(args.red_events_topic),
                    "blue_events_topic": str(args.blue_events_topic),
                    "playback_rate": float(args.bag_rate),
                    "loop": bool(args.bag_loop),
                },
                name="MightyMultiAprilTagBagPlayback",
                daemon=True,
            )
        )
    else:
        red_device = MightyWebDevice(base_url=str(args.red_host), connect_timeout_s=3.0, read_timeout_s=1.0)
        blue_device = MightyWebDevice(base_url=str(args.blue_host), connect_timeout_s=3.0, read_timeout_s=1.0)
        red_client = MightyClient(red_device, auto_reconnect=True, reconnect_delay_s=float(args.reconnect_delay))
        blue_client = MightyClient(blue_device, auto_reconnect=True, reconnect_delay_s=float(args.reconnect_delay))
        clients = [red_client, blue_client]

        wire_client_callbacks(red_client, red_state, role="red", recorder=recorder)
        wire_client_callbacks(blue_client, blue_state, role="blue", recorder=recorder)

        workers.extend(
            [
                threading.Thread(
                    target=run_camera_lifecycle,
                    args=(red_state, red_client, red_device, stop_event, bool(args.start_vio)),
                    name="MightyRedLifecycle",
                    daemon=True,
                ),
                threading.Thread(
                    target=run_camera_lifecycle,
                    args=(blue_state, blue_client, blue_device, stop_event, bool(args.start_vio)),
                    name="MightyBlueLifecycle",
                    daemon=True,
                ),
            ]
        )

    for worker in workers:
        worker.start()

    def start_vio() -> Dict[str, Any]:
        red_state.clear_trajectory()
        blue_state.clear_trajectory()
        return command_both(clients, "start_vio")

    def stop_vio() -> Dict[str, Any]:
        return command_both(clients, "stop_vio")

    def toggle_recording() -> Any:
        if bag_mode:
            return False, "recording disabled during bag replay"
        if recorder.active():
            return recorder.stop()
        return recorder.start()

    try:
        launch_gui(
            red_state,
            blue_state,
            recorder=recorder,
            fps=int(args.fps),
            on_start_vio=start_vio,
            on_stop_vio=stop_vio,
            on_toggle_recording=toggle_recording,
        )
    finally:
        stop_event.set()
        for client in clients:
            try:
                client.disconnect()
            except Exception:
                pass
        for worker in workers:
            worker.join(timeout=3.0)
        was_recording = recorder.active()
        stats = recorder.snapshot()
        recorder.close()
        if was_recording:
            print(f"Closed ROS bag {stats.get('path', '')}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Two-camera Mighty preview, synced recording, and optional stereo depth.

This file is intentionally kept as the docs-friendly entry point. Supporting
math, drawing, stream callbacks, timestamp utilities, and rosbag writing live in
neighboring modules so the app flow is easy to read.
"""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path


THIS_FILE = Path(__file__).resolve()
SDK_PY_DIR = THIS_FILE.parents[3] / "python"
if str(SDK_PY_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_PY_DIR))

from mighty_sdk import MightyClient, MightyWebDevice  # noqa: E402

from depth import StereoDepthProcessor  # noqa: E402
from recording import RecorderController  # noqa: E402
from state import CameraState, ImagePairRefiner  # noqa: E402
from stream import run_camera_lifecycle, wire_client_callbacks  # noqa: E402
from viewer import launch_opencv_gui  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Synced stereo Mighty camera preview")

    # Camera endpoints. The Mighty SDK accepts a base URL, so two USB-networked
    # cameras can be opened by pointing each client at its own subnet.
    parser.add_argument("--left-host", default="http://192.168.7.1", help="Left camera base URL")
    parser.add_argument("--right-host", default="http://192.168.8.1", help="Right camera base URL")
    parser.add_argument("--left-name", default="left", help="Left camera display name")
    parser.add_argument("--right-name", default="right", help="Right camera display name")

    # Timestamp alignment. Clock sync gives both devices a shared host-time base;
    # image refinement then removes residual offset using the shared shutter pin.
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="SDK reconnect delay in seconds")
    parser.add_argument("--clock-samples", type=int, default=8, help="Clock command samples per sync pass")
    parser.add_argument("--clock-sample-delay", type=float, default=0.04, help="Delay between clock samples")
    parser.add_argument("--resync-interval", type=float, default=30.0, help="Periodic clock resync interval; 0 disables")
    parser.add_argument("--no-image-refine", action="store_true", help="Disable shared-shutter frame-pair correction")
    parser.add_argument("--image-refine-min-pairs", type=int, default=6, help="Pairs needed before applying correction")
    parser.add_argument("--pair-arrival-window-ms", type=float, default=50.0, help="Max host arrival gap for image pairing")

    # Viewer controls. The raw preview always renders timestamp-matched stereo
    # pairs; depth is optional because SGM/SGBM is CPU-heavy.
    parser.add_argument("--imu-window", type=float, default=10.0, help="IMU plot history in seconds")
    parser.add_argument("--fps", type=int, default=20, help="GUI refresh FPS")
    parser.add_argument("--preview-width", type=int, default=640, help="Rendered width per camera")
    parser.add_argument("--preview-height", type=int, default=360, help="Rendered height per camera")
    parser.add_argument("--render-pair-max-delta-ms", type=float, default=8.0, help="Max synced timestamp gap for displayed image pairs")
    parser.add_argument("--depth", default="", metavar="CALIB_YAML", help="Enable stereo depth view using a Kalibr camchain YAML")
    parser.add_argument("--depth-scale", type=float, default=0.5, help="Depth compute scale relative to camera frame size")
    parser.add_argument("--depth-min-m", type=float, default=0.2, help="Nearest metric depth mapped to the hottest color")
    parser.add_argument("--depth-max-m", type=float, default=5.0, help="Farthest metric depth mapped to the coolest color")

    # ROS bag recording. The button in the OpenCV window starts/stops recording;
    # --record just starts immediately for scripted runs.
    parser.add_argument("--record", action="store_true", help="Start recording immediately; otherwise use the Start Rec button")
    parser.add_argument("--record-dir", default="~", help="Directory for button-created bags; defaults to home")
    parser.add_argument("--record-out", default="", help="Exact .bag path for the next recording; defaults to --record-dir/synced_stereo_<timestamp>.bag")
    parser.add_argument("--record-pair-max-delta-ms", type=float, default=8.0, help="Max synced timestamp gap for recorded image pairs")
    parser.add_argument("--record-jpeg-quality", type=int, default=90, help="JPEG quality for compressed image topics")
    parser.add_argument("--record-before-refine", action="store_true", help="Record image pairs before frame-pair correction has converged")
    parser.add_argument("--left-image-topic", default="/cam0/image_raw/compressed", help="ROS topic for left compressed images")
    parser.add_argument("--right-image-topic", default="/cam1/image_raw/compressed", help="ROS topic for right compressed images")
    parser.add_argument("--left-imu-topic", default="/imu0", help="ROS topic for left IMU")
    parser.add_argument("--right-imu-topic", default="/imu1", help="ROS topic for right IMU")

    parser.add_argument("--start-vio", action="store_true", help="Send start_vio to both cameras on launch")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Shared state owns the latest images, IMU samples, and timestamp offsets.
    left_state = CameraState(name=str(args.left_name), base_url=str(args.left_host))
    right_state = CameraState(name=str(args.right_name), base_url=str(args.right_host))
    refiner = ImagePairRefiner(
        left_state,
        right_state,
        enabled=not bool(args.no_image_refine),
        min_pairs=int(args.image_refine_min_pairs),
        arrival_window_ms=float(args.pair_arrival_window_ms),
    )

    # Optional depth uses the same synced image pair shown in the raw preview.
    depth_processor = None
    if str(args.depth or "").strip():
        depth_processor = StereoDepthProcessor(
            Path(str(args.depth)),
            depth_scale=float(args.depth_scale),
            min_depth_m=float(args.depth_min_m),
            max_depth_m=float(args.depth_max_m),
        )
        print(f"Depth enabled: {depth_processor.summary()}")

    recorder = RecorderController(
        record_dir=Path(str(args.record_dir)),
        record_out=str(args.record_out),
        left_image_topic=str(args.left_image_topic),
        right_image_topic=str(args.right_image_topic),
        left_imu_topic=str(args.left_imu_topic),
        right_imu_topic=str(args.right_imu_topic),
        jpeg_quality=int(args.record_jpeg_quality),
        pair_max_delta_ms=float(args.record_pair_max_delta_ms),
        require_refined_pairs=not bool(args.record_before_refine),
    )
    if bool(args.record):
        ok, message = recorder.start()
        if not ok:
            raise RuntimeError(f"Failed to start ROS bag recording: {message}")
        print(f"Recording ROS bag to {message}")

    # Each camera gets its own SDK client and lifecycle thread. The lifecycle
    # thread performs clock sync, connects the streaming client, and resyncs.
    left_device = MightyWebDevice(base_url=str(args.left_host), connect_timeout_s=3.0, read_timeout_s=1.0)
    right_device = MightyWebDevice(base_url=str(args.right_host), connect_timeout_s=3.0, read_timeout_s=1.0)
    left_client = MightyClient(left_device, auto_reconnect=True, reconnect_delay_s=float(args.reconnect_delay))
    right_client = MightyClient(right_device, auto_reconnect=True, reconnect_delay_s=float(args.reconnect_delay))

    wire_client_callbacks(
        left_client,
        left_state,
        refiner,
        role="left",
        recorder=recorder,
        left_state=left_state,
        right_state=right_state,
    )
    wire_client_callbacks(
        right_client,
        right_state,
        refiner,
        role="right",
        recorder=recorder,
        left_state=left_state,
        right_state=right_state,
    )

    stop_event = threading.Event()
    workers = [
        threading.Thread(
            target=run_camera_lifecycle,
            args=(
                left_state,
                left_client,
                left_device,
                stop_event,
                int(args.clock_samples),
                float(args.clock_sample_delay),
                float(args.resync_interval),
                bool(args.start_vio),
            ),
            name="MightyLeftLifecycle",
            daemon=True,
        ),
        threading.Thread(
            target=run_camera_lifecycle,
            args=(
                right_state,
                right_client,
                right_device,
                stop_event,
                int(args.clock_samples),
                float(args.clock_sample_delay),
                float(args.resync_interval),
                bool(args.start_vio),
            ),
            name="MightyRightLifecycle",
            daemon=True,
        ),
    ]
    for worker in workers:
        worker.start()

    try:
        # The OpenCV viewer owns the UI loop, render-pair selection, record
        # button, and optional depth panes.
        launch_opencv_gui(
            left_state,
            right_state,
            refiner,
            recorder,
            depth_processor=depth_processor,
            imu_window_s=float(args.imu_window),
            fps=int(args.fps),
            preview_width=int(args.preview_width),
            preview_height=int(args.preview_height),
            max_render_pair_delta_ms=float(args.render_pair_max_delta_ms),
        )
    finally:
        # Always close SDK streams and the bag writer, even if the window is
        # closed or the user hits Ctrl-C.
        stop_event.set()
        for client in (left_client, right_client):
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

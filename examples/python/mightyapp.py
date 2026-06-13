#!/usr/bin/env python3
"""
Mighty SDK example app:
- Uses MightyWebDevice host fallback (default constructor)
- Connects with MightyClient(auto_reconnect=True)
- Streams image / pose / IMU / status
- Renders dashboard UI (via uihelpers)
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Dict
import sys


# Make local SDK importable when running from source tree.
THIS_FILE = Path(__file__).resolve()
SDK_PY_DIR = THIS_FILE.parents[2] / "python"
if str(SDK_PY_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_PY_DIR))

from mighty_sdk import MightyClient, MightyWebDevice  # noqa: E402

from uihelpers import (  # noqa: E402
    DashboardState,
    decode_raw_to_rgb,
    launch_gui,
)

R_VIZ_FROM_ODOM = (
    (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
)
Q_VIZ_FROM_ODOM = (0.5, -0.5, 0.5, 0.5)  # xyzw from R_VIZ_FROM_ODOM


def _map_position_odom_to_viz(position):
    if position is None or len(position) < 3:
        return position
    x = float(position[0])
    y = float(position[1])
    z = float(position[2])
    return (
        R_VIZ_FROM_ODOM[0][0] * x + R_VIZ_FROM_ODOM[0][1] * y + R_VIZ_FROM_ODOM[0][2] * z,
        R_VIZ_FROM_ODOM[1][0] * x + R_VIZ_FROM_ODOM[1][1] * y + R_VIZ_FROM_ODOM[1][2] * z,
        R_VIZ_FROM_ODOM[2][0] * x + R_VIZ_FROM_ODOM[2][1] * y + R_VIZ_FROM_ODOM[2][2] * z,
    )


def _quat_multiply_xyzw(a, b):
    ax, ay, az, aw = [float(v) for v in a]
    bx, by, bz, bw = [float(v) for v in b]
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _map_quat_odom_to_viz(quat):
    if quat is None or len(quat) != 4:
        return quat
    q = _quat_multiply_xyzw(Q_VIZ_FROM_ODOM, quat)
    n = math.sqrt(sum(float(v) * float(v) for v in q))
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(v) / n for v in q)


def wire_client_callbacks(client: MightyClient, state: DashboardState) -> None:
    def on_image(img: Dict[str, object]) -> None:
        kind = img.get("kind")
        if kind == "raw":
            rgb = decode_raw_to_rgb(img)
            if rgb is None:
                return
            channel = str(img.get("channel_alias") or img.get("channel") or "cam0")
            ts = int(img.get("timestamp_ns") or 0)
            state.update_image(rgb, channel, ts)
            return

        if kind == "stereo_raw":
            left = img.get("left") or {}
            right = img.get("right") or {}
            frame = left
            if (left.get("channel_alias") or left.get("channel")) not in ("cam0", "preview", "left"):
                frame = right
            rgb = decode_raw_to_rgb(frame)
            if rgb is None:
                return
            channel = str(frame.get("channel_alias") or frame.get("channel") or "cam0")
            ts = int(frame.get("timestamp_ns") or 0)
            state.update_image(rgb, channel, ts)

    def on_pose(p: Dict[str, object]) -> None:
        pos_viz = _map_position_odom_to_viz(p.get("position_m"))
        quat_viz = _map_quat_odom_to_viz(p.get("orientation_xyzw"))
        state.update_pose(pos_viz, quat_viz)

    def on_imu(imu: Dict[str, object]) -> None:
        state.update_imu(imu.get("samples", []))

    def on_vio_state(vs: Dict[str, object]) -> None:
        state.update_vio_state(vs)

    def on_status(st: Dict[str, object]) -> None:
        state.update_status(str(st.get("text", "") or ""))

    def on_keyframe(kf: Dict[str, object]) -> None:
        state.update_status(f"keyframe {int(kf.get('timestamp_ns') or 0)} dim={int(kf.get('descriptor_dim') or 0)}")

    def on_reset(_: Dict[str, object]) -> None:
        state.update_status("N/A")

    def on_error(err: Dict[str, object]) -> None:
        msg = f"{err.get('scope', 'unknown')}:{err.get('code', 'unknown')} {err.get('message', '')}"
        state.set_error(msg)

    client.on_image(on_image)
    client.on_pose(on_pose)
    client.on_imu(on_imu)
    client.on_vio_state(on_vio_state)
    client.on_keyframe(on_keyframe)
    client.on_status(on_status)
    client.on_reset(on_reset)
    client.on_error(on_error)


def run_client_lifecycle(
    state: DashboardState,
    stop_event: threading.Event,
    client: MightyClient,
    device: MightyWebDevice,
) -> None:
    state.set_connection("connecting", device.get_info().get("source", ""))
    client.connect()

    while not stop_event.is_set():
        connected = bool(client.is_connected())
        source = str(device.get_info().get("source", "") or "")
        state.set_connection("connected" if connected else "disconnected", source)
        stop_event.wait(0.2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live Mighty VIO Python dashboard")
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Reconnect delay (seconds)")
    parser.add_argument("--imu-window", type=float, default=10.0, help="IMU plot history window (seconds)")
    parser.add_argument("--fps", type=int, default=20, help="GUI refresh FPS")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    # Plain constructor: uses SDK default host fallback list.
    device = MightyWebDevice()

    state = DashboardState()
    client = MightyClient(device, auto_reconnect=True, reconnect_delay_s=float(args.reconnect_delay))
    wire_client_callbacks(client, state)

    stop_event = threading.Event()
    worker = threading.Thread(
        target=run_client_lifecycle,
        args=(state, stop_event, client, device),
        name="MightyClientLifecycle",
        daemon=True,
    )
    worker.start()

    def start_vio_from_ui() -> Dict[str, object]:
        return client.start_vio()

    try:
        launch_gui(
            state,
            imu_window_s=float(args.imu_window),
            fps=int(args.fps),
            on_start_vio=start_vio_from_ui,
        )
    finally:
        stop_event.set()
        try:
            client.disconnect()
        except Exception:
            pass
        worker.join(timeout=3.0)


if __name__ == "__main__":
    main()

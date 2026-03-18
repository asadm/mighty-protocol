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
import mighty_protocol as mp  # noqa: E402

from uihelpers import (  # noqa: E402
    DashboardState,
    decode_jpeg_to_rgb,
    decode_raw_to_rgb,
    launch_gui,
)


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

    def on_any(evt: Dict[str, object]) -> None:
        # Python SDK currently maps RAW/SRAW to image events.
        # Handle JPG fallback via unknown event.
        if evt.get("type") != "unknown":
            return
        raw_type = evt.get("raw_type")
        payload = evt.get("payload", b"")
        if raw_type not in ("JPG ", "RJPG"):
            return
        try:
            decoded = mp.decode_jpg_payload(payload, raw_type == "RJPG")
        except Exception:
            return
        rgb = decode_jpeg_to_rgb(decoded.get("data", b""))
        if rgb is None:
            return
        channel = "ref" if raw_type == "RJPG" else str(decoded.get("channel") or "cam0")
        if channel == "preview":
            channel = "cam0"
        ts = int(decoded.get("timestamp_ns") or 0)
        state.update_image(rgb, channel, ts)

    def on_pose(p: Dict[str, object]) -> None:
        state.update_pose(p.get("position"), p.get("quat"))

    def on_imu(imu: Dict[str, object]) -> None:
        state.update_imu(imu.get("samples", []))

    def on_vio_state(vs: Dict[str, object]) -> None:
        state.update_vio_state(vs)

    def on_status(st: Dict[str, object]) -> None:
        state.update_status(str(st.get("text", "") or ""))

    def on_reset(_: Dict[str, object]) -> None:
        state.update_status("N/A")

    def on_error(err: Dict[str, object]) -> None:
        msg = f"{err.get('scope', 'unknown')}:{err.get('code', 'unknown')} {err.get('message', '')}"
        state.set_error(msg)

    client.on_image(on_image)
    client.on_any(on_any)
    client.on_pose(on_pose)
    client.on_imu(on_imu)
    client.on_vio_state(on_vio_state)
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
    was_connected = False

    while not stop_event.is_set():
        connected = bool(client.is_connected())
        source = str(device.get_info().get("source", "") or "")
        state.set_connection("connected" if connected else "disconnected", source)

        if connected and not was_connected:
            try:
                cmd = client.start_vio()
                if not cmd.get("ok", False):
                    state.update_status(f"start_vio failed: {cmd.get('message', 'unknown')}")
            except Exception as exc:
                state.update_status(f"start_vio error: {exc}")
        was_connected = connected
        stop_event.wait(0.2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live Mighty VIO Python dashboard")
    parser.add_argument(
        "--hosts",
        default="",
        help="Comma-separated host base URLs. Empty means SDK defaults.",
    )
    parser.add_argument("--reconnect-delay", type=float, default=1.0, help="Reconnect delay (seconds)")
    parser.add_argument("--imu-window", type=float, default=10.0, help="IMU plot history window (seconds)")
    parser.add_argument("--fps", type=int, default=20, help="GUI refresh FPS")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    hosts = [h.strip() for h in str(args.hosts).split(",") if h.strip()]
    if hosts:
        device = MightyWebDevice(
            base_urls=hosts,
            stream_path="/stream",
            command_path="/command",
            connect_timeout_s=2.0,
            read_timeout_s=2.0,
        )
    else:
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

    try:
        launch_gui(state, imu_window_s=float(args.imu_window), fps=int(args.fps))
    finally:
        stop_event.set()
        try:
            client.disconnect()
        except Exception:
            pass
        worker.join(timeout=3.0)


if __name__ == "__main__":
    main()

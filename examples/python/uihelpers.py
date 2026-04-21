from __future__ import annotations

import threading
import time
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Optional, Sequence, Tuple

import numpy as np

import mighty_protocol as mp

STATE_LABELS = {
    0: "OFF",
    1: "INITIALIZING",
    2: "TRACKING",
    3: "DEGRADED",
    4: "LOST",
    5: "LOW_LIGHT",
}


def quat_xyzw_to_rotmat(quat: Sequence[float]) -> np.ndarray:
    if quat is None or len(quat) != 4:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = [float(v) for v in quat]
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def decode_raw_to_rgb(raw: Dict[str, Any]) -> Optional[np.ndarray]:
    width = int(raw.get("width", 0) or 0)
    height = int(raw.get("height", 0) or 0)
    if width <= 0 or height <= 0:
        return None

    data = raw.get("data", b"") or b""
    arr = np.frombuffer(data, dtype=np.uint8)
    fmt = int(raw.get("format", mp.RAW_FORMAT["UNKNOWN"]) or 0)

    need = width * height
    if fmt == mp.RAW_FORMAT["GRAY8"]:
        if arr.size < need:
            return None
        g = arr[:need].reshape((height, width))
        return np.stack([g, g, g], axis=-1)

    if fmt == mp.RAW_FORMAT["RGB24"]:
        need3 = need * 3
        if arr.size < need3:
            return None
        return arr[:need3].reshape((height, width, 3))

    if fmt == mp.RAW_FORMAT["BGR24"]:
        need3 = need * 3
        if arr.size < need3:
            return None
        bgr = arr[:need3].reshape((height, width, 3))
        return bgr[:, :, ::-1]

    if fmt == mp.RAW_FORMAT["RGBA32"]:
        need4 = need * 4
        if arr.size < need4:
            return None
        rgba = arr[:need4].reshape((height, width, 4))
        return rgba[:, :, :3]

    if fmt == mp.RAW_FORMAT["BGRA32"]:
        need4 = need * 4
        if arr.size < need4:
            return None
        bgra = arr[:need4].reshape((height, width, 4))
        return bgra[:, :, [2, 1, 0]]

    if fmt in (mp.RAW_FORMAT["YUV420SP"], mp.RAW_FORMAT["YUV420P"]):
        if arr.size < need:
            return None
        y = arr[:need].reshape((height, width))
        return np.stack([y, y, y], axis=-1)

    return None


@dataclass
class DashboardState:
    lock: threading.Lock = field(default_factory=threading.Lock)

    connection_state: str = "disconnected"
    connection_source: str = ""
    last_error: str = ""
    rx_events: int = 0
    last_data_time_s: float = 0.0

    status_text: str = "N/A"
    host_version: str = "Unknown"
    vio_state_text: str = "STATE_NA"
    vio_state_code: Optional[int] = None
    fps_text: str = "FPS: NA"
    imu_hz_text: str = "IMU Hz: NA"
    features_text: str = "Features: NA"
    pose_conf_text: str = "Pose Conf: NA"

    image_rgb: Optional[np.ndarray] = None
    image_channel: str = ""
    image_timestamp_ns: int = 0

    pose_path: Deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=3000))
    pose_latest: Optional[np.ndarray] = None
    pose_quat_latest: Optional[np.ndarray] = None

    imu_samples: Deque[Tuple[float, float, float, float, float, float, float]] = field(
        default_factory=lambda: deque(maxlen=6000)
    )

    def set_connection(self, state: str, source: str = "") -> None:
        with self.lock:
            self.connection_state = state
            self.connection_source = source

    def set_error(self, msg: str) -> None:
        with self.lock:
            self.last_error = msg

    def update_image(self, rgb: np.ndarray, channel: str, timestamp_ns: int) -> None:
        with self.lock:
            self.image_rgb = rgb
            self.image_channel = channel
            self.image_timestamp_ns = int(timestamp_ns)
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_pose(self, position: Sequence[float], quat: Optional[Sequence[float]]) -> None:
        if position is None or len(position) < 3:
            return
        pos = np.array([float(position[0]), float(position[1]), float(position[2])], dtype=np.float64)
        q = None
        if quat is not None and len(quat) == 4:
            q = np.array([float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])], dtype=np.float64)
        with self.lock:
            self.pose_latest = pos
            self.pose_quat_latest = q
            self.pose_path.append(pos)
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_imu(self, samples: Sequence[Dict[str, Any]]) -> None:
        if not samples:
            return
        with self.lock:
            for s in samples:
                ts_ns = s.get("timestamp_ns")
                if isinstance(ts_ns, int):
                    t = float(ts_ns) / 1e9
                else:
                    t = time.time()
                self.imu_samples.append(
                    (
                        t,
                        float(s.get("ax", 0.0)),
                        float(s.get("ay", 0.0)),
                        float(s.get("az", 0.0)),
                        float(s.get("gx", 0.0)),
                        float(s.get("gy", 0.0)),
                        float(s.get("gz", 0.0)),
                    )
                )
            self.rx_events += len(samples)
            self.last_data_time_s = time.time()

    def update_vio_state(self, s: Dict[str, Any]) -> None:
        code = s.get("state")
        try:
            code_int = int(code)
        except Exception:
            code_int = None
        with self.lock:
            self.vio_state_code = code_int
            self.vio_state_text = STATE_LABELS.get(code_int, f"STATE_{code_int}")
            light_level = s.get("light_level01")
            if isinstance(light_level, (int, float)) and math.isfinite(float(light_level)):
                light_required = s.get("light_required01")
                required = float(light_required) if isinstance(light_required, (int, float)) and math.isfinite(float(light_required)) else 0.0
                self.vio_state_text = f"{self.vio_state_text} darkness {float(light_level):.4f}/{required:.4f}"
            if s.get("build_version"):
                self.host_version = str(s.get("build_version"))
            fps_cur = s.get("fps_current")
            fps_avg = s.get("fps_average")
            if isinstance(fps_cur, (int, float)) and isinstance(fps_avg, (int, float)):
                self.fps_text = f"FPS: {fps_cur:.1f} (avg {fps_avg:.1f})"
            imu_cur = s.get("imu_hz_current")
            imu_avg = s.get("imu_hz_average_5s")
            if isinstance(imu_cur, (int, float)) and isinstance(imu_avg, (int, float)):
                self.imu_hz_text = f"IMU Hz: {imu_cur:.1f} (avg5s {imu_avg:.1f})"
            nfeat = s.get("num_features")
            if isinstance(nfeat, int):
                self.features_text = f"Features: {nfeat}"
            conf = s.get("pose_confidence")
            if isinstance(conf, (int, float)):
                level = "HIGH" if conf > 0.4 else "LOW"
                self.pose_conf_text = f"Pose Conf: {conf:.3f} ({level})"
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def update_status(self, text: str) -> None:
        with self.lock:
            if text.startswith("HOST_VERSION:"):
                self.host_version = text.replace("HOST_VERSION:", "").strip() or "Unknown"
                self.rx_events += 1
                self.last_data_time_s = time.time()
                return
            self.status_text = text
            self.rx_events += 1
            self.last_data_time_s = time.time()

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "connection_state": self.connection_state,
                "connection_source": self.connection_source,
                "last_error": self.last_error,
                "rx_events": self.rx_events,
                "last_data_time_s": self.last_data_time_s,
                "status_text": self.status_text,
                "host_version": self.host_version,
                "vio_state_text": self.vio_state_text,
                "vio_state_code": self.vio_state_code,
                "fps_text": self.fps_text,
                "imu_hz_text": self.imu_hz_text,
                "features_text": self.features_text,
                "pose_conf_text": self.pose_conf_text,
                "image_rgb": None if self.image_rgb is None else self.image_rgb.copy(),
                "image_channel": self.image_channel,
                "image_timestamp_ns": self.image_timestamp_ns,
                "pose_latest": None if self.pose_latest is None else self.pose_latest.copy(),
                "pose_quat_latest": None if self.pose_quat_latest is None else self.pose_quat_latest.copy(),
                "pose_path": [p.copy() for p in self.pose_path],
                "imu_samples": list(self.imu_samples),
            }


def set_axis_limits_from_points(ax: Any, points: np.ndarray) -> float:
    if points.size == 0:
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(-1.0, 1.0)
        return 1.0

    mn = points.min(axis=0)
    mx = points.max(axis=0)
    center = (mn + mx) * 0.5
    radius = float(np.max(mx - mn) * 0.5)
    radius = max(radius, 0.5)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    return radius


def launch_gui(
    state: DashboardState,
    imu_window_s: float = 10.0,
    fps: int = 20,
    on_start_vio: Optional[Any] = None,
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib import gridspec
        from matplotlib.widgets import Button
    except Exception as exc:
        raise RuntimeError(
            "matplotlib is required for this example. Install with: pip install matplotlib"
        ) from exc

    fig = plt.figure(figsize=(14, 8))
    outer = gridspec.GridSpec(1, 2, width_ratios=[1, 2], wspace=0.12)
    left = gridspec.GridSpecFromSubplotSpec(
        4,
        1,
        subplot_spec=outer[0],
        height_ratios=[1.2, 2.3, 1.5, 1.5],
        hspace=0.35,
    )

    ax_status = fig.add_subplot(left[0])
    ax_image = fig.add_subplot(left[1])
    ax_acc = fig.add_subplot(left[2])
    ax_gyro = fig.add_subplot(left[3])
    ax_pose = fig.add_subplot(outer[1], projection="3d")

    ax_status.axis("off")
    status_text_artist = ax_status.text(
        0.01,
        1.06,
        "",
        va="top",
        ha="left",
        family="monospace",
        fontsize=9,
        linespacing=1.0,
        clip_on=False,
        transform=ax_status.transAxes,
    )

    status_pos = ax_status.get_position()
    btn_w = status_pos.width * 0.42
    btn_h = min(0.04, status_pos.height * 0.28)
    btn_x = status_pos.x1 - btn_w
    btn_y = status_pos.y1 - btn_h
    ax_start_vio_btn = fig.add_axes([btn_x, btn_y, btn_w, btn_h])
    start_vio_btn = Button(ax_start_vio_btn, "Start VIO")

    def _on_start_vio_clicked(_: Any) -> None:
        if on_start_vio is None:
            state.update_status("start_vio unavailable")
            return

        def _run() -> None:
            try:
                res = on_start_vio()
                if isinstance(res, dict) and not res.get("ok", False):
                    state.update_status(f"start_vio failed: {res.get('message', 'unknown')}")
                    return
                state.update_status("start_vio sent")
            except Exception as exc:
                state.update_status(f"start_vio error: {exc}")

        threading.Thread(target=_run, name="StartVioButton", daemon=True).start()

    start_vio_btn.on_clicked(_on_start_vio_clicked)

    ax_image.set_title("Camera")
    ax_image.set_xticks([])
    ax_image.set_yticks([])
    img_artist = ax_image.imshow(np.zeros((2, 2, 3), dtype=np.uint8), interpolation="nearest")

    ax_acc.set_title("IMU Accel")
    ax_acc.set_xlabel("t (s)")
    ax_acc.set_ylabel("m/s^2")
    acc_x, = ax_acc.plot([], [], label="ax")
    acc_y, = ax_acc.plot([], [], label="ay")
    acc_z, = ax_acc.plot([], [], label="az")
    ax_acc.legend(loc="upper right", fontsize=8)
    ax_acc.grid(True, alpha=0.3)

    ax_gyro.set_title("IMU Gyro")
    ax_gyro.set_xlabel("t (s)")
    ax_gyro.set_ylabel("rad/s")
    gyro_x, = ax_gyro.plot([], [], label="gx")
    gyro_y, = ax_gyro.plot([], [], label="gy")
    gyro_z, = ax_gyro.plot([], [], label="gz")
    ax_gyro.legend(loc="upper right", fontsize=8)
    ax_gyro.grid(True, alpha=0.3)

    ax_pose.set_title("Pose (3D)")
    ax_pose.set_xlabel("X")
    ax_pose.set_ylabel("Y")
    ax_pose.set_zlabel("Z")
    traj_line, = ax_pose.plot([], [], [], color="tab:blue", lw=1.5, label="trajectory")
    pose_dot, = ax_pose.plot([], [], [], "o", color="tab:red", ms=6, label="current")
    axis_x, = ax_pose.plot([], [], [], color="r", lw=2)
    axis_y, = ax_pose.plot([], [], [], color="g", lw=2)
    axis_z, = ax_pose.plot([], [], [], color="b", lw=2)
    ax_pose.legend(loc="upper right", fontsize=8)

    def update(_: int):
        snap = state.snapshot()

        status_lines = [
            f"Conn: {snap['connection_state']} {snap['connection_source']}",
            f"VIO:  {snap['vio_state_text']} ({snap['vio_state_code']})",
            f"RX events: {snap['rx_events']}",
            f"{snap['fps_text']}",
            f"{snap['imu_hz_text']}",
            f"{snap['features_text']}",
            f"{snap['pose_conf_text']}",
            f"Host: {snap['host_version']}",
            f"Status: {snap['status_text']}",
        ]
        if snap["last_error"]:
            status_lines.append(f"Err: {snap['last_error']}")
        status_text_artist.set_text("\n".join(status_lines))

        rgb = snap["image_rgb"]
        if rgb is not None:
            img_artist.set_data(rgb)
            h, w = rgb.shape[:2]
            ax_image.set_title(f"Camera [{snap['image_channel']}] {w}x{h}")

        imu = snap["imu_samples"]
        if imu:
            imu_arr = np.asarray(imu, dtype=np.float64)
            t = imu_arr[:, 0]
            t_rel = t - t[-1]
            keep = t_rel >= -float(imu_window_s)
            t_rel = t_rel[keep]
            vals = imu_arr[keep, :]

            ax_vals = vals[:, 1:4]
            gx_vals = vals[:, 4:7]

            acc_x.set_data(t_rel, ax_vals[:, 0])
            acc_y.set_data(t_rel, ax_vals[:, 1])
            acc_z.set_data(t_rel, ax_vals[:, 2])
            gyro_x.set_data(t_rel, gx_vals[:, 0])
            gyro_y.set_data(t_rel, gx_vals[:, 1])
            gyro_z.set_data(t_rel, gx_vals[:, 2])

            ax_acc.set_xlim(-imu_window_s, 0.0)
            ax_gyro.set_xlim(-imu_window_s, 0.0)

            if ax_vals.size:
                amax = float(np.max(np.abs(ax_vals)))
                amax = max(amax, 0.1)
                ax_acc.set_ylim(-1.15 * amax, 1.15 * amax)
            if gx_vals.size:
                gmax = float(np.max(np.abs(gx_vals)))
                gmax = max(gmax, 0.1)
                ax_gyro.set_ylim(-1.15 * gmax, 1.15 * gmax)

        path = snap["pose_path"]
        if path:
            p = np.asarray(path, dtype=np.float64)
            traj_line.set_data_3d(p[:, 0], p[:, 1], p[:, 2])
            latest = p[-1]
            pose_dot.set_data_3d([latest[0]], [latest[1]], [latest[2]])
            radius = set_axis_limits_from_points(ax_pose, p)

            q = snap["pose_quat_latest"]
            if q is not None and len(q) == 4:
                rot = quat_xyzw_to_rotmat(q)
                axis_len = max(0.1, radius * 0.2)
                origin = latest
                ex = origin + rot[:, 0] * axis_len
                ey = origin + rot[:, 1] * axis_len
                ez = origin + rot[:, 2] * axis_len
                axis_x.set_data_3d([origin[0], ex[0]], [origin[1], ex[1]], [origin[2], ex[2]])
                axis_y.set_data_3d([origin[0], ey[0]], [origin[1], ey[1]], [origin[2], ey[2]])
                axis_z.set_data_3d([origin[0], ez[0]], [origin[1], ez[1]], [origin[2], ez[2]])

        return (
            status_text_artist,
            img_artist,
            acc_x,
            acc_y,
            acc_z,
            gyro_x,
            gyro_y,
            gyro_z,
            traj_line,
            pose_dot,
            axis_x,
            axis_y,
            axis_z,
        )

    anim = FuncAnimation(fig, update, interval=max(25, int(1000 / max(1, fps))), blit=False)
    setattr(fig, "_mighty_anim_ref", anim)
    plt.show()

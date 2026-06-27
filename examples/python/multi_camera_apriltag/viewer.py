from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from state import (
    CameraPanelState,
    event_summary,
    quat_xyzw_to_rotmat,
    set_axis_limits_from_points,
)


CommandCallback = Callable[[], Any]


def _button_thread(name: str, callback: Optional[CommandCallback], set_note: Callable[[str], None]) -> None:
    if callback is None:
        set_note(f"{name} unavailable")
        return
    try:
        result = callback()
        if isinstance(result, dict):
            ok = bool(result.get("ok", True))
            message = str(result.get("message") or result.get("path") or "")
            set_note(f"{name}: {'ok' if ok else 'failed'} {message}".strip())
        elif isinstance(result, tuple) and len(result) >= 2:
            ok = bool(result[0])
            set_note(f"{name}: {'ok' if ok else 'failed'} {result[1]}")
        elif result is not None:
            set_note(f"{name}: {result}")
        else:
            set_note(f"{name}: sent")
    except Exception as exc:
        set_note(f"{name}: {exc}")


def _event_lines(snapshots: Sequence[Dict[str, Any]], max_lines: int = 9) -> List[str]:
    rows: List[Tuple[float, str]] = []
    for snap in snapshots:
        name = str(snap["name"])
        for event in snap.get("events", [])[:max_lines]:
            rows.append((float(event.get("time_s") or 0.0), f"{name}: {event_summary(event)}"))
    rows.sort(key=lambda item: item[0], reverse=True)
    return [text for _, text in rows[:max_lines]]


def _draw_tags(ax: Any, tags: Iterable[Dict[str, Any]], color: str, recent_tag_id: Optional[int], recent_at_s: float) -> None:
    now_s = time.time()
    for tag in tags:
        corners = list(tag.get("corners", []) or [])
        if len(corners) >= 4:
            xs = [float(p[0]) for p in corners] + [float(corners[0][0])]
            ys = [float(p[1]) for p in corners] + [float(corners[0][1])]
            is_recent = (
                recent_tag_id is not None
                and int(tag.get("id", -1)) == int(recent_tag_id)
                and now_s - float(recent_at_s or 0.0) < 4.0
            )
            ax.plot(xs, ys, color=color, lw=3.2 if is_recent else 2.0)
        center = tag.get("center", (0.0, 0.0))
        if len(center) >= 2:
            ax.text(
                float(center[0]),
                float(center[1]),
                f"#{int(tag.get('id', 0))}",
                color="white",
                fontsize=8,
                ha="center",
                va="center",
                bbox={"facecolor": color, "edgecolor": "white", "boxstyle": "round,pad=0.18", "alpha": 0.9},
            )


def _draw_preview(ax: Any, snap: Dict[str, Any]) -> None:
    ax.clear()
    rgb = snap.get("image_rgb")
    ax.set_xticks([])
    ax.set_yticks([])
    title = f"{snap['name']} preview"
    if rgb is None:
        ax.set_title(title)
        ax.text(0.5, 0.5, "waiting for image", ha="center", va="center", transform=ax.transAxes)
        return
    ax.imshow(rgb, interpolation="nearest")
    h, w = rgb.shape[:2]
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_title(f"{title} [{snap.get('image_channel') or 'cam0'}] {w}x{h}")
    _draw_tags(
        ax,
        snap.get("tags", []),
        str(snap.get("color") or "#2563eb"),
        snap.get("last_relocalized_tag_id"),
        float(snap.get("last_relocalized_at_s") or 0.0),
    )


def _status_text(snap: Dict[str, Any]) -> str:
    lines = [
        f"{snap['name']}  {snap['connection_state']}  {snap['connection_source']}",
        f"VIO: {snap['vio_state_text']}  {snap['fps_text']}  {snap['pose_conf_text']}",
        f"RX image/pose/tags/imu: {snap['rx_images']}/{snap['rx_poses']}/{snap['rx_tags']}/{snap['rx_imu_samples']}",
        f"Status: {snap['status_text']}",
    ]
    if snap.get("last_error"):
        lines.append(f"Err: {snap['last_error']}")
    return "\n".join(lines)


def _update_pose_artists(
    ax_pose: Any,
    snaps: Sequence[Dict[str, Any]],
    traj_artists: Dict[str, Any],
    dot_artists: Dict[str, Any],
    axis_artists: Dict[str, Tuple[Any, Any, Any]],
) -> None:
    all_points: List[np.ndarray] = []
    for snap in snaps:
        name = str(snap["name"])
        color = str(snap["color"])
        path = snap.get("pose_path") or []
        if path:
            p = np.asarray(path, dtype=np.float64)
            traj_artists[name].set_data_3d(p[:, 0], p[:, 1], p[:, 2])
            all_points.append(p)
            latest = p[-1]
            dot_artists[name].set_data_3d([latest[0]], [latest[1]], [latest[2]])
            quat = snap.get("pose_quat_latest")
            if quat is not None and len(quat) == 4:
                rot = quat_xyzw_to_rotmat(quat)
                axis_len = 0.16
                origin = latest
                ex = origin + rot[:, 0] * axis_len
                ey = origin + rot[:, 1] * axis_len
                ez = origin + rot[:, 2] * axis_len
                axis_x, axis_y, axis_z = axis_artists[name]
                axis_x.set_data_3d([origin[0], ex[0]], [origin[1], ex[1]], [origin[2], ex[2]])
                axis_y.set_data_3d([origin[0], ey[0]], [origin[1], ey[1]], [origin[2], ey[2]])
                axis_z.set_data_3d([origin[0], ez[0]], [origin[1], ez[1]], [origin[2], ez[2]])
        else:
            traj_artists[name].set_data_3d([], [], [])
            dot_artists[name].set_data_3d([], [], [])
            for artist in axis_artists[name]:
                artist.set_data_3d([], [], [])

    if all_points:
        points = np.concatenate(all_points, axis=0)
        radius = set_axis_limits_from_points(ax_pose, points)
        for snap in snaps:
            name = str(snap["name"])
            axis_len = max(0.1, radius * 0.14)
            path = snap.get("pose_path") or []
            quat = snap.get("pose_quat_latest")
            if not path or quat is None or len(quat) != 4:
                continue
            latest = np.asarray(path[-1], dtype=np.float64)
            rot = quat_xyzw_to_rotmat(quat)
            axis_x, axis_y, axis_z = axis_artists[name]
            ex = latest + rot[:, 0] * axis_len
            ey = latest + rot[:, 1] * axis_len
            ez = latest + rot[:, 2] * axis_len
            axis_x.set_data_3d([latest[0], ex[0]], [latest[1], ex[1]], [latest[2], ex[2]])
            axis_y.set_data_3d([latest[0], ey[0]], [latest[1], ey[1]], [latest[2], ey[2]])
            axis_z.set_data_3d([latest[0], ez[0]], [latest[1], ez[1]], [latest[2], ez[2]])
    else:
        set_axis_limits_from_points(ax_pose, np.empty((0, 3), dtype=np.float64))


def launch_gui(
    red_state: CameraPanelState,
    blue_state: CameraPanelState,
    *,
    recorder: Optional[Any] = None,
    fps: int = 20,
    on_start_vio: Optional[CommandCallback] = None,
    on_stop_vio: Optional[CommandCallback] = None,
    on_toggle_recording: Optional[CommandCallback] = None,
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib import gridspec
        from matplotlib.widgets import Button
    except Exception as exc:
        raise RuntimeError("matplotlib is required for this example. Install with: pip install matplotlib") from exc

    fig = plt.figure(figsize=(15, 8.4))
    outer = gridspec.GridSpec(1, 2, width_ratios=[1.05, 1.65], wspace=0.12)
    left = gridspec.GridSpecFromSubplotSpec(
        4,
        1,
        subplot_spec=outer[0],
        height_ratios=[1.0, 2.0, 2.0, 1.25],
        hspace=0.32,
    )

    ax_status = fig.add_subplot(left[0])
    ax_red = fig.add_subplot(left[1])
    ax_blue = fig.add_subplot(left[2])
    ax_events = fig.add_subplot(left[3])
    ax_pose = fig.add_subplot(outer[1], projection="3d")

    ax_status.axis("off")
    ax_events.axis("off")
    status_artist = ax_status.text(
        0.01,
        1.05,
        "",
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.5,
        linespacing=1.0,
        clip_on=False,
        transform=ax_status.transAxes,
    )
    events_artist = ax_events.text(
        0.01,
        1.0,
        "",
        va="top",
        ha="left",
        family="monospace",
        fontsize=8.5,
        linespacing=1.08,
        clip_on=False,
        transform=ax_events.transAxes,
    )

    note = {"text": "", "until_s": 0.0}

    def set_note(text: str) -> None:
        note["text"] = str(text)
        note["until_s"] = time.time() + 5.0

    status_pos = ax_status.get_position()
    btn_w = status_pos.width * 0.27
    btn_h = min(0.038, status_pos.height * 0.28)
    gap = status_pos.width * 0.025
    btn_y = status_pos.y1 - btn_h
    btn_x0 = status_pos.x1 - (btn_w * 3.0 + gap * 2.0)
    ax_start = fig.add_axes([btn_x0, btn_y, btn_w, btn_h])
    ax_stop = fig.add_axes([btn_x0 + btn_w + gap, btn_y, btn_w, btn_h])
    ax_record = fig.add_axes([btn_x0 + 2.0 * (btn_w + gap), btn_y, btn_w, btn_h])

    start_btn = Button(ax_start, "Start VIO")
    stop_btn = Button(ax_stop, "Stop VIO")
    rec_btn = Button(ax_record, "Record")

    start_btn.on_clicked(lambda _: threading.Thread(target=_button_thread, args=("start_vio", on_start_vio, set_note), daemon=True).start())
    stop_btn.on_clicked(lambda _: threading.Thread(target=_button_thread, args=("stop_vio", on_stop_vio, set_note), daemon=True).start())
    rec_btn.on_clicked(lambda _: threading.Thread(target=_button_thread, args=("record", on_toggle_recording, set_note), daemon=True).start())

    ax_pose.set_title("Relocalized Camera Poses")
    ax_pose.set_xlabel("X")
    ax_pose.set_ylabel("Y")
    ax_pose.set_zlabel("Z")
    ax_pose.grid(True, alpha=0.25)

    snaps0 = [red_state.snapshot(copy_image=False), blue_state.snapshot(copy_image=False)]
    traj_artists: Dict[str, Any] = {}
    dot_artists: Dict[str, Any] = {}
    axis_artists: Dict[str, Tuple[Any, Any, Any]] = {}
    for snap in snaps0:
        name = str(snap["name"])
        color = str(snap["color"])
        traj_artists[name], = ax_pose.plot([], [], [], color=color, lw=1.8, label=f"{name} path")
        dot_artists[name], = ax_pose.plot([], [], [], "o", color=color, ms=6, label=f"{name} current")
        axis_artists[name] = (
            ax_pose.plot([], [], [], color=color, lw=2.0, alpha=0.95)[0],
            ax_pose.plot([], [], [], color=color, lw=1.4, alpha=0.70)[0],
            ax_pose.plot([], [], [], color=color, lw=1.0, alpha=0.45)[0],
        )
    ax_pose.legend(loc="upper right", fontsize=8)

    def update(_: int):
        red = red_state.snapshot()
        blue = blue_state.snapshot()
        snaps = [red, blue]
        _draw_preview(ax_red, red)
        _draw_preview(ax_blue, blue)
        _update_pose_artists(ax_pose, snaps, traj_artists, dot_artists, axis_artists)

        record_line = "Record: unavailable"
        if recorder is not None:
            stats = recorder.snapshot()
            active = bool(stats.get("active"))
            record_line = (
                f"Record: {'ON' if active else 'off'} "
                f"images={stats.get('image_messages', 0)} "
                f"poses={stats.get('red_pose_messages', 0)}/{stats.get('blue_pose_messages', 0)} "
                f"events={stats.get('red_event_messages', 0)}/{stats.get('blue_event_messages', 0)}"
            )
            if stats.get("last_error"):
                record_line += f" err={stats.get('last_error')}"

        note_line = str(note["text"]) if time.time() < float(note["until_s"]) else ""
        status_artist.set_text(
            "\n\n".join([_status_text(red), _status_text(blue), record_line, note_line]).strip()
        )
        event_text = "\n".join(_event_lines(snaps)) or "EVNT: waiting"
        events_artist.set_text(event_text)
        return []

    anim = FuncAnimation(fig, update, interval=max(25, int(1000 / max(1, int(fps)))), blit=False)
    setattr(fig, "_mighty_multi_apriltag_anim", anim)
    plt.show()

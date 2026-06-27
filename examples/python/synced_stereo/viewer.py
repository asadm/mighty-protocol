from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from state import NsTuple, select_synced_image_pair
from utils import fmt_ms, fmt_ns, fmt_rel_s, image_pair_age_ms


def build_status_lines(
    snap: Dict[str, Any],
    sync_zero_ns: Optional[int],
    refiner_stats: Optional[Dict[str, Any]] = None,
    display_frame: Optional[Dict[str, Any]] = None,
    display_pair: Optional[Dict[str, Any]] = None,
    recorder_stats: Optional[Dict[str, Any]] = None,
    view_fps: Optional[float] = None,
) -> List[str]:
    shown_raw_ns = snap["image_raw_ns"]
    shown_synced_ns = snap["image_synced_ns"]
    if display_frame is not None:
        shown_raw_ns = int(display_frame.get("raw_ns") or 0)
        shown_synced_ns = display_frame.get("synced_ns")

    lines = [
        f"{snap['name']}  {snap['base_url']}",
        f"Conn: {snap['connection_state']} {snap['connection_source']}",
        f"Clock: {snap['clock_source']} off={fmt_ms(snap['clock_offset_ns'])} rtt={fmt_ms(snap['clock_rtt_ns'])}",
        f"Frame corr: {fmt_ms(snap['frame_correction_ns'])} pairs={snap['frame_correction_samples']}",
        f"Shown raw:  {fmt_ns(shown_raw_ns)}",
        f"Shown sync: {fmt_ns(shown_synced_ns)} (+{fmt_rel_s(shown_synced_ns, sync_zero_ns)})",
        f"IMU raw:    {fmt_ns(snap['imu_raw_ns'])}",
        f"IMU sync:   {fmt_ns(snap['imu_synced_ns'])} (+{fmt_rel_s(snap['imu_synced_ns'], sync_zero_ns)})",
        f"RX image/imu: {snap['rx_images']}/{snap['rx_imu_samples']}",
        f"{snap['fps_text']} | {snap['imu_hz_text']}",
    ]
    if view_fps is not None:
        lines.append(f"View FPS: {view_fps:.1f}")
    if display_pair is not None:
        age_ms = image_pair_age_ms(display_pair)
        age_text = "NA" if age_ms is None else f"{age_ms:.1f} ms"
        lines.append(f"Rendered pair: delta={fmt_ms(display_pair['delta_ns'])} age={age_text}")
    else:
        lines.append("Rendered pair: waiting")
    if recorder_stats is not None:
        if recorder_stats.get("active"):
            lines.append(
                "Record: "
                f"pairs={recorder_stats.get('image_pairs', 0)} "
                f"imu={recorder_stats.get('left_imu_messages', 0)}/{recorder_stats.get('right_imu_messages', 0)}"
            )
        else:
            lines.append("Record: off")
        if recorder_stats.get("last_error"):
            lines.append(f"Record err: {str(recorder_stats['last_error'])[:80]}")
    if snap["last_error"]:
        lines.append(f"Err: {snap['last_error'][:90]}")
    elif snap["status_text"] and snap["status_text"] != "N/A":
        lines.append(f"Status: {snap['status_text'][:90]}")
    if refiner_stats is not None:
        lines.append(
            "Pair delta raw/median/synced: "
            f"{fmt_ms(refiner_stats['last_raw_delta_ns'])} / "
            f"{fmt_ms(refiner_stats['median_base_delta_ns'])} / "
            f"{fmt_ms(refiner_stats['last_synced_delta_ns'])}"
        )
    return lines


def draw_text_lines(cv2: Any, canvas: np.ndarray, lines: Sequence[str], x: int, y: int, max_width: int) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    line_h = 16
    max_chars = max(12, max_width // 8)
    for index, line in enumerate(lines):
        text = str(line)
        if len(text) > max_chars:
            text = text[: max_chars - 1] + "~"
        color = (45, 45, 45)
        if text.startswith("Err:"):
            color = (32, 32, 220)
        cv2.putText(canvas, text, (x, y + index * line_h), font, 0.42, color, 1, cv2.LINE_AA)


def draw_recording_toolbar(
    cv2: Any,
    canvas: np.ndarray,
    recorder_stats: Dict[str, Any],
    button_rect: Tuple[int, int, int, int],
    note: str,
) -> None:
    x0, y0, x1, y1 = button_rect
    active = bool(recorder_stats.get("active"))
    fill = (70, 70, 210) if active else (70, 150, 75)
    label = "Stop Rec" if active else "Start Rec"

    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], y1 + 10), (232, 232, 232), -1)
    cv2.rectangle(canvas, (x0, y0), (x1, y1), fill, -1)
    cv2.rectangle(canvas, (x0, y0), (x1, y1), (50, 50, 50), 1)
    cv2.putText(canvas, label, (x0 + 18, y0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    path = str(recorder_stats.get("path") or "")
    if active:
        status = f"Recording: {path}"
    elif path:
        status = f"Saved: {path}"
    else:
        status = "Recording off"
    if note:
        status = note
    max_chars = max(24, (x0 - 24) // 8)
    if len(status) > max_chars:
        status = status[: max_chars - 1] + "~"
    color = (40, 40, 40) if not recorder_stats.get("last_error") else (32, 32, 220)
    cv2.putText(canvas, status, (12, y0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)


def image_to_bgr_fit(cv2: Any, image: Optional[np.ndarray], width: int, height: int) -> np.ndarray:
    out = np.full((height, width, 3), 24, dtype=np.uint8)
    if image is None:
        cv2.putText(out, "waiting for image", (18, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1, cv2.LINE_AA)
        return out

    h, w = image.shape[:2]
    if h <= 0 or w <= 0:
        return out
    scale = min(float(width) / float(w), float(height) / float(h))
    dst_w = max(1, int(round(w * scale)))
    dst_h = max(1, int(round(h * scale)))
    resized = cv2.resize(image, (dst_w, dst_h), interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_NEAREST)
    if resized.ndim == 2:
        resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
    elif resized.shape[2] == 4:
        resized = cv2.cvtColor(resized, cv2.COLOR_BGRA2BGR)

    x0 = (width - dst_w) // 2
    y0 = (height - dst_h) // 2
    out[y0 : y0 + dst_h, x0 : x0 + dst_w] = resized
    return out


def draw_depth_tile(
    cv2: Any,
    canvas: np.ndarray,
    image: Optional[np.ndarray],
    x: int,
    y: int,
    w: int,
    h: int,
    title: str,
    subtitle: str = "",
) -> None:
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (255, 255, 255), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (210, 210, 210), 1)
    tile = image_to_bgr_fit(cv2, image, w - 2, h - 2)
    canvas[y + 1 : y + h - 1, x + 1 : x + w - 1] = tile
    cv2.putText(canvas, title, (x + 10, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, title, (x + 10, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (18, 18, 18), 1, cv2.LINE_AA)
    if subtitle:
        cv2.rectangle(canvas, (x + 1, y + h - 27), (x + w - 1, y + h - 1), (245, 245, 245), -1)
        cv2.putText(canvas, subtitle[:76], (x + 8, y + h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (35, 35, 35), 1, cv2.LINE_AA)


def draw_depth_band(
    cv2: Any,
    canvas: np.ndarray,
    depth_result: Optional[Dict[str, Any]],
    depth_error: str,
    display_pair: Optional[Dict[str, Any]],
    x: int,
    y: int,
    w: int,
    tile_h: int,
) -> None:
    header_h = 28
    if depth_result is not None:
        stats = depth_result.get("stats", {})
        header = (
            "SGBM metric depth  "
            f"pair_delta={fmt_ms((display_pair or {}).get('delta_ns'))}  "
            f"{int(stats.get('width', 0))}x{int(stats.get('height', 0))}  "
            f"range={float(stats.get('min_depth_m', 0.0)):.2f}-{float(stats.get('max_depth_m', 0.0)):.2f}m  "
            f"baseline={float(stats.get('baseline_m', 0.0)):.4f}m  "
            f"sgbm={float(stats.get('sgbm_compute_ms', 0.0)):.1f}ms  "
            f"valid={float(stats.get('sgbm_valid_pct', 0.0)):.1f}%  "
            f"in-range={float(stats.get('sgbm_in_range_pct', 0.0)):.1f}%  "
            f"medianZ={float(stats.get('sgbm_median_depth_m', 0.0)):.2f}m"
        )
        color = (45, 45, 45)
    elif depth_error:
        header = f"Depth error: {depth_error[:150]}"
        color = (32, 32, 220)
    else:
        header = "Depth from rendered pair: waiting for synced stereo frames"
        color = (75, 75, 75)

    cv2.putText(canvas, header, (x, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)

    gap = 10
    tile_y = y + header_h
    tile_w = max(180, (w - gap * 2) // 3)
    tiles = [
        ("Left rect", (depth_result or {}).get("left_rect") if depth_result is not None else None, ""),
        ("Right rect", (depth_result or {}).get("right_rect") if depth_result is not None else None, ""),
        ("SGBM metric", (depth_result or {}).get("sgbm") if depth_result is not None else None, "warm=near, cool=far"),
    ]
    for index, (title, image, subtitle) in enumerate(tiles):
        draw_depth_tile(cv2, canvas, image, x + index * (tile_w + gap), tile_y, tile_w, tile_h, title, subtitle)


def draw_imu_series(
    cv2: Any,
    canvas: np.ndarray,
    samples: Sequence[NsTuple],
    x: int,
    y: int,
    w: int,
    h: int,
    title: str,
    cols: Tuple[int, int, int],
    labels: Tuple[str, str, str],
    window_s: float,
) -> None:
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (235, 235, 235), 1)
    cv2.putText(canvas, title, (x + 6, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (35, 35, 35), 1, cv2.LINE_AA)
    if not samples:
        cv2.putText(canvas, "waiting for IMU", (x + 12, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
        return

    arr = np.asarray(samples, dtype=np.float64)
    t = arr[:, 0]
    t_rel = t - t[-1]
    keep = t_rel >= -float(window_s)
    t_rel = t_rel[keep]
    values = arr[keep][:, list(cols)]
    if values.size == 0:
        return

    if t_rel.size > w:
        stride = int(math.ceil(float(t_rel.size) / float(w)))
        t_rel = t_rel[::stride]
        values = values[::stride, :]

    vmax = float(np.max(np.abs(values))) if values.size else 1.0
    vmax = max(vmax, 0.05)
    plot_x0 = x + 34
    plot_y0 = y + 24
    plot_w = max(10, w - 46)
    plot_h = max(10, h - 36)
    mid_y = plot_y0 + plot_h // 2
    cv2.line(canvas, (plot_x0, mid_y), (plot_x0 + plot_w, mid_y), (210, 210, 210), 1)

    colors = ((50, 70, 230), (60, 165, 75), (230, 135, 40))
    denom_t = max(float(window_s), 1e-6)
    for col_i, color in enumerate(colors):
        xs = plot_x0 + np.clip(((t_rel + float(window_s)) / denom_t) * plot_w, 0, plot_w).astype(np.int32)
        ys = mid_y - np.clip((values[:, col_i] / vmax) * (plot_h * 0.46), -plot_h * 0.46, plot_h * 0.46).astype(np.int32)
        if xs.size >= 2:
            pts = np.column_stack([xs, ys]).reshape((-1, 1, 2))
            cv2.polylines(canvas, [pts], False, color, 1, cv2.LINE_AA)
        cv2.putText(canvas, labels[col_i], (x + 7 + col_i * 28, y + h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)


def draw_opencv_panel(
    cv2: Any,
    canvas: np.ndarray,
    snap: Dict[str, Any],
    display_frame: Optional[Dict[str, Any]],
    x: int,
    y: int,
    panel_w: int,
    status_h: int,
    image_w: int,
    image_h: int,
    plot_h: int,
    sync_zero_ns: Optional[int],
    refiner_stats: Optional[Dict[str, Any]],
    display_pair: Optional[Dict[str, Any]],
    recorder_stats: Optional[Dict[str, Any]],
    imu_window_s: float,
    view_fps: float,
) -> None:
    cv2.rectangle(canvas, (x, y), (x + panel_w, y + status_h + image_h + plot_h + 36), (255, 255, 255), -1)
    cv2.rectangle(canvas, (x, y), (x + panel_w, y + status_h + image_h + plot_h + 36), (210, 210, 210), 1)

    lines = build_status_lines(
        snap,
        sync_zero_ns,
        refiner_stats=refiner_stats,
        display_frame=display_frame,
        display_pair=display_pair,
        recorder_stats=recorder_stats,
        view_fps=view_fps,
    )
    draw_text_lines(cv2, canvas, lines, x + 10, y + 18, panel_w - 20)

    image_y = y + status_h + 8
    shown_image = display_frame.get("image") if display_frame is not None else snap["image_rgb"]
    preview = image_to_bgr_fit(cv2, shown_image, image_w, image_h)
    canvas[image_y : image_y + image_h, x + 10 : x + 10 + image_w] = preview
    shown_channel = display_frame.get("channel") if display_frame is not None else snap["image_channel"]
    title = f"{snap['name']} [{shown_channel}]"
    if display_pair is not None:
        title += f" paired d={float(display_pair['delta_ns']) / 1e6:.3f}ms"
    else:
        title += " waiting for pair"
    cv2.putText(canvas, title, (x + 14, image_y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, title, (x + 14, image_y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 1, cv2.LINE_AA)

    plot_y = image_y + image_h + 8
    half_h = (plot_h - 8) // 2
    draw_imu_series(
        cv2,
        canvas,
        snap["imu_samples"],
        x + 10,
        plot_y,
        image_w,
        half_h,
        "Accel",
        (1, 2, 3),
        ("ax", "ay", "az"),
        imu_window_s,
    )
    draw_imu_series(
        cv2,
        canvas,
        snap["imu_samples"],
        x + 10,
        plot_y + half_h + 8,
        image_w,
        half_h,
        "Gyro",
        (4, 5, 6),
        ("gx", "gy", "gz"),
        imu_window_s,
    )


def launch_opencv_gui(
    left_state: CameraState,
    right_state: CameraState,
    refiner: ImagePairRefiner,
    recorder: RecorderController,
    depth_processor: Optional[StereoDepthProcessor],
    imu_window_s: float,
    fps: int,
    preview_width: int,
    preview_height: int,
    max_render_pair_delta_ms: float,
) -> None:
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("opencv-python is required for the OpenCV viewer") from exc

    margin = 12
    image_w = max(240, int(preview_width))
    image_h = max(160, int(preview_height))
    status_h = 275
    plot_h = 230
    toolbar_h = 50
    body_h = status_h + image_h + plot_h + 36
    depth_enabled = depth_processor is not None
    depth_tile_h = max(150, min(260, int(round(image_h * 0.62))))
    depth_extra_h = (margin + 28 + depth_tile_h) if depth_enabled else 0
    panel_w = image_w + 20
    canvas_w = margin * 3 + panel_w * 2
    canvas_h = margin * 2 + toolbar_h + body_h + depth_extra_h
    target_dt = 1.0 / max(1, int(fps))
    window_name = "Mighty Synced Stereo Preview"
    button_rect = (canvas_w - margin - 132, 10, canvas_w - margin, 40)
    record_note = {"text": "", "until_s": 0.0}

    def set_record_note(text: str) -> None:
        record_note["text"] = str(text)
        record_note["until_s"] = time.monotonic() + 4.0

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        x0, y0, x1, y1 = button_rect
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            return
        if recorder.active():
            ok, message = recorder.stop()
            set_record_note(f"Saved: {message}" if ok and message else f"Stop failed: {message}")
        else:
            ok, message = recorder.start()
            set_record_note(f"Recording: {message}" if ok else f"Start failed: {message}")

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, canvas_w, canvas_h)
    cv2.setMouseCallback(window_name, on_mouse)

    sync_zero_ns: Optional[int] = None
    frames = 0
    fps_started_s = time.monotonic()
    view_fps = 0.0
    display_pair: Optional[Dict[str, Any]] = None
    depth_pair_key: Tuple[int, int] = (0, 0)
    depth_result: Optional[Dict[str, Any]] = None
    depth_error = ""

    try:
        while True:
            frame_start_s = time.monotonic()
            left_snap = left_state.snapshot(copy_image=False)
            right_snap = right_state.snapshot(copy_image=False)
            refiner_stats = refiner.snapshot()
            recorder_stats = recorder.snapshot()
            candidate_pair = select_synced_image_pair(
                left_state.frame_buffer_snapshot(copy_image=False),
                right_state.frame_buffer_snapshot(copy_image=False),
                max_delta_ms=float(max_render_pair_delta_ms),
            )
            if candidate_pair is not None:
                display_pair = candidate_pair
            elif display_pair is not None:
                age_ms = image_pair_age_ms(display_pair)
                if age_ms is not None and age_ms > 1000.0:
                    display_pair = None
            if sync_zero_ns is None:
                for snap in (left_snap, right_snap):
                    if snap["image_synced_ns"] is not None:
                        sync_zero_ns = int(snap["image_synced_ns"])
                        break

            left_display = (display_pair or {}).get("left") if display_pair is not None else None
            right_display = (display_pair or {}).get("right") if display_pair is not None else None
            if depth_processor is not None and left_display is not None and right_display is not None:
                pair_key = (int(left_display.get("raw_ns") or 0), int(right_display.get("raw_ns") or 0))
                if pair_key != depth_pair_key:
                    depth_pair_key = pair_key
                    try:
                        depth_result = depth_processor.compute(cv2, left_display["image"], right_display["image"])
                        depth_error = ""
                    except Exception as exc:
                        depth_result = None
                        depth_error = str(exc)
            elif depth_processor is not None:
                depth_pair_key = (0, 0)

            canvas = np.full((canvas_h, canvas_w, 3), 242, dtype=np.uint8)
            note_text = str(record_note["text"]) if time.monotonic() < float(record_note["until_s"]) else ""
            draw_recording_toolbar(cv2, canvas, recorder_stats, button_rect, note_text)
            panel_y = margin + toolbar_h
            draw_opencv_panel(
                cv2,
                canvas,
                left_snap,
                left_display,
                margin,
                panel_y,
                panel_w,
                status_h,
                image_w,
                image_h,
                plot_h,
                sync_zero_ns,
                None,
                display_pair,
                recorder_stats,
                float(imu_window_s),
                view_fps,
            )
            draw_opencv_panel(
                cv2,
                canvas,
                right_snap,
                right_display,
                margin * 2 + panel_w,
                panel_y,
                panel_w,
                status_h,
                image_w,
                image_h,
                plot_h,
                sync_zero_ns,
                refiner_stats,
                display_pair,
                recorder_stats,
                float(imu_window_s),
                view_fps,
            )

            if depth_processor is not None:
                depth_y = panel_y + body_h + margin
                draw_depth_band(
                    cv2,
                    canvas,
                    depth_result,
                    depth_error,
                    display_pair,
                    margin,
                    depth_y,
                    canvas_w - margin * 2,
                    depth_tile_h,
                )

            cv2.imshow(window_name, canvas)
            frames += 1
            now_s = time.monotonic()
            if now_s - fps_started_s >= 1.0:
                view_fps = frames / max(1e-6, now_s - fps_started_s)
                frames = 0
                fps_started_s = now_s

            elapsed_s = time.monotonic() - frame_start_s
            wait_ms = max(1, int((target_dt - elapsed_s) * 1000.0))
            key = cv2.waitKey(wait_ms) & 0xFF
            if key in (27, ord("q")):
                break
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except Exception:
                pass
    finally:
        try:
            cv2.destroyWindow(window_name)
        except Exception:
            pass

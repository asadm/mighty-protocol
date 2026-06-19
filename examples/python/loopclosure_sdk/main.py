#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List


THIS_FILE = Path(__file__).resolve()
SDK_PY_DIR = THIS_FILE.parents[3] / "python"
if str(SDK_PY_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_PY_DIR))

from mighty_sdk import MightyClient, MightyWebDevice  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mighty Python SDK loopclosure example")
    parser.add_argument("--host", default="", help="Device/app base URL, e.g. http://127.0.0.1:18091")
    parser.add_argument("--seconds", type=float, default=120.0, help="Run duration")
    parser.add_argument("--out", default="loopclosure_python.svg", help="Output plot path")
    parser.add_argument("--calibration", default="", help="Calibration YAML file; defaults to device config calib")
    parser.add_argument("--library", default="", help="Native loopclosure library path override")
    parser.add_argument("--no-start", action="store_true", help="Do not send start_vio")
    return parser.parse_args()


def load_text(path: str) -> str:
    if not path:
        return ""
    return Path(path).read_text(encoding="utf-8")


def plot_loopclosure(events: List[Dict[str, object]], keyframes: List[Dict[str, object]], out_path: str) -> None:
    raw_xy = [
        ((p.get("raw_position_m") or p["position_m"])[0], (p.get("raw_position_m") or p["position_m"])[1])
        for p in keyframes
    ]
    opt_xy = [(p["position_m"][0], p["position_m"][1]) for p in keyframes]
    loop_edges = [
        (int(e["current_keyframe"]), int(e["matched_keyframe"]))
        for e in events
        if "current_keyframe" in e and "matched_keyframe" in e
    ]

    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        write_svg_fallback(raw_xy, opt_xy, loop_edges, out_path)
        return

    fig, axes = plt.subplots(1, 2, figsize=(13.2, 7.6), constrained_layout=True)
    fig.patch.set_facecolor("#f8f8f4")
    for ax, title, points, color, dashed in [
        (axes[0], "Unoptimized keyframes", raw_xy, "#666666", True),
        (axes[1], "Optimized keyframes", opt_xy, "#ff0055", False),
    ]:
        ax.set_facecolor("#ffffff")
        ax.set_title(title, loc="left", color="#0f172a")
        ax.grid(True, color="#e4e4df", linewidth=1.0)
        ax.set_aspect("equal", adjustable="box")
        if len(points) >= 2:
            xs, ys = zip(*points)
            ax.plot(xs, ys, color=color, linewidth=2.5, linestyle="--" if dashed else "-")
            ax.scatter(xs, ys, s=28, c="#ffcc00", edgecolors=color, linewidths=0.8, zorder=3)
        for current, candidate in loop_edges:
            if current < len(points) and candidate < len(points):
                ax.plot(
                    [points[current][0], points[candidate][0]],
                    [points[current][1], points[candidate][1]],
                    color="#0099ff",
                    linestyle="--",
                    linewidth=1.5,
                    alpha=0.85,
                )

    fig.suptitle(
        f"Mighty Loop Closure SDK | keyframes: {len(keyframes)} | loops: {len(loop_edges)}",
        x=0.02,
        ha="left",
        color="#0f172a",
        fontweight="bold",
    )
    fig.savefig(out_path, facecolor=fig.get_facecolor(), dpi=160)
    plt.close(fig)


def write_svg_fallback(raw_xy, opt_xy, loop_edges, out_path: str) -> None:
    all_points = list(raw_xy) + list(opt_xy)
    if not all_points:
        all_points = [(0.0, 0.0), (1.0, 1.0)]
    min_x = min(p[0] for p in all_points)
    max_x = max(p[0] for p in all_points)
    min_y = min(p[1] for p in all_points)
    max_y = max(p[1] for p in all_points)
    if abs(max_x - min_x) < 1e-9:
        min_x -= 1.0
        max_x += 1.0
    if abs(max_y - min_y) < 1e-9:
        min_y -= 1.0
        max_y += 1.0

    width, height = 1320.0, 760.0
    margin, header_h, gap = 48.0, 128.0, 42.0
    plot_w = (width - 2.0 * margin - gap) / 2.0
    plot_h = height - header_h - margin
    scale = min(plot_w / (max_x - min_x), plot_h / (max_y - min_y))
    x_offset = (plot_w - (max_x - min_x) * scale) * 0.5
    y_offset = (plot_h - (max_y - min_y) * scale) * 0.5

    def sx(point, x0):
        return x0 + x_offset + (point[0] - min_x) * scale

    def sy(point):
        return header_h + plot_h - y_offset - (point[1] - min_y) * scale

    def polyline(points, x0):
        return " ".join(f"{sx(p, x0):.2f},{sy(p):.2f}" for p in points)

    raw_x = margin
    opt_x = margin + plot_w + gap
    with open(out_path, "w", encoding="utf-8") as out:
        out.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width:g} {height:g}" width="{width:g}" height="{height:g}">')
        out.write(f'<rect width="{width:g}" height="{height:g}" fill="#f8f8f4"/>')
        out.write(f'<text x="48" y="40" font-family="system-ui" font-size="18" font-weight="700" fill="#0f172a">Mighty Python Loop Closure SDK</text>')
        out.write(f'<text x="48" y="66" font-family="system-ui" font-size="13" fill="#0f172a">keyframes: {len(opt_xy)} | loops: {len(loop_edges)} | install matplotlib for PNG/PDF output</text>')
        for x0, title in ((raw_x, "Unoptimized keyframes"), (opt_x, "Optimized keyframes")):
            out.write(f'<rect x="{x0:g}" y="{header_h:g}" width="{plot_w:g}" height="{plot_h:g}" fill="#fff" stroke="#d4d4ce"/>')
            for i in range(1, 10):
                gx = x0 + plot_w * i / 10.0
                gy = header_h + plot_h * i / 10.0
                out.write(f'<line x1="{gx:g}" y1="{header_h:g}" x2="{gx:g}" y2="{header_h + plot_h:g}" stroke="#e4e4df"/>')
                out.write(f'<line x1="{x0:g}" y1="{gy:g}" x2="{x0 + plot_w:g}" y2="{gy:g}" stroke="#e4e4df"/>')
            out.write(f'<text x="{x0:g}" y="{header_h - 14:g}" font-family="system-ui" font-size="14" font-weight="700" fill="#0f172a">{title}</text>')
        for points, x0 in ((raw_xy, raw_x), (opt_xy, opt_x)):
            for current, candidate in loop_edges:
                if current < len(points) and candidate < len(points):
                    a, b = points[current], points[candidate]
                    out.write(f'<line x1="{sx(a, x0):g}" y1="{sy(a):g}" x2="{sx(b, x0):g}" y2="{sy(b):g}" stroke="#0099ff" stroke-width="2.5" stroke-dasharray="8 5" opacity="0.85"/>')
        if len(raw_xy) >= 2:
            out.write(f'<polyline points="{polyline(raw_xy, raw_x)}" fill="none" stroke="#666666" stroke-width="4" opacity="0.65" stroke-dasharray="10 8"/>')
        if len(opt_xy) >= 2:
            out.write(f'<polyline points="{polyline(opt_xy, opt_x)}" fill="none" stroke="#ff0055" stroke-width="4" opacity="0.95"/>')
        for points, x0, stroke in ((raw_xy, raw_x, "#666666"), (opt_xy, opt_x, "#ff0055")):
            for p in points:
                out.write(f'<circle cx="{sx(p, x0):g}" cy="{sy(p):g}" r="3.5" fill="#ffcc00" stroke="{stroke}" stroke-width="1"/>')
        out.write("</svg>\n")


def main() -> None:
    args = parse_args()
    device = MightyWebDevice(base_url=args.host) if args.host else MightyWebDevice()
    loop_events: List[Dict[str, object]] = []
    keyframes: List[Dict[str, object]] = []
    stop_event = threading.Event()

    client = MightyClient(
        device,
        loopclosure=True,
        loopclosure_calibration_yaml=load_text(args.calibration),
        loopclosure_library=args.library,
        auto_reconnect=True,
    )

    def on_loopclosure(event: Dict[str, object]) -> None:
        loop_events.append(event)
        event_type = str(event.get("type") or "")
        if event_type == "loop_closure":
            print(
                "[loop]",
                event_type,
                "current=",
                event.get("current_keyframe"),
                "matched=",
                event.get("matched_keyframe"),
                flush=True,
            )
        if keyframes:
            plot_loopclosure(loop_events, keyframes, args.out)

    client.on_loopclosure(on_loopclosure)
    client.on_pose(lambda pose: keyframes.append(pose) if pose.get("is_keyframe") and pose.get("position_m") else None)
    client.on_error(lambda e: print(f"[error] {e.get('scope')}:{e.get('code')} {e.get('message')}", flush=True))

    client.connect()
    print(f"transport=http source={device.get_info().get('source', '')}", flush=True)

    if not args.calibration:
        for _ in range(40):
            cfg = client.config_get("calib", as_text=True)
            if cfg.get("ok") and cfg.get("found") and cfg.get("value"):
                ok = client.set_loopclosure_calibration_yaml(str(cfg["value"]))
                print(f"loop-closure calibration loaded={ok}", flush=True)
                break
            time.sleep(0.25)

    if not args.no_start:
        res = client.start_vio()
        print(f"start_vio ok={res.get('ok')} message={res.get('message', '')}", flush=True)

    deadline = time.monotonic() + max(1.0, float(args.seconds))
    try:
        while time.monotonic() < deadline and not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not args.no_start:
                client.stop_vio()
        except Exception:
            pass
        client.disconnect()

    if keyframes:
        plot_loopclosure(loop_events, keyframes, args.out)
    print(f"wrote {args.out}", flush=True)
    print(f"stats {client.stats()}", flush=True)


if __name__ == "__main__":
    main()

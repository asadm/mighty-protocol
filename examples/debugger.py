#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import mighty_protocol as mp  # noqa: E402

MAX_PAYLOAD_BYTES = 16 * 1024 * 1024


def parse_frames_with_debug(buf: bytes, debug: bool = True):
    frames = []
    offset = 0
    min_size = 4 + 4 + 4 + 4 + 4
    skipped = 0
    while len(buf) - offset >= min_size:
        if buf[offset : offset + 4] != mp.HEADER_MAGIC:
            offset += 1
            skipped += 1
            continue
        tcode = buf[offset + 4 : offset + 8]
        length = int.from_bytes(buf[offset + 8 : offset + 12], "big")
        if length > MAX_PAYLOAD_BYTES:
            if debug:
                print(f"warn: oversized payload len={length}")
            offset += 1
            skipped += 1
            continue
        pkt_size = min_size + length
        if len(buf) - offset < pkt_size:
            if debug:
                print(f"warn: incomplete frame len={length}")
            break
        payload = buf[offset + 12 : offset + 12 + length]
        recv_crc = int.from_bytes(buf[offset + 12 + length : offset + 16 + length], "big")
        footer = buf[offset + 16 + length : offset + pkt_size]
        offset += pkt_size
        if footer != mp.FOOTER_MAGIC:
            if debug:
                print("warn: corrupt frame footer")
            continue
        if length and mp._crc32(payload) != recv_crc:
            if debug:
                print("warn: corrupt frame crc")
            continue
        frames.append({"type": tcode.decode("ascii"), "payload": payload})
    if skipped and debug:
        print(f"warn: skipped {skipped} bytes (desync)")
    return frames, buf[offset:]


def _shorten(text: str, max_len: int) -> str:
    if max_len <= 0:
        return ""
    if len(text) <= max_len:
        return text
    if max_len <= 3:
        return text[:max_len]
    return text[: max_len - 3] + "..."


def decode_summary(frame, max_text_len: int, decode: bool) -> str:
    tcode = frame["type"]
    payload = frame["payload"]
    if not decode:
        return f"payload_len={len(payload)}"
    try:
        if tcode in ("JPG ", "RJPG"):
            info = mp.decode_jpg_payload(payload, tcode == "RJPG")
            channel = info.get("channel") or ("ref" if tcode == "RJPG" else "preview")
            return f"ts={info['timestamp_ns']} ch={channel} bytes={len(info['data'])}"
        if tcode in ("POSE", "UPOS"):
            info = mp.decode_pose_payload(payload)
            pos = info["position"]
            has_quat = "yes" if info["quat"] else "no"
            return f"type={info['pose_type']} pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) quat={has_quat}"
        if tcode == "LCON":
            segs = mp.decode_constraints_payload(payload)
            return f"segments={len(segs)}"
        if tcode == "VIZ ":
            data = mp.decode_viz_payload(payload)
            subtype = data.get("subtype")
            if subtype == 0:
                return f"subtype=features count={len(data.get('features', []))}"
            if subtype == 1:
                return f"subtype=detections count={len(data.get('detections', []))}"
            if subtype == 2:
                return f"subtype=matches count={len(data.get('matches', []))}"
            return f"subtype={subtype}"
        if tcode == "IMU ":
            samples = mp.decode_imu_payload(payload)
            if samples:
                return f"samples={len(samples)} ts=[{samples[0]['timestamp_ns']},{samples[-1]['timestamp_ns']}]"
            return "samples=0"
        if tcode == "STAT":
            text = mp.decode_status_payload(payload)
            return f"text={_shorten(text, max_text_len)}"
        if tcode == "RSET":
            return "reset"
        if tcode == "FEA3":
            feats = mp.decode_fea3_payload(payload)
            return f"features={len(feats)}"
        if tcode == "PCLD":
            data = mp.decode_pcld_payload(payload)
            ps = data.get("point_size")
            ps_text = f"{ps:.3f}" if isinstance(ps, float) else "none"
            return f"points={len(data.get('points', []))} point_size={ps_text}"
        if tcode == "CMD ":
            cmd = mp.decode_command_payload(payload)
            return f"req_id={cmd['req_id']} name={cmd['name']} data_len={len(cmd['data'])}"
        if tcode == "CRES":
            res = mp.decode_command_response_payload(payload)
            msg = _shorten(res["message"], max_text_len)
            return f"req_id={res['req_id']} status={res['status']} msg={msg} data_len={len(res['data'])}"
        return f"payload_len={len(payload)}"
    except Exception as exc:
        return f"decode_error={exc} payload_len={len(payload)}"


def _pick_device(cli_device: str) -> str:
    if cli_device:
        return cli_device
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Provide --device explicitly.")
        sys.exit(1)
    print("Available serial ports:")
    for idx, port in enumerate(ports, start=1):
        desc = port.description or "Unknown"
        hwid = port.hwid or "n/a"
        print(f"  {idx}) {port.device} - {desc} ({hwid})")
    while True:
        choice = input("Select port number: ").strip()
        if not choice:
            continue
        try:
            sel = int(choice)
            if 1 <= sel <= len(ports):
                return ports[sel - 1].device
        except ValueError:
            pass
        print("Invalid selection, try again.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Read mighty-protocol frames over serial and log summaries.")
    parser.add_argument("-d", "--device", default="", help="Serial device path (leave empty to select)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--timeout", type=float, default=0.1, help="Serial read timeout seconds")
    parser.add_argument("--no-decode", action="store_true", help="Do not decode payloads")
    parser.add_argument("--max-text", type=int, default=120, help="Max chars for status/command text")
    parser.add_argument("--debug", action="store_true", help="Log resync/corrupt frame warnings")
    args = parser.parse_args()

    device = _pick_device(args.device)
    try:
        ser = serial.Serial(device, baudrate=args.baud, timeout=args.timeout)
    except Exception as exc:
        print(f"Failed to open serial device {device}: {exc}")
        return 1

    print(f"Listening on {device} @ {args.baud} (ctrl-c to quit)")
    buf = b""

    try:
        while True:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            if args.debug:
                frames, buf = parse_frames_with_debug(buf, debug=True)
            else:
                frames, buf = mp.parse_frames(buf)
            for frame in frames:
                now = time.strftime("%H:%M:%S")
                summary = decode_summary(frame, args.max_text, not args.no_decode)
                print(f"[{now}] {frame['type']} {summary}")
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

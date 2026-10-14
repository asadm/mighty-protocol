"""C++/JS/Python pair wire compatibility, validation, fragmented dispatch."""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
import mighty_protocol as mp
from decoded_dispatcher import DecodedDispatcher

left = {"timestamp_ns": 123, "channel": "cam0", "data": bytes([255, 216, 1, 255, 217])}
right = {"timestamp_ns": 456, "channel": "cam1", "data": bytes([255, 216, 2, 3, 255, 217])}
payload = mp.build_stereo_jpg_payload(left, right)
assert mp.decode_stereo_jpg_payload(payload) == {"left": left, "right": right}
for bad in (payload[:-1], payload + b"\0", b"\xff" * 4 + payload[4:]):
    try:
        mp.decode_stereo_jpg_payload(bad)
    except ValueError:
        pass
    else:
        raise AssertionError("accepted malformed pair")
dispatcher = DecodedDispatcher()
pairs = []
dispatcher.on_stereo_jpg = lambda l, r: pairs.append((l, r))
packet = mp.make_packet(mp.TYPE["SJPG"], payload)
dispatcher.feed(packet[:-1]); assert not pairs
dispatcher.feed(packet[-1:]); assert pairs == [(left, right)]
with tempfile.TemporaryDirectory() as directory:
    binary = Path(directory) / "stereo-jpg"
    subprocess.run(["c++", "-std=c++17", str(ROOT / "tests/cpp_stereo_jpg_test.cpp"), "-o", str(binary)], check=True)
    assert subprocess.check_output([str(binary)], text=True).strip() == payload.hex()
assert subprocess.check_output(["node", str(ROOT / "tests/node_stereo_jpg.test.js")], text=True).strip() == payload.hex()
print("PASS SJPG C++/JS/Python wire parity, validation, fragmented transport and SDK dispatch")

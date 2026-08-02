import os
import sys

HERE = os.path.dirname(__file__)
sys.path.append(os.path.join(HERE, "..", "python"))

import mighty_protocol as mp  # noqa: E402
from decoded_dispatcher import DecodedDispatcher  # noqa: E402
from mighty_sdk import (  # noqa: E402
    MightyClient,
    RgbdSynchronizer,
    depth_at_meters,
    rectify_image_to_depth,
)


TIMESTAMP_NS = 123456789


def make_payload():
    return mp.build_depth_payload(
        timestamp_ns=TIMESTAMP_NS,
        width=2,
        height=2,
        source_width=2,
        source_height=2,
        depth_intrinsics=(1.0, 1.0, 0.0, 0.0),
        source_intrinsics=(1.0, 1.0, 0.0, 0.0),
        distortion=(0.0, 0.0, 0.0, 0.0),
        depth_mm=(0, 1250, 2500, 10000),
    )


payload = make_payload()
depth = mp.decode_depth_payload(payload)
assert list(depth["depth_mm"]) == [0, 1250, 2500, 10000]
assert abs(depth_at_meters(depth, 1, 0) - 1.25) < 1e-6
assert depth_at_meters(depth, 0, 0) is None

raw = {
    "kind": "raw",
    "timestamp_ns": TIMESTAMP_NS,
    "width": 2,
    "height": 2,
    "format": mp.RAW_FORMAT["GRAY8"],
    "channel": "preview",
    "data": bytes((10, 20, 30, 40)),
}
rectified = rectify_image_to_depth(raw, depth)
assert bytes(rectified["rgba"]) == bytes((
    10, 10, 10, 255, 20, 20, 20, 255,
    30, 30, 30, 255, 40, 40, 40, 255,
))

pairs = []
synchronizer = RgbdSynchronizer(pairs.append)
synchronizer.push_image({**raw, "channel_alias": "cam0"})
synchronizer.push_depth(depth)
assert len(pairs) == 1
assert pairs[0]["image"]["timestamp_ns"] == pairs[0]["depth"]["timestamp_ns"]

seen = []
dispatcher = DecodedDispatcher()
dispatcher.on_depth = seen.append
dispatcher.feed(mp.make_packet(mp.TYPE["DPT"], payload))
assert len(seen) == 1
assert seen[0]["frame_id"] == "cam0_rectified"


class DummyDevice:
    def connect(self, _on_bytes):
        return None

    def disconnect(self):
        return None


client_depths = []
client_pairs = []
client = MightyClient(DummyDevice(), auto_reconnect=False)
client.on_depth(client_depths.append)
client.on_rgbd(client_pairs.append)
client._handle_frame({
    "type": "RAW ",
    "payload": mp.build_raw_payload(
        TIMESTAMP_NS, 2, 2, mp.RAW_FORMAT["GRAY8"], "preview", raw["data"]
    ),
})
client._handle_frame({"type": "DPT ", "payload": payload})
assert len(client_depths) == 1
assert len(client_pairs) == 1

print("python depth tests passed")

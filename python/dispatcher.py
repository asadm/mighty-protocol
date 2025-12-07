from typing import Callable
import mighty_protocol as mp

class FrameDispatcher:
    """
    Feed raw bytes; calls on_frame(frame_dict) for each complete frame.
    frame_dict has keys: type (str), payload (bytes).
    """
    def __init__(self, on_frame: Callable[[dict], None]):
        self.on_frame = on_frame
        self._buffer = b""

    def feed(self, data: bytes):
        self._buffer += data
        frames, rest = mp.parse_frames(self._buffer)
        self._buffer = rest
        for f in frames:
            self.on_frame(f)

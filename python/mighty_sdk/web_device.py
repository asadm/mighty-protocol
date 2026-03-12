import socket
import threading
import urllib.error
import urllib.request
from typing import Callable, Dict, Optional

from .utils import to_bytes


class MightyWebDevice:
    """
    HTTP transport for Mighty protocol.

    - Stream ingress: GET /stream, pass raw bytes to callback.
    - Command egress: POST /command with CMD payload body, returns CRES payload body.
    """

    def __init__(
        self,
        base_url: str,
        stream_path: str = "/stream",
        command_path: str = "/command",
        headers: Optional[Dict[str, str]] = None,
        connect_timeout_s: float = 5.0,
        read_timeout_s: float = 1.0,
        read_chunk_size: int = 64 * 1024,
    ):
        if not base_url:
            raise ValueError("base_url is required")
        self.base_url = base_url[:-1] if base_url.endswith("/") else base_url
        self.stream_path = stream_path or "/stream"
        self.command_path = command_path or "/command"
        self.headers = dict(headers or {})
        self.connect_timeout_s = float(connect_timeout_s)
        self.read_timeout_s = float(read_timeout_s)
        self.read_chunk_size = max(1, int(read_chunk_size))

        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._connected = False
        self._active_stream = None

    def get_info(self) -> Dict[str, str]:
        return {"transport": "http", "source": self.base_url}

    def _url(self, path: str) -> str:
        if path.startswith("http://") or path.startswith("https://"):
            return path
        if path.startswith("/"):
            return f"{self.base_url}{path}"
        return f"{self.base_url}/{path}"

    def connect(self, on_bytes: Callable[[bytes], None]) -> None:
        if not callable(on_bytes):
            raise ValueError("connect requires callable on_bytes")

        with self._state_lock:
            if self._connected:
                raise RuntimeError("stream already connected")
            self._connected = True
            self._stop_event.clear()

        timeout = max(self.connect_timeout_s, self.read_timeout_s)
        req = urllib.request.Request(
            self._url(self.stream_path),
            method="GET",
            headers={"Accept": "application/octet-stream", **self.headers},
        )

        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                with self._state_lock:
                    self._active_stream = resp
                while not self._stop_event.is_set():
                    try:
                        chunk = resp.read(self.read_chunk_size)
                    except (socket.timeout, TimeoutError):
                        if self._stop_event.is_set():
                            break
                        continue

                    if not chunk:
                        break
                    on_bytes(to_bytes(chunk))
        except Exception:
            if not self._stop_event.is_set():
                raise
        finally:
            with self._state_lock:
                self._active_stream = None
                self._connected = False

    def disconnect(self) -> None:
        self._stop_event.set()
        stream = None
        with self._state_lock:
            stream = self._active_stream
        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass

    def send_command_payload(self, cmd_payload: bytes) -> bytes:
        payload = to_bytes(cmd_payload)
        req = urllib.request.Request(
            self._url(self.command_path),
            method="POST",
            data=payload,
            headers={
                "Content-Type": "application/octet-stream",
                "Accept": "application/octet-stream",
                **self.headers,
            },
        )
        timeout = max(self.connect_timeout_s, self.read_timeout_s)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return to_bytes(resp.read())
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"command request failed ({exc.code})") from exc

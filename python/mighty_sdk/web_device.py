import socket
import threading
import urllib.error
import urllib.request
from typing import Callable, Dict, List, Optional

from .utils import to_bytes

DEFAULT_BASE_URLS = [
    "http://192.168.7.1",
    "http://localhost:8080",
    "http://localhost:8084",
]


def _normalize_base_url(base_url: str) -> str:
    if not isinstance(base_url, str):
        return ""
    value = base_url.strip()
    if not value:
        return ""
    if value.endswith("/"):
        value = value[:-1]
    if not (value.startswith("http://") or value.startswith("https://")):
        return ""
    return value


def _append_unique(out: List[str], value: str) -> None:
    v = _normalize_base_url(value)
    if not v:
        return
    if v in out:
        return
    out.append(v)


class MightyWebDevice:
    """
    HTTP transport for Mighty protocol.

    - Stream ingress: GET /stream, pass raw bytes to callback.
    - Command egress: POST /command with CMD payload body, returns CRES payload body.
    """

    def __init__(
        self,
        base_url: Optional[str] = None,
        base_urls: Optional[List[str]] = None,
        stream_path: str = "/stream",
        command_path: str = "/command",
        headers: Optional[Dict[str, str]] = None,
        connect_timeout_s: float = 5.0,
        read_timeout_s: float = 1.0,
        read_chunk_size: int = 64 * 1024,
    ):
        resolved: List[str] = []
        if isinstance(base_urls, list) and len(base_urls) > 0:
            for b in base_urls:
                _append_unique(resolved, b)
        elif isinstance(base_url, str) and base_url.strip():
            _append_unique(resolved, base_url)
        else:
            for b in DEFAULT_BASE_URLS:
                _append_unique(resolved, b)

        if not resolved:
            raise ValueError("MightyWebDevice requires at least one valid base URL")

        self.base_urls = list(resolved)
        self.base_url = self.base_urls[0]
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
        self._active_base_url = self.base_url

    def get_info(self) -> Dict[str, str]:
        return {"transport": "http", "source": self._active_base_url or self.base_url}

    def _url(self, base_url: str, path: str) -> str:
        if path.startswith("http://") or path.startswith("https://"):
            return path
        base = _normalize_base_url(base_url) or self.base_url
        if path.startswith("/"):
            return f"{base}{path}"
        return f"{base}/{path}"

    def _ordered_base_urls(self) -> List[str]:
        ordered: List[str] = []
        _append_unique(ordered, self._active_base_url or "")
        for b in self.base_urls:
            _append_unique(ordered, b)
        return ordered

    def connect(self, on_bytes: Callable[[bytes], None]) -> None:
        if not callable(on_bytes):
            raise ValueError("connect requires callable on_bytes")

        with self._state_lock:
            if self._connected:
                raise RuntimeError("stream already connected")
            self._connected = True
            self._stop_event.clear()

        timeout = max(self.connect_timeout_s, self.read_timeout_s)
        last_err: Optional[Exception] = None

        try:
            for base in self._ordered_base_urls():
                if self._stop_event.is_set():
                    return
                req = urllib.request.Request(
                    self._url(base, self.stream_path),
                    method="GET",
                    headers={"Accept": "application/octet-stream", **self.headers},
                )

                try:
                    with urllib.request.urlopen(req, timeout=timeout) as resp:
                        with self._state_lock:
                            self._active_stream = resp
                            self._active_base_url = base
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
                        return
                except Exception as exc:
                    if self._stop_event.is_set():
                        return
                    last_err = exc
                    continue

            if last_err is not None:
                raise RuntimeError(f"stream request failed on all hosts: {last_err}") from last_err
            raise RuntimeError("stream request failed on all hosts")
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
        timeout = max(self.connect_timeout_s, self.read_timeout_s)
        last_err: Optional[Exception] = None
        for base in self._ordered_base_urls():
            req = urllib.request.Request(
                self._url(base, self.command_path),
                method="POST",
                data=payload,
                headers={
                    "Content-Type": "application/octet-stream",
                    "Accept": "application/octet-stream",
                    **self.headers,
                },
            )
            try:
                with urllib.request.urlopen(req, timeout=timeout) as resp:
                    with self._state_lock:
                        self._active_base_url = base
                    return to_bytes(resp.read())
            except urllib.error.HTTPError as exc:
                last_err = RuntimeError(f"command request failed ({exc.code})")
            except Exception as exc:
                last_err = exc

        if last_err is not None:
            raise RuntimeError(f"command request failed on all hosts: {last_err}") from last_err
        raise RuntimeError("command request failed on all hosts")

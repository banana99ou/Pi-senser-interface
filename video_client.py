#!/usr/bin/env python3
"""
VideoClient: connect to Pi H.264 over TCP (MPEG-TS), decode frames, yield BGR ndarrays.
No saving, auto-reconnect.
"""
from __future__ import annotations
import time
from typing import Generator, Tuple, Optional
import av  # pip install av
import numpy as np

class VideoClient:
    def __init__(self, host: str = "10.55.0.1", port: int = 6000,
                 reconnect_delay: float = 1.0, open_timeout_s: int = 5):
        self.url = f"tcp://{host}:{port}"
        self.reconnect_delay = reconnect_delay
        self._open_opts = {"rw_timeout": str(open_timeout_s * 1_000_000)}  # µs
        self._container: Optional[av.container.input.InputContainer] = None
        self._vstream: Optional[av.video.stream.VideoStream] = None
        self._closed = False

    def _open(self) -> None:
        self._container = av.open(self.url, format="mpegts", options=self._open_opts)
        self._vstream = next(s for s in self._container.streams if s.type == "video")

    def frames(self) -> Generator[Tuple[np.ndarray, float], None, None]:
        while not self._closed:
            try:
                if self._container is None:
                    self._open()
                for packet in self._container.demux(self._vstream):
                    if self._closed:
                        break
                    for frame in packet.decode():
                        img_bgr = frame.to_ndarray(format="bgr24")
                        tb = frame.time_base or self._vstream.time_base
                        pts_s = float(frame.pts * tb) if frame.pts is not None else float("nan")
                        yield img_bgr, pts_s
            except av.AVError:
                self._cleanup(); time.sleep(self.reconnect_delay)
            except Exception:
                self._cleanup(); time.sleep(self.reconnect_delay)

    def close(self) -> None:
        self._closed = True
        self._cleanup()

    def _cleanup(self) -> None:
        try:
            if self._container:
                self._container.close()
        except Exception:
            pass
        self._container = None
        self._vstream = None

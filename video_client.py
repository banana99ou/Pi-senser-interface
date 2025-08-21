#!/usr/bin/env python3
"""
VideoClient: connect to a Pi H.264 stream over TCP (MPEG-TS),
decode frames, and yield them as BGR numpy arrays.

- No disk I/O or saving.
- Reconnects automatically if the socket drops.
- Color is correct (BGR for OpenCV).
"""
from __future__ import annotations
import time
from typing import Generator, Tuple, Optional

import av  # PyAV
import numpy as np
import cv2

class VideoClient:
    def __init__(self, host: str = "10.55.0.1", port: int = 6000,
                 reconnect_delay: float = 1.0, open_timeout_s: int = 5):
        self.url = f"tcp://{host}:{port}"
        self.reconnect_delay = reconnect_delay
        # rw_timeout in microseconds for FFmpeg/Libav
        self._open_opts = {"rw_timeout": str(open_timeout_s * 1_000_000)}
        self._container: Optional[av.container.input.InputContainer] = None
        self._vstream: Optional[av.video.stream.VideoStream] = None
        self._closed = False

    def _open(self) -> None:
        # Open MPEG-TS over TCP
        self._container = av.open(self.url, format="mpegts", options=self._open_opts)
        self._vstream = next(s for s in self._container.streams if s.type == "video")

    def frames(self) -> Generator[Tuple[np.ndarray, float], None, None]:
        """
        Yields (frame_bgr, pts_seconds). No saving, no side effects.
        Reconnects automatically on errors.
        """
        while not self._closed:
            try:
                if self._container is None:
                    self._open()
                assert self._container and self._vstream
                for packet in self._container.demux(self._vstream):
                    if self._closed:
                        break
                    for frame in packet.decode():
                        # Convert to OpenCV-friendly BGR
                        img_bgr = frame.to_ndarray(format="bgr24")
                        tb = frame.time_base or self._vstream.time_base
                        pts_s = float(frame.pts * tb) if frame.pts is not None else np.nan
                        yield img_bgr, pts_s
            except av.AVError as e:
                # Any network/decoder hiccup → brief backoff and reconnect
                self._cleanup()
                time.sleep(self.reconnect_delay)
            except Exception:
                self._cleanup()
                time.sleep(self.reconnect_delay)

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

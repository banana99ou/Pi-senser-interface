#!/usr/bin/env python3
"""
IMUClient: connect to Pi TCP :6001, receive struct <Q6f> samples.
No saving, auto-reconnect.
"""
import socket, struct, time
from typing import Generator, Tuple, Optional

FMT = struct.Struct("<Q6f")   # t_us, ax, ay, az, gx, gy, gz

class IMUClient:
    def __init__(self, host="10.55.0.1", port=6001, reconnect_delay=1.0):
        self.host, self.port = host, port
        self.delay = reconnect_delay
        self.sock: Optional[socket.socket] = None
        self.closed = False

    def _connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.connect((self.host, self.port))
        self.sock = s

    def samples(self) -> Generator[Tuple[int, float, float, float, float, float, float], None, None]:
        buf = b""
        while not self.closed:
            try:
                if self.sock is None:
                    self._connect()
                    buf = b""
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("server closed")
                buf += chunk
                while len(buf) >= FMT.size:
                    t_us, ax, ay, az, gx, gy, gz = FMT.unpack_from(buf, 0)
                    buf = buf[FMT.size:]
                    yield (t_us, ax, ay, az, gx, gy, gz)
            except Exception:
                if self.sock:
                    try: self.sock.close()
                    except Exception: pass
                self.sock = None
                time.sleep(self.delay)

    def close(self):
        self.closed = True
        if self.sock:
            try: self.sock.close()
            except Exception: pass
            self.sock = None

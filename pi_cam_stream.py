#!/usr/bin/env python3
import socket, struct, threading, time, sys
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder

PI_LISTEN_IP = "0.0.0.0"
VID_PORT     = 6000
IMU_PORT     = 6001

W, H, FPS    = 1280, 720, 60
BITRATE      = 10_000_000      # 8–12 Mbps is a good range
GOP          = 120             # keyframe every 2 s @60fps

# ── Wire formats ─────────────────────────────────────────────────────────────
# Video frame header (little-endian):
#   type(1)='V', width(u16), height(u16), t_us(u64), nbytes(u32), key(u8)
VHDR = struct.Struct("<cHHQIB")  # total 1+2+2+8+4+1 = 18 bytes
IMU  = struct.Struct("<Q6f")     # t_us, ax,ay,az,gx,gy,gz  (32 bytes)

def monotonic_us() -> int:
    return time.monotonic_ns() // 1_000

def serve_imu(conn):
    """
    100 Hz placeholder loop.
    Replace the body with MPU6050 reads and keep the same packing.
    Units: ax,ay,az [m/s^2], gx,gy,gz [rad/s], axes mapped to your convention.
    """
    try:
        period = 1.0 / 100.0
        next_t = time.perf_counter()
        while True:
            next_t += period
            t_us = monotonic_us()
            # TODO: read real sensor here; keep units as specified
            ax=ay=az=gx=gy=gz=0.0
            conn.sendall(IMU.pack(t_us, ax, ay, az, gx, gy, gz))
            # precise pacing
            dt = next_t - time.perf_counter()
            if dt > 0: time.sleep(dt)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        try: conn.close()
        except: pass

class TcpVideoOutput:
    """
    Picamera2 encoded-output sink. Called once per encoded frame.
    Sends a small header + Annex-B H.264 bytes on a persistent TCP socket.
    """
    def __init__(self, conn, width, height):
        self.conn = conn
        self.width = width
        self.height = height

    # Picamera2 will call this with (buf, keyframe, *args)
    def outputframe(self, buf, keyframe=True, *args, **kwargs):
        # buf is a Python bytes-like object for this encoded frame
        t_us = monotonic_us()  # use monotonic capture/send timestamp in µs
        payload = memoryview(buf)
        hdr = VHDR.pack(b"V", self.width, self.height, t_us, len(payload), 1 if keyframe else 0)
        try:
            self.conn.sendall(hdr)
            self.conn.sendall(payload)
        except (BrokenPipeError, ConnectionResetError, OSError):
            # stop streaming if the client vanishes
            raise

def accept_one(port, name):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((PI_LISTEN_IP, port))
    srv.listen(1)
    print(f"[Pi] Waiting for {name} on :{port} …")
    conn, addr = srv.accept()
    srv.close()
    print(f"[Pi] {name} connected from {addr}")
    # nodelay helps reduce small latencies on the video path
    try: conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except: pass
    return conn

def main():
    # Accept both sockets before starting camera (order does not matter)
    vid_conn = accept_one(VID_PORT, "video client")
    imu_conn = accept_one(IMU_PORT, "IMU client")

    # Start IMU sender thread
    imu_th = threading.Thread(target=serve_imu, args=(imu_conn,), daemon=True)
    imu_th.start()

    # Camera + hardware H.264 encoder
    cam = Picamera2()
    cfg = cam.create_video_configuration(main={"size": (W, H), "format": "RGB888"}, controls={"FrameRate": FPS})
    cam.configure(cfg)

    enc = H264Encoder(bitrate=BITRATE)
    # Inline SPS/PPS at every keyframe and set GOP (intra period)
    try:
        enc.encoder._intra_period = GOP                 # picamera2 exposes this on encoder.impl
    except Exception:
        pass
    try:
        enc.repeat = True                                # repeat sps/pps (inline_headers)
    except Exception:
        pass

    out = TcpVideoOutput(vid_conn, W, H)

    try:
        cam.start_recording(enc, out)
        print(f"[Pi] Streaming H.264 {W}x{H}@{FPS}  bitrate≈{BITRATE} …  (Ctrl+C to stop)")
        while True:
            time.sleep(1)
    except (BrokenPipeError, ConnectionResetError):
        print("[Pi] client disconnected, stopping.")
    except KeyboardInterrupt:
        print("\n[Pi] Ctrl+C, stopping.")
    finally:
        try: cam.stop_recording()
        except: pass
        try: vid_conn.close()
        except: pass

if __name__ == "__main__":
    main()

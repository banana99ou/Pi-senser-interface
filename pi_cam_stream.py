#!/usr/bin/env python3
"""
Pi Camera + MPU6050 IMU TCP streamer with control plane.

Ports:
  - Video (TCP)   : 6000  → H.264 in MPEG-TS, 1280x720@60, ~10 Mbps
  - IMU (TCP)     : 6001  → struct '<Q6f' @100 Hz (t_us, ax, ay, az, gx, gy, gz)
  - Control (TCP) : 6002  → ASCII commands: STATUS | RESET_VID | RESET_IMU | RESTART_ALL

Notes:
  - Accepts only CONTROL from ALLOW_IPS.
  - RESET_VID tears down camera/encoder and the current client socket immediately.
  - RESET_IMU drops the current IMU client (sensor keeps running).
  - RESTART_ALL exits(0); use systemd to respawn (recommended).

Requires:
  sudo apt install python3-picamera2 libcamera-apps python3-smbus2
"""

import os, sys, time, socket, threading, struct, select, signal
from dataclasses import dataclass, field
from typing import Optional, Tuple

from picamera2 import Picamera2
from libcamera import Transform
from picamera2.encoders import H264Encoder
from picamera2.outputs import PyavOutput   # mpegts muxer via libavformat
from smbus2 import SMBus

# ---------------- Constants ----------------
VID_PORT   = 6000
IMU_PORT   = 6001
CTRL_PORT  = 6002

WIDTH, HEIGHT = 1280, 720
FPS           = 60
BITRATE       = 10_000_000   # ~10 Mbps
GOP           = 120          # keyframe every ~2s @ 60 fps
ROTATE_DEG    = 180          # your camera is upside-down

I2C_BUS = 1
MPU_ADDR = 0x68
IMU_HZ   = 100
IMU_STRUCT = struct.Struct("<Q6f")  # t_us, ax, ay, az, gx, gy, gz

# Only allow control from your PC:
ALLOW_IPS = {"10.55.0.160"}

# ---------------- Shared State ----------------
@dataclass
class SharedState:
    stop_evt: threading.Event = field(default_factory=threading.Event)

    # Video state
    vid_state: str = "idle"       # "idle" | "streaming"
    vid_epoch: int = 0            # increments each new recording start
    vid_reset_evt: threading.Event = field(default_factory=threading.Event)
    vid_conn_lock: threading.Lock = field(default_factory=threading.Lock)
    vid_conn: Optional[socket.socket] = None

    # IMU state
    imu_state: str = "listening"  # "listening" | "streaming"
    imu_conn_lock: threading.Lock = field(default_factory=threading.Lock)
    imu_conn: Optional[socket.socket] = None

STATE = SharedState()

# ---------------- Utilities ----------------
def _make_transform():
    d = ROTATE_DEG % 360
    if d == 0:
        return Transform()
    if d == 180:
        return Transform(hflip=True, vflip=True)
    # If you ever need 90/270, you can use transpose + flip, but
    # that changes W/H. For now we keep 1280x720 and do 180°.
    return Transform()

def _safe_close(sock: Optional[socket.socket]):
    if not sock: return
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass

def _now_us() -> int:
    return int(time.time_ns() // 1000)

# ---------------- Video Server ----------------
def video_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", VID_PORT))
    srv.listen(8)
    print(f"[VID] Listening on :{VID_PORT} (H.264 {WIDTH}x{HEIGHT}@{FPS}, {BITRATE//1_000_000} Mbps)")

    while not STATE.stop_evt.is_set():
        try:
            srv.settimeout(1.0)
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue

            print(f"[VID] Client {addr} connected")
            # record conn so control plane can reset it
            with STATE.vid_conn_lock:
                STATE.vid_conn = conn

            try:
                # fast dead-peer detection params (helps, but we rely on control plane)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                for name, val in (("TCP_KEEPIDLE", 60), ("TCP_KEEPINTVL", 10), ("TCP_KEEPCNT", 3)):
                    if hasattr(socket, name):
                        conn.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), val)

                # fresh camera per client
                print("[VID] CAM: create")
                cam = Picamera2()
                xform = _make_transform()
                print("[VID] CAM: configure")
                cfg = cam.create_video_configuration(
                    main={"size": (WIDTH, HEIGHT), "format": "YUV420"},
                    controls={"FrameRate": FPS},
                    transform=xform,
                )
                cam.configure(cfg)

                enc = H264Encoder(bitrate=BITRATE)
                enc.repeat  = True   # SPS/PPS inline at each IDR
                enc.iperiod = GOP

                out = PyavOutput(f"pipe:{conn.fileno()}", format="mpegts")
                print("[VID] CAM: start_recording")
                cam.start_recording(enc, out, name="main")

                STATE.vid_state = "streaming"
                STATE.vid_epoch += 1
                this_epoch = STATE.vid_epoch

                # loop until stop/reset/peer error
                # We keep a small poll to break fast on peer close, but main kill switch is RESET_VID.
                p = select.poll()
                p.register(conn, select.POLLHUP | select.POLLERR | select.POLLNVAL)
                while not STATE.stop_evt.is_set():
                    if STATE.vid_reset_evt.is_set():
                        raise ConnectionAbortedError("reset requested")
                    events = p.poll(250)
                    if events:
                        raise ConnectionAbortedError(f"peer closed (poll events={events})")

                print("[VID] stop requested")
            except Exception as e:
                print(f"[VID] Stream ended: {e}")
            finally:
                try:
                    cam.stop_recording()
                except Exception:
                    pass
                try:
                    cam.close()
                except Exception:
                    pass
                with STATE.vid_conn_lock:
                    _safe_close(STATE.vid_conn)
                    STATE.vid_conn = None
                STATE.vid_state = "idle"
                STATE.vid_reset_evt.clear()
                print("[VID] Client disconnected; returning to accept()")
        except Exception as e:
            print(f"[VID] server loop error: {e}")
            time.sleep(0.3)

    srv.close()
    print("[VID] Server stopped")

# ---------------- IMU (MPU6050) ----------------
def _mpu_init(bus: SMBus):
    # Wake up device, +/-2g, +/-250 dps, sample rate divider to match 100Hz, etc.
    # Registers
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    ACCEL_CONFIG = 0x1C

    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x01)   # clock X gyro, wake
    bus.write_byte_data(MPU_ADDR, SMPLRT_DIV, 9)      # 1kHz/(9+1)=100Hz
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x03)       # DLPF ~44Hz
    bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x00)  # ±250 dps
    bus.write_byte_data(MPU_ADDR, ACCEL_CONFIG, 0x00) # ±2g

def _mpu_read(bus: SMBus) -> Tuple[float, float, float, float, float, float]:
    # Read accel/gyro raw and convert to SI units
    # register blocks
    ACCEL_XOUT_H = 0x3B
    data = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)
    ax = struct.unpack_from(">h", bytes(data), 0)[0]
    ay = struct.unpack_from(">h", bytes(data), 2)[0]
    az = struct.unpack_from(">h", bytes(data), 4)[0]
    gx = struct.unpack_from(">h", bytes(data), 8)[0]
    gy = struct.unpack_from(">h", bytes(data),10)[0]
    gz = struct.unpack_from(">h", bytes(data),12)[0]

    # scale: accel 16384 LSB/g; gyro 131 LSB/(deg/s)
    ax = (ax / 16384.0) * 9.81
    ay = (ay / 16384.0) * 9.81
    az = (az / 16384.0) * 9.81
    gx = (gx / 131.0)   * 3.141592653589793 / 180.0
    gy = (gy / 131.0)   * 3.141592653589793 / 180.0
    gz = (gz / 131.0)   * 3.141592653589793 / 180.0
    return ax, ay, az, gx, gy, gz

def imu_server():
    print("[IMU] Initializing I2C/MPU6050…")
    bus = SMBus(I2C_BUS)
    _mpu_init(bus)
    print("[IMU] Done.")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", IMU_PORT))
    srv.listen(4)
    print(f"[IMU] Listening on :{IMU_PORT} (100 Hz, struct <Q6f>)")

    srv.settimeout(1.0)
    conn = None
    next_t = time.perf_counter()

    try:
        while not STATE.stop_evt.is_set():
            if conn is None:
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                print(f"[IMU] Client {addr} connected")
                with STATE.imu_conn_lock:
                    STATE.imu_conn = conn
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                STATE.imu_state = "streaming"

            # 100 Hz loop
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            next_t = now + 1.0 / IMU_HZ

            try:
                t_us = _now_us()
                az, ay, ax, gz, gy, gx = _mpu_read(bus)
                ax = -1 * ax
                gz = -1 * gz
                payload = IMU_STRUCT.pack(t_us, ax, ay, az, gx, gy, gz)
                conn.sendall(payload)
            except (BrokenPipeError, ConnectionError, OSError) as e:
                print(f"[IMU] Stream ended: {e}")
                with STATE.imu_conn_lock:
                    _safe_close(STATE.imu_conn)
                    STATE.imu_conn = None
                conn = None
                STATE.imu_state = "listening"
            except Exception as e:
                print(f"[IMU] error: {e}")
                time.sleep(0.01)
    finally:
        with STATE.imu_conn_lock:
            _safe_close(STATE.imu_conn)
            STATE.imu_conn = None
        srv.close()
        bus.close()
        print("[IMU] Server stopped")

# ---------------- Control Server ----------------
def _ctrl_reply(conn: socket.socket, text: str):
    try:
        conn.sendall((text + "\n").encode())
    except Exception:
        pass

def control_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", CTRL_PORT))
    srv.listen(4)
    print(f"[CTL] Listening on :{CTRL_PORT}")

    while not STATE.stop_evt.is_set():
        try:
            srv.settimeout(1.0)
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue

            peer_ip = addr[0]
            if ALLOW_IPS and (peer_ip not in ALLOW_IPS):
                print(f"[CTL] Reject {addr} not in allowlist")
                _ctrl_reply(conn, "ERR FORBIDDEN")
                _safe_close(conn)
                continue

            conn.settimeout(5.0)
            with conn:
                data = b""
                while True:
                    try:
                        chunk = conn.recv(1024)
                        if not chunk:
                            break
                        data += chunk
                        if b"\n" not in data:
                            continue
                        line, data = data.split(b"\n", 1)
                        cmd = line.decode(errors="ignore").strip().upper()

                        if cmd == "STATUS":
                            _ctrl_reply(conn, f"OK STATUS vid={STATE.vid_state} epoch={STATE.vid_epoch} imu={STATE.imu_state}")

                        elif cmd == "RESET_VID":
                            # set event AND close the socket (if any)
                            STATE.vid_reset_evt.set()
                            with STATE.vid_conn_lock:
                                _safe_close(STATE.vid_conn)
                                STATE.vid_conn = None
                            _ctrl_reply(conn, f"OK RESET_VID {STATE.vid_epoch}")

                        elif cmd == "RESET_IMU":
                            with STATE.imu_conn_lock:
                                _safe_close(STATE.imu_conn)
                                STATE.imu_conn = None
                            STATE.imu_state = "listening"
                            _ctrl_reply(conn, "OK RESET_IMU")

                        elif cmd == "RESTART_ALL":
                            _ctrl_reply(conn, "OK RESTART_ALL")
                            conn.close()
                            # Let systemd restart us (or if running manually, just re-run)
                            os._exit(0)

                        else:
                            _ctrl_reply(conn, "ERR UNKNOWN")
                    except socket.timeout:
                        break
                    except Exception as e:
                        _ctrl_reply(conn, f"ERR {e}")
                        break
        except Exception as e:
            print(f"[CTL] loop error: {e}")
            time.sleep(0.3)
    srv.close()
    print("[CTL] Server stopped")

# ---------------- Main ----------------
def main():
    # SIGTERM → stop
    def _sigterm(*_):
        STATE.stop_evt.set()
    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGINT, _sigterm)

    t_vid = threading.Thread(target=video_server,   daemon=True)
    t_imu = threading.Thread(target=imu_server,     daemon=True)
    t_ctl = threading.Thread(target=control_server, daemon=True)
    t_vid.start(); t_imu.start(); t_ctl.start()

    try:
        while not STATE.stop_evt.is_set():
            time.sleep(0.5)
    finally:
        STATE.stop_evt.set()
        t_vid.join(timeout=2.0)
        t_imu.join(timeout=2.0)
        t_ctl.join(timeout=2.0)
        print("[MAIN] Exiting.")

if __name__ == "__main__":
    main()

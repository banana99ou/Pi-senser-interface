#!/usr/bin/env python3
"""
Pi 4 server:
  - TCP :6000 → H.264 1280x720 @ 60 fps inside MPEG-TS (Picamera2 hardware encoder)
  - TCP :6001 → IMU (MPU6050) samples at 100 Hz, struct <Q6f> (t_us, ax, ay, az, gx, gy, gz)

Timestamps:
  - IMU: t_us = time.monotonic_ns() // 1000 (microseconds, monotonic)
  - Video: carried in the TS/encoder PTS (the PC client uses that only for display)

Requires:
  sudo apt install python3-picamera2 python3-av python3-smbus i2c-tools
  (Enable I2C in raspi-config)
"""
import socket, threading, time, math, signal, struct
from smbus import SMBus
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import PyavOutput
try:
    from libcamera import Transform          # preferred
except ImportError:
    from picamera2 import Transform          # fallback for older builds

# ---- Video config ----
WIDTH, HEIGHT, FPS = 1280, 720, 60
BITRATE = 10_000_000     # 10 Mbps
GOP     = 120            # keyframe every ~2 s at 60 fps
VID_PORT = 6000
ROTATE_DEG = 180

# ---- IMU config ----
I2C_BUS   = 1
MPU_ADDR  = 0x68         # 0x69 if AD0 high
IMU_PORT  = 6001
IMU_HZ    = 100.0
APPLY_AXIS_MAP = True    # match your RS mapping: ax=-z, ay=+x, az=+y; gx=+z, gy=-x, gz=-y

# MPU6050 regs
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
WHO_AM_I     = 0x75

ACC_LSB_PER_G    = 16384.0
GYRO_LSB_PER_DPS = 16.4
G_TO_MS2 = 9.80665
DEG2RAD  = math.pi / 180.0

def _twos(h, l):
    v = (h << 8) | l
    return v - 65536 if v & 0x8000 else v

def mpu_init(bus: SMBus):
    # wake
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.05)
    # DLPF = 42 Hz (3)
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x03)
    # sample = 1kHz/(1+div) => div=9 → 100 Hz
    bus.write_byte_data(MPU_ADDR, SMPLRT_DIV, 9)
    # gyro ±2000 dps
    bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x18)
    # accel ±2g
    bus.write_byte_data(MPU_ADDR, ACCEL_CONFIG, 0x00)
    time.sleep(0.05)

def mpu_read(bus: SMBus):
    raw = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)
    ax = _twos(raw[0], raw[1]);  ay = _twos(raw[2], raw[3]);  az = _twos(raw[4], raw[5])
    gx = _twos(raw[8], raw[9]);  gy = _twos(raw[10], raw[11]); gz = _twos(raw[12], raw[13])
    ax = (ax/ACC_LSB_PER_G) * G_TO_MS2
    ay = (ay/ACC_LSB_PER_G) * G_TO_MS2
    az = (az/ACC_LSB_PER_G) * G_TO_MS2
    gx = (gx/GYRO_LSB_PER_DPS) * DEG2RAD
    gy = (gy/GYRO_LSB_PER_DPS) * DEG2RAD
    gz = (gz/GYRO_LSB_PER_DPS) * DEG2RAD
    return ax, ay, az, gx, gy, gz

def mpu_calibrate(bus: SMBus, seconds=1.5):
    print(f"[IMU] Calibrating gyro {seconds}s… keep still")
    t_end = time.monotonic() + seconds
    n=0; sx=sy=sz=0.0
    while time.monotonic() < t_end:
        _,_,_,gx,gy,gz = mpu_read(bus)
        sx+=gx; sy+=gy; sz+=gz; n+=1
        time.sleep(0.002)
    if n==0: return 0.0,0.0,0.0
    print("[IMU] Done.")
    return sx/n, sy/n, sz/n

def video_server(stop_evt: threading.Event):
    cam = Picamera2()

    # Build a libcamera Transform for rotation
    if ROTATE_DEG % 360 == 0:
        xform = Transform()
    elif ROTATE_DEG % 360 == 180:
        # 180° = hflip + vflip (keeps 1280x720)
        xform = Transform(hflip=True, vflip=True)
    else:
        # For 90/270 you can use transpose + one flip; see notes below.
        xform = Transform()

    cfg = cam.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "YUV420"},
        controls={"FrameRate": FPS},
        transform=xform,  # <-- rotation applied in ISP before encode
    )
    cam.configure(cfg)
    enc = H264Encoder(bitrate=BITRATE)
    enc.repeat  = True     # SPS/PPS inline at each IDR
    enc.iperiod = GOP

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", VID_PORT))
    srv.listen(1)
    print(f"[VID] Listening on :{VID_PORT} (H.264 {WIDTH}x{HEIGHT}@{FPS}, {BITRATE//1_000_000} Mbps)")

    try:
        while not stop_evt.is_set():
            srv.settimeout(1.0)
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print(f"[VID] Client {addr} connected")
            try:
                out = PyavOutput(f"pipe:{conn.fileno()}", format="mpegts")
                cam.start_recording(enc, out, name="main")
                while not stop_evt.is_set():
                    time.sleep(0.25)  # keep thread alive; PyavOutput pushes internally
            except Exception as e:
                print(f"[VID] Stream ended: {e}")
            finally:
                try: cam.stop_recording()
                except Exception: pass
                try: conn.close()
                except Exception: pass
                print("[VID] Client disconnected")
    finally:
        srv.close()
        print("[VID] Server stopped")

def imu_server(stop_evt: threading.Event):
    bus = SMBus(I2C_BUS)
    try:
        who = bus.read_byte_data(MPU_ADDR, WHO_AM_I)
        print(f"[IMU] WHO_AM_I=0x{who:02X}")
    except Exception as e:
        print(f"[IMU] WHO_AM_I read failed: {e}")
    mpu_init(bus)
    bias_gx, bias_gy, bias_gz = mpu_calibrate(bus, 1.5)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", IMU_PORT))
    srv.listen(1)
    print(f"[IMU] Listening on :{IMU_PORT} (100 Hz, struct <Q6f>)")

    period = 1.0/IMU_HZ
    try:
        while not stop_evt.is_set():
            srv.settimeout(1.0)
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print(f"[IMU] Client {addr} connected")
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                next_t = time.monotonic()
                while not stop_evt.is_set():
                    t_us = time.monotonic_ns() // 1000
                    ax, ay, az, gx, gy, gz = mpu_read(bus)
                    gx -= bias_gx; gy -= bias_gy; gz -= bias_gz
                    if APPLY_AXIS_MAP:
                        # ax=-z, ay=+x, az=+y; gx=+z, gy=-x, gz=-y
                        ax, ay, az, gx, gy, gz = (-az, ax, ay, gz, -gx, -gy)
                    packet = struct.pack("<Q6f", t_us, ax, ay, az, gx, gy, gz)
                    conn.sendall(packet)

                    next_t += period
                    sleep = next_t - time.monotonic()
                    if sleep > 0:
                        time.sleep(sleep)
                    else:
                        next_t = time.monotonic()
            except Exception as e:
                print(f"[IMU] Stream ended: {e}")
            finally:
                try: conn.close()
                except Exception: pass
                print("[IMU] Client disconnected")
    finally:
        bus.close()
        srv.close()
        print("[IMU] Server stopped")

def main():
    stop_evt = threading.Event()
    def _sig(*_): stop_evt.set()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    tv = threading.Thread(target=video_server, args=(stop_evt,), daemon=True)
    ti = threading.Thread(target=imu_server,   args=(stop_evt,), daemon=True)
    tv.start(); ti.start()

    while not stop_evt.is_set():
        time.sleep(0.2)

if __name__ == "__main__":
    main()

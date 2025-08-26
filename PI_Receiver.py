#!/usr/bin/env python3
"""
PC-side recorder for Pi camera + Pi IMU + 2x Serial IMUs.

Changes vs previous version:
  - Live preview added (before heartbeat). Recording still begins ONLY after heartbeat.
  - Press ESC or 'q' in preview window to exit cleanly.

Outputs (same as known-working):
  recording_<YYYYMMDD_HHMMSS_mmm>/
    ├─ frames/frame_<rel_color_us>.jpg       # resized to IMG_W×IMG_H, JPEG_QUALITY
    ├─ imu_raw.csv                           # Pi IMU @100 Hz, header identical
    ├─ CentC_serial_*.csv  (optional)
    └─ HeadR_serial_*.csv  (optional)

Run:
  pip install av opencv-python numpy pyserial
  python pc_recorder_pi_receiver.py --pi-ip 10.55.0.1 --portA COM4 --portB COM6
"""
import threading, time, struct, socket, datetime, csv, os, argparse
import numpy as np, cv2, serial, zlib
from serial import SerialException
import av  # PyAV
try:
    from av import AVError as _PyAVError
except Exception:
    class _PyAVError(Exception): pass

# ================= User Config =================
# Heartbeat listener (UNCHANGED semantics): start recording only after any UDP arrives here.
SIMULINK_IP = "127.0.0.1"
START_PORT  = 6000

# Pi endpoints
PI_IP        = "10.55.0.1"
PI_VID_PORT  = 6000
PI_IMU_PORT  = 6001

# Output & rates
IMG_W, IMG_H   = 320, 240
JPEG_QUALITY   = 90
HB_TIMEOUT     = 0.5  # not used to stop; heartbeat only gates start
PI_IMU_STRUCT  = struct.Struct("<Q6f")  # t_us, ax, ay, az, gx, gy, gz
# =================================================

# ---------------- Serial IMU (unchanged) ----------------
class SerialIMUThread(threading.Thread):
    """Read lines from Serial, parse CSV+CRC, write to its own imu_serial.csv."""
    def __init__(self, port, baudrate, out_dir, last_hb, hb_timeout, tag="imu1", max_gap=1.0):
        super().__init__(daemon=True)
        self.port, self.baudrate = port, baudrate
        self.max_gap, self.out_dir, self.tag = max_gap, out_dir, tag
        self.last_hb, self.hb_timeout = last_hb, hb_timeout

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except SerialException as e:
            print(f"[ERROR][{self.tag}] Could not open serial port {self.port!r}: {e}")
            return
        time.sleep(0.3)
        ser.reset_input_buffer()

        fname = os.path.join(self.out_dir, f"{self.tag}_serial_{datetime.datetime.now():%Y%m%d_%H%M%S}_{self.port}.csv")
        with open(fname, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_us','t_s','t_rel', 'ax','ay','az', 'gx','gy','gz','crc'])

            state = "waiting"
            t0 = None
            last_recv = time.time()
            print(f"[SERIAL {self.tag}] Logging to {fname}")

            while True:
                hb = self.last_hb[0]  # gate by heartbeat
                try:
                    line = ser.readline().decode(errors='ignore').strip()
                except SerialException as e:
                    print(f"[ERROR][{self.tag}] Serial read error on {self.port}: {e}")
                    break
                except Exception as e:
                    print(f"[ERROR][{self.tag}] Unexpected serial error: {e}")
                    break

                if not line:
                    if time.time() - last_recv > self.max_gap:
                        print(f"[SERIAL {self.tag}] ⚠️  No data for {self.max_gap}s")
                        last_recv = time.time()
                    continue
                last_recv = time.time()

                if state == "waiting":
                    if "Calibrate" in line:
                        state = "calibrating"; print(f"[SERIAL {self.tag}] Calibrating IMU… please wait")
                    continue
                if state == "calibrating":
                    if "DMP ready" in line:
                        state = "running"; print(f"[SERIAL {self.tag}] ✅ Calibration complete! Logging data now.")
                    continue

                if hb is None:
                    continue  # do not write until heartbeat

                parts = line.split(',')
                if len(parts) != 11:
                    print(f"[SERIAL {self.tag}] ⚠️ Bad field count: {line}")
                    continue

                data_str = ','.join(parts[:-1])
                recv_crc = int(parts[-1], 16)
                calc_crc = zlib.crc32(data_str.encode()) & 0xFFFFFFFF
                if calc_crc != recv_crc:
                    print(f"[SERIAL {self.tag}] ❌ CRC mismatch: {parts[-1]} vs {calc_crc:08X}")
                    continue

                t_us = int(parts[0]); t_s = t_us/1e6
                if t0 is None: t0 = t_s
                t_rel = t_s - t0

                ax, ay, az = map(float, parts[1:4])
                gx, gy, gz = map(float, parts[4:7])
                # units
                ax *= 9.81; ay *= 9.81; az *= 9.81
                gx = np.deg2rad(gx); gy = np.deg2rad(gy); gz = np.deg2rad(gz)

                writer.writerow([
                    t_us, f"{t_s:.6f}", f"{t_rel:.6f}",
                    f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}",
                    f"{gx:.6f}", f"{gy:.6f}", f"{gz:.6f}",
                    parts[10]
                ])
                f.flush(); os.fsync(f.fileno())

        ser.close()
        print(f"[SERIAL {self.tag}] Thread exiting cleanly.")

# ---------------- Heartbeat ----------------
def start_heartbeat_listener(ip, port, last_hb):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"[HB] Listening for heartbeat on {ip}:{port}")
    while True:
        try:
            sock.recvfrom(16)
            last_hb[0] = time.perf_counter()
        except Exception:
            break

# ---------------- Video client (Pi → PC, TCP :6000) ----------------
class VideoClient:
    """Connect to Pi H.264 over TCP (mpegts), decode frames, yield (bgr, pts_us)."""
    def __init__(self, host: str, port: int, reconnect_delay: float = 1.0, open_timeout_s: int = 5):
        self.url = f"tcp://{host}:{port}"
        self.reconnect_delay = reconnect_delay
        self._open_opts = {"rw_timeout": str(open_timeout_s * 1_000_000)}  # µs
        self._container = None; self._vstream = None; self._closed = False

    def _open(self):
        self._container = av.open(self.url, format="mpegts", options=self._open_opts)
        self._vstream   = next(s for s in self._container.streams if s.type == "video")
        print(f"[VID] Opened {self.url} OK, codec={self._vstream.codec_context.name}, tb={self._vstream.time_base}")

    def frames(self):
        while not self._closed:
            try:
                if self._container is None:
                    self._open()
                for packet in self._container.demux(self._vstream):
                    for frame in packet.decode():
                        img_bgr = frame.to_ndarray(format="bgr24")
                        tb = frame.time_base or self._vstream.time_base
                        pts_us = int(frame.pts * tb * 1_000_000) if (frame.pts is not None and tb is not None) else None
                        yield img_bgr, pts_us
            except (_PyAVError, OSError) as e:   # <- catch PyAV + socket layer
                if self._closed:
                    break
                print(f"[VID] Open/demux error on {self.url}: {e}; reconnecting…")
                self._cleanup()
                time.sleep(self.reconnect_delay)

            except Exception as e:
                if self._closed:
                    break
                print(f"[VID] Unexpected error: {type(e).__name__}: {e}; reconnecting…")
                self._cleanup()
                time.sleep(self.reconnect_delay)
        
    def close(self):
        self._closed = True; self._cleanup()

    def _cleanup(self):
        try:
            if self._container: self._container.close()
        except Exception: pass
        self._container = None; self._vstream = None

# ---------------- IMU client (Pi → PC, TCP :6001) ----------------
class IMUClient:
    """Connect to Pi IMU TCP :6001, receive struct <Q6f> samples."""
    def __init__(self, host: str, port: int, reconnect_delay: float = 1.0):
        self.host, self.port, self.reconnect_delay = host, port, reconnect_delay
        self.sock = None; self.closed = False

    def _connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.connect((self.host, self.port))
        s.settimeout(2.0)
        self.sock = s

    def samples(self):
        buf = b""
        while not self.closed:
            try:
                if self.sock is None:
                    self._connect(); buf = b""
                    print(f"[IMU] Connected to {self.host}:{self.port}")
                chunk = self.sock.recv(4096)
                if not chunk: raise ConnectionError("server closed")
                buf += chunk
                while len(buf) >= PI_IMU_STRUCT.size:
                    t_us, ax, ay, az, gx, gy, gz = PI_IMU_STRUCT.unpack_from(buf, 0)
                    buf = buf[PI_IMU_STRUCT.size:]
                    yield t_us, ax, ay, az, gx, gy, gz
            except Exception as e:
                print(f"[IMU] Error: {e}; reconnecting…")
                if self.sock:
                    try: self.sock.close()
                    except Exception: pass
                self.sock = None; time.sleep(self.reconnect_delay)

    def close(self):
        self.closed = True
        if self.sock:
            try: self.sock.close()
            except Exception: pass
            self.sock = None

# ---------------- Utils ----------------
def ensure_dir(p): os.makedirs(p, exist_ok=True); return p

# ---------------- Main ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi-ip", default=PI_IP)
    ap.add_argument("--vid-port", type=int, default=PI_VID_PORT)
    ap.add_argument("--imu-port", type=int, default=PI_IMU_PORT)
    ap.add_argument("--portA", default="COM4")
    ap.add_argument("--portB", default="COM6")
    ap.add_argument("--no-preview", action="store_true", help="disable live preview window")
    args = ap.parse_args()
    PREVIEW = not args.no_preview

    # ----- Output directory -----
    session_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    record_dir = f"recording_{session_ts}"
    frames_dir = ensure_dir(os.path.join(record_dir, "frames"))
    imu_csv_path = os.path.join(record_dir, "imu_raw.csv")
    print(f"[INFO] Recording into: {record_dir}")

    # ----- Heartbeat -----
    last_hb = [None]
    threading.Thread(target=start_heartbeat_listener,
                     args=(SIMULINK_IP, START_PORT, last_hb),
                     daemon=True).start()

    # ----- Serial IMUs (unchanged) -----
    SERIAL_IMUS = [
        {"port": args.portA, "baudrate": 115200, "tag": "CentC"},
        {"port": args.portB, "baudrate": 115200, "tag": "HeadR"},
    ]
    for cfg in SERIAL_IMUS:
        t = SerialIMUThread(port=cfg["port"], baudrate=cfg["baudrate"],
                            out_dir=record_dir, tag=cfg["tag"],
                            max_gap=1.0, last_hb=last_hb, hb_timeout=HB_TIMEOUT)
        t.start()

    # ----- Pi video + Pi IMU -----
    vc = VideoClient(args.pi_ip, args.vid_port)
    ic = IMUClient(args.pi_ip, args.imu_port)

    # CSV for Pi IMU (same header)
    csv_file = open(imu_csv_path, "w", newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['t_accel_us','t_color_us','ax','ay','az','gx','gy','gz','wall_clock'])

    # shared state for timestamps
    t0_color_us = None     # set on first color frame AFTER heartbeat
    t0_accel_us = None     # set on first imu sample AFTER t0_color_us
    last_color_rel_us = None
    stop = threading.Event()

    # --- Video thread: preview always; save only after heartbeat ---
    def video_thread():
        nonlocal t0_color_us, last_color_rel_us
        fps_count = 0; fps_t0 = time.time(); fps = 0.0
        win = "Preview (waiting for heartbeat)" if PREVIEW else None
        if PREVIEW:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        try:
            for img_bgr, pts_us in vc.frames():
                if stop.is_set(): break

                # live preview (even before heartbeat)
                if PREVIEW:
                    fps_count += 1
                    now = time.time()
                    if now - fps_t0 >= 0.5:
                        fps = fps_count / (now - fps_t0)
                        fps_count = 0; fps_t0 = now

                    disp = img_bgr.copy()
                    # status overlay
                    status = ("REC" if t0_color_us is not None else "PREVIEW (waiting for heartbeat)")
                    color  = (0,0,255) if t0_color_us is not None else (0,255,255)
                    cv2.putText(disp, f"{status}  {fps:4.1f} FPS",
                                (8,28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
                    cv2.imshow(win, disp)
                    k = cv2.waitKey(1) & 0xFF
                    if k in (27, ord('q')):
                        stop.set(); break

                # ignore saving until we have heartbeat + pts
                if last_hb[0] is None or pts_us is None:
                    continue

                if t0_color_us is None:
                    t0_color_us = pts_us
                    if PREVIEW:
                        cv2.setWindowTitle(win, "REC")
                    print("[VID] First heartbeat-aligned frame → t0_color set")

                rel_color_us = int(pts_us - t0_color_us)
                last_color_rel_us = rel_color_us

                # resize to legacy size for saving
                if (img_bgr.shape[1], img_bgr.shape[0]) != (IMG_W, IMG_H):
                    img_bgr = cv2.resize(img_bgr, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)

                fname = os.path.join(frames_dir, f"frame_{rel_color_us}.jpg")
                ok = cv2.imwrite(fname, img_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if not ok:
                    print(f"[VID] ⚠️ Failed to write {fname}")

        finally:
            vc.close()
            if PREVIEW:
                try: cv2.destroyWindow(win)
                except Exception: pass
            print("[VID] thread exit")

    # --- IMU thread: write imu_raw.csv after heartbeat & after t0_color_us ---
    def imu_thread():
        nonlocal t0_accel_us
        try:
            for t_us, ax, ay, az, gx, gy, gz in ic.samples():
                if stop.is_set(): break
                if (last_hb[0] is None) or (t0_color_us is None):
                    continue
                if t0_accel_us is None:
                    t0_accel_us = t_us
                    print("[IMU] First heartbeat-aligned sample → t0_accel set")

                rel_accel_us = int(t_us - t0_accel_us)
                wallclock = datetime.datetime.now().strftime("%H%M%S_%f")
                csv_writer.writerow((
                    rel_accel_us,
                    last_color_rel_us if last_color_rel_us is not None else "",
                    f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}",
                    f"{gx:.6f}", f"{gy:.6f}", f"{gz:.6f}",
                    wallclock
                ))
                csv_file.flush(); os.fsync(csv_file.fileno())
        finally:
            ic.close()
            print("[IMU] thread exit")

    tv = threading.Thread(target=video_thread)
    ti = threading.Thread(target=imu_thread)
    tv.start()
    ti.start()

    try:
        while not stop.is_set() and tv.is_alive() and ti.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop.set()
    finally:
        vc.close(); ic.close()
        tv.join(timeout=3.0); ti.join(timeout=3.0)   # <-- wait for clean exit
        try: csv_file.close()
        except Exception: pass
        if PREVIEW:
            try: cv2.destroyAllWindows()
            except Exception: pass
        print("[INFO] Done.")

if __name__ == "__main__":
    main()

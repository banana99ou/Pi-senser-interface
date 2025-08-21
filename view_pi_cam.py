#!/usr/bin/env python3
import socket, struct, os, csv, datetime, threading

PI_IP    = "10.55.0.1"   # adjust if needed
VID_PORT = 6000
IMU_PORT = 6001

VHDR = struct.Struct("<cHHQIB")  # 'V', w, h, t_us, nbytes, key
IMU  = struct.Struct("<Q6f")     # t_us, ax ay az gx gy gz

def read_exact(sock, n):
    buf = bytearray(n)
    mv  = memoryview(buf)
    got = 0
    while got < n:
        chunk = sock.recv(n - got)
        if not chunk:
            raise ConnectionResetError("socket closed")
        mv[got:got+len(chunk)] = chunk
        got += len(chunk)
    return mv

def video_thread(ip, port, out_dir):
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    h264_path = os.path.join(out_dir, f"video_{ts}.h264")
    csv_path  = os.path.join(out_dir, f"frame_timestamps_{ts}.csv")
    os.makedirs(out_dir, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    # grow buffers to smooth bursts
    try: sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
    except: pass

    with open(h264_path, "wb") as vf, open(csv_path, "w", newline="") as cf:
        wtr = csv.writer(cf); wtr.writerow(["idx","t_us","keyframe","bytes"])
        idx = 0
        print(f"[PC] Video → {h264_path}")
        try:
            while True:
                hdr = read_exact(sock, VHDR.size)
                typ, w, h, t_us, nbytes, key = VHDR.unpack(hdr)
                if typ != b"V":
                    raise RuntimeError(f"Bad type byte: {typ!r}")
                payload = read_exact(sock, nbytes)
                vf.write(payload)
                wtr.writerow([idx, t_us, int(key), nbytes])
                idx += 1
        except (ConnectionResetError, OSError):
            pass
    sock.close()
    print("[PC] Video thread ended.")

def imu_thread(ip, port, out_dir):
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    csv_path = os.path.join(out_dir, f"imu_{ts}.csv")
    os.makedirs(out_dir, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    try: sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1*1024*1024)
    except: pass

    with open(csv_path, "w", newline="") as cf:
        wtr = csv.writer(cf); wtr.writerow(["t_us","ax","ay","az","gx","gy","gz"])
        print(f"[PC] IMU  → {csv_path}")
        try:
            pkt_sz = IMU.size
            while True:
                pkt = read_exact(sock, pkt_sz)
                t_us, ax, ay, az, gx, gy, gz = IMU.unpack(pkt)
                wtr.writerow([t_us, ax, ay, az, gx, gy, gz])
        except (ConnectionResetError, OSError):
            pass
    sock.close()
    print("[PC] IMU thread ended.")

def main():
    out_dir = os.path.join(os.getcwd(), f"recording_{datetime.datetime.now():%Y%m%d_%H%M%S}")
    t1 = threading.Thread(target=video_thread, args=(PI_IP, VID_PORT, out_dir), daemon=True)
    t2 = threading.Thread(target=imu_thread,   args=(PI_IP, IMU_PORT, out_dir), daemon=True)
    t1.start(); t2.start()
    print(f"[PC] Saving to: {out_dir}")
    try:
        while t1.is_alive() and t2.is_alive():
            t1.join(0.5); t2.join(0.5)
    except KeyboardInterrupt:
        print("\n[PC] Ctrl+C – stopping.")
    # threads exit on socket close from Pi

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Live visualization (no saving):
- Shows 720p video with FPS and video PTS
- Overlays latest IMU values and IMU rate
Press ESC or 'q' to exit.
"""
import threading, time
import cv2
from video_client import VideoClient
from imu_client import IMUClient

HOST = "10.55.0.1"
VID_PORT = 6000
IMU_PORT = 6001

def main():
    vc = VideoClient(HOST, VID_PORT)
    ic = IMUClient(HOST, IMU_PORT)

    last_imu = {"t_us": None, "ax":0.0,"ay":0.0,"az":0.0,"gx":0.0,"gy":0.0,"gz":0.0}
    imu_rate = {"count":0, "t0":time.time(), "hz":0.0}

    def imu_thread():
        for t_us, ax, ay, az, gx, gy, gz in ic.samples():
            last_imu.update({"t_us":t_us,"ax":ax,"ay":ay,"az":az,"gx":gx,"gy":gy,"gz":gz})
            imu_rate["count"] += 1
            now = time.time()
            if now - imu_rate["t0"] >= 1.0:
                imu_rate["hz"] = imu_rate["count"] / (now - imu_rate["t0"])
                imu_rate["count"] = 0
                imu_rate["t0"] = now

    th = threading.Thread(target=imu_thread, daemon=True); th.start()

    win = "Pi 720p60 + IMU"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    t0 = time.time(); frames = 0; fps = 0.0

    try:
        for img, pts_s in vc.frames():
            frames += 1
            now = time.time()
            if now - t0 >= 0.5:
                fps = frames / (now - t0)
                frames = 0; t0 = now

            y = 28; lh = 26
            def put(txt):
                nonlocal y
                cv2.putText(img, txt, (8,y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2, cv2.LINE_AA)
                y += lh

            put(f"Video {fps:5.1f} FPS  " + (f"t={pts_s*1e3:8.1f} ms" if pts_s==pts_s else "t=--.-"))
            t_us = last_imu["t_us"]
            if t_us is not None:
                age_ms = (time.time_ns()/1e6) - (t_us/1e3)
                put(f"IMU   {imu_rate['hz']:5.1f} Hz  age={age_ms:6.1f} ms")
                put(f"ax={last_imu['ax']: .2f} ay={last_imu['ay']: .2f} az={last_imu['az']: .2f} m/s^2")
                put(f"gx={last_imu['gx']: .2f} gy={last_imu['gy']: .2f} gz={last_imu['gz']: .2f} rad/s")
            else:
                put("IMU   --.- Hz  (waiting)")

            cv2.imshow(win, img)
            if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):
                break
    finally:
        vc.close(); ic.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

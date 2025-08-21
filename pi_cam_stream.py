#!/usr/bin/env python3
import socket, struct, cv2, time
from picamera2 import Picamera2

HOST = "0.0.0.0"
PORT = 5000
RES  = (640, 480)
FPS  = 30
JPEG_QUALITY = 80

cam = Picamera2()

# Always request RGB from the camera, then convert to BGR ourselves.
cfg = cam.create_video_configuration(main={"size": RES, "format": "RGB888"})
cam.configure(cfg)
try:
    cam.set_controls({"FrameRate": FPS})
except Exception:
    pass
cam.start()

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind((HOST, PORT))
srv.listen(1)
print(f"[Pi] Waiting for viewer on port {PORT} …")
conn, addr = srv.accept()
print(f"[Pi] Viewer connected from {addr}")

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
pack_u32 = struct.Struct(">L").pack

try:
    while True:
        # Camera delivers RGB; convert to BGR for OpenCV encoding.
        frame_rgb = cam.capture_array("main")
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode(".jpg", frame_bgr, encode_param)
        if not ok:
            continue
        data = jpg.tobytes()
        conn.sendall(pack_u32(len(data)) + data)
        time.sleep(1 / FPS)
except (BrokenPipeError, ConnectionResetError, KeyboardInterrupt):
    pass
finally:
    conn.close()
    srv.close()

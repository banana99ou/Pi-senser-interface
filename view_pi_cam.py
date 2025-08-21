#!/usr/bin/env python3
import socket, struct, cv2, numpy as np

PI_IP = "10.55.0.1"   # change if needed
PORT  = 443

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((PI_IP, PORT))
print("[Laptop] Connected – press ESC to quit")

payload_len = struct.Struct(">L").size
buf = b""

try:
    while True:
        # --- read 4-byte size header ---
        while len(buf) < payload_len:
            buf += sock.recv(4096)
        frame_len = struct.unpack(">L", buf[:payload_len])[0]
        buf = buf[payload_len:]

        # --- read the JPEG payload ---
        while len(buf) < frame_len:
            buf += sock.recv(4096)
        jpg = buf[:frame_len]
        buf = buf[frame_len:]

        # --- decode & show ---
        frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow("PiCam", frame)
        if cv2.waitKey(1) == 27:  # ESC
            break
finally:
    sock.close()
    cv2.destroyAllWindows()
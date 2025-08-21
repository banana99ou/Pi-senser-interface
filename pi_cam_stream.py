#!/usr/bin/env python3
import socket, threading
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import PyavOutput

WIDTH, HEIGHT, FPS = 1280, 720, 60
BITRATE = 10_000_000      # 10 Mbps is a good starting point
GOP     = 120             # keyframe every ~2 s at 60 fps
PORT    = 6000            # video TCP port

def main():
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "YUV420"},
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)

    enc = H264Encoder(bitrate=BITRATE)
    enc.repeat  = True     # repeat SPS/PPS at each IDR (SPS/PPS inline)
    enc.iperiod = GOP      # GOP length (frames between I-frames)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)
    print(f"[Pi] Listening on :{PORT} (H.264 {WIDTH}x{HEIGHT}@{FPS}, {BITRATE//1_000_000} Mbps, GOP {GOP})")

    try:
        while True:
            conn, addr = srv.accept()
            print(f"[Pi] Client connected: {addr}")
            stop_evt = threading.Event()

            # Stream H.264 inside an MPEG-TS container to the TCP socket
            out = PyavOutput(f"pipe:{conn.fileno()}", format="mpegts")
            out.error_callback = lambda e: (print(f"[Pi] Output error/disconnect: {e}"), stop_evt.set())

            try:
                picam2.start_recording(enc, out, name="main")
                stop_evt.wait()  # wait until client disconnects or an error occurs
            finally:
                try: picam2.stop_recording()
                except Exception: pass
                try: conn.close()
                except Exception: pass
                print("[Pi] Client disconnected; waiting for next client…")
    finally:
        srv.close()

if __name__ == "__main__":
    main()

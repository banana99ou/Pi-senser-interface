#!/usr/bin/env python3
import argparse, time
import cv2
from video_client import VideoClient

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="10.55.0.1")
    ap.add_argument("--port", type=int, default=6000)
    args = ap.parse_args()

    vc = VideoClient(args.host, args.port)
    win = "Pi 720p60"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    t0 = time.time(); frames = 0

    try:
        for img, pts_s in vc.frames():
            frames += 1
            now = time.time()
            if now - t0 >= 0.5:
                fps = frames / (now - t0)
                t0 = now; frames = 0
            # else:
            #     fps = None

            if fps is not None:
                text = f"{fps:.1f} FPS"
                if pts_s == pts_s:  # not NaN
                    text += f" | t={pts_s*1e3:,.1f} ms"
            cv2.putText(img, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2, cv2.LINE_AA)

            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
    finally:
        vc.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

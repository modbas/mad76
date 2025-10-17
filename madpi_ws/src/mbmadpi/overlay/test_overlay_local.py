import os
import cv2
import numpy as np
from overlay_preview import draw_leaderboard, draw_bottom_status


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(script_dir, 'track_preview.jpg')
    out_path = os.path.join(script_dir, 'track_preview_out_camera_overlay.jpg')

    if os.path.exists(img_path):
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"Error: '{img_path}' exists but couldn't be read as an image.")
            return
    else:
        frame = np.full((600, 800, 3), 110, dtype=np.uint8)
        cv2.putText(frame, 'Placeholder Track Preview', (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (200,200,200), 2)

    target_w, target_h = 800, 600
    h, w = frame.shape[:2]
    if w == target_w and h == target_h:
        canvas = frame.copy()
    else:
        scale = min(target_w / float(w), target_h / float(h))
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
        canvas = np.full((target_h, target_w, 3), 110, dtype=np.uint8)
        x_off = (target_w - new_w) // 2
        y_off = (target_h - new_h) // 2
        canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized

    frame = canvas
    draw_leaderboard(frame)
    draw_bottom_status(frame)

    ok = cv2.imwrite(out_path, frame)
    if ok:
        print(f"Preview image written to: {out_path}")
    else:
        print("Failed to write preview image.")


if __name__ == '__main__':
    main()

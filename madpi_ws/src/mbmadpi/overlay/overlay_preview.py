import os
import cv2
import numpy as np

def get_mad76_data(): # Mock function to simulate data retrieval
    # In a real scenario, this function would fetch data from the MAD76 management interface
    return [
        {'car': 1, 'pos': 1, 'driver': 'name_1', 'lap': 12, 'time': 110.56, 'speed': 136.8, 'mode': 'User'},
        {'car': 2, 'pos': 2, 'driver': 'name_2', 'lap': 11, 'time': 112.34, 'speed': 0.5, 'mode': 'AI'},
        {'car': 3, 'pos': 3, 'driver': 'name_3', 'lap': 11, 'time': 114.01, 'speed': 0, 'mode': 'Not On Track'},
        {'car': 4, 'pos': 4, 'driver': 'name_4', 'lap': 10, 'time': 120.45, 'speed': 0, 'mode': 'Not On Track'},
    ]

def draw_text(img, text, org, font_scale=0.6, thickness=1, color=(255,255,255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, cv2.LINE_AA)

def draw_leaderboard(frame, x=10, y=10, width=300):
    car_data = get_mad76_data()
    entry_h = 25
    header_h = 25
    padding = 8
    height = header_h + padding + entry_h * len([c for c in car_data if c['mode'].lower() != 'not on track']) + padding

    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + width, y + height), (18, 18, 18), -1)
    alpha = 0.5
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    draw_text(frame, 'MAD76', (x + 12, y + 26), font_scale=0.6, thickness=2)

    visible_entries = sorted([c for c in car_data if c['mode'].lower() != 'not on track'], key=lambda e: e['pos'])
    for idx, entry in enumerate(visible_entries):
        line_y = y + header_h + padding + idx * entry_h
        color_map = {1: (0, 0, 255), 2: (255, 255, 255), 3: (255, 0, 0), 4: (0, 255, 0)}
        car_color = color_map.get(entry['car'], (200, 200, 200))

        rect_w = 6
        rect_h = entry_h - 6
        rx = x + 8
        ry = line_y + 4
        cv2.rectangle(frame, (rx, ry), (rx + rect_w, ry + rect_h), car_color, -1)

        pos_text = f"{entry['pos']:>2}."
        car_text = f"{entry['driver']}"
        lap_text = f"Lap{entry['lap']}"
        time_text = f"{entry['time']:.2f}s"

        tx = rx + rect_w + 8
        draw_text(frame, pos_text, (tx, line_y + 22), font_scale=0.5, thickness=2, color=(245,245,245))
        draw_text(frame, car_text, (tx + 30, line_y + 22), font_scale=0.55, thickness=1, color=(220,220,220))
        draw_text(frame, lap_text, (x + width - 145, line_y + 22), font_scale=0.50, thickness=1, color=(160,255,160))
        draw_text(frame, time_text, (x + width - 80, line_y + 22), font_scale=0.50, thickness=1, color=(200,200,255))

    cv2.rectangle(frame, (x, y), (x + width, y + height), (150, 150, 150), 1)

def draw_bottom_status(frame):
    car_info = get_mad76_data()
    cols = len(car_info)

    start_y = frame.shape[0] - 64
    box_height = 75
    box_width = int((frame.shape[1] - 40) / max(1, cols))

    overlay = frame.copy()
    bar_y = start_y - 4
    cv2.rectangle(overlay, (10, bar_y), (frame.shape[1] - 10, bar_y + box_height + 12), (18, 18, 18), -1)
    cv2.addWeighted(overlay, 1, frame, 1 - .5, 0, frame)

    for idx, car in enumerate(car_info):
        x = 10 + idx * box_width
        y = start_y
        inner_w = box_width - 12
        inner_h = box_height

        car_name_x = x + 16
        car_name_y = y + 16
        car_scale = 0.60
        car_th = 2
        driver_scale = 0.5
        driver_th = 1
        mode_scale = 0.5
        cmd_scale = 0.5

        car_text = f"Car{str(car['car'])}"
        driver_name = car.get('driver', '')

        (car_w, _), _ = cv2.getTextSize(car_text, cv2.FONT_HERSHEY_SIMPLEX, car_scale, car_th)
        (driver_w, _), _ = cv2.getTextSize(driver_name, cv2.FONT_HERSHEY_SIMPLEX, driver_scale, driver_th) if driver_name else ((0,0), 0)
        gap = 4

        mode_text = f"Mode: {car['mode']}"
        speed_text = f"Speed: {car['speed']}"
        (mode_w, _), _ = cv2.getTextSize(mode_text, cv2.FONT_HERSHEY_SIMPLEX, mode_scale, 1)
        (cmd_w, _), _ = cv2.getTextSize(speed_text, cv2.FONT_HERSHEY_SIMPLEX, cmd_scale, 1)

        needed_w = car_w + gap + driver_w + 36
        needed_w = max(needed_w, mode_w + 36, cmd_w + 36)

        max_inner_w = (frame.shape[1] - 10) - (x + 6)
        inner_w_draw = min(max(inner_w, int(needed_w)), max_inner_w)

        draw_text(frame, car_text, (car_name_x, car_name_y), font_scale=car_scale, thickness=car_th)
        if driver_name:
            driver_x = car_name_x + car_w + gap
            draw_text(frame, driver_name, (driver_x, car_name_y), font_scale=driver_scale, thickness=driver_th, color=(190,190,190))

        mode_y = y + 40
        cmd_y = y + 56
        draw_text(frame, mode_text, (car_name_x, mode_y), font_scale=mode_scale, thickness=1, color=(180,255,255))
        draw_text(frame, speed_text, (car_name_x, cmd_y), font_scale=cmd_scale, thickness=1, color=(255,255,200))

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(script_dir, 'track_preview.jpg')
    out_path = os.path.join(script_dir, 'track_preview_out.jpg')

    if os.path.exists(img_path):
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"Error: '{img_path}' exists but couldn't be read as an image.")
            return
    else:
        print(f"'{img_path}' not found; creating a placeholder background.")
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

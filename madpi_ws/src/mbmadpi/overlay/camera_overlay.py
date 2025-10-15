import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mbmadmsgs.msg import CarOutputsExtList
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import json
import urllib.request
from urllib.error import URLError


# --- Aggregator for live data ---
class CarState:
    def __init__(self):
        # keyed by carid (int)
        self._cars = {}
        self._lock = threading.Lock()

    def update_from_outputs_ext_list(self, msg_list):
        # msg_list: mbmadmsgs/CarOutputsExtList, iterate msg.list
        with self._lock:
            now = time.time()
            for entry in getattr(msg_list, 'list', []):
                cid = int(entry.carid)
                rec = self._cars.setdefault(cid, {})
                rec['car'] = cid
                rec['speed'] = float(getattr(entry, 'v', 0.0))
                s = getattr(entry, 's', None)
                if s and len(s) >= 2:
                    rec['pos_xy'] = (float(s[0]), float(s[1]))
                rec['prob'] = float(getattr(entry, 'prob', 0.0))
                rec['last_seen'] = now

    def update_from_ranking(self, ranking_json):
        # ranking_json: list of {driver, laptime, avgspeed, active}
        with self._lock:
            for idx, item in enumerate(ranking_json, start=1):
                rec = self._cars.setdefault(idx, {})
                rec['driver'] = item.get('driver')
                rec['pos'] = idx
                rec['laptime'] = item.get('laptime') or item.get('time')
                rec['avgspeed'] = item.get('avgspeed') or item.get('speed')

    def snapshot_list(self):
        with self._lock:
            out = []
            # sort by pos when available, else by carid
            items = sorted(self._cars.items(), key=lambda kv: kv[1].get('pos', kv[0]))
            for cid, rec in items:
                item = {
                    'car': rec.get('car', cid),
                    'pos': rec.get('pos', None) or cid,
                    'driver': rec.get('driver', '') or f'car_{cid}',
                    'lap': rec.get('lap', 0),
                    'time': rec.get('laptime', 0.0),
                    'speed': rec.get('speed', 0.0),
                    'mode': 'On Track' if rec.get('prob', 0.0) > 0.1 else 'Not On Track'
                }
                out.append(item)
            return out


# shared aggregator instance
CAR_STATE = CarState()


# data will be pulled from CAR_STATE.snapshot_list() inside the draw functions


def draw_text(img, text, org, font_scale=0.6, thickness=1, color=(255, 255, 255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, cv2.LINE_AA)


def draw_leaderboard(frame, x=10, y=10, width=300):
    car_data = CAR_STATE.snapshot_list()
    entry_h = 25
    header_h = 25
    padding = 8
    visible = [c for c in car_data if c['mode'].lower() != 'not on track']
    height = header_h + padding + entry_h * len(visible) + padding

    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + width, y + height), (18, 18, 18), -1)
    alpha = 0.5
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    draw_text(frame, 'MAD76', (x + 12, y + 26), font_scale=0.6, thickness=2)

    visible_entries = sorted(visible, key=lambda e: e['pos'])
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
        draw_text(frame, pos_text, (tx, line_y + 22), font_scale=0.5, thickness=2, color=(245, 245, 245))
        draw_text(frame, car_text, (tx + 30, line_y + 22), font_scale=0.55, thickness=1, color=(220, 220, 220))
        draw_text(frame, lap_text, (x + width - 145, line_y + 22), font_scale=0.50, thickness=1, color=(160, 255, 160))
        draw_text(frame, time_text, (x + width - 80, line_y + 22), font_scale=0.50, thickness=1, color=(200, 200, 255))

    cv2.rectangle(frame, (x, y), (x + width, y + height), (150, 150, 150), 1)


def draw_bottom_status(frame):
    car_info = CAR_STATE.snapshot_list()
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
        (driver_w, _), _ = cv2.getTextSize(driver_name, cv2.FONT_HERSHEY_SIMPLEX, driver_scale, driver_th) if driver_name else ((0, 0), 0)
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
            draw_text(frame, driver_name, (driver_x, car_name_y), font_scale=driver_scale, thickness=driver_th, color=(190, 190, 190))

        mode_y = y + 40
        cmd_y = y + 56
        draw_text(frame, mode_text, (car_name_x, mode_y), font_scale=mode_scale, thickness=1, color=(180, 255, 255))
        draw_text(frame, speed_text, (car_name_x, cmd_y), font_scale=cmd_scale, thickness=1, color=(255, 255, 200))


class OverlayNode(Node):
    def __init__(self):
        super().__init__('overlay_node')
        self.subscription = self.create_subscription(
            Image,
            '/mad/camera/image_raw',
            self.listener_callback,
            10)
        # subscribe to aggregated car outputs from locate
        try:
            self.sub_car_outputs_ext = self.create_subscription(
                CarOutputsExtList,
                '/mad/locate/caroutputsext',
                self.car_outputs_ext_callback,
                10)
        except Exception:
            # if message types are not available in analysis environment this will fail here; node runtime will succeed when ROS is sourced
            self.get_logger().warning('Could not create subscription to /mad/locate/caroutputsext (message types may not be built yet)')

        # periodic mgmt fetch (1 Hz) to retrieve driver names / ranking
        self.mgmt_url = 'http://localhost:8082/getranking'
        self.create_timer(1.0, self._mgmt_fetch_timer)
        self.bridge = CvBridge()
        cv2.namedWindow("MAD76 - Camera Overlay", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("MAD76 - Camera Overlay", 800, 600)

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # target window size
        target_w, target_h = 800, 600
        h, w = frame.shape[:2]
        if w == target_w and h == target_h:
            canvas = frame
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

        # Draw overlays (using preview design)
        draw_leaderboard(frame)
        draw_bottom_status(frame)

        cv2.imshow("MAD76 - Camera Overlay", frame)
        cv2.waitKey(1)

    def car_outputs_ext_callback(self, msg):
        try:
            CAR_STATE.update_from_outputs_ext_list(msg)
        except Exception as e:
            self.get_logger().error(f'Error updating car state from outputs ext: {e}')

    def _mgmt_fetch_timer(self):
        # fetch mgmt ranking JSON
        try:
            with urllib.request.urlopen(self.mgmt_url, timeout=0.8) as resp:
                raw = resp.read()
                try:
                    data = json.loads(raw.decode('utf-8'))
                    if isinstance(data, list):
                        CAR_STATE.update_from_ranking(data)
                except Exception as e:
                    self.get_logger().warning(f'Failed to parse mgmt ranking JSON: {e}')
        except URLError as e:
            # mgmt server not available; silently ignore
            pass
        except Exception as e:
            self.get_logger().warning(f'Error fetching mgmt ranking: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
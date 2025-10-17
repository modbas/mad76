import rclpy
from rclpy.node import Node
try:
    from sensor_msgs.msg import Image
except Exception:
    Image = None

try:
    from mbmadmsgs.msg import CarOutputsExtList
except Exception:
    CarOutputsExtList = None
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import urllib.request
from urllib.error import URLError

from .overlay_getcardata import CAR_STATE
from .overlay_preview import draw_leaderboard, draw_bottom_status, draw_text


class OverlayNode(Node):
    def __init__(self):
        super().__init__('overlay_node')
        # ROS parameter for mgmt server URL (GET /getranking)
        self.declare_parameter('mgmt_url', 'http://localhost:8082/getranking')
        self.mgmt_url = self.get_parameter('mgmt_url').get_parameter_value().string_value
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
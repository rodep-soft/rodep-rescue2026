import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar.pyzbar import decode

class QRReader(Node):
    def __init__(self):
        super().__init__("qr_reader")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # print("Callback alive")
        # Convert YUYV → BGR
        try:
            yuyv = np.frombuffer(msg.data, dtype=np.uint8)
           
            yuyv = yuyv.reshape((msg.height, msg.width, 2))

            frame = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        except Exception as e:
            self.get_logger().error(f"Frame conversion error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # -------- PyZBar detection --------
        codes = decode(gray)

        if not codes:
            print("No QR detected")
            # return

        for c in codes:
            text = c.data.decode("utf-8")
            bbox = c.polygon
            print(f"[QR] {text}")
            print(f"[BBOX] {bbox}")

            # Draw box for debug
            pts = np.array([[p.x, p.y] for p in bbox], dtype=np.int32)
            cv2.polylines(frame, [pts], True, (0,255,0), 2)
            cv2.putText(frame, text, (pts[0][0], pts[0][1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # print("Before imshow")
        cv2.imshow("frame", frame)
        cv2.waitKey(1)
        cv2.imwrite('/root/frame.jpg', frame)


def main(args=None):
    rclpy.init(args=args)
    node = QRReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


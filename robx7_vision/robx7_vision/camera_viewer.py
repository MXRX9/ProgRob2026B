import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def annotate(frame_bgr, text_lines):
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(out, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
    cv2.line(out, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)
    cv2.circle(out, (cx, cy), 4, (0, 255, 255), 1)
    for i, line in enumerate(text_lines):
        cv2.putText(out, line, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return out


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('debug_topic', '/camera/image_debug')
        self.declare_parameter('snapshot_path', '')

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('info_topic').value
        debug_topic = self.get_parameter('debug_topic').value
        self.snapshot_path = self.get_parameter('snapshot_path').value

        self.bridge = CvBridge()
        self.frames = 0
        self.t_last = time.monotonic()
        self.fps = 0.0
        self.info_logged = False
        self.frame_id_logged = False

        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.pub_debug = self.create_publisher(Image, debug_topic, 10)

        self.get_logger().info(f'Esperando imagenes en {image_topic} y {info_topic} ...')

    def on_info(self, msg: CameraInfo):
        if self.info_logged:
            return
        self.info_logged = True
        k = np.array(msg.k).reshape(3, 3)
        self.get_logger().info(
            f'camera_info: {msg.width}x{msg.height}, frame_id="{msg.header.frame_id}"\n'
            f'  fx={k[0, 0]:.1f} fy={k[1, 1]:.1f} cx={k[0, 2]:.1f} cy={k[1, 2]:.1f}')

    def on_image(self, msg: Image):
        if not self.frame_id_logged:
            self.frame_id_logged = True
            self.get_logger().info(
                f'Primera imagen: {msg.width}x{msg.height}, encoding={msg.encoding}, '
                f'frame_id="{msg.header.frame_id}"')

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.frames += 1
        if self.frames % 15 == 0:
            now = time.monotonic()
            self.fps = 15.0 / (now - self.t_last)
            self.t_last = now

        h, w = frame.shape[:2]
        b, g, r = frame[h // 2, w // 2]
        lines = [f'{w}x{h}  {self.fps:4.1f} FPS',
                 f'pixel central BGR=({b},{g},{r})']
        debug = annotate(frame, lines)

        out = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        out.header = msg.header          
        self.pub_debug.publish(out)

        if self.snapshot_path and self.frames == 10:
            cv2.imwrite(self.snapshot_path, frame)
            self.get_logger().info(f'Imagen guardada en {self.snapshot_path}')


def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

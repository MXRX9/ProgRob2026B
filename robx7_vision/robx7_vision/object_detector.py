import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robx7_interfaces.msg import DetectedObject, DetectedObjectArray
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robx7_vision import detector_core as core

# Color con el que se dibuja cada clase en RViz (r, g, b entre 0 y 1)
RGB = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.1), 'blue': (0.2, 0.3, 1.0)}


def put_text(img, text, org, scale=0.55):
    """Texto blanco con borde negro (se lee sobre cualquier fondo)."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # ---- Parametros (se pueden cambiar desde el launch o con --ros-args -p nombre:=valor)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        # Se usa este marco para no depender de Gazebo
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('classify_by', 'color')          # variante A - 'color', B - 'shape'
        self.declare_parameter('object_height', 0.04)           # altura de los objetos [m]
        self.declare_parameter('min_area', 150.0)
        self.declare_parameter('max_area', 3000.0)
        self.declare_parameter('circularity_threshold', 0.82)
        # Zona de entrada (en 'world') lo que quede fuera se ignora
        self.declare_parameter('roi_x_min', 0.50)
        self.declare_parameter('roi_x_max', 0.90)
        self.declare_parameter('roi_y_min', -0.20)
        self.declare_parameter('roi_y_max', 0.20)
        self.declare_parameter('ground_truth_file', '')
        # Si camera_info no llega, calcular K con este campo de vision [rad] (0.0 = desactivado).
        self.declare_parameter('fallback_hfov', 0.0)

        gp = self.get_parameter
        self.image_topic = gp('image_topic').value
        self.info_topic = gp('info_topic').value
        self.camera_frame = gp('camera_frame').value
        self.target_frame = gp('target_frame').value
        self.classify_by = gp('classify_by').value
        self.obj_h = gp('object_height').value
        self.min_area = gp('min_area').value
        self.max_area = gp('max_area').value
        self.circ_thr = gp('circularity_threshold').value
        self.roi = (gp('roi_x_min').value, gp('roi_x_max').value,
                    gp('roi_y_min').value, gp('roi_y_max').value)
        self.fallback_hfov = gp('fallback_hfov').value
        self.ground_truth = self.load_ground_truth(gp('ground_truth_file').value)

        # ---- Estado interno
        self.bridge = CvBridge()        # convierte sensor_msgs/Image <-> arreglo de OpenCV
        self.K = None                   # matriz intrinseca (llega en camera_info)
        self.n_images = 0               # contadores para el diagnostico
        self.n_info = 0
        self.tf_ok = False
        self.n_objects = 0
        self.last_report = 0.0

        # ---- El buffer guarda las transformadas que escucha el listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- Entradas y salidas
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image,
                                 qos_profile_sensor_data)
        self.pub_objects = self.create_publisher(DetectedObjectArray, '/vision/objects', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/vision/markers', 10)
        self.pub_debug = self.create_publisher(Image, '/vision/image_debug', 10)

        # Cada 5 s dice que esta pasando
        self.create_timer(5.0, self.watchdog)

        self.get_logger().info(
            f'Detector listo. Clasificacion por "{self.classify_by}". '
            f'Esperando imagen en {self.image_topic} y camera_info en {self.info_topic}')

    # ------------------------------------------------------------------ utilidades
    def load_ground_truth(self, path):
        if not path:
            path = os.path.join(get_package_share_directory('robx7_vision'),
                                'config', 'ground_truth.yaml')
        try:
            with open(path) as f:
                return yaml.safe_load(f)['objects']
        except (OSError, KeyError, yaml.YAMLError) as e:
            self.get_logger().warn(f'Sin verdad de terreno ({e})')
            return []

    def in_roi(self, p):
        return self.roi[0] <= p[0] <= self.roi[1] and self.roi[2] <= p[1] <= self.roi[3]

    def on_info(self, msg: CameraInfo):
        self.n_info += 1
        if self.K is None or self.n_info == 1:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f'camera_info recibido: fx={self.K[0, 0]:.1f} fy={self.K[1, 1]:.1f} '
                f'cx={self.K[0, 2]:.1f} cy={self.K[1, 2]:.1f}')

    def watchdog(self):
        """Mensajes de diagnostico"""
        log = self.get_logger()
        if self.n_images == 0:
            log.warn(f'NO llegan imagenes en {self.image_topic}. Revisa el puente y la camara: '
                     f'ros2 topic hz {self.image_topic}')
        elif self.n_info == 0 and self.fallback_hfov <= 0.0:
            log.warn(f'Llegan imagenes pero NO camera_info en {self.info_topic}. '
                     f'Revisa: ign topic -l | grep camera  y  ros2 topic list | grep camera. '
                     f'Solucion rapida: -p fallback_hfov:=1.2')
        elif not self.tf_ok:
            log.warn(f'Falta la transformada {self.target_frame} <- {self.camera_frame}. '
                     f'Revisa: ros2 run tf2_ros tf2_echo {self.target_frame} {self.camera_frame}')
        else:
            log.info(f'Estado OK | imagenes={self.n_images} camera_info={self.n_info} '
                     f'| objetos en el ultimo cuadro={self.n_objects}')

    def publish_debug(self, frame, header, lines):
        """Publica la imagen de depuracion con lineas de texto en la esquina."""
        for i, line in enumerate(lines):
            put_text(frame, line, (10, 22 + 20 * i))
        img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img.header = header
        self.pub_debug.publish(img)

    # ------------------------------------------------------------------ callback principal
    def on_image(self, msg: Image):
        self.n_images += 1
        # sensor_msgs/Image -> arreglo NumPy de forma (alto, ancho, 3) en orden B, G, R
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Sin intrinsecos no se puede calcular nada 3D
        if self.K is None and self.fallback_hfov > 0.0:
            self.K = core.intrinsics_from_fov(msg.width, msg.height, self.fallback_hfov)
            self.get_logger().warn('Usando K calculada con fallback_hfov (no llego camera_info)')
        if self.K is None:
            self.publish_debug(frame, msg.header, ['ESPERANDO camera_info...'])
            return

        # Pose de la camara en 'world' (TF2). Time() = "la transformada mas reciente"
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, Time())
            self.tf_ok = True
        except TransformException as e:
            self.tf_ok = False
            self.get_logger().warn(f'Sin TF {self.target_frame}<-{self.camera_frame}: {e}',
                                   throttle_duration_sec=2.0)
            self.publish_debug(frame, msg.header, [f'SIN TF {self.target_frame}<-'
                                                   f'{self.camera_frame}'])
            return
        q, tr = tf.transform.rotation, tf.transform.translation
        R_wc = core.quat_to_matrix(q.x, q.y, q.z, q.w)
        t_wc = np.array([tr.x, tr.y, tr.z])

        # blobs de color -> forma
        blobs = core.find_blobs(frame, self.min_area, self.max_area)
        debug = frame.copy()

        # Pixel -> punto 3D (interseccion del rayo con el plano de la cara superior)
        detections = []
        for b in blobs:
            p = core.pixel_to_world(b.cx, b.cy, self.K, R_wc, t_wc, self.obj_h)
            if p is None or not self.in_roi(p):
                cv2.drawContours(debug, [b.contour], -1, (140, 140, 140), 1)   # descartado
                continue
            shape = core.classify_shape(b.circularity, self.circ_thr)
            label = b.color if self.classify_by == 'color' else shape
            detections.append((b, p, shape, label))
        detections.sort(key=lambda d: (d[0].color, d[1][1]))      # orden estable
        self.n_objects = len(detections)

        # Mensaje, TF, marcadores y depuracion
        out = DetectedObjectArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        markers = MarkerArray()
        tfs = []
        counts = {}
        for i, (b, p, shape, label) in enumerate(detections):
            k = counts.get(b.color, 0)
            counts[b.color] = k + 1
            obj_id = f'{b.color}_{k}'

            o = DetectedObject()
            o.id, o.label, o.color, o.shape = obj_id, label, b.color, shape
            o.area_px = float(b.area)
            o.pose.position.x, o.pose.position.y = float(p[0]), float(p[1])
            o.pose.position.z = self.obj_h / 2.0            # centro del objeto
            o.pose.orientation.w = 1.0                      # no se estima la orientacion
            out.objects.append(o)

            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = f'obj_{obj_id}'
            t.transform.translation.x = o.pose.position.x
            t.transform.translation.y = o.pose.position.y
            t.transform.translation.z = o.pose.position.z
            t.transform.rotation.w = 1.0
            tfs.append(t)

            markers.markers.extend(self.make_markers(i, o, out.header))

            cv2.drawContours(debug, [b.contour], -1, (255, 255, 255), 2)
            put_text(debug, f'{label} ({p[0]:.3f}, {p[1]:.3f})',
                     (int(b.cx) - 55, int(b.cy) - 22), 0.5)

        self.pub_objects.publish(out)
        if tfs:
            self.tf_broadcaster.sendTransform(tfs)
        self.pub_markers.publish(markers)

        # Se proyecta al pixel y se mide el error
        self.draw_ground_truth(debug, R_wc, t_wc)
        self.report_error(detections)

        self.publish_debug(debug, msg.header, [
            f'{len(detections)} objeto(s) | clase por: {self.classify_by}',
            'cruz magenta = posicion real (SDF)'])

    # ------------------------------------------------------------------ salidas
    def make_markers(self, idx, o, header):
        r, g, b = RGB[o.color]
        life = Duration(seconds=0.5).to_msg()

        body = Marker()
        body.header = header
        body.ns, body.id = 'objects', idx
        body.type = Marker.CYLINDER if o.shape == 'cylinder' else Marker.CUBE
        body.action = Marker.ADD
        body.pose = o.pose
        body.scale.x = body.scale.y = 0.04
        body.scale.z = 0.05 if o.shape == 'cylinder' else 0.04
        body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 0.8
        body.lifetime = life

        text = Marker()
        text.header = header
        text.ns, text.id = 'labels', idx
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = o.pose.position.x
        text.pose.position.y = o.pose.position.y
        text.pose.position.z = 0.10
        text.pose.orientation.w = 1.0
        text.scale.z = 0.035
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'{o.label}\n({o.pose.position.x:.3f}, {o.pose.position.y:.3f})'
        text.lifetime = life
        return [body, text]

    def draw_ground_truth(self, img, R_wc, t_wc):
        for gt in self.ground_truth:
            px = core.world_to_pixel([gt['x'], gt['y'], self.obj_h], self.K, R_wc, t_wc)
            if px is not None:
                u, v = int(round(px[0])), int(round(px[1]))
                cv2.drawMarker(img, (u, v), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 1)

    def report_error(self, detections):
        """Error de posicion [mm] frente a la verdad de terreno (cada 2 s)."""
        now = time.monotonic()
        if now - self.last_report < 2.0 or not detections:
            return
        self.last_report = now
        errs = []
        for b, p, _, _ in detections:
            cands = [g for g in self.ground_truth if g['color'] == b.color]
            if cands:
                e = min(math.hypot(p[0] - g['x'], p[1] - g['y']) for g in cands) * 1000.0
                errs.append((b.color, e))
        if errs:
            txt = ', '.join(f'{c}: {e:.1f} mm' for c, e in errs)
            mean = sum(e for _, e in errs) / len(errs)
            self.get_logger().info(
                f'{len(detections)} objeto(s) | error {txt} | media {mean:.1f} mm')


def main():
    rclpy.init()
    node = ObjectDetector()
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

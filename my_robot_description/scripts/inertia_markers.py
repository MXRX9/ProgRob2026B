#!/usr/bin/env python3
# Publica en /inertia_markers: una esfera en el centro de masa de cada link y una
# caja equivalente de inercia (la caja sólida que tendría el mismo tensor Ixx, Iyy, Izz).
import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class InertiaMarkers(Node):
    def __init__(self):
        super().__init__('inertia_markers')
        # robot_state_publisher publica /robot_description con durabilidad TRANSIENT_LOCAL,
        # así que nos suscribimos igual para recibir el mensaje aunque lleguemos tarde.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_urdf, qos)
        self.pub = self.create_publisher(MarkerArray, 'inertia_markers', 10)
        self.markers = MarkerArray()
        self.create_timer(1.0, self.publish)  # republica cada segundo

    def on_urdf(self, msg):
        root = ET.fromstring(msg.data)
        array = MarkerArray()
        for i, link in enumerate(root.findall('link')):
            inertial = link.find('inertial')
            if inertial is None:
                continue
            m = float(inertial.find('mass').get('value'))
            origin = inertial.find('origin')
            xyz = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            inr = inertial.find('inertia')
            ixx, iyy, izz = (float(inr.get(k)) for k in ('ixx', 'iyy', 'izz'))

            # Dimensiones de la caja equivalente (se despejan de las fórmulas de inercia)
            sx = math.sqrt(max(6.0 * (iyy + izz - ixx) / m, 1e-6))
            sy = math.sqrt(max(6.0 * (ixx + izz - iyy) / m, 1e-6))
            sz = math.sqrt(max(6.0 * (ixx + iyy - izz) / m, 1e-6))

            box = Marker()
            box.header.frame_id = link.get('name')  # el marcador se mueve con el link
            box.ns = 'caja_inercia'
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = xyz
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = sx, sy, sz
            box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.2, 0.2, 0.45
            array.markers.append(box)

            com = Marker()
            com.header.frame_id = link.get('name')
            com.ns = 'centro_de_masa'
            com.id = 1000 + i
            com.type = Marker.SPHERE
            com.action = Marker.ADD
            com.pose.position.x, com.pose.position.y, com.pose.position.z = xyz
            com.pose.orientation.w = 1.0
            com.scale.x = com.scale.y = com.scale.z = 0.03
            com.color.r, com.color.g, com.color.b, com.color.a = 0.0, 1.0, 0.0, 1.0
            array.markers.append(com)

        self.markers = array
        self.get_logger().info('URDF recibido: %d marcadores' % len(array.markers))

    def publish(self):
        self.pub.publish(self.markers)


def main():
    rclpy.init()
    rclpy.spin(InertiaMarkers())


if __name__ == '__main__':
    main()
    
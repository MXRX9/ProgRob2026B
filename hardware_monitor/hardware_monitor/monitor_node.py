"""
Nodo Monitor (Subscriber + Service Server).

- Se suscribe a /hardware_status y guarda el último estado recibido.
- Declara el parámetro estático 'umbral_temperatura' (por defecto 50),
  que se lee una sola vez durante la inicialización. Se marca como
  read_only para que no pueda cambiarse con 'ros2 param set'.
- Emite una advertencia en los logs cuando la temperatura supera el umbral.
- Expone el servicio /check_system (robx7_interfaces/srv/CheckSystem), que
  responde si el sistema es seguro para operar según el último estado.
"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from robx7_interfaces.msg import HardwareStatus
from robx7_interfaces.srv import CheckSystem


class MonitorNode(Node):
    """Supervisa el hardware y autoriza o bloquea la operación."""

    def __init__(self):
        super().__init__('monitor_node')

        # Parámetro estático: se declara y se lee solo aquí, en el arranque
        descriptor = ParameterDescriptor(
            description='Temperatura máxima segura en °C (solo lectura)',
            read_only=True)
        self.declare_parameter('umbral_temperatura', 50, descriptor)
        self.umbral = self.get_parameter('umbral_temperatura').value

        # Último estado conocido del hardware (None hasta recibir el primero)
        self.ultimo_estado = None

        self.subscription_ = self.create_subscription(
            HardwareStatus, 'hardware_status', self.callback_estado, 10)
        self.service_ = self.create_service(
            CheckSystem, 'check_system', self.callback_check_system)

        self.get_logger().info(
            f'Monitor iniciado. umbral_temperatura = {self.umbral} °C. '
            'Servicio /check_system disponible.')

    def callback_estado(self, msg):
        """Guarda cada lectura y advierte si se supera el umbral."""
        self.ultimo_estado = msg
        if msg.temperatura > self.umbral:
            self.get_logger().warn(
                f'Temperatura {msg.temperatura} °C supera el umbral de '
                f'{self.umbral} °C ({msg.debug_msg})')

    def callback_check_system(self, request, response):
        """Evalúa el último estado y responde si es seguro operar."""
        estado = self.ultimo_estado

        if estado is None:
            response.seguro = False
            response.temperatura = 0
            response.motores_listos = False
            response.mensaje = 'Bloqueo: aún no se reciben datos del hardware'
        else:
            response.temperatura = estado.temperatura
            response.motores_listos = estado.motores_listos
            if estado.temperatura > self.umbral:
                response.seguro = False
                response.mensaje = (
                    f'Bloqueo por temperatura alta ({estado.temperatura} °C '
                    f'> {self.umbral} °C)')
            elif not estado.motores_listos:
                response.seguro = False
                response.mensaje = (
                    f'Bloqueo: motores no listos ({estado.debug_msg})')
            else:
                response.seguro = True
                response.mensaje = 'Autorización de movimiento concedida'

        self.get_logger().info(
            f'Petición a /check_system -> seguro={response.seguro}: '
            f'{response.mensaje}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
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

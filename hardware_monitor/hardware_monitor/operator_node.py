"""
Nodo Operador (Service Client).

Actúa como supervisor de alto nivel: cada 'periodo_consulta' segundos llama
al servicio /check_system del Nodo Monitor e imprime en consola si se
concede o se bloquea el movimiento.

La llamada se hace de forma asíncrona (call_async) con un callback, para no
bloquear el executor mientras se espera la respuesta.
"""

import rclpy
from rclpy.node import Node

from robx7_interfaces.srv import CheckSystem


class OperatorNode(Node):
    """Consulta periódicamente el estado de seguridad del sistema."""

    def __init__(self):
        super().__init__('operator_node')

        self.declare_parameter('periodo_consulta', 3.0)
        periodo = self.get_parameter('periodo_consulta').value

        self.client_ = self.create_client(CheckSystem, 'check_system')
        self.peticion_pendiente = False
        self.timer_ = self.create_timer(periodo, self.consultar_sistema)

        self.get_logger().info(
            f'Operador iniciado. Consultando /check_system cada {periodo} s.')

    def consultar_sistema(self):
        """Callback del timer: envía una petición si el servicio existe."""
        if not self.client_.service_is_ready():
            self.get_logger().warn('Esperando al servicio /check_system...')
            return
        if self.peticion_pendiente:
            # Evita acumular peticiones si el servidor tarda en responder
            return

        self.peticion_pendiente = True
        future = self.client_.call_async(CheckSystem.Request())
        future.add_done_callback(self.callback_respuesta)

    def callback_respuesta(self, future):
        """Procesa e imprime la respuesta del Nodo Monitor."""
        self.peticion_pendiente = False
        try:
            respuesta = future.result()
        except Exception as error:  # noqa: B902
            self.get_logger().error(f'Fallo al llamar al servicio: {error}')
            return

        texto = (f'{respuesta.mensaje} | temp: {respuesta.temperatura} °C, '
                 f'motores_listos: {respuesta.motores_listos}')
        if respuesta.seguro:
            self.get_logger().info(f'[OK] {texto}')
        else:
            self.get_logger().warn(f'[BLOQUEO] {texto}')


def main(args=None):
    rclpy.init(args=args)
    node = OperatorNode()
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

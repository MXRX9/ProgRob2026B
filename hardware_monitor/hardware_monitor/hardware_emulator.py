"""
Nodo Emulador de Hardware (Publisher).

Publica periódicamente un mensaje robx7_interfaces/msg/HardwareStatus en el
tópico /hardware_status. Simula un ciclo térmico sencillo:

  NORMAL      -> la temperatura oscila en un rango seguro, motores listos.
  CALENTANDO  -> la temperatura sube de forma sostenida hasta un máximo.
  ENFRIANDO   -> se detienen los motores y la temperatura baja.

Además, con una probabilidad pequeña se simula un "Fallo de sensor".
"""

import random

import rclpy
from rclpy.node import Node

from robx7_interfaces.msg import HardwareStatus


class HardwareEmulator(Node):
    """Genera y publica lecturas simuladas del hardware del robot."""

    # Límites del ciclo térmico simulado (°C)
    TEMP_INICIAL = 40
    TEMP_INICIO_CALENTAMIENTO = 55
    TEMP_MAXIMA = 90
    TEMP_FIN_ENFRIAMIENTO = 45

    # Probabilidad de simular un fallo de sensor en cada publicación
    PROB_FALLO_SENSOR = 0.05

    def __init__(self):
        super().__init__('hardware_emulator')

        # Frecuencia de publicación configurable (por defecto 2 Hz)
        self.declare_parameter('frecuencia_hz', 2.0)
        frecuencia = self.get_parameter('frecuencia_hz').value

        self.publisher_ = self.create_publisher(
            HardwareStatus, 'hardware_status', 10)
        self.timer_ = self.create_timer(1.0 / frecuencia, self.publicar_estado)

        # Estado interno de la simulación
        self.temperatura = self.TEMP_INICIAL
        self.fase = 'NORMAL'
        self.ciclos_normal = 0

        self.get_logger().info(
            f'Emulador de hardware iniciado, publicando a {frecuencia} Hz '
            'en /hardware_status')

    def actualizar_simulacion(self):
        """Avanza un paso la máquina de estados térmica."""
        if self.fase == 'NORMAL':
            # Pequeñas variaciones alrededor de un valor seguro
            self.temperatura += random.randint(-1, 3)
            self.ciclos_normal += 1
            if (self.temperatura >= self.TEMP_INICIO_CALENTAMIENTO
                    or self.ciclos_normal > 20):
                self.fase = 'CALENTANDO'
                self.ciclos_normal = 0

        elif self.fase == 'CALENTANDO':
            # Subida sostenida de temperatura
            self.temperatura += random.randint(2, 4)
            if self.temperatura >= self.TEMP_MAXIMA:
                self.fase = 'ENFRIANDO'

        elif self.fase == 'ENFRIANDO':
            # Motores detenidos, la temperatura baja
            self.temperatura -= random.randint(3, 5)
            if self.temperatura <= self.TEMP_FIN_ENFRIAMIENTO:
                self.fase = 'NORMAL'

    def publicar_estado(self):
        """Callback del timer: construye y publica el mensaje."""
        self.actualizar_simulacion()

        msg = HardwareStatus()
        msg.temperatura = int(self.temperatura)

        if random.random() < self.PROB_FALLO_SENSOR:
            # Fallo de sensor: por seguridad se reportan motores no listos
            msg.motores_listos = False
            msg.debug_msg = 'Fallo de sensor'
        elif self.fase == 'NORMAL':
            msg.motores_listos = True
            msg.debug_msg = 'Operación normal'
        elif self.fase == 'CALENTANDO':
            msg.motores_listos = True
            msg.debug_msg = 'Calentando'
        else:
            msg.motores_listos = False
            msg.debug_msg = 'Enfriando: motores detenidos'

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publicado -> temp: {msg.temperatura} °C | '
            f'motores_listos: {msg.motores_listos} | {msg.debug_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = HardwareEmulator()
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

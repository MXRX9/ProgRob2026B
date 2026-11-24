"""
Launch hardware_system: arranca los tres nodos del sistema.

Uso:
    ros2 launch hardware_monitor hardware_system.launch.py

El parámetro 'umbral_temperatura' del Nodo Monitor se sobrescribe aquí
(75 °C) en lugar de usar el valor por defecto del código (50 °C).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Nodo 1: emulador de hardware (publisher de /hardware_status)
    emulador = Node(
        package='hardware_monitor',
        executable='hardware_emulator',
        name='hardware_emulator',
        output='screen',
        parameters=[{'frecuencia_hz': 2.0}],
    )

    # Nodo 2: monitor (subscriber + servidor de /check_system)
    monitor = Node(
        package='hardware_monitor',
        executable='monitor_node',
        name='monitor_node',
        output='screen',
        parameters=[{'umbral_temperatura': 75}],  # sobrescribe el 50 por defecto
    )

    # Nodo 3: operador (cliente de /check_system)
    operador = Node(
        package='hardware_monitor',
        executable='operator_node',
        name='operator_node',
        output='screen',
        parameters=[{'periodo_consulta': 3.0}],
    )

    return LaunchDescription([emulador, monitor, operador])

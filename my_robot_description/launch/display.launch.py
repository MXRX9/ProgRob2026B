# display.launch.py
# Visualiza el robot en RViz2 con deslizadores para mover las articulaciones.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'my_robot.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'display.rviz')

    # Ejecuta 'xacro <archivo>' y guarda el URDF resultante como texto
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Publica el TF de cada link y el topic /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Ventana con deslizadores: publica /joint_states con los ángulos elegidos
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Abre RViz2 cargando la configuración guardada (RobotModel + TF)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Dibuja masas e inercias como marcadores (para la captura de inercias)
    inertia_markers = Node(
        package='my_robot_description',
        executable='inertia_markers.py',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
        inertia_markers,
    ])
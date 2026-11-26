# gazebo.launch.py
# Abre Gazebo con un mundo vacío e inserta el robot desde /robot_description.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    xacro_file = os.path.join(pkg, 'urdf', 'my_robot.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1) Arranca Gazebo con un mundo vacío. '-r' = empieza a simular de inmediato
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2) Publica /robot_description (el nodo 'create' lo lee de ahí)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # 3) Inserta el robot en Gazebo desde el topic /robot_description
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot', '-z', '0.01'],
        output='screen',
    )

    return LaunchDescription([gazebo, robot_state_publisher, spawn])
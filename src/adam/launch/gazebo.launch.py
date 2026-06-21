import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('adam')
    urdf_file = os.path.join(pkg, 'urdf', 'adam.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        # Iniciar Gazebo com mundo vazio
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'empty_world.launch.py')
            )
        ),

        # Publicar robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Spawnar o robô no Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'adam'],
            output='screen',
        ),
    ])

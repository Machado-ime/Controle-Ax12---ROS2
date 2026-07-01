#!/usr/bin/env python3
"""
Marcha manual: você escolhe a coluna da matriz pelo slider e o robô REAL vai
para aquela pose, com o RViz espelhando a posição real (telemetria do
ax12_controller). Tudo numa máquina só (pensado para rodar na Raspberry Pi,
ou no PC com a OpenCR conectada, com os motores ligados).

Uso:
    ros2 launch ax12_control marcha_manual.launch.py                  # matriz otimizada
    ros2 launch ax12_control marcha_manual.launch.py matriz:=cin_inve
    ros2 launch ax12_control marcha_manual.launch.py device:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_ax12 = get_package_share_directory('ax12_control')
    adam_urdf = os.path.join(
        get_package_share_directory('adam_urdf'), 'urdf', 'adam_fixed.urdf')

    with open(adam_urdf, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'matriz',
            default_value='otimizada',
            description='Matriz de marcha (sem extensão): otimizada ou cin_inve.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Porta serial dos motores AX-12.',
        ),

        # Publica /tf e /robot_description a partir do URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Interface de hardware REAL — escreve nos motores e publica /joint_states
        # a partir da telemetria real (é isso que faz o RViz espelhar o robô).
        Node(
            package='ax12_control',
            executable='ax12_controller',
            parameters=[{'device': LaunchConfiguration('device')}],
            output='screen',
        ),

        # RViz com config pré-configurado para o Adam
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_ax12, 'adam.rviz')],
            output='screen',
        ),

        # Janela com slider/botões para escolher a coluna da matriz de marcha
        Node(
            package='ax12_control',
            executable='marcha_manual',
            parameters=[{'matriz': LaunchConfiguration('matriz')}],
            output='screen',
        ),
    ])

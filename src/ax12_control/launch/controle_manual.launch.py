#!/usr/bin/env python3
"""
Jog manual: sliders por junta comandam os motores reais e o RViz mostra a
posição real (via telemetria do ax12_controller) — tudo na mesma máquina
(pensado para rodar direto na Raspberry Pi, com os motores ligados).

Uso:
    ros2 launch ax12_control controle_manual.launch.py

    # Porta serial diferente do padrão, ou velocidade de jog mais lenta:
    ros2 launch ax12_control controle_manual.launch.py device:=/dev/ttyUSB0 velocidade:=0.5
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
            'device',
            default_value='/dev/ttyACM0',
            description='Porta serial dos motores AX-12.',
        ),
        DeclareLaunchArgument(
            'velocidade',
            default_value='1.0',
            description='Velocidade do jog em rad/s, enviada em cada comando de slider.',
        ),

        # Publica /tf e /robot_description a partir do URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Interface de hardware REAL — escreve nos motores e publica /joint_states
        # a partir da telemetria de verdade (é isso que faz o RViz espelhar o robô).
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

        # Janela com um slider por junta — cada slider publica /joint_trajectory
        Node(
            package='ax12_control',
            executable='controle_manual',
            parameters=[{'velocidade': LaunchConfiguration('velocidade')}],
            output='screen',
        ),
    ])

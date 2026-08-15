#!/usr/bin/env python3
"""
IK cartesiana do pé no robô REAL: roll central + X/Z de cada pé (sliders),
pitchs resolvidos por cinemática inversa com pé paralelo ao chão, RViz
espelhando a posição real via telemetria do ax12_controller.

Uso:
    ros2 launch ax12_control controle_pe.launch.py
    ros2 launch ax12_control controle_pe.launch.py matriz:=matriz_zmp velocidade:=0.3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_ax12 = get_package_share_directory('ax12_control')
    urdf_path = os.path.join(
        get_package_share_directory('adam_description'), 'urdf', 'adam_fixed.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'matriz',
            default_value='cin_inve_2',
            description='Matriz cuja COLUNA 1 define a postura base.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Porta serial dos motores AX-12.',
        ),
        DeclareLaunchArgument(
            'velocidade',
            default_value='0.5',
            description='Velocidade dos comandos em rad/s.',
        ),

        # Publica /tf e /robot_description a partir do URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Interface de hardware REAL — escreve nos motores e publica
        # /joint_states a partir da telemetria (RViz espelha o robô).
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

        # Janela de IK cartesiana (roll + X/Z por pé); recebe o URDF para a FK
        Node(
            package='ax12_control',
            executable='controle_pe',
            parameters=[{
                'matriz': LaunchConfiguration('matriz'),
                'velocidade': LaunchConfiguration('velocidade'),
                'robot_description': robot_description,
            }],
            output='screen',
        ),
    ])

#!/usr/bin/env python3
"""
Medidor de roll: pitchs fixos na coluna 1 da matriz + UM slider comandando as
4 juntas de roll juntas (cada uma com seu sinal), no robô REAL, com o RViz
espelhando a posição real via telemetria do ax12_controller.

Uso:
    ros2 launch ax12_control medir_roll.launch.py
    ros2 launch ax12_control medir_roll.launch.py matriz:=matriz_zmp velocidade:=0.3
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
            description='Matriz cuja COLUNA 1 define a postura base dos pitchs.',
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

        # Janela com o slider único de roll
        Node(
            package='ax12_control',
            executable='medir_roll',
            parameters=[{
                'matriz': LaunchConfiguration('matriz'),
                'velocidade': LaunchConfiguration('velocidade'),
            }],
            output='screen',
        ),
    ])

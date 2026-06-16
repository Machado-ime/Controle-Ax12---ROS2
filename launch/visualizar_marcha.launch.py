#!/usr/bin/env python3
"""
Sobe robot_state_publisher + visualizar_marcha + RViz2.

Uso:
    ros2 launch ax12_control visualizar_marcha.launch.py \
        urdf:=/caminho/para/adam.urdf \
        [matriz:=cin_inve] \
        [passo_s:=0.0]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory('ax12_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf',
            description='Caminho absoluto para o URDF do robô (ex.: /home/pi/adam.urdf).',
        ),
        DeclareLaunchArgument(
            'matriz',
            default_value='cin_inve',
            description='Arquivo de marcha sem extensão: cin_inve ou otimizada.',
        ),
        DeclareLaunchArgument(
            'passo_s',
            default_value='0.0',
            description='Tempo por passo em segundos. 0 = usa o valor do YAML.',
        ),

        # Publica /tf e /robot_description a partir do URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', LaunchConfiguration('urdf')]),
                    value_type=str,
                ),
            }],
        ),

        # Publica /joint_states ciclando pela matriz de marcha
        Node(
            package='ax12_control',
            executable='visualizar_marcha',
            parameters=[{
                'matriz': LaunchConfiguration('matriz'),
                'passo_s': LaunchConfiguration('passo_s'),
            }],
            output='screen',
        ),

        # RViz com config pré-configurado para o Adam
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'adam.rviz')],
            output='screen',
        ),
    ])

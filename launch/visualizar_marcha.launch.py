#!/usr/bin/env python3
"""
Sobe robot_state_publisher + visualizar_marcha + RViz2.

Uso:
    # Usa o URDF do pacote adam automaticamente:
    ros2 launch ax12_control visualizar_marcha.launch.py

    # Ou passe o arquivo de marcha e velocidade:
    ros2 launch ax12_control visualizar_marcha.launch.py matriz:=otimizada passo_s:=0.5
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
        get_package_share_directory('adam'), 'urdf', 'adam_fixed.urdf')

    with open(adam_urdf, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
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
            parameters=[{'robot_description': robot_description}],
        ),

        # Publica /joint_states ciclando pela matriz de marcha
        # (juntas ausentes na matriz são publicadas em 0 usando o URDF)
        Node(
            package='ax12_control',
            executable='visualizar_marcha',
            parameters=[{
                'matriz': LaunchConfiguration('matriz'),
                'passo_s': LaunchConfiguration('passo_s'),
                'robot_description': robot_description,
            }],
            output='screen',
        ),

        # RViz com config pré-configurado para o Adam
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_ax12, 'adam.rviz')],
            output='screen',
        ),

        # Slider Qt para selecionar o passo manualmente (modo passo_s=0)
        Node(
            package='ax12_control',
            executable='passo_slider',
            parameters=[{'matriz': LaunchConfiguration('matriz')}],
            output='screen',
        ),
    ])

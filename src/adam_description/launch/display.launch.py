#!/usr/bin/env python3
"""
Visualiza o modelo do Adam no RViz, em dois modos.

    ros2 launch adam_description display.launch.py                         # sliders manuais
    ros2 launch adam_description display.launch.py use_gui_sliders:=false  # espelha o robô real

Com `use_gui_sliders:=true` (padrão), o `joint_state_publisher_gui` publica
/joint_states a partir dos sliders — modelo sozinho, sem hardware.

Com `:=false`, nada aqui publica /joint_states: o modelo segue o que vier da
rede, tipicamente a telemetria real do `ax12_controller` rodando na Raspberry
Pi. É por isso que o slider é condicional — os dois publicando /joint_states
ao mesmo tempo fariam o RViz "tremer" entre a posição alvo e a real (mesma
decisão de design do `controle_manual`, ver docs/arquitetura.md).

Só as 10 juntas das pernas estão no barramento, então no modo espelho os 6
joints dos braços ficam sem estado (o RViz não desenha a TF deles).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('adam_description')
    urdf_file = os.path.join(pkg, 'urdf', 'adam_fixed.urdf')
    rviz_config = os.path.join(pkg, 'config', 'adam.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui_sliders',
            default_value='true',
            description='true = sliders do joint_state_publisher_gui; '
                        'false = espelha o robô real (/joint_states da rede).',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui_sliders')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])

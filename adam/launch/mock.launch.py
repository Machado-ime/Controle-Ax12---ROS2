"""
Digital twin do Adam com hardware mock (ros2_control + RViz).

Sobe o controller_manager com o mock_components/GenericSystem, o
robot_state_publisher, os spawners dos controllers e o RViz. Sem MoveIt
ainda (isso vem na Fase 2/3); aqui se valida o pipeline ros2_control.

    ros2 launch adam mock.launch.py

Para comandar uma junta (exemplo, perna direita):
    ros2 topic pub -1 /perna_direita_controller/joint_trajectory \\
      trajectory_msgs/msg/JointTrajectory \\
      '{joint_names: [pd_picht_quadril_7, pd_picht_joelho_5,
                      pd_picht_tornozelo_3, pd_roll_tornozelo_1],
        points: [{positions: [0.0, 0.5, 0.0, 0.0], time_from_start: {sec: 1}}]}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock_hardware')

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution(
                [FindPackageShare('adam'), 'urdf', 'adam.urdf.xacro']),
            ' use_mock_hardware:=', use_mock,
        ])
    }

    controllers = PathJoinSubstitution(
        [FindPackageShare('adam'), 'config', 'ros2_controllers.yaml'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('adam'), 'config', 'adam.rviz'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen',
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    # Publica posicao=0 para joints NAO controlados pelo ros2_control
    # (quadril roll + bracos). Lê os 8 joints controlados via source_list
    # e preenche os 8 restantes para o robot_state_publisher ter todos.
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ['/joint_states'], 'rate': 50.0}],
        output='screen',
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        # CycloneDDS evita incompatibilidade ABI entre libfastrtps 2.14.5 e libfastcdr 2.2.7
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='true',
            description='true = mock_components; false = driver AX-12 real (Fase 4)'),
        control_node,
        rsp_node,
        jsp_node,
        spawner('joint_state_broadcaster'),
        spawner('perna_direita_controller'),
        spawner('perna_esquerda_controller'),
        rviz_node,
    ])

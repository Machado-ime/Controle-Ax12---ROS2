"""
Digital twin completo: mock hardware + MoveIt2 + RViz com MotionPlanning.

Sobe em sequencia:
  1. controller_manager (mock_components/GenericSystem)
  2. robot_state_publisher
  3. joint_state_publisher (cobre as 8 juntas nao controladas)
  4. joint_state_broadcaster, perna_direita_controller, perna_esquerda_controller
  5. move_group (MoveIt2)
  6. RViz com o plugin MotionPlanning

    ros2 launch adam_moveit_config demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    adam_pkg = get_package_share_directory("adam")
    moveit_pkg = get_package_share_directory("adam_moveit_config")

    use_mock = LaunchConfiguration("use_mock_hardware")

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        os.path.join(adam_pkg, "urdf", "adam.urdf.xacro"),
        " use_mock_hardware:=", use_mock,
    ])
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder("adam", package_name="adam_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                    os.path.join(adam_pkg, "config", "ros2_controllers.yaml")],
        output="screen",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Publica posicao=0 para juntas NAO controladas (hip roll + bracos)
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"source_list": ["/joint_states"], "rate": 50.0}],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_config.joint_limits,
            {
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(moveit_pkg, "config", "moveit.rviz")],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    return LaunchDescription([
        # CycloneDDS evita incompatibilidade ABI do fastrtps
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description="true = mock; false = driver AX-12 real (Fase 4)"),
        control_node,
        rsp_node,
        jsp_node,
        spawner("joint_state_broadcaster"),
        spawner("perna_direita_controller"),
        spawner("perna_esquerda_controller"),
        move_group_node,
        rviz_node,
    ])

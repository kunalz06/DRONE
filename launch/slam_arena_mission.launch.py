#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    bridge_config = os.path.join(repo_root, "config", "arena_bridge_topics.yaml")
    slam_params = os.path.join(repo_root, "config", "slam_toolbox_arena.yaml")
    mission_script = os.path.join(repo_root, "scripts", "slam_boundary_mapping_mission.py")
    rviz_config = "/opt/ros/jazzy/share/slam_toolbox/config/slam_toolbox_default.rviz"

    use_rviz = LaunchConfiguration("use_rviz")

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py")
        ),
        launch_arguments={
            "bridge_name": "arena_bridge",
            "config_file": bridge_config,
            "use_composition": "False",
            "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    mission = ExecuteProcess(
        cmd=["python3", mission_script, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    rviz = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz map window",
            ),
            bridge_launch,
            slam_launch,
            mission,
            rviz,
        ]
    )

#!/usr/bin/env python3
"""
Hardware Mission Launch File
Launches the autonomous mission for hardware deployment with MAVROS.

Usage:
    ros2 launch hardware_mission.launch.py

Prerequisites:
    - MAVROS node running with Pixhawk connection
    - Sensors publishing on expected topics
    - Battery state available

Author: Drone Project Team
Version: 2.0.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Repository root
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    
    # Configuration files
    mission_params = os.path.join(repo_root, "config", "mission_params.yaml")
    
    # Launch arguments
    use_mavros_arg = DeclareLaunchArgument(
        "use_mavros",
        default_value="true",
        description="Launch MAVROS node"
    )
    
    use_safety_arg = DeclareLaunchArgument(
        "use_safety_manager",
        default_value="true",
        description="Launch safety manager node"
    )
    
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="Launch SLAM Toolbox for mapping"
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz for visualization (optional, for debugging)"
    )
    
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id",
        default_value="1",
        description="MAVLink system ID"
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyACM0:57600",
        description="FCU connection URL (serial or UDP)"
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        "gcs_url",
        default_value="",
        description="Ground control station URL (optional)"
    )
    
    # MAVROS launch (if enabled)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mavros"), "launch", "node.launch.py")
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
            "gcs_url": LaunchConfiguration("gcs_url"),
            "system_id": LaunchConfiguration("vehicle_id"),
            "namespace": "mavros",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_mavros")),
    )
    
    # SLAM Toolbox launch (if enabled)
    slam_params = os.path.join(repo_root, "config", "slam_toolbox_arena.yaml")
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": slam_params,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_slam")),
    )
    
    # Hardware mission node
    hardware_mission_node = Node(
        package="",  # Will be set when packaged
        executable="hardware_mission.py",
        name="hardware_mission",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            mission_params,
        ],
        arguments=["--mode", "hardware"],
    )
    
    # Safety manager node
    safety_manager_node = Node(
        package="",
        executable="safety_manager.py",
        name="safety_manager",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_safety_manager")),
    )
    
    # RViz (optional, for debugging)
    rviz_config = os.path.join(get_package_share_directory("slam_toolbox"), "config", "slam_toolbox_default.rviz")
    rviz_node = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config, "--ros-args", "-p", "use_sim_time:=false"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    
    return LaunchDescription([
        # Arguments
        use_mavros_arg,
        use_safety_arg,
        use_slam_arg,
        use_rviz_arg,
        vehicle_id_arg,
        fcu_url_arg,
        gcs_url_arg,
        # Nodes
        mavros_launch,
        slam_launch,
        hardware_mission_node,
        safety_manager_node,
        rviz_node,
    ])

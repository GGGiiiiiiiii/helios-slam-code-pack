"""仿真 playground 场地 + 预建地图 AMCL + Nav2 导航。

用法：
  ros2 launch helios_gazebo sim_playground_nav.launch.py

可选：
  ros2 launch helios_gazebo sim_playground_nav.launch.py \\
    map:=$(ros2 pkg prefix helios_bringup)/share/helios_bringup/maps/playground.yaml

RViz：先用 2D Pose Estimate 设初始位姿，再用 Nav2 Goal 发目标。
依赖 ~/.gazebo/models/ 中的 playground、pine_tree 等模型（linorobot2 或 Gazebo Fuel）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("helios_gazebo")
    pkg_bringup = get_package_share_directory("helios_bringup")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    nav2_params = os.path.join(pkg_bringup, "config", "nav2_params.yaml")
    default_world = os.path.join(pkg_gazebo, "worlds", "playground.world")
    default_map = os.path.join(pkg_bringup, "maps", "playground.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "merge": "true",
            "world": world,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "params_file": nav2_params,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
    )

    delayed = TimerAction(period=8.0, actions=[localization, navigation])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Gazebo world 文件"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="预建地图 yaml（AMCL）"),
        sim,
        delayed,
    ])

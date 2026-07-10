"""建图：bringup + slam_toolbox（async 在线建图）。

职责分工（真机）：
  - sr_amr_control ：雷达、里程计、odom -> base_footprint TF
  - helios_bringup ：仅预处理 SDK 雷达 -> /scan；不发底盘动态 TF
  - slam_toolbox   ：建图并发布 map -> odom

用法见文末「真机启动顺序」。
建图完成后保存地图：
  ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("helios_bringup")
    slam_params = os.path.join(pkg, "config", "slam_toolbox.yaml")

    merge = LaunchConfiguration("merge")

    return LaunchDescription([
        DeclareLaunchArgument("merge", default_value="false",
                              description="是否合并前后雷达（建图建议先用单雷达验证）"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "bringup.launch.py")),
            launch_arguments={
                "merge": merge,
                "relay_cmd_vel": "true",
                "use_odom": "false",
                "use_ekf": "false",
            }.items(),
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_params],
                ),
            ],
        ),
    ])

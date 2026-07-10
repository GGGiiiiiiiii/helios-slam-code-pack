"""仿真 + SLAM 一键建图测试。

包含：
- sim.launch.py（Gazebo + 机器人 + 雷达预处理 + RViz）
- slam_toolbox（在线异步建图，用 helios_bringup/config/slam_toolbox.yaml）

用法：
  ros2 launch helios_gazebo sim_slam.launch.py
另开终端遥控走一圈建图：
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
建好后保存地图：
  ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_sim
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
    slam_params = os.path.join(pkg_bringup, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

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

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )
    delayed_slam = TimerAction(period=8.0, actions=[slam])

    default_world = os.path.join(pkg_gazebo, "worlds", "minimal.world")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Gazebo world 文件"),
        sim,
        delayed_slam,
    ])

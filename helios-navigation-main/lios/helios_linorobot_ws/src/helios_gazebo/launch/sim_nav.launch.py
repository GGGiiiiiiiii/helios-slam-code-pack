"""仿真 + SLAM + Nav2 全栈，RViz 里点目标点自主导航测试。

包含：
- sim.launch.py（Gazebo + 机器人 + 雷达预处理 + RViz）
- slam_toolbox（在线建图，边走边建）
- nav2（用 nav2_bringup/navigation_launch.py，自动注入 use_sim_time=true）

用法：
  ros2 launch helios_gazebo sim_nav.launch.py
在 RViz 里用工具栏 **Nav2 Goal**（nav2_rviz_plugins/GoalTool）点目标点；
不要用默认 SetGoal（/goal_pose），Nav2 不会响应。
Fixed Frame 应为 map。

说明：用 SLAM 在线建图代替 amcl，无需预先地图；
要测 amcl+已知地图，改用 helios_bringup/launch/navigation.launch.py（真机流程）。
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

    slam_params = os.path.join(pkg_bringup, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(pkg_bringup, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "sim.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time, "merge": "true"}.items(),
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
    )

    delayed = TimerAction(period=8.0, actions=[slam, nav2])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        sim,
        delayed,
    ])

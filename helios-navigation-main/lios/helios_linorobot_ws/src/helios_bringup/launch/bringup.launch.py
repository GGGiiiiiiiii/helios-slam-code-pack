"""底盘对接层 bringup（仅做 SDK 数据预处理，不重复发布底盘 TF）。

启动：
- robot_state_publisher：URDF 静态链 base_footprint -> base_link -> 雷达
- laser_preprocess     ：订阅 SDK /sr_amr_control/{front,rear}/scan，过滤/合并 -> /scan
- cmd_vel_relay        ：/cmd_vel -> /sr_amr_control/remote_control_cmd_vel
- odom_publisher / EKF ：仅仿真或 SDK 不可用时启用（真机默认关闭，避免与 SDK 抢 TF）

真机里程计与动态 TF（odom -> base_footprint）由 sr_amr_control 独占发布。
helios_bringup 不再重复发 odom TF；建图时 map -> odom 由 slam_toolbox 独占。

不包含 SLAM / 导航，供建图或导航 launch 复用。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("helios_bringup")
    pkg_desc = get_package_share_directory("helios_description")
    urdf = os.path.join(pkg_desc, "urdf", "helios.urdf.xacro")

    merge = LaunchConfiguration("merge")
    use_static_tf = LaunchConfiguration("use_static_tf")
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")
    relay_cmd_vel = LaunchConfiguration("relay_cmd_vel")
    use_odom = LaunchConfiguration("use_odom")
    use_ekf = LaunchConfiguration("use_ekf")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ekf_params = os.path.join(pkg, "config", "ekf.yaml")
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("merge", default_value="false",
                              description="是否合并前后雷达为单个 /scan"),
        DeclareLaunchArgument("use_static_tf", default_value="false",
                              description="是否发布占位雷达 TF（与 URDF 二选一）"),
        DeclareLaunchArgument("use_robot_state_publisher", default_value="true",
                              description="是否用 URDF 发布 base->雷达 TF"),
        DeclareLaunchArgument("relay_cmd_vel", default_value="true",
                              description="是否启用 cmd_vel 中继到底盘"),
        DeclareLaunchArgument("use_odom", default_value="false",
                              description="是否启用本地 odom 积分（真机请 false，TF 由 SDK 发布）"),
        DeclareLaunchArgument("use_ekf", default_value="false",
                              description="是否启用 EKF 融合原生 /sr_amr_control/odom + IMU（真机推荐）"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="仿真时设为 true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            condition=IfCondition(use_robot_state_publisher),
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "static_tf.launch.py")),
            condition=IfCondition(use_static_tf),
        ),

        Node(
            package="helios_bringup",
            executable="laser_preprocess",
            name="laser_preprocess",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "front_topic": "/sr_amr_control/front/scan",
                "rear_topic": "/sr_amr_control/rear/scan",
                "output_topic": "/scan",
                "merge": merge,
                "target_frame": "base_footprint",
                "invalid_value": 101.0,
                "range_min": 0.05,
                "range_max": 25.0,
                "stamp_now": True,
            }],
        ),

        Node(
            package="helios_bringup",
            executable="odom_publisher",
            name="odom_publisher",
            output="screen",
            condition=IfCondition(use_odom),
            parameters=[{
                "use_sim_time": use_sim_time,
                "system_state_topic": "/sr_amr_control/system_state",
                "odom_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "publish_tf": True,
            }],
        ),

        Node(
            package="helios_bringup",
            executable="cmd_vel_relay",
            name="cmd_vel_relay",
            output="screen",
            condition=IfCondition(relay_cmd_vel),
            parameters=[{
                "use_sim_time": use_sim_time,
                "in_topic": "/cmd_vel",
                "out_topic": "/sr_amr_control/remote_control_cmd_vel",
                "enable_on_start": True,
                "oba_on_start": True,
            }],
        ),

        # 真机推荐：EKF 融合原生 odom + IMU，发布 odom->base_footprint TF。
        # 启用时请把 use_odom 设为 false，避免 TF 双发布者冲突。
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            condition=IfCondition(use_ekf),
            parameters=[ekf_params, {"use_sim_time": use_sim_time}],
        ),
    ])

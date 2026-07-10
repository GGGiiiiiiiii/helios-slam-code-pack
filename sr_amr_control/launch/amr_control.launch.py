from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  declared_args = [
    DeclareLaunchArgument(
      'connect_ip',
    ),
    DeclareLaunchArgument('connect_username', default_value=''),
    DeclareLaunchArgument('connect_passwd', default_value=''),
    DeclareLaunchArgument(
      'frame_id',
      default_value='base_footprint',
      description='odom 子坐标系，与 helios URDF / slam 一致',
    ),
    DeclareLaunchArgument('parent_frame_id', default_value='map'),
    DeclareLaunchArgument('odom_frame_id', default_value='odom'),
    DeclareLaunchArgument('odom_topic', default_value='odom'),
    DeclareLaunchArgument('odom_use_integration', default_value='true'),
    DeclareLaunchArgument('lidar_points_frame_id', default_value='map'),
    DeclareLaunchArgument(
      'lidar_front_frame_id', default_value='right_front_laser_link'
    ),
    DeclareLaunchArgument(
      'lidar_rear_frame_id', default_value='left_behind_laser_link'
    ),
    DeclareLaunchArgument('lidar', default_value='false'),
    DeclareLaunchArgument('move_to_station_no_rotate', default_value='true'),
    DeclareLaunchArgument(
      'publish_tf',
      default_value='true',
      description='发布 odom->base_footprint TF（helios 侧不再重复发布）',
    ),
    DeclareLaunchArgument(
      'publish_map_to_odom',
      default_value='true',
      description='建图时请 false，由 slam_toolbox 独占 map->odom',
    ),
  ]

  connect_ip = LaunchConfiguration('connect_ip')
  connect_username = LaunchConfiguration('connect_username')
  connect_passwd = LaunchConfiguration('connect_passwd')
  frame_id = LaunchConfiguration('frame_id')
  parent_frame_id = LaunchConfiguration('parent_frame_id')
  odom_frame_id = LaunchConfiguration('odom_frame_id')
  odom_topic = LaunchConfiguration('odom_topic')
  odom_use_integration = ParameterValue(
    LaunchConfiguration('odom_use_integration'), value_type=bool
  )
  lidar_points_frame_id = LaunchConfiguration('lidar_points_frame_id')
  lidar_front_frame_id = LaunchConfiguration('lidar_front_frame_id')
  lidar_rear_frame_id = LaunchConfiguration('lidar_rear_frame_id')
  lidar = ParameterValue(LaunchConfiguration('lidar'), value_type=bool)
  move_to_station_no_rotate = ParameterValue(
    LaunchConfiguration('move_to_station_no_rotate'), value_type=bool
  )
  publish_tf = ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool)
  publish_map_to_odom = ParameterValue(
    LaunchConfiguration('publish_map_to_odom'), value_type=bool
  )

  control_node = Node(
    package='sr_amr_control',
    executable='control_node',
    namespace='sr_amr_control',
    output='screen',
    parameters=[
      {
        'connect_ip': connect_ip,
        'connect_username': connect_username,
        'connect_passwd': connect_passwd,
        'frame_id': frame_id,
        'parent_frame_id': parent_frame_id,
        'odom_frame_id': odom_frame_id,
        'odom_topic': odom_topic,
        'odom_use_integration': odom_use_integration,
        'lidar_points_frame_id': lidar_points_frame_id,
        'lidar_front_frame_id': lidar_front_frame_id,
        'lidar_rear_frame_id': lidar_rear_frame_id,
        'lidar': lidar,
        'move_to_station_no_rotate': move_to_station_no_rotate,
        'publish_tf': publish_tf,
        'publish_map_to_odom': publish_map_to_odom,
      }
    ],
  )

  return LaunchDescription(declared_args + [control_node])

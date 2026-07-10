import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from sros_sdk_py import SrpClient
from sros_sdk_py.main_pb2 import (
  HardwareState as HardwareStateProtocol,
)
from sros_sdk_py.main_pb2 import (
  MovementTask as MovementTaskProtocol,
)
from sros_sdk_py.main_pb2 import (
  SystemState as SystemStateProtocol,
)
from tf2_ros import TransformBroadcaster, TransformStamped

from sr_amr_interfaces.msg import (
  BatteryState as BatteryStateMsg,
)
from sr_amr_interfaces.msg import (
  SystemState as SystemStateMsg,
)

from .estop_srv import EStopStateChecker
from .remote_control import RemoteControlStateChecker
from .utils.ros_bridge import srp_to_ros_unit, to_ros_pose


class MovementTaskStateCtrl:
  @staticmethod
  def is_moving(task: MovementTaskProtocol) -> bool:
    return task.state not in [
      MovementTaskProtocol.MT_NA,
      MovementTaskProtocol.MT_FINISHED,
    ]


class StatePublisher:
  """把 SROS SystemState 转成 ROS 话题，并发布 odom→base_footprint TF。

  真机建图时的关键职责：
  - 独占发布 TF：odom → base_footprint（frame_id 常设为 base_footprint）
  - 发布话题：/sr_amr_control/odom
  - 建图时通常 publish_map_to_odom=false，把 map→odom 留给 slam_toolbox
  """

  _node: Node
  _srp_client: SrpClient
  _sys_state_publisher: Publisher
  _battery_state_publisher: Publisher
  _odom_publisher: Publisher
  # 坐标系命名（launch 传入）：
  #   _frame_id        = 子坐标系，真机建图常用 "base_footprint"
  #   _odom_frame_id   = 父坐标系，固定 "odom"
  #   _parent_frame_id = map 系，仅在发 map→odom 时用到
  _frame_id: str
  _parent_frame_id: str
  _odom_frame_id: str
  _odom_topic: str
  _remote_control_state_checker: RemoteControlStateChecker
  _tf_broadcaster: TransformBroadcaster
  # 当前里程计位姿（在 odom 坐标系下）：车在哪、朝哪
  _odom_x: float
  _odom_y: float
  _odom_yaw: float
  _last_odom_time_ns: int | None
  _odom_initialized: bool
  # True：用 vx/vy/w 积分推算位姿；False：直接用 SROS location_pose
  _odom_use_integration: bool
  _publish_odom: bool
  # True 才会发 map→odom；建图时必须 False，避免和 slam 抢 TF
  _publish_map_to_odom: bool
  # True 才会真正 sendTransform；关掉则只算位姿、不发 TF
  _enable_tf_publish: bool
  # 缓存最近一次 TF，供 50Hz 定时器反复刷新时间戳后重发
  _last_odom_to_base_tf: TransformStamped | None
  _last_map_to_odom_tf: TransformStamped | None
  # 最近一次车体系速度（m/s, rad/s），供高频积分使用
  _last_vx: float
  _last_vy: float
  _last_w: float

  def __init__(
    self,
    node: Node,
    client: SrpClient,
    frame_id: str,
    parent_frame_id: str,
    odom_frame_id: str,
    odom_topic: str,
    odom_use_integration: bool,
    publish_map_to_odom: bool = True,
    publish_tf: bool = True,
    publish_odom: bool = True,
  ):
    self._node = node
    self._srp_client = client
    # 真机建图 launch 典型值：
    #   frame_id=base_footprint, odom_frame_id=odom, parent_frame_id=map
    self._frame_id = frame_id
    self._parent_frame_id = parent_frame_id
    self._odom_frame_id = odom_frame_id
    self._odom_topic = odom_topic
    self._odom_use_integration = odom_use_integration
    self._publish_odom = publish_odom
    self._publish_map_to_odom = publish_map_to_odom
    self._enable_tf_publish = publish_tf
    self._remote_control_state_checker = RemoteControlStateChecker(client)
    # TF 广播器：最终靠它发出 odom→base_footprint
    self._tf_broadcaster = TransformBroadcaster(node)
    self._odom_x = 0.0
    self._odom_y = 0.0
    self._odom_yaw = 0.0
    self._last_odom_time_ns = None
    self._odom_initialized = False
    self._last_odom_to_base_tf = None
    self._last_map_to_odom_tf = None
    self._last_vx = 0.0
    self._last_vy = 0.0
    self._last_w = 0.0

    self._sys_state_publisher = self._node.create_publisher(
      SystemStateMsg, 'system_state', 10
    )
    self._battery_state_publisher = self._node.create_publisher(
      BatteryStateMsg, 'battery_state', 10
    )
    # 标准里程计话题（相对命名空间后通常是 /sr_amr_control/odom）
    self._odom_publisher = self._node.create_publisher(Odometry, self._odom_topic, 10)

    # SROS 推送 SystemState 时回调 publish_sys_state（约 3.33Hz）
    self._srp_client.add_system_state_callback(self.publish_sys_state)
    self._srp_client.add_hardware_state_callback(self.publish_battery_state)

    # 开启 odom+TF 时：先发一帧单位变换，再 50Hz 持续刷新 odom→base
    if self._publish_odom and self._enable_tf_publish:
      self._bootstrap_odom_tf()
      # 0.02s = 50Hz，比雷达(~5Hz)和 system_state(~3Hz) 更密，
      # 保证 slam 按任意 scan 时间戳都能查到附近的 odom TF
      self._node.create_timer(0.02, self._integrate_and_publish_odom)

  def _integrate_odom_to(self, target_time_ns: int) -> None:
    """把里程计位姿从上次时刻积分到 target_time_ns。

    全向底盘：车体系速度 (vx, vy) 先旋到 odom 系，再乘 dt 累加位移；
    w 直接累加航向。公式：
      dx = (vx*cos(yaw) - vy*sin(yaw)) * dt
      dy = (vx*sin(yaw) + vy*cos(yaw)) * dt
      dyaw = w * dt
    """
    if not self._odom_initialized or self._last_odom_time_ns is None:
      return
    if target_time_ns <= self._last_odom_time_ns:
      return

    dt = (target_time_ns - self._last_odom_time_ns) / 1e9
    cos_yaw = math.cos(self._odom_yaw)
    sin_yaw = math.sin(self._odom_yaw)
    # 车体系 → odom 系旋转后积分
    self._odom_x += (self._last_vx * cos_yaw - self._last_vy * sin_yaw) * dt
    self._odom_y += (self._last_vx * sin_yaw + self._last_vy * cos_yaw) * dt
    self._odom_yaw += self._last_w * dt
    self._last_odom_time_ns = target_time_ns

  def _integrate_and_publish_odom(self):
    """50Hz 定时器：积分位姿 → 更新缓存 TF → sendTransform(odom→base)。

    这是高频发布 odom→base_footprint 的主路径。
    system_state 回调也会更新位姿，但频率较低；这里用最近速度填补中间时刻。
    """
    if not self._enable_tf_publish or self._last_odom_to_base_tf is None:
      return

    now = self._node.get_clock().now()
    now_ns = now.nanoseconds

    # 积分模式：用缓存速度把位姿推到当前时刻
    if self._odom_use_integration and self._odom_initialized:
      self._integrate_odom_to(now_ns)

    # ---- 组装 / 刷新 odom→base_footprint ----
    stamp = now.to_msg()
    self._last_odom_to_base_tf.header.stamp = stamp
    # parent=odom, child=base_footprint：表示“车在 odom 系下的位姿”
    self._last_odom_to_base_tf.transform.translation.x = self._odom_x
    self._last_odom_to_base_tf.transform.translation.y = self._odom_y
    qx, qy, qz, qw = self._quaternion_from_yaw(self._odom_yaw)
    self._last_odom_to_base_tf.transform.rotation.x = qx
    self._last_odom_to_base_tf.transform.rotation.y = qy
    self._last_odom_to_base_tf.transform.rotation.z = qz
    self._last_odom_to_base_tf.transform.rotation.w = qw

    if self._last_map_to_odom_tf is not None:
      self._last_map_to_odom_tf.header.stamp = stamp

    # 真正广播 TF（内部会 sendTransform）
    self._send_cached_tf()

    # 同步发 nav_msgs/Odometry 话题（内容和 TF 位姿一致）
    odom_msg = Odometry()
    odom_msg.header.stamp = stamp
    odom_msg.header.frame_id = self._odom_frame_id      # "odom"
    odom_msg.child_frame_id = self._frame_id            # "base_footprint"
    odom_msg.pose.pose.position.x = self._odom_x
    odom_msg.pose.pose.position.y = self._odom_y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.x = qx
    odom_msg.pose.pose.orientation.y = qy
    odom_msg.pose.pose.orientation.z = qz
    odom_msg.pose.pose.orientation.w = qw
    odom_msg.twist.twist.linear.x = self._last_vx
    odom_msg.twist.twist.linear.y = self._last_vy
    odom_msg.twist.twist.angular.z = self._last_w
    self._odom_publisher.publish(odom_msg)

  def _bootstrap_odom_tf(self):
    """上电立刻发一帧单位变换 odom→base，避免首帧 system_state 到来前 TF 断链。

    单位变换 = 平移(0,0,0) + 四元数(0,0,0,1)，表示“车还在 odom 原点”。
    slam / laser_preprocess 启动早期查 odom→base 时就不会失败。
    """
    self._odom_initialized = True
    self._last_odom_time_ns = self._node.get_clock().now().nanoseconds
    now = self._node.get_clock().now()

    odom_to_base = TransformStamped()
    odom_to_base.header.stamp = now.to_msg()
    # ★ 这里就是 odom → base_footprint 的父子关系定义
    odom_to_base.header.frame_id = self._odom_frame_id  # parent: "odom"
    odom_to_base.child_frame_id = self._frame_id        # child:  "base_footprint"
    # translation 默认 0；只设 w=1 表示无旋转
    odom_to_base.transform.rotation.w = 1.0

    self._last_odom_to_base_tf = odom_to_base
    self._send_cached_tf()

  def _send_cached_tf(self):
    """把缓存的 TransformStamped 真正广播出去。

    必发：odom→base_footprint（_last_odom_to_base_tf）
    可选：map→odom（仅 _publish_map_to_odom=True 时）
    """
    if not self._enable_tf_publish or self._last_odom_to_base_tf is None:
      return

    # 至少发 odom→base
    transforms = [self._last_odom_to_base_tf]
    # 建图时 publish_map_to_odom=false，这里不会插入 map→odom
    if self._publish_map_to_odom and self._last_map_to_odom_tf is not None:
      transforms.insert(0, self._last_map_to_odom_tf)
    # ★ TF 真正发出的唯一出口
    self._tf_broadcaster.sendTransform(transforms)

  def _srp_location_state_to_msg_location_state(
    self, location_state: SystemStateProtocol.LocationState
  ) -> int:
    State = SystemStateProtocol.LocationState

    match location_state:
      case State.LOCATION_STATE_ZERO:
        return SystemStateMsg.LOCATION_STATE_ZERO
      case State.LOCATION_STATE_NONE:
        return SystemStateMsg.LOCATION_STATE_NONE
      case State.LOCATION_STATE_INITIALING:
        return SystemStateMsg.LOCATION_STATE_INITIALING
      case State.LOCATION_STATE_RUNNING:
        return SystemStateMsg.LOCATION_STATE_RUNNING
      case State.LOCATION_STATE_RELOCATING:
        return SystemStateMsg.LOCATION_STATE_RELOCATING
      case State.LOCATION_STATE_ERROR:
        return SystemStateMsg.LOCATION_STATE_ERROR

    return SystemStateMsg.LOCATION_STATE_UNKNOWN

  def publish_sys_state(self, system_state: SystemStateProtocol):
    if system_state is None:
      return

    current_pose = to_ros_pose(system_state.location_pose)

    msg = SystemStateMsg()
    msg.header.stamp = self._node.get_clock().now().to_msg()
    msg.header.frame_id = self._parent_frame_id

    msg.current_pose = current_pose
    msg.current_pose.header = msg.header

    msg.remote_control_active = (
      self._remote_control_state_checker.get_remote_control_status()
    )
    msg.remote_control_oba_active = (
      self._remote_control_state_checker.get_remote_control_oba_status()
    )
    msg.estop_active = EStopStateChecker.is_emergency_stopped(system_state)

    msg.current_map_name = system_state.map_name
    msg.current_station_id = system_state.station_no

    msg.location_confidence_score = system_state.location_pose.confidence
    msg.location_state = self._srp_location_state_to_msg_location_state(
      system_state.location_state
    )

    # SROS 速度单位：v_x/v_y=mm/s，w=mrad/s → 除以 1000 变成 m/s、rad/s
    mc_state = system_state.mc_state
    msg.linear_velocity_x = srp_to_ros_unit(mc_state.v_x)
    msg.linear_velocity_y = srp_to_ros_unit(mc_state.v_y)
    msg.angular_velocity = srp_to_ros_unit(mc_state.w)

    msg.executing_movement_task = MovementTaskStateCtrl.is_moving(
      system_state.movement_state
    )
    msg.movement_task_manual_paused = (
      system_state.sys_state == SystemStateProtocol.SYS_STATE_TASK_MANUAL_PAUSED
    )
    msg.movement_task_obstacle_paused = (
      system_state.sys_state == SystemStateProtocol.SYS_STATE_TASK_NAV_PAUSED
    )

    if rclpy.ok():
      try:
        # 缓存速度，供 50Hz 定时器在两次 system_state 之间做积分
        self._last_vx = msg.linear_velocity_x
        self._last_vy = msg.linear_velocity_y
        self._last_w = msg.angular_velocity
        self._sys_state_publisher.publish(msg)
        # 用最新 location_pose + 速度，更新 odom 位姿并（可选）发 TF
        self._publish_tf(
          system_state.location_pose,
          msg.linear_velocity_x,
          msg.linear_velocity_y,
          msg.angular_velocity,
        )
      except Exception as e:
        self._node.get_logger().error(f'Publish failed: {e}')
    else:
      self._node.get_logger().debug(
        'Skipping publish: ROS2 context is invalid (shutting down)'
      )

  def _battery_state_to_msg_battery_state(
    self, battery_state: HardwareStateProtocol.BatteryState
  ) -> int:
    State = HardwareStateProtocol.BatteryState

    match battery_state:
      case State.BATTERY_NA:
        return BatteryStateMsg.STATE_NA
      case State.BATTERY_CHARGING:
        return BatteryStateMsg.STATE_CHARING
      case State.BATTERY_NO_CHARGING:
        return BatteryStateMsg.STATE_NO_CHARING

    return BatteryStateMsg.STATE_UNKNOWN

  def publish_battery_state(self, hardware_state: HardwareStateProtocol):
    if hardware_state is None:
      return

    msg = BatteryStateMsg()

    msg.remaining_percentage = hardware_state.battery_percentage
    msg.remaining_capacity = hardware_state.battery_remain_capacity
    msg.remaining_time = hardware_state.battery_remain_time

    msg.state = self._battery_state_to_msg_battery_state(hardware_state.battery_state)

    msg.voltage = hardware_state.battery_voltage
    msg.current = hardware_state.battery_current
    msg.temperature = hardware_state.battery_temperature

    msg.nominal_capacity = hardware_state.battery_nominal_capacity
    msg.use_cycles = hardware_state.battery_use_cycles

    msg.sn = hardware_state.battery_sn

    if rclpy.ok():
      try:
        self._battery_state_publisher.publish(msg)
      except Exception as e:
        self._node.get_logger().error(f'Publish failed: {e}')
    else:
      self._node.get_logger().debug(
        'Skipping publish: ROS2 context is invalid (shutting down)'
      )

  def _yaw_from_quaternion(self, x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

  def _quaternion_from_yaw(self, yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

  def _compose_2d(
    self,
    x1: float,
    y1: float,
    yaw1: float,
    x2: float,
    y2: float,
    yaw2: float,
  ) -> tuple[float, float, float]:
    cos_yaw = math.cos(yaw1)
    sin_yaw = math.sin(yaw1)
    x = x1 + cos_yaw * x2 - sin_yaw * y2
    y = y1 + sin_yaw * x2 + cos_yaw * y2
    return (x, y, yaw1 + yaw2)

  def _inverse_2d(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    inv_x = -(cos_yaw * x + sin_yaw * y)
    inv_y = -(-sin_yaw * x + cos_yaw * y)
    return (inv_x, inv_y, -yaw)

  def _publish_tf(self, system_pose, vx: float, vy: float, w: float):
    """system_state 回调入口：根据 SROS 位姿/速度更新 odom，并缓存 odom→base TF。

    参数：
      system_pose: SROS location_pose（mm / mrad，map 系）
      vx, vy, w:   已换算成 ROS 单位的车体系速度

    两条路径都会更新 _last_odom_to_base_tf：
      1) 本函数（~3.33Hz，跟 system_state）
      2) _integrate_and_publish_odom（50Hz 定时器）
    最终都经 _send_cached_tf() → TransformBroadcaster.sendTransform()
    """
    if system_pose is None:
      return

    if not self._publish_odom and not self._publish_map_to_odom:
      return

    now = self._node.get_clock().now()
    now_ns = now.nanoseconds

    # SROS Pose(mm) → ROS PoseStamped(m)；这是 MATRIX 给出的 map 系位姿
    ros_pose = to_ros_pose(system_pose)
    map_position = ros_pose.pose.position
    map_orientation = ros_pose.pose.orientation
    map_yaw = self._yaw_from_quaternion(
      map_orientation.x,
      map_orientation.y,
      map_orientation.z,
      map_orientation.w,
    )

    if self._publish_odom:
      # ---- 更新内部里程计状态 (_odom_x/y/yaw) ----
      if not self._odom_use_integration:
        # 非积分：直接把 map 位姿当作 odom 位姿（odom 与 map 重合）
        self._odom_x = map_position.x
        self._odom_y = map_position.y
        self._odom_yaw = map_yaw
        self._odom_initialized = True
        self._last_odom_time_ns = now_ns
      elif not self._odom_initialized:
        # 积分模式首帧：用 map 位姿做初值，之后靠速度积分
        self._odom_x = map_position.x
        self._odom_y = map_position.y
        self._odom_yaw = map_yaw
        self._odom_initialized = True
        self._last_odom_time_ns = now_ns
      else:
        # 积分模式后续帧：用速度把位姿推到当前时刻
        self._integrate_odom_to(now_ns)

      # ---- 组装 odom → base_footprint（核心）----
      odom_to_base = TransformStamped()
      odom_to_base.header.stamp = now.to_msg()
      # parent / child 决定 TF 树边：odom → base_footprint
      odom_to_base.header.frame_id = self._odom_frame_id  # "odom"
      odom_to_base.child_frame_id = self._frame_id        # "base_footprint"
      # 平移：车在 odom 系下的 (x, y)
      odom_to_base.transform.translation.x = self._odom_x
      odom_to_base.transform.translation.y = self._odom_y
      odom_to_base.transform.translation.z = 0.0
      # 旋转：只绕 Z 轴的 yaw → 四元数
      qx, qy, qz, qw = self._quaternion_from_yaw(self._odom_yaw)
      odom_to_base.transform.rotation.x = qx
      odom_to_base.transform.rotation.y = qy
      odom_to_base.transform.rotation.z = qz
      odom_to_base.transform.rotation.w = qw
      # 写入缓存；50Hz 定时器也会读这份缓存重发
      self._last_odom_to_base_tf = odom_to_base

      # 同步发 /odom 话题（和 TF 内容一致，方便不看 TF 的节点订阅）
      odom_msg = Odometry()
      odom_msg.header.stamp = now.to_msg()
      odom_msg.header.frame_id = self._odom_frame_id
      odom_msg.child_frame_id = self._frame_id
      odom_msg.pose.pose.position.x = self._odom_x
      odom_msg.pose.pose.position.y = self._odom_y
      odom_msg.pose.pose.position.z = 0.0
      odom_msg.pose.pose.orientation.x = qx
      odom_msg.pose.pose.orientation.y = qy
      odom_msg.pose.pose.orientation.z = qz
      odom_msg.pose.pose.orientation.w = qw
      odom_msg.twist.twist.linear.x = vx
      odom_msg.twist.twist.linear.y = vy
      odom_msg.twist.twist.angular.z = w
      self._odom_publisher.publish(odom_msg)
    elif self._publish_map_to_odom:
      self._last_map_to_odom_tf = None

    # ---- 可选：算 map→odom（建图时关掉，避免和 slam_toolbox 冲突）----
    if self._publish_map_to_odom and self._publish_odom:
      # map_T_odom = map_T_base ∘ base_T_odom = map_pose ∘ inverse(odom_pose)
      odom_to_base_inv = self._inverse_2d(self._odom_x, self._odom_y, self._odom_yaw)
      map_to_odom = self._compose_2d(
        map_position.x,
        map_position.y,
        map_yaw,
        odom_to_base_inv[0],
        odom_to_base_inv[1],
        odom_to_base_inv[2],
      )
      map_to_odom_tf = TransformStamped()
      map_to_odom_tf.header.stamp = now.to_msg()
      map_to_odom_tf.header.frame_id = self._parent_frame_id  # "map"
      map_to_odom_tf.child_frame_id = self._odom_frame_id     # "odom"
      map_to_odom_tf.transform.translation.x = map_to_odom[0]
      map_to_odom_tf.transform.translation.y = map_to_odom[1]
      map_to_odom_tf.transform.translation.z = 0.0
      qx, qy, qz, qw = self._quaternion_from_yaw(map_to_odom[2])
      map_to_odom_tf.transform.rotation.x = qx
      map_to_odom_tf.transform.rotation.y = qy
      map_to_odom_tf.transform.rotation.z = qz
      map_to_odom_tf.transform.rotation.w = qw
      self._last_map_to_odom_tf = map_to_odom_tf
    else:
      # 建图推荐：这里为 None，SDK 不发 map→odom
      self._last_map_to_odom_tf = None

    # 立刻广播一次（定时器还会以 50Hz 继续刷）
    if self._enable_tf_publish and self._publish_odom:
      self._send_cached_tf()

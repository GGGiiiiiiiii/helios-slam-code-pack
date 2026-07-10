# Helios 真机 SLAM 集成代码

本包包含真机建图导航的两部分改动源码（2026-07-03 更新，雷达已回退 loc 模式）：

| 目录 | 说明 |
|------|------|
| `helios-navigation-main/` | 导航栈（helios_bringup 等），真机 SLAM/Nav2 |
| `sr_amr_control/` | 珞石 AMR ROS2 SDK 封装（雷达/里程计/TF） |
| `docs-代码改动说明.md` | 详细改动说明 |
| `diffs/` | 各文件 unified diff |

## 编译

```bash
# 导航
cd helios-navigation-main/lios/helios_linorobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# SDK（放入 robot_ws/src 后）
cd robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sr_amr_control
```

## 真机建图启动

```bash
# 终端1 SDK
ros2 launch sr_amr_control amr_control.launch.py \
  connect_ip:=192.168.71.50 lidar:=true \
  frame_id:=base_footprint publish_tf:=true publish_map_to_odom:=false

# 终端2 建图
ros2 launch helios_bringup mapping.launch.py merge:=true
```

## 保存地图

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_map
```

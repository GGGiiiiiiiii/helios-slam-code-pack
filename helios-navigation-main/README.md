# Helios 移动底盘 2D 导航

Helios 轮臂机器人**移动底盘**的 ROS 2 Humble 导航栈：SLAM 建图、`AMCL` 定位、`Nav2` 路径规划与避障。全向四轮底盘，对角双 2D 激光雷达；仿真基于 Gazebo，真机通过桥接层对接 `sr_amr_control` SDK。

---

## 依赖

- Ubuntu 22.04，ROS 2 Humble
- `ros-humble-navigation2` `ros-humble-nav2-bringup` `ros-humble-slam-toolbox`
- `ros-humble-gazebo-ros-pkgs` `ros-humble-teleop-twist-keyboard`
- Playground 仿真场景需 `~/.gazebo/models/` 中有 `playground`、`pine_tree` 等模型

---

## 编译

```bash
git clone https://github.com/<your-org>/helios-navigation.git
cd helios-navigation/lios/helios_linorobot_ws

export PATH="/usr/bin:$PATH"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 仿真 Demo

### 1. 预建地图导航（推荐入门）

```bash
ros2 launch helios_gazebo sim_playground_nav.launch.py
```

1. 等待 Nav2 就绪：`ros2 lifecycle get /bt_navigator` → `active [3]`
2. RViz：**2D Pose Estimate** 设置初始位姿
3. RViz：**Nav2 Goal** 发送目标点

### 2. 建图

```bash
# 终端 A
ros2 launch helios_gazebo sim_slam.launch.py

# 终端 B：遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 终端 C：保存地图
ros2 run nav2_map_server map_saver_cli -f ~/helios_sim_map
```

Playground 场地：

```bash
ros2 launch helios_gazebo sim_slam.launch.py \
  world:=$(ros2 pkg prefix helios_gazebo)/share/helios_gazebo/worlds/playground.world
```

### 3. 用自存地图导航

```bash
ros2 launch helios_gazebo sim_playground_nav.launch.py map:=~/helios_sim_map.yaml
```

先 **2D Pose Estimate**，再 **Nav2 Goal**。

### 4. 在线建图 + 导航（一条命令）

```bash
./scripts/run_sim_nav.bash
```

RViz 直接 **Nav2 Goal** 即可。

---

## Launch 一览

| Launch | 说明 |
|--------|------|
| `helios_gazebo/sim.launch.py` | 仿真 + 激光 |
| `helios_gazebo/sim_slam.launch.py` | 仿真 + SLAM 建图 |
| `helios_gazebo/sim_nav.launch.py` | 仿真 + SLAM + Nav2 |
| `helios_gazebo/sim_playground_nav.launch.py` | Playground + 预建图 + AMCL + Nav2 |
| `helios_bringup/mapping.launch.py` | 真机建图 |
| `helios_bringup/navigation.launch.py` | 真机导航（`map:=` 指定地图） |

---

## 仓库结构

```
helios-navigation/
├── docs/                           # 文档
└── lios/helios_linorobot_ws/
    ├── scripts/run_sim_nav.bash
    └── src/
        ├── helios_bringup/         # 桥接、Nav2/SLAM 配置、launch
        ├── helios_description/     # URDF、mesh
        └── helios_gazebo/          # 仿真、worlds、rviz
```

| 资源 | 路径 |
|------|------|
| 默认仿真场地 | `helios_gazebo/worlds/minimal.world` |
| Playground 场地 | `helios_gazebo/worlds/playground.world` |
| Playground 地图 | `helios_bringup/maps/playground.yaml` |

---

## 文档

| 文档 | 内容 |
|------|------|
| [docs/建图与导航.md](docs/建图与导航.md) | 建图、存图、导航流程 |
| [docs/技术说明.md](docs/技术说明.md) | 架构与数据流 |
| [docs/仿真手册.md](docs/仿真手册.md) | 仿真操作 |
| [docs/真机手册.md](docs/真机手册.md) | 真机联调 |
| [docs/典型问题.md](docs/典型问题.md) | 常见问题 |

---

## 真机

需安装厂家 `standard_robots_amr_ros2` SDK，启动底盘后：

```bash
ros2 launch helios_bringup mapping.launch.py          # 建图
ros2 launch helios_bringup navigation.launch.py map:=/path/to/map.yaml
```

详见 [docs/真机手册.md](docs/真机手册.md)。

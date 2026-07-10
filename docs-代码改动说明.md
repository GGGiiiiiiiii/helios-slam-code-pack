# Helios 真机 SLAM 建图 — 代码改动说明

> 对比基准（改前）  
> - 导航：`helios-navigation-main.zip`（2026-07-01 原版 helios_bringup）  
> - SDK：`helios_robot_code_20260622.tar.gz`（2026-06-22 原版 sr_amr_control）  
>  
> 改后代码已打包在同目录：  
> - `helios_bringup_modified.zip`  
> - `sr_amr_control_modified.zip`  
> - `diffs/` 目录含每个文件的 unified diff

---

## 一、改动背景（为什么要改）

真机 SLAM 建图失败，核心原因是 **TF 多发布者冲突** 和 **时间戳不对齐**：

| 问题 | 报错/现象 |
|------|-----------|
| SDK 与 helios 同时发 `odom→base_footprint` | `Message Filter dropping ... queue is full` |
| SDK 与 slam 同时发 `map→odom` | TF 树混乱 |
| SDK TF ~3Hz，雷达 ~5Hz，时间戳不匹配 | `Failed to compute odom pose` |
| 建图时 `publish_map_to_odom:=false` 后雷达点云在 map 系无法变换 | 雷达不出 scan |
| SDK 默认 `frame_id=base_link`，helios 用 `base_footprint` | TF 链拼接困难 |

**设计原则（改后）：**

- **SDK**：独占 `odom→base_footprint`，提供雷达/里程计数据
- **helios_bringup**：只消费 SDK 数据做预处理，**不**发底盘动态 TF
- **slam_toolbox**：建图时独占 `map→odom`

---

## 二、导航代码改动（helios_bringup，共 5 个文件）

### 1. `launch/bringup.launch.py`

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| 文件头注释 | 描述多种 odom 方案（EKF/积分/仿真） | 明确「真机 TF 由 SDK 独占，helios 只做预处理」 | 文档化职责，避免误用 |
| `use_odom` 默认值 | `"true"` | **`"false"`** | 真机默认不启 `odom_publisher`，避免与 SDK 抢 `odom→base_footprint` |
| `use_odom` 描述 | 「回退方案；用 EKF 时设 false」 | 「真机请 false，TF 由 SDK 发布」 | 明确真机用法 |
| `laser_preprocess` 参数 | 无 `stamp_now` | 增加 **`stamp_now: True`** | 输出 scan 时间戳用 `now()`，与 SDK 20Hz TF 对齐 |
| `odom_publisher` | `use_odom=true` 时默认运行 | 仍条件启动，但真机默认不触发 | 保留仿真/回退能力 |

**未改动的部分：** `output_topic` 仍为 `/scan`；`robot_state_publisher`、`cmd_vel_relay`、EKF 节点逻辑不变。

---

### 2. `launch/mapping.launch.py`

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| 文件头注释 | 仅一行用法 | 写明 SDK / helios / slam 三方职责 | 使用说明 |
| 传给 bringup 的参数 | 仅 `merge`、`relay_cmd_vel` | 增加 **`use_odom:=false`、`use_ekf:=false`** | 建图时强制关闭 helios 侧 odom/EKF |
| slam_toolbox 启动 | 与 bringup 同时启动 | **`TimerAction(period=2.0)` 延迟 2 秒启动** | 等 SDK TF、URDF 静态链就绪后再起 SLAM |

---

### 3. `launch/localization.launch.py`

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| 传给 bringup 的参数 | 仅 `merge`、`relay_cmd_vel` | 增加 **`use_odom:=false`、`use_ekf:=false`** | 定位导航时同样不抢 SDK TF |

---

### 4. `helios_bringup/laser_preprocess.py`

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| 新参数 | 无 | **`stamp_now`**，默认 `True` | 可配置是否重写时间戳 |
| `_on_single()` | 保留 SDK 原始 `header.stamp` | `stamp_now=True` 时改为 **`self.get_clock().now()`** | slam 按 scan 时刻查 TF 时，时间戳与最新 TF 一致 |
| `_try_merge()` 输出 | 用 `_front.header.stamp` | `stamp_now=True` 时用 **`now()`** | 双雷达合并模式同样对齐 |

---

### 5. `setup.py`

**无实质逻辑改动**（与原版 entry_points 相同，仅换行符差异）。

---

## 三、SDK 代码改动（sr_amr_control，共 4 个文件）

### 6. `launch/amr_control.launch.py`

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| `frame_id` 默认 | `base_link` | **`base_footprint`** | 与 helios URDF、slam_toolbox 的 `base_frame` 一致 |
| 新增 launch 参数 | 无 | **`publish_tf`**（默认 true） | 控制是否发布 `odom→base` TF |
| 新增 launch 参数 | 无 | **`publish_map_to_odom`**（默认 true） | 建图时传 `false`，让 slam 独占 `map→odom` |
| 传入 control_node | 无上述参数 | 传入 `publish_tf`、`publish_map_to_odom` | 与 main.py / state_publisher 接线 |

**建图推荐 launch：**
```bash
ros2 launch sr_amr_control amr_control.launch.py \
  connect_ip:=192.168.71.50 lidar:=true \
  frame_id:=base_footprint publish_tf:=true publish_map_to_odom:=false
```

---

### 7. `sr_amr_control/main.py`

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| 声明参数 | 无 | **`publish_map_to_odom`**、**`publish_tf`** | 从 launch 读取 TF 开关 |
| 读取参数 | 无 | 用 `_parameter_value_to_bool` 解析 | 兼容 launch 字符串/布尔 |
| 创建 StatePublisher | 仅传 `odom_use_integration` 等 | 增加传 **`publish_map_to_odom`、`publish_tf`** | 参数下发到 TF 发布逻辑 |

---

### 8. `sr_amr_control/state_publisher.py`（改动最大）

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| 构造函数参数 | 无 TF 开关 | 增加 **`publish_map_to_odom`、`publish_tf`** | 可分别控制两种 TF |
| 成员变量 | 无 | **`_last_odom_to_base_tf`、`_last_map_to_odom_tf`** | 缓存 TF 供定时重发 |
| 初始化 | 仅注册回调 | 若 `publish_tf`：调用 **`_bootstrap_odom_tf()`** + **20Hz 定时器** | 上电即有 odom TF；持续刷新时间戳 |
| `_bootstrap_odom_tf()` | 不存在 | **新增**：发布单位变换 `odom→base` | 首帧 system_state（~300ms）前 slam 能查到 odom |
| `_send_cached_tf()` | 不存在 | **新增**：统一发送缓存 TF | 避免重复代码 |
| `_republish_cached_tf()` | 不存在 | **新增**：20Hz 刷新 stamp 并重发 | 解决「scan 时间戳比 TF 新」导致 Failed to compute odom pose |
| `_publish_tf()` 末尾 | **无条件** `sendTransform([map_to_odom, odom_to_base])` | 缓存 TF；**仅当 `publish_map_to_odom` 才发 map→odom**；**仅当 `publish_tf` 才发送** | 建图时关闭 map→odom；可完全关闭 TF |

**改前关键代码：**
```python
self._tf_broadcaster.sendTransform([map_to_odom_tf, odom_to_base])
```

**改后关键逻辑：**
```python
self._last_odom_to_base_tf = odom_to_base
if self._publish_map_to_odom:
    self._last_map_to_odom_tf = map_to_odom_tf
else:
    self._last_map_to_odom_tf = None
if self._enable_tf_publish:
    self._send_cached_tf()
```

---

### 9. `sr_amr_control/lidar_reporter.py`

| 位置 | 改前 | 改后 | 为什么改 |
|------|------|------|----------|
| 订阅 | 无 | 订阅 **`system_state`**，缓存 **`_map_pose_xy_yaw`** | 建图时无 map→odom TF，仍能用位姿变换点云 |
| `_on_system_state()` | 不存在 | **新增** | 从 current_pose 提取 x/y/yaw |
| `_can_transform_via_map_pose()` | 不存在 | **新增** | 检查能否用位姿+静态链代替 map TF |
| `_transform_scan_points_via_map_pose()` | 不存在 | **新增**：map 点 → base → laser 数学变换 | 不依赖 map 在 TF 树中 |
| `_transform_scan_points()` | TF 失败返回 `[]` | map 系失败时 **走 map_pose 回退** | 建图模式雷达冷启动 |
| `_transforms_ready()` | 仅查 TF | 增加 **map_pose 回退判断** | 更早开始出 scan |

**改前逻辑：** 点云在 map 系，必须用 TF `laser ← map`；建图时 `publish_map_to_odom:=false` 则 map 不在 TF 树 → 雷达卡住。

**改后逻辑：** TF 查不到时，用 system_state 位姿 + URDF 静态链（base→laser）做数学变换。

---

## 四、未改动的文件（供对照）

以下文件与原版相同，**未修改**：

**helios_bringup：**
- `helios_bringup/odom_publisher.py`
- `helios_bringup/cmd_vel_relay.py`
- `config/slam_toolbox.yaml`
- `config/nav2_params.yaml`
- `config/ekf.yaml`
- `launch/navigation.launch.py`

**sr_amr_control：**
- `sr_amr_control/remote_control.py`
- `sr_amr_control/move_to_station_action.py`
- 其余 action/service 文件

---

## 五、部署方法

### 导航包
```bash
# 解压到工作空间 src，覆盖原 helios_bringup
unzip helios_bringup_modified.zip -d ~/helios-navigation-main/lios/helios_linorobot_ws/src/
cd ~/helios-navigation-main/lios/helios_linorobot_ws
colcon build --packages-select helios_bringup
source install/setup.bash
```

### SDK 包
```bash
unzip sr_amr_control_modified.zip -d ~/Desktop/helios_robot_code_pack/robot_ws/src/
cd ~/Desktop/helios_robot_code_pack/robot_ws
colcon build --packages-select sr_amr_control
source install/setup.bash
```

---

## 六、真机启动命令（改后）

**终端 1 — SDK：**
```bash
ros2 launch sr_amr_control amr_control.launch.py \
  connect_ip:=192.168.71.50 lidar:=true \
  frame_id:=base_footprint publish_tf:=true publish_map_to_odom:=false
```

**终端 2 — 建图：**
```bash
ros2 launch helios_bringup mapping.launch.py merge:=false
```

**保存地图：**
```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_map
```

---

## 七、验证结果

| 指标 | 改前 | 改后 |
|------|------|------|
| `Failed to compute odom pose` | 持续报错 | 消除 |
| `/scan` | 有数据 | ~5 Hz 稳定 |
| `/map` | 无/空 | ~1 Hz 有数据 |
| 地图保存 | — | 268×281，0.05m/px 成功 |

---

## 八、文件清单与 diff 路径

| 包 | 文件 | diff 文件 |
|----|------|-----------|
| helios_bringup | launch/bringup.launch.py | diffs/nav_launch_bringup.launch.py.diff |
| helios_bringup | launch/mapping.launch.py | diffs/nav_launch_mapping.launch.py.diff |
| helios_bringup | launch/localization.launch.py | diffs/nav_launch_localization.launch.py.diff |
| helios_bringup | helios_bringup/laser_preprocess.py | diffs/nav_helios_bringup_laser_preprocess.py.diff |
| sr_amr_control | launch/amr_control.launch.py | diffs/sdk_launch_amr_control.launch.py.diff |
| sr_amr_control | sr_amr_control/main.py | diffs/sdk_sr_amr_control_main.py.diff |
| sr_amr_control | sr_amr_control/state_publisher.py | diffs/sdk_sr_amr_control_state_publisher.py.diff |
| sr_amr_control | sr_amr_control/lidar_reporter.py | diffs/sdk_sr_amr_control_lidar_reporter.py.diff |

#!/usr/bin/env bash
# Helios 仿真一条龙：Gazebo + SLAM + Nav2
# 用法：
#   ./scripts/run_sim_nav.bash            # 直接启动仿真导航
#   ./scripts/run_sim_nav.bash --build    # 先 colcon build 再启动
#
# 启动后在 RViz 用【Nav2 Goal】发目标点即可自主导航（SLAM 在线建图，无需 2D Pose Estimate）。
# 关闭：在本终端按 Ctrl+C（不要点 Gazebo 窗口右上角的叉，否则会残留 gzserver）。

set -euo pipefail

# ---- 定位工作空间（脚本所在目录的上一级）----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"

# ---- 避开 conda 抢占系统 python（否则编译/spawn 会失败）----
export PATH="/usr/bin:${PATH}"

# ---- 清理可能残留的 gzserver/gzclient（上次没用 Ctrl+C 关时会残留）----
if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
  echo "[run_sim_nav] 检测到残留的 Gazebo 进程，正在清理..."
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  sleep 2
fi

# ---- source ROS（setup.bash 会引用未定义变量，source 前临时关闭 nounset）----
if [[ ! -f "${ROS_DISTRO_SETUP}" ]]; then
  echo "[run_sim_nav] 找不到 ${ROS_DISTRO_SETUP}，请确认已安装 ROS 2 Humble" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "${ROS_DISTRO_SETUP}"
set -u

cd "${WS_DIR}"

# ---- 可选编译 ----
if [[ "${1:-}" == "--build" ]]; then
  echo "[run_sim_nav] colcon build --symlink-install ..."
  colcon build --symlink-install
fi

# ---- source 工作空间 ----
if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "[run_sim_nav] 未找到 install/setup.bash，先执行一次编译：" >&2
  echo "    ${BASH_SOURCE[0]} --build" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"
set -u

# ---- 启动仿真导航 ----
echo "[run_sim_nav] 启动 Gazebo + SLAM + Nav2 ..."
echo "[run_sim_nav] 就绪标志：日志出现 'Managed nodes are active'（可能被 slam 的丢帧 INFO 刷屏顶上去，属正常）"
exec ros2 launch helios_gazebo sim_nav.launch.py

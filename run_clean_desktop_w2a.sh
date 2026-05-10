#!/usr/bin/env bash
set -euo pipefail

# Service Robot B task startup helper for the original W2A launch flow.
# Flow:
# 1. roslaunch clean_desktop_robot clean_desktop_robot_w2a.launch
# 2. roslaunch clean_desktop_robot view_grab.launch
# 3. wait for operator confirmation
# 4. roslaunch clean_desktop_robot complete_flow.launch

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}"
LOG_DIR="${WORKSPACE_DIR}/logs"
mkdir -p "${LOG_DIR}"

PIDS=()

cleanup() {
  echo
  echo "[启动脚本] 收到退出信号，正在关闭已启动的 roslaunch..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
  echo "[启动脚本] 已退出。"
}

trap cleanup EXIT INT TERM

source_if_exists() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck disable=SC1090
    source "${setup_file}"
    echo "[启动脚本] 已加载: ${setup_file}"
  fi
}

echo "============================================================"
echo "[启动脚本] 服务机器人 B 赛项 W2A 原版流程"
echo "============================================================"

source_if_exists "/opt/ros/noetic/setup.bash"
source_if_exists "${WORKSPACE_DIR}/devel/setup.bash"
source_if_exists "${HOME}/upros_class_code/devel/setup.bash"
source_if_exists "${HOME}/clean_robot_code/devel/setup.bash"

echo
echo "[启动脚本] 检查 ROS 包..."
if ! command -v rospack >/dev/null 2>&1; then
  echo "[启动脚本][错误] 找不到 rospack。请先 source ROS 环境，例如 /opt/ros/noetic/setup.bash。"
  exit 1
fi

for pkg in clean_desktop_robot upros_bringup w2u_navigation; do
  if ! rospack find "${pkg}" >/dev/null 2>&1; then
    echo "[启动脚本][错误] 找不到 ROS 包: ${pkg}"
    echo "[启动脚本][建议] 请确认 clean_robot_code 和 upros_class_code 都已经 catkin_make，并 source 对应 devel/setup.bash。"
    exit 1
  fi
done

echo "[启动脚本] ROS 包检查通过。"

echo
echo "[启动脚本] 第 1 步: 启动底盘、导航、AprilTag 识别..."
roslaunch clean_desktop_robot clean_desktop_robot_w2a.launch \
  2>&1 | tee "${LOG_DIR}/01_clean_desktop_w2a.log" &
PIDS+=("$!")

echo "[启动脚本] 等待系统初始化 10 秒..."
sleep 2

echo
echo "[启动脚本] 第 2 步: 启动 RViz 抓取视图..."
roslaunch clean_desktop_robot view_grab.launch \
  2>&1 | tee "${LOG_DIR}/02_view_grab.log" &
PIDS+=("$!")

echo
echo "================================================------------"
echo "[启动脚本] 启动任务前请在 RViz 检查:"
echo "  1. /map 是否正常"
echo "  2. /scan 或 /scan_filtered 是否正常"
echo "  3. TF: map -> odom_combined -> base_footprint -> base_link"
echo "  4. /move_base/local_costmap/costmap 中机器人是否没有压在障碍物里"
echo "  5. 能否看到 tag_1，且 base_link -> tag_1 稳定"
echo "================================================------------"
read -r -p "[启动脚本] 确认无误后按回车启动 complete_flow，或按 Ctrl+C 退出..."

echo
echo "[启动脚本] 第 3 步: 启动比赛完整流程..."
roslaunch clean_desktop_robot complete_flow.launch \
  2>&1 | tee "${LOG_DIR}/03_complete_flow.log" &
PIDS+=("$!")

echo
echo "[启动脚本] complete_flow 已启动。日志目录: ${LOG_DIR}"
echo "[启动脚本] 当前终端保持运行；按 Ctrl+C 会关闭本脚本启动的 roslaunch。"

wait "${PIDS[-1]}"

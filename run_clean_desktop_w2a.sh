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
BCSH_DIR="$(cd "${WORKSPACE_DIR}/.." && pwd)"
LOG_DIR="${WORKSPACE_DIR}/logs"
LAUNCH_DIR="${WORKSPACE_DIR}/src/clean_desktop_robot/launch"
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

require_source() {
  local setup_file="$1"
  if [[ ! -f "${setup_file}" ]]; then
    echo "[启动脚本][错误] 找不到当前仓库环境文件: ${setup_file}"
    echo "[启动脚本][建议] 请先在对应工作空间执行 catkin_make。"
    exit 1
  fi
  source_if_exists "${setup_file}"
}

require_source "${BCSH_DIR}/upros_class_code/devel/setup.bash"
require_source "${BCSH_DIR}/clean_robot_code/devel/setup.bash"

# 当前 devel/setup.bash 里可能残留旧的 chained workspace 路径。
# 这条启动链只允许解析当前仓库和系统 ROS，避免 roslaunch 看到 /home/bcsh 下的同名包。
export CMAKE_PREFIX_PATH="${BCSH_DIR}/clean_robot_code/devel:${BCSH_DIR}/upros_class_code/devel:/opt/ros/noetic"
export ROS_PACKAGE_PATH="${BCSH_DIR}/clean_robot_code/src:${BCSH_DIR}/upros_class_code/src:/opt/ros/noetic/share"

echo
echo "[启动脚本] 检查 ROS 包..."
if ! command -v rospack >/dev/null 2>&1; then
  echo "[启动脚本][错误] 找不到 rospack。请先 source ROS 环境，例如 /opt/ros/noetic/setup.bash。"
  exit 1
fi
rospack profile >/dev/null 2>&1 || true

require_executable() {
  local executable_path="$1"
  local description="$2"
  if [[ ! -x "${executable_path}" ]]; then
    echo "[启动脚本][错误] 找不到可执行节点: ${description}"
    echo "  期望路径: ${executable_path}"
    echo "[启动脚本][建议] 请在当前仓库对应工作空间重新编译:"
    echo "  cd ${BCSH_DIR}/upros_class_code && catkin_make"
    if [[ -x "/home/bcsh/upros_class_code/devel/lib/orbbec_camera/orbbec_camera_node" ]]; then
      echo "[启动脚本][提示] 检测到旧工作空间里有该节点:"
      echo "  /home/bcsh/upros_class_code/devel/lib/orbbec_camera/orbbec_camera_node"
      echo "  但本脚本当前固定使用 ${BCSH_DIR} 下的工作空间，避免同名包混用。"
    fi
    exit 1
  fi
}

require_image_stream() {
  local topic="$1"
  local info

  info="$(timeout 5 rostopic info "${topic}" 2>/dev/null || true)"
  if [[ -z "${info}" || "${info}" == *"Publishers: None"* ]]; then
    echo "[启动脚本][错误] 相机图像话题没有发布者: ${topic}"
    echo "[启动脚本][建议] 请检查 Orbbec 相机节点是否启动成功。常用排查命令:"
    echo "  rosnode list | grep camera"
    echo "  rostopic info ${topic}"
    echo "  tail -n 80 ${LOG_DIR}/01_clean_desktop_w2a.log"
    exit 1
  fi

  if ! timeout 8 rostopic echo -n 1 "${topic}/header" >/dev/null 2>&1; then
    echo "[启动脚本][错误] ${topic} 有发布者，但 8 秒内没有收到图像帧。"
    echo "[启动脚本][建议] 请重插相机或单独测试:"
    echo "  roslaunch orbbec_camera dabai_dcw2.launch"
    exit 1
  fi
}

require_topic_message() {
  local topic="$1"
  local description="$2"

  if ! timeout 8 rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "[启动脚本][错误] ${description} 没有收到数据: ${topic}"
    echo "[启动脚本][建议] 请检查对应硬件驱动和话题发布者:"
    echo "  rostopic info ${topic}"
    echo "  tail -n 120 ${LOG_DIR}/01_clean_desktop_w2a.log"
    exit 1
  fi
}

require_tf_ready() {
  local target_frame="$1"
  local source_frame="$2"
  local description="$3"

  timeout 8 rosrun tf tf_echo "${target_frame}" "${source_frame}" >/tmp/clean_robot_tf_check.log 2>&1 || true
  if ! grep -q "Translation:" /tmp/clean_robot_tf_check.log; then
    echo "[启动脚本][错误] ${description} 未就绪: ${target_frame} -> ${source_frame}"
    echo "[启动脚本][建议] 请检查 AMCL 初始位姿、里程计、雷达和 TF 树:"
    echo "  rosrun tf view_frames"
    echo "  rosrun tf tf_echo ${target_frame} ${source_frame}"
    echo "  tail -n 120 ${LOG_DIR}/01_clean_desktop_w2a.log"
    exit 1
  fi
}

require_rosnode_alive() {
  local node_name="$1"
  local description="$2"

  if ! timeout 5 rosnode ping -c 1 "${node_name}" >/dev/null 2>&1; then
    echo "[启动脚本][错误] ${description} 节点未运行或无响应: ${node_name}"
    echo "[启动脚本][建议] 请查看启动日志:"
    echo "  tail -n 160 ${LOG_DIR}/01_clean_desktop_w2a.log"
    exit 1
  fi
}

for pkg in \
  clean_desktop_robot \
  upros_bringup \
  w2u_navigation \
  upros_navigation \
  upros_lidar_filter \
  upros_arm \
  upros_odometry \
  zoo_bringup \
  zoo_description \
  bluesea2 \
  handsfree_ros_imu \
  orbbec_camera; do
  pkg_path="$(rospack find "${pkg}" 2>/dev/null || true)"
  if [[ -z "${pkg_path}" ]]; then
    echo "[启动脚本][错误] 找不到 ROS 包: ${pkg}"
    echo "[启动脚本][建议] 请确认 clean_robot_code 和 upros_class_code 都已经 catkin_make，并 source 对应 devel/setup.bash。"
    exit 1
  fi
  if [[ "${pkg_path}" != "${BCSH_DIR}/"* ]]; then
    echo "[启动脚本][错误] ROS 包 ${pkg} 当前解析到了非当前仓库路径:"
    echo "  ${pkg_path}"
    echo "[启动脚本][建议] 请检查 ROS_PACKAGE_PATH，确保使用 ${BCSH_DIR} 下的工作空间。"
    exit 1
  fi
done

echo "[启动脚本] ROS 包检查通过。"

require_executable \
  "${BCSH_DIR}/upros_class_code/devel/lib/orbbec_camera/orbbec_camera_node" \
  "orbbec_camera/orbbec_camera_node"
require_executable \
  "${BCSH_DIR}/upros_class_code/devel/lib/zoo_bringup/zoo_driver" \
  "zoo_bringup/zoo_driver"
require_executable \
  "${BCSH_DIR}/upros_class_code/devel/lib/bluesea2/bluesea2_node" \
  "bluesea2/bluesea2_node"
require_executable \
  "${BCSH_DIR}/upros_class_code/src/upros_hardware/handsfree_ros_imu/scripts/hfi_b9_ros.py" \
  "handsfree_ros_imu/hfi_b9_ros.py"
require_executable \
  "${BCSH_DIR}/upros_class_code/devel/lib/handsfree_ros_imu/imu_calibrate" \
  "handsfree_ros_imu/imu_calibrate"
require_executable \
  "${BCSH_DIR}/upros_class_code/devel/lib/upros_arm/upros_arm_control" \
  "upros_arm/upros_arm_control"
require_executable \
  "${BCSH_DIR}/upros_class_code/src/upros_odometry/scripts/odom_ekf.py" \
  "upros_odometry/odom_ekf.py"
require_executable \
  "${BCSH_DIR}/upros_class_code/devel/lib/libcostmap_prohibition_layer.so" \
  "costmap_prohibition_layer_namespace::CostmapProhibitionLayer"
require_executable \
  "${BCSH_DIR}/upros_class_code/devel/lib/libteb_local_planner.so" \
  "teb_local_planner/TebLocalPlannerROS"

echo
echo "[启动脚本] 第 1 步: 启动底盘、导航、AprilTag 识别..."
roslaunch "${LAUNCH_DIR}/clean_desktop_robot_w2a.launch" \
  2>&1 | tee "${LOG_DIR}/01_clean_desktop_w2a.log" &
PIDS+=("$!")

echo "[启动脚本] 等待系统初始化 10 秒..."
sleep 10
echo "[启动脚本] 检查相机图像流 /camera/color/image_raw ..."
require_image_stream "/camera/color/image_raw"
echo "[启动脚本] 相机图像流正常。"
echo "[启动脚本] 检查雷达原始数据 /scan ..."
require_topic_message "/scan" "雷达原始扫描"
echo "[启动脚本] 检查雷达过滤数据 /scan_filtered ..."
require_topic_message "/scan_filtered" "雷达过滤扫描"
echo "[启动脚本] 检查地图 /map ..."
require_topic_message "/map" "地图"
echo "[启动脚本] 检查定位 TF map -> base_footprint ..."
require_tf_ready "map" "base_footprint" "定位 TF"
echo "[启动脚本] 检查 move_base 导航服务器 ..."
require_rosnode_alive "/move_base" "move_base"
echo "[启动脚本] 导航基础数据正常。"

echo
echo "[启动脚本] 第 2 步: 启动 RViz 抓取视图..."
roslaunch "${LAUNCH_DIR}/view_grab.launch" \
  2>&1 | tee "${LOG_DIR}/02_view_grab.log" &
PIDS+=("$!")

echo
echo "================================================------------"
echo "[启动脚本] 启动任务前请在 RViz 检查:"
echo "  1. /map 是否正常"
echo "  2. /scan 或 /scan_filtered 是否正常"
echo "  3. TF: map -> odom_combined -> base_footprint -> base_link"
echo "  4. /move_base/local_costmap/costmap 中机器人是否没有压在障碍物里"
echo "  5. Camera Raw 是否能看到 /camera/color/image_raw 画面"
echo "  6. 能否看到 tag_1，且 base_link -> tag_1 稳定"
echo "================================================------------"
read -r -p "[启动脚本] 确认无误后按回车启动 complete_flow，或按 Ctrl+C 退出..."

echo
echo "[启动脚本] 第 3 步: 启动比赛完整流程..."
roslaunch "${LAUNCH_DIR}/complete_flow.launch" \
  2>&1 | tee "${LOG_DIR}/03_complete_flow.log" &
PIDS+=("$!")

echo
echo "[启动脚本] complete_flow 已启动。日志目录: ${LOG_DIR}"
echo "[启动脚本] 当前终端保持运行；按 Ctrl+C 会关闭本脚本启动的 roslaunch。"

wait "${PIDS[-1]}"

#!/bin/bash
#====================================================================
# carry_robot TF & AprilTag 诊断脚本
# 运行方式: 在机器人终端执行  bash diagnose_tf.sh
# 前提: roscore 和所有节点已启动
#====================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo ""
echo "============================================================"
echo "  carry_robot TF & AprilTag 诊断"
echo "  运行时间: $(date)"
echo "============================================================"
echo ""

# ---- 1. 检查 map -> base_link TF ----
echo -e "${YELLOW}[1/5] TF: map -> base_link${NC}"
result=$(timeout 2 rosrun tf tf_echo map base_link 2>&1 || true)
if echo "$result" | grep -q "Exception\|ERROR\|Failed"; then
    echo -e "  ${RED}✗ 异常:${NC}"
    echo "$result" | grep -i "Exception\|ERROR" | head -3
    echo "  建议: 在 RViz 中用 2D Pose Estimate 重设机器人起始位姿"
else
    echo -e "  ${GREEN}✓ 正常${NC}"
    echo "$result" | tail -1
fi
echo ""

# ---- 2. 检查 odom -> base_link TF ----
echo -e "${YELLOW}[2/5] TF: odom -> base_link${NC}"
result=$(timeout 2 rosrun tf tf_echo odom base_link 2>&1 || true)
if echo "$result" | grep -q "Exception\|ERROR\|Failed"; then
    echo -e "  ${RED}✗ 异常:${NC}"
    echo "$result" | grep -i "Exception\|ERROR" | head -3
    echo "  建议: 检查轮式里程计或 IMU 是否发布 odom"
else
    echo -e "  ${GREEN}✓ 正常${NC}"
    echo "$result" | tail -1
fi
echo ""

# ---- 3. 检查 tag 是否在 TF 树中 ----
echo -e "${YELLOW}[3/5] AprilTag TF 检测${NC}"
# 用 view_frames 生成 TF 树 PDF
timeout 5 rosrun tf view_frames 2>/dev/null || true
TAG_FOUND=0
if [ -f frames.pdf ]; then
    # pdftotext 提取文本搜索 tag 帧名
    if command -v pdftotext &> /dev/null; then
        tag_text=$(pdftotext frames.pdf - 2>/dev/null | grep -i "tag36h11\|tag_")
        if [ -n "$tag_text" ]; then
            echo -e "  ${GREEN}✓ TF 树中发现 tag 帧:${NC}"
            echo "$tag_text"
            TAG_FOUND=1
        fi
    fi
    rm -f frames.pdf
fi

# 额外用 rostopic 检查
echo "  ---"
echo "  话题检测:"
tag_topics=$(timeout 2 rostopic list 2>/dev/null | grep -i tag || true)
if [ -n "$tag_topics" ]; then
    echo -e "  ${GREEN}✓ AprilTag 话题存在:${NC}"
    echo "$tag_topics"
    TAG_FOUND=1
else
    echo -e "  ${YELLOW}  未找到 AprilTag 话题${NC}"
fi

# 检查 tag_detections 内容
detect_result=$(timeout 2 rostopic echo /tag_detections -n 1 2>/dev/null || true)
if [ -n "$detect_result" ] && ! echo "$detect_result" | grep -q "WARNING\|ERROR"; then
    num_tags=$(echo "$detect_result" | grep -c "id:")
    echo -e "  ${GREEN}✓ 检测到 $num_tags 个 tag${NC}"
    echo "$detect_result" | grep -E "id:|pose:" | head -6
    TAG_FOUND=1
fi

if [ "$TAG_FOUND" -eq 0 ]; then
    echo -e "  ${RED}✗ TF 树中没有 tag 帧${NC}"
    echo "  建议:"
    echo "    (1) 确认相机已启动: rostopic list | grep camera"
    echo "    (2) 确认 apriltag_ros 已启动: rosnode list | grep apriltag"
    echo "    (3) 确认 tags.yaml 中 id 配置正确"
    echo "    (4) 确认 settings.yaml 中 publish_tf: true"
fi
echo ""

# ---- 4. TEB/local_costmap 相关参数 ----
echo -e "${YELLOW}[4/5] 代价地图与规划器参数${NC}"
echo "  ---"
echo "  当前 footprint:"
rosparam get /move_base/local_costmap/footprint 2>/dev/null || echo "  (无法获取)"
echo ""
echo "  当前 inflation_radius:"
rosparam get /move_base/local_costmap/inflation_layer/inflation_radius 2>/dev/null || echo "  (无法获取)"
echo ""
echo "  当前 min_obstacle_dist (TEB):"
rosparam get /move_base/TebLocalPlannerROS/min_obstacle_dist 2>/dev/null || echo "  (无法获取)"
echo ""
echo "  当前 xy_goal_tolerance (TEB):"
rosparam get /move_base/TebLocalPlannerROS/xy_goal_tolerance 2>/dev/null || echo "  (无法获取)"
echo ""

# ---- 5. 检查关键节点是否运行 ----
echo -e "${YELLOW}[5/5] 关键节点状态${NC}"
for node in "/move_base" "/apriltag_ros_continuous_node" "/camera" "/amcl"; do
    if rosnode list 2>/dev/null | grep -q "$node"; then
        echo -e "  ${GREEN}✓${NC} $node  running"
    else
        echo -e "  ${RED}✗${NC} $node  NOT running"
    fi
done

echo ""
echo "============================================================"
echo "  诊断完毕"
echo "  RViz 推荐检查项:"
echo "    - Add → Map: /move_base/local_costmap/costmap"
echo "    - Add → Polygon: /move_base/local_costmap/footprint"
echo "    - Add → LaserScan: /scan_filtered"
echo "    - Add → Path: /move_base/TebLocalPlannerROS/local_plan"
echo "============================================================"

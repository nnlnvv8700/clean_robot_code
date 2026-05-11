# clean_robot_code

服务机器人竞赛任务代码，当前主要用于 B 赛项桌面清洁流程。

本仓库是 ROS1 catkin 工作空间，需要和 `upros_class_code` 一起部署、编译后运行，不能只 clone 后直接执行。

## 目录结构

- `src/clean_robot`：A 赛项地面清洁相关节点和 launch。
- `src/clean_desktop_robot`：B 赛项桌面清洁相关节点和 launch。
- `src/carry_robot`：C 赛项搬运相关节点和 launch。
- `run_clean_desktop_w2a.sh`：W2A 小车运行 B 赛项桌面清洁流程的一键启动脚本。

## 运行环境

- Ubuntu + ROS Noetic。
- 已安装并编译 `upros_class_code`。
- 小车硬件、串口权限、雷达、相机、机械臂等设备配置正常。

本工程依赖 `upros_class_code` 中的基础包，例如：

- `upros_bringup`
- `w2u_navigation`
- `upros_message`

## 部署到小车

建议把两个工作空间都放在小车用户的家目录下，因为启动脚本会自动 source：

```bash
cd ~
git clone <clean_robot_code 仓库地址> clean_robot_code
git clone <upros_class_code 仓库地址> upros_class_code
```

如果小车上已经有 `~/upros_class_code`，可以只更新它，不必重复 clone。

## 编译顺序

先编译基础工程：

```bash
cd ~/upros_class_code
rosdepc install --from-paths src --ignore-src --rosdistro=noetic -y
catkin_make --pkg upros_message
catkin_make
```

再编译本工程：

```bash
cd ~/clean_robot_code
rosdepc install --from-paths src --ignore-src --rosdistro=noetic -y
catkin_make
```

## 运行 B 赛项 W2A 桌面清洁流程

```bash
cd ~/clean_robot_code
chmod +x run_clean_desktop_w2a.sh
./run_clean_desktop_w2a.sh
```

脚本会依次执行：

```bash
roslaunch clean_desktop_robot clean_desktop_robot_w2a.launch
roslaunch clean_desktop_robot view_grab.launch
roslaunch clean_desktop_robot complete_flow.launch
```

启动 `complete_flow.launch` 前，脚本会暂停等待人工确认。需要在 RViz 中检查：

- `/map` 是否正常。
- `/scan` 或 `/scan_filtered` 是否正常。
- TF 是否正常，重点检查 `map -> odom_combined -> base_footprint -> base_link`。
- `/move_base/local_costmap/costmap` 中小车是否没有压在障碍物里。
- 是否能看到 `tag_1`，且 `base_link -> tag_1` 稳定。

确认无误后按回车继续。

## 手动 source 环境

如果不使用启动脚本，或新开终端手动调试，需要先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/upros_class_code/devel/setup.bash
source ~/clean_robot_code/devel/setup.bash
```

可以用下面命令检查 ROS 包是否能找到：

```bash
rospack find clean_desktop_robot
rospack find upros_bringup
rospack find w2u_navigation
```

## 常见问题

### 找不到 ROS 包

重新 source 两个工作空间：

```bash
source ~/upros_class_code/devel/setup.bash
source ~/clean_robot_code/devel/setup.bash
```

如果仍然找不到，重新按编译顺序执行 `catkin_make`。

### 找不到 `upros_message`

`clean_desktop_robot` 编译依赖 `upros_message`，需要先在 `upros_class_code` 中单独编译消息包：

```bash
cd ~/upros_class_code
catkin_make --pkg upros_message
catkin_make
```

### 脚本中文显示乱码

确认终端使用 UTF-8：

```bash
export LANG=zh_CN.UTF-8
```

不影响 ROS 节点运行。

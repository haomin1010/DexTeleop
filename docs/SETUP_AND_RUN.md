# 环境与执行

## 1. 环境准备

DexProj 运行时通常需要两层环境：

- `conda`：给 DexProj 本地 Python 编排、检查和冒烟测试用
- ROS2 工作区：给 `wuji-hand-teleop`、`camera`、`openvr_input`、`controller` 等硬件节点用

这两层环境可能会在 `PYTHONPATH`、`PATH`、`LD_LIBRARY_PATH` 上互相影响，所以建议固定顺序：

1. 先 `conda activate dexproj`
2. 再 `source /opt/ros/<ros_distro>/setup.bash`
3. 再 `source <workspace>/install/setup.bash`

如果你发现某个 ROS2 包和 conda 里的 Python 包冲突，优先以 ROS2 工作区里的依赖为准。

## 1.0 拉取仓库

如果你是从零开始，先克隆主仓库，再初始化子模块：

```bash
git clone <DexProj-repo-url>
cd DexProj
git submodule update --init --recursive
```

如果你是第一次拿到这个仓库，建议先确认：

- `wuji-hand-teleop/` 已经被拉成 submodule
- `wuji-retargeting/` 已经被拉成 submodule
- `TJ/` 只作为历史参考，不需要参与正常执行

## 1.1 什么是 bringup

`bringup` 就是把底层 ROS2 硬件链路一次性拉起来的启动层。

在 DexProj 里，它主要负责：

- 启动机械臂相关 ROS2 launch
- 启动手部相关节点
- 按需启动相机
- 按需启动 RViz

如果你只是跑完整的遥操+采集，通常不需要单独执行 bringup，因为 `run_session` 会一起拉起它。
只有在你想单独调试硬件节点时，才单独跑 `bringup_teleop.sh`。

## 2. 创建 Conda 环境

```bash
conda create -n dexproj python=3.10 -y
conda activate dexproj
```

如果你后面要安装 `wuji-retargeting` 的 Python 依赖，建议也在这个环境里装。

## 3. 安装 Python 依赖

DexProj 本地代码目前主要依赖：

- `PyYAML`
- `inputs`（如果要用手柄触发）
- `pytest`（如果要补本地测试）
- `numpy`（`wuji-hand-teleop` 的多个 Python 包会用到）
- ROS2 Python 包运行时依赖由 ROS2 环境提供

```bash
pip install pyyaml inputs pytest numpy
```

如果后面你要加更多本地工具，也都优先装在这个 conda 环境里。

如果你需要同时跑 `wuji-retargeting`，再到对应子模块里装它自己的依赖：

```bash
cd wuji-retargeting
pip install -r requirements.txt
```

如果你要频繁改 `wuji-retargeting`，也可以在它子模块目录内单独装开发依赖。

`wuji-hand-teleop` 这边虽然没有统一的 `requirements.txt`，但它的多个 ROS2 Python 包本身也会依赖：

- `numpy`
- `pyyaml`
- `setuptools`

另外它更重要的一部分依赖其实在 ROS2 工作区和系统包里，例如：

- `rclpy`
- `launch`
- `launch_ros`
- `tf2_ros`
- `sensor_msgs`
- `cv_bridge`
- `realsense2_camera`
- `usb_cam`

所以可以简单理解成：

- `wuji-retargeting` 更偏 `pip` 依赖
- `wuji-hand-teleop` 更偏 ROS2 工作区依赖

## 4. ROS2 环境

先激活 conda，再 source ROS2：

```bash
source /opt/ros/<ros_distro>/setup.bash
source /path/to/your_ros2_ws/install/setup.bash
```

如果 `wuji-hand-teleop`、`camera`、`controller`、`openvr_input` 都在同一个工作区里，也要 source 那个工作区的 `install/setup.bash`。

如果你还没编译过工作区，先在 ROS2 工作区里执行一次 `colcon build`，再 source `install/setup.bash`。

如果你担心环境变量冲突，可以记住一句话：

- Python 依赖装在 conda
- ROS2 节点和 launch 用 ROS2 工作区
- 真正跑硬件前再 source ROS2，不要反过来乱叠

## 5. 推荐执行顺序

1. `conda activate dexproj`
2. `source /opt/ros/<ros_distro>/setup.bash`
3. `source <workspace>/install/setup.bash`
4. `git submodule update --init --recursive`
5. `./scripts/check_devices.sh`
6. `./scripts/run_session.sh`

## 6. 常用命令

### 设备检查

```bash
./scripts/check_devices.sh
```

### 启动会话

```bash
./scripts/run_session.sh
```

### 单独拉起 bringup

```bash
./scripts/bringup_teleop.sh
```

这一步不是必须的。它主要用于单独调试机械臂、手和相机是否能正常启动。

### 拉取子模块

```bash
git submodule update --init --recursive
```

如果子模块远端有更新，你还可以在主仓库里执行：

```bash
git submodule update --remote --merge
```

## 7. ROS2 常用命令

### 查看 topic

```bash
ros2 topic list
```

### 查看单个 topic

```bash
ros2 topic echo /cam_head/color/image_raw
```

### 查看节点

```bash
ros2 node list
```

### 查看节点图

```bash
rqt_graph
```

## 8. 说明

- `conda` 负责 DexProj 的 Python 编排层和本地依赖
- ROS2 负责真实硬件节点和 launch
- 两者都需要，不是二选一
- `trigger_mode: both` 时，默认同时支持手柄和键盘触发

# Ubuntu 22.04 原生部署指南

这份文档说明如何在一台全新的 Ubuntu 22.04 机器上，不使用 Docker，直接部署并运行当前仓库这套 DexProj + `wuji-hand-teleop` 遥操作链路。

目标是把下面两类流程都跑通：

1. Tianji 真机手臂 + HTC Vive Tracker + Wuji glove 的原生 bringup。
2. DexProj session 录制流程，也就是 [run_session.sh](/home/user/workspace/DexProj_back_up_0602/scripts/run_session.sh) 对应的原生版本。

这份文档基于当前仓库代码整理，不是假设一个“理想中的未来环境”。

## 1. 范围和结论

原生部署是可行的，但有两个前提要先说清楚：

1. Ubuntu 22.04 只是让 ROS 2 Humble 能原生安装，不会自动替你解决 SteamVR、OpenVR、Wuji SDK、工作区构建和 Python 环境问题。
2. 仓库里有些脚本默认会强制走 Docker；原生部署时要么给它们加环境变量绕过，要么直接使用它们等价的底层命令。

当前仓库里和 Docker 的关系现在是这样：

- [bringup_teleop.sh](/home/user/workspace/DexProj_back_up_0602/scripts/bringup_teleop.sh)、[run_session.sh](/home/user/workspace/DexProj_back_up_0602/scripts/run_session.sh)、[check_devices.sh](/home/user/workspace/DexProj_back_up_0602/scripts/check_devices.sh)、[get_wuji_glove_sn.sh](/home/user/workspace/DexProj_back_up_0602/scripts/get_wuji_glove_sn.sh) 可以通过设置 `DEXPROJ_RUNNING_IN_CONTAINER=1` 直接在宿主机运行。
- [run_tianji_real_teleop.sh](/home/user/workspace/DexProj_back_up_0602/scripts/run_tianji_real_teleop.sh) 现在已经支持 native 分支；在宿主机上设置 `DEXPROJ_RUNNING_IN_CONTAINER=1` 或 `DEXPROJ_NATIVE_MODE=1` 后，会直接走原生 ROS 工作区，不再强制进 Docker。

## 2. 目录约定

建议在新机器上直接采用下面的目录布局，能最大限度减少路径改动：

```bash
/home/<user>/workspace/DexProj
/home/<user>/ros2_ws
```

下文默认：

- DexProj 根目录是 `/home/<user>/workspace/DexProj`
- ROS 工作区根目录是 `/home/<user>/ros2_ws`

如果你使用别的路径，后面所有绝对路径都要按你的机器改掉。

## 3. 机器准备

### 3.1 系统和账号

- Ubuntu 22.04 LTS
- 普通用户登录，不建议长期用 root 跑
- 能 `sudo`

### 3.2 建议的基础软件

```bash
sudo apt update
sudo apt install -y \
  git git-lfs curl wget vim tmux ffmpeg adb \
  net-tools iputils-ping usbutils udev kmod \
  build-essential cmake pkg-config \
  software-properties-common ca-certificates gnupg lsb-release locales \
  python3-pip python3-dev python3-pybind11 python3-venv \
  libusb-1.0-0-dev libncursesw5-dev libgl1-mesa-glx libgl1-mesa-dri \
  fonts-noto-cjk libspdlog-dev
```

然后初始化 Git LFS：

```bash
git lfs install
```

## 4. 安装 ROS 2 Humble

### 4.1 官方安装方式

ROS 2 Humble 官方文档明确支持 Ubuntu 22.04 Jammy，官方推荐 deb 包安装。

参考官方文档：

- ROS 2 Humble 安装总览: https://docs.ros.org/en/humble/Installation.html
- Ubuntu 22.04 deb 安装: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

按官方流程，至少需要做：

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

export ROS_APT_SOURCE_VERSION=$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F'"' '{print $4}'
)
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
```

注意：ROS 官方文档特别提醒，在全新 22.04 系统上，安装 ROS 2 之前应先做 `apt upgrade`，否则可能触发 `systemd` / `udev` 相关包的异常移除。

### 4.2 补充 ROS 包

本项目不是只要 `ros-humble-desktop` 就够，建议额外装上这些包：

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-realsense2-camera \
  ros-humble-usb-cam \
  ros-humble-rmw-cyclonedds-cpp
```

初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

## 5. 克隆仓库

```bash
mkdir -p ~/workspace
cd ~/workspace
git clone <your-dexproj-repo-url> DexProj
cd DexProj
git submodule update --init --recursive
git lfs pull
```

你需要确认两个子模块确实已经拉下来：

- `wuji-hand-teleop/`
- `wuji-retargeting/`

## 6. 安装 Steam 和 SteamVR

HTC Vive Tracker 方案依赖 SteamVR，而且这是原生部署里最容易被忽略的一段。

### 6.1 安装 Steam

Ubuntu 22.04 上优先使用系统包或应用商店安装 Steam。命令行常见做法是：

```bash
sudo apt install -y steam-installer
```

如果你的软件源里包名不是 `steam-installer`，就直接用 Ubuntu Software 安装 Steam。

### 6.2 安装 SteamVR

打开 Steam，在库里安装 SteamVR。

安装后确认目录存在：

```bash
ls ~/.steam/debian-installation/steamapps/common/SteamVR/
```

### 6.3 配置无头模式

当前 HTC tracker 方案默认是无头模式，需要启用 null driver。可以参考仓库里的 [STEAMVR.md](/home/user/workspace/DexProj_back_up_0602/wuji-hand-teleop/docker/STEAMVR.md)。

关键设置有两处：

1. `~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings`
2. `~/.steam/debian-installation/config/steamvr.vrsettings`

核心目标是保证：

- `driver_null.enable = true`
- `requireHmd = false`
- `forcedDriver = "null"`
- `activateMultipleDrivers = true`

### 6.4 Wayland 机器的启动方式

如果新机器是 Ubuntu 默认的 Gnome Wayland，会比 X11 更容易出 SteamVR 启动问题。建议按仓库现有文档中的方式启动：

```bash
GDK_BACKEND=x11 QT_QPA_PLATFORM=xcb steam steam://rungameid/250820
```

### 6.5 验证 SteamVR

```bash
ps aux | grep vrserver
grep "null" ~/.steam/debian-installation/logs/vrserver.txt | tail -3
```

## 7. 安装 Wuji Hand SDK

### 7.1 安装 `wujihandcpp`

`wujihandros2` 的 C++ 驱动依赖 `wujihandcpp`。仓库里也明确写了缺它会 build fail。

推荐先安装与当前仓库比较接近的版本：

```bash
cd /tmp
wget https://github.com/wuji-technology/wujihandpy/releases/download/v1.5.1/wujihandcpp-1.5.1-amd64.deb
sudo apt install -y ./wujihandcpp-1.5.1-amd64.deb
```

安装后验证：

```bash
dpkg -l | grep wujihandcpp
ls /usr/include/wujihandcpp
```

### 7.2 准备 Wuji glove 标定数据

当前 Docker 方案会把宿主机的 `~/.wuji/sdk/params` 和 `~/.wuji/sdk/models` 同步进容器。原生部署时，这些数据就应该直接在宿主机上存在。

建议确认以下目录存在：

```bash
ls ~/.wuji/sdk/params
ls ~/.wuji/sdk/models
```

如果旧机器上已经跑通过，最稳妥的方法就是把整个 `~/.wuji/` 目录完整拷到新机器同一路径下。

## 8. 安装 Miniconda 和 DexProj Python 环境

虽然是原生部署，但当前仓库仍然假设有一层 conda 环境，名字通常叫 `dexproj`。

### 8.1 安装 Miniconda

```bash
cd /tmp
wget https://repo.anaconda.com/miniconda/Miniconda3-py310_25.3.1-1-Linux-x86_64.sh -O miniconda.sh
bash miniconda.sh -b -p "$HOME/miniconda3"
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda init bash
```

重开一个 shell，或者再次执行：

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
```

### 8.2 创建 `dexproj` 环境

```bash
conda create -y -n dexproj python=3.10
conda activate dexproj
```

### 8.3 一键准备 Python 环境

仓库里现在提供了一个原生 helper：

```bash
cd ~/workspace/DexProj
./scripts/setup_native_python_env.sh
```

它会：

- 安装 Miniconda 到 `~/miniconda3`，如果本机还没有
- 创建 conda env `dexproj`
- 安装当前项目所需 Python 包
- 把本地 `wuji-retargeting` 以 editable 模式装进这个环境

### 8.4 手工安装 Python 依赖

下面这组依赖是根据仓库当前脚本和容器环境整理出来的：

```bash
pip install --no-cache-dir \
  pyyaml pytest inputs numpy scipy \
  openvr==2.12.1401 \
  opencv-python==4.13.0.92 \
  wuji-sdk==0.10.0 \
  wujihandpy==1.7.0 \
  avp_stream==2.51 \
  pin==3.9.0 \
  nlopt==2.10.0 \
  empy==3.3.4 \
  catkin_pkg \
  lark
```

安装后建议验证：

```bash
python - <<'PY'
import yaml, openvr, numpy, scipy, cv2, inputs, wuji_sdk, wujihandpy
print("python deps ok")
PY
```

### 8.5 安装本地 `wuji-retargeting`

当前流程里，手套到灵巧手重定向会直接调用 `wuji-retargeting` 下的 `teleop_real.py`，所以需要把这个子模块装进当前 conda 环境。

```bash
cd ~/workspace/DexProj/wuji-retargeting
pip install -e .
```

如果遇到构建元数据问题，也可以参考 Dockerfile 里的做法自行补 `setup.py` shim；但多数情况下 `pip install -e .` 应该足够。

## 9. 建立 ROS 工作区

### 9.1 创建工作区

```bash
mkdir -p ~/ros2_ws/src
```

### 9.2 把 `wuji-hand-teleop/src` 接到工作区

当前容器里的做法是把 `DexProj/wuji-hand-teleop/src` 挂到 `/home/wuji/ros2_ws/src`。原生部署建议直接用软链接，保持仓库内代码单一来源：

```bash
ln -sfn ~/workspace/DexProj/wuji-hand-teleop/src ~/ros2_ws/src/wuji-hand-teleop-src
```

### 9.3 可选：屏蔽不需要的包

如果你明确只跑当前这条链路：

- HTC Vive Tracker
- Wuji glove
- Tianji arm
- 可选 camera

那么 Manus、PICO 那些路径暂时不必作为主链路依赖。

如果后面 build 遇到旁支包报错，可以只构建项目真正需要的这几个包，而不是全量 `colcon build`。

## 10. 准备原生环境激活顺序

当前仓库推荐的环境叠加顺序是：

1. conda
2. `/opt/ros/humble`
3. `~/ros2_ws/install/setup.bash`

可以直接复用 [activate_dexproj_env.sh](/home/user/workspace/DexProj_back_up_0602/scripts/activate_dexproj_env.sh)，但要给它正确的环境变量：

```bash
export DEXPROJ_RUNNING_IN_CONTAINER=1
export DEXPROJ_CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
export DEXPROJ_ROS_WS_SETUP="$HOME/ros2_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
```

然后：

```bash
cd ~/workspace/DexProj
source scripts/activate_dexproj_env.sh
```

如果这里没有报错，继续验证：

```bash
which python3
which ros2
echo "$CONDA_DEFAULT_ENV"
```

预期是：

- `python3` 指向 conda env
- `ros2` 可用
- `CONDA_DEFAULT_ENV=dexproj`

## 11. 构建 ROS 工作区

### 11.0 一键准备 ROS 工作区

仓库里现在也提供了原生 helper：

```bash
cd ~/workspace/DexProj
./scripts/setup_native_ros_workspace.sh
```

它会：

- 创建或复用 `~/ros2_ws`
- 把 `DexProj/wuji-hand-teleop/src` 软链接进工作区
- 执行 `rosdep install`
- 构建当前主链路需要的 ROS 包

默认会连 `camera` 一起 build。如果你想先只打通手臂和手套链路：

```bash
DEXPROJ_BUILD_CAMERA=0 ./scripts/setup_native_ros_workspace.sh
```

### 11.1 先装 rosdep 依赖

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 11.2 构建最小必需包

当前项目原生真机链路最稳妥的 build 命令是：

```bash
cd ~/workspace/DexProj
export DEXPROJ_RUNNING_IN_CONTAINER=1
export DEXPROJ_CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
export DEXPROJ_ROS_WS_SETUP="$HOME/ros2_ws/install/setup.bash"
source scripts/activate_dexproj_env.sh

cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  common_input \
  openvr_input \
  wuji_glove_input_py \
  wujihand_output \
  tianji_output \
  controller \
  wuji_teleop_bringup \
  camera \
  --cmake-args -DPYTHON_EXECUTABLE:FILEPATH="$(command -v python3)"
```

如果暂时不需要相机，可以把 `camera` 去掉。

构建成功后：

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix wuji_teleop_bringup
```

## 12. 配置相机

如果你只想先把手臂和手套链路跑通，这一节可以先跳过。

如果你需要完整 session 采集，建议把相机一次性配置好。

### 12.1 安装 udev 规则

```bash
cd ~/workspace/DexProj/wuji-hand-teleop/src/camera
bash setup_cameras.sh
```

它会安装 [99-teleop-cameras.rules](/home/user/workspace/DexProj_back_up_0602/wuji-hand-teleop/src/camera/config/udev/99-teleop-cameras.rules) 并尝试创建：

- `/dev/stereo_camera`
- `/dev/cam_left_wrist`
- `/dev/cam_right_wrist`

### 12.2 修改 wrist 相机序列号

编辑 [camera_config.yaml](/home/user/workspace/DexProj_back_up_0602/wuji-hand-teleop/src/camera/config/camera_config.yaml)，填入：

- `left_wrist.serial_number`
- `right_wrist.serial_number`

如果换了 D405，还要同步修改 udev rules 里的 `ATTRS{serial}`。

## 13. 配置 OpenVR 和 tracker

### 13.1 填 tracker 序列号

编辑 [openvr_input.yaml](/home/user/workspace/DexProj_back_up_0602/wuji-hand-teleop/src/input_devices/openvr_input/config/openvr_input.yaml) 或 [htc_openvr_tracker.yaml](/home/user/workspace/DexProj_back_up_0602/config/htc_openvr_tracker.yaml)，填入你的新机器上实际 tracker SN。

最少要保证：

- `chest`
- `right_wrist`
- `right_arm`

### 13.2 获取 tracker SN

在 SteamVR 已经启动、tracker 已连接的前提下：

```bash
python3 -c "import openvr; openvr.init(openvr.VRApplication_Other); vr=openvr.VRSystem(); [print(vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)) for i in range(64) if vr.getTrackedDeviceClass(i)==openvr.TrackedDeviceClass_GenericTracker and vr.isTrackedDeviceConnected(i)]; openvr.shutdown()"
```

## 14. 配置 Wuji glove 和灵巧手序列号

### 14.1 自动写入 glove SN

原生机上可以直接跑：

```bash
cd ~/workspace/DexProj
export DEXPROJ_RUNNING_IN_CONTAINER=1
source scripts/activate_dexproj_env.sh
./scripts/get_wuji_glove_sn.sh --update-config config/hand_teleop_wuji_glove.yaml
```

### 14.2 检查 hand SN

编辑 [hand_teleop_wuji_glove.yaml](/home/user/workspace/DexProj_back_up_0602/config/hand_teleop_wuji_glove.yaml)，确认：

- `hands.left.hand_sn`
- `hands.right.hand_sn`

这些是灵巧手硬件的 SN，不一定会自动更新。

## 15. 修改原生机必须调整的配置

这是 native 部署里以前最容易忽略的一步，但仓库现在已经把主配置改成了 repo 相对路径。

当前 [bringup_htc.yaml](/home/user/workspace/DexProj_back_up_0602/config/bringup_htc.yaml) 会通过 `dexproj.integration.bringup` 自动解析：

- 相对路径
- 容器遗留的 `/workspace/DexProj/...` 路径

也就是说，只要主仓库路径本身正确，这一项通常不需要你再手工改绝对路径。

## 16. 设备预检查

这是原生 bringup 之前最推荐做的一步。

```bash
cd ~/workspace/DexProj
export DEXPROJ_RUNNING_IN_CONTAINER=1
source scripts/activate_dexproj_env.sh
./scripts/check_devices.sh
```

它会检查三类东西：

- Wuji hands
- Wuji gloves
- HTC trackers

如果 `OpenVR tracker scan failed`，优先去检查：

- SteamVR 是否真的在宿主机运行
- tracker 是否已经配对
- base station 是否工作
- 你的 `openvr` Python 包是否安装在当前 conda env

## 17. 原生启动流程

### 17.1 最小真机 bringup

这是 native 22.04 上最建议先跑通的第一条链路。

```bash
cd ~/workspace/DexProj
export DEXPROJ_RUNNING_IN_CONTAINER=1
export DEXPROJ_CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
export DEXPROJ_ROS_WS_SETUP="$HOME/ros2_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
source scripts/activate_dexproj_env.sh

ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py \
  arm_input:=tracker \
  dry_run:=true \
  read_only:=false \
  feedback_handshake:=false \
  sdk_executor_enable:=true \
  sim_viz:=true \
  enable_rviz:=true \
  controller_config:=/home/<user>/workspace/DexProj/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output_real_teleop.yaml \
  openvr_config:=/home/<user>/workspace/DexProj/wuji-hand-teleop/src/input_devices/openvr_input/config/openvr_input.yaml
```

也可以直接使用脚本：

```bash
cd ~/workspace/DexProj
export DEXPROJ_RUNNING_IN_CONTAINER=1
export DEXPROJ_ROS_WS_ROOT="$HOME/ros2_ws"
source scripts/activate_dexproj_env.sh
./scripts/run_tianji_real_teleop.sh
```

它内部会先在原生工作区里增量 build，再 launch 真机链路。

### 17.2 用 `bringup_teleop.sh` 走原生入口

当你确认配置文件里的绝对路径都已经改好之后，可以直接用仓库脚本：

```bash
cd ~/workspace/DexProj
export DEXPROJ_RUNNING_IN_CONTAINER=1
source scripts/activate_dexproj_env.sh
./scripts/bringup_teleop.sh --dry-run --skip-preflight
./scripts/bringup_teleop.sh --skip-preflight
```

如果不想跳过预检查，就去掉 `--skip-preflight`。

### 17.3 启动完整 session 录制

```bash
cd ~/workspace/DexProj
export DEXPROJ_RUNNING_IN_CONTAINER=1
source scripts/activate_dexproj_env.sh
./scripts/run_session.sh --task "<your_task_name>"
```

当前默认配置来自：

- [session_htc_wuji_glove.yaml](/home/user/workspace/DexProj_back_up_0602/config/session_htc_wuji_glove.yaml)
- [bringup_htc.yaml](/home/user/workspace/DexProj_back_up_0602/config/bringup_htc.yaml)
- [hand_teleop_wuji_glove.yaml](/home/user/workspace/DexProj_back_up_0602/config/hand_teleop_wuji_glove.yaml)

## 18. 推荐验证顺序

不要一上来就跑完整 session，建议按这个顺序推进：

1. `source scripts/activate_dexproj_env.sh` 成功。
2. `python3 -c "import openvr, wuji_sdk, wujihandpy"` 成功。
3. `./scripts/check_devices.sh` 能看到 tracker 和 glove。
4. `ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py ...` 能起。
5. 再跑 `./scripts/bringup_teleop.sh`。
6. 最后再跑 `./scripts/run_session.sh`。

## 19. 常见问题

### 19.1 `bringup_teleop.sh` 还是进 Docker

说明你没设置：

```bash
export DEXPROJ_RUNNING_IN_CONTAINER=1
```

### 19.2 `run_tianji_real_teleop.sh` 还是进 Docker

说明你没有给它 native 标记。先确认：

```bash
export DEXPROJ_RUNNING_IN_CONTAINER=1
```

或者：

```bash
export DEXPROJ_NATIVE_MODE=1
```

### 19.3 `Package not found`

先检查：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep wuji_teleop_bringup
```

如果没有，说明工作区没 build 成功，或者 build 后没 source。

### 19.4 `wujihandcpp not found`

重装 `wujihandcpp` deb，并确认：

```bash
ls /usr/include/wujihandcpp
ldconfig -p | grep wujihandcpp
```

### 19.5 `OpenVR tracker scan failed`

优先检查：

1. SteamVR 是否正在宿主机运行。
2. tracker 是否配对成功。
3. base station 是否亮绿灯并覆盖到 tracker。
4. 当前 shell 是否已经 `conda activate dexproj`。
5. `python -c "import openvr"` 是否成功。

### 19.6 配置路径还是 `/workspace/DexProj`

这是容器时代遗留路径。所有这种绝对路径都要替换成新机器上的真实路径。

## 20. 建议保留的环境变量

建议在 native 机器上长期保留下面这些变量，可以写进 `~/.bashrc`：

```bash
export DEXPROJ_RUNNING_IN_CONTAINER=1
export DEXPROJ_CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
export DEXPROJ_ROS_WS_SETUP="$HOME/ros2_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
```

如果你不想每次都手动 source，可以再定义一个 helper：

```bash
dexproj_env() {
  cd ~/workspace/DexProj || return 1
  source scripts/activate_dexproj_env.sh
}
```

## 21. 这份文档没有覆盖的内容

这份文档聚焦你现在最关心的链路：

- Ubuntu 22.04
- 不用 Docker
- HTC Vive Tracker
- Wuji glove
- Tianji arm
- DexProj session

它没有展开写的内容：

- PICO 路线
- Manus 路线
- 多机 ROS 2 联网
- 更复杂的自定义 camera 拓扑

如果后面要把这份 native 文档继续细化，最值得补的下一步是把第 17 节里的手工启动命令再封装成一套 `*_native.sh` 脚本。

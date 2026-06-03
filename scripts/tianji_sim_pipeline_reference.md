# Tianji 仿真遥操作链路 — 文件与功能对照

本文档把 **当前金标准仿真链路**（`read_only` + TJ SDK IK + `mapped_tf`）从启动命令到每个源文件、话题、采集物逐项对齐。路径均相对于仓库根目录 `DexProj/`。

---

## 1. 一条命令启动什么

**推荐启动（与 baseline 采集一致）：**

```bash
./scripts/ensure_docker_exec.sh -- bash -lc '
  source scripts/activate_dexproj_env.sh
  cd /home/wuji/ros2_ws && source install/setup.bash
  ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py \
    arm_input:=tracker \
    read_only:=true \
    feedback_handshake:=true \
    sim_viz:=true \
    enable_rviz:=true \
    enable_mujoco:=true \
    controller_config:=/workspace/DexProj/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output_sim.yaml \
    openvr_config:=/workspace/DexProj/wuji-hand-teleop/src/input_devices/openvr_input/config/openvr_input.yaml
'
```

**主机上一键采集：**

```bash
./scripts/run_tianji_sim_baseline_capture.sh
```

---

## 2. 数据流总览

```text
┌─────────────────────────────────────────────────────────────────────────┐
│ 输入：HTC Vive / SteamVR                                                 │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ openvr_input 节点                                                        │
│  文件: wuji-hand-teleop/src/input_devices/openvr_input/                  │
│  配置: .../config/openvr_input.yaml (tracker SN, wrist_offset)           │
│  输出 TF (parent=world): chest, right_wrist, right_arm, ...              │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                    static_transforms (chest→right_chest, wrist→tianji_right)
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ tianji_arm_controller 节点 (package: controller)                         │
│  文件: wuji-hand-teleop/src/controller/controller/tianji_arm_node.py    │
│  库:   .../tianji_output/tianji_arm_controller.py + Marvin SDK (.so)     │
│  配置: tianji_output_sim.yaml                                            │
│  · 查 TF: right_chest → tianji_right                                     │
│  · 增量零点: 当前 tracker ↔ 真机 FK (需连机 + feedback)                  │
│  · IK: move_to_matrix_direct (SDK, 非 Pinocchio)                         │
│  · read_only: 算 IK、发 joint_command，不向真机 send_cmd                  │
└─────────────────────────────────────────────────────────────────────────┘
          │                              │
          │ joint_command              │ joint_state (真机反馈)
          ▼                              ▼
┌──────────────────────┐    ┌──────────────────────────────────────────┐
│ tianji_mujoco_viewer │    │ 物理 Tianji (192.168.8.166)               │
│ 跟 command 显示      │    │ read_only 下关节不动，只提供反馈           │
└──────────────────────┘    └──────────────────────────────────────────┘
          │
          │ sim_viz → RViz markers; enable_rviz → OpenVR TF 视图
          ▼
```

**关键结论（避免表象误判）：**

| 话题 | 含义 | 仿真里谁用它 |
|------|------|----------------|
| `/tianji_arm/right/joint_command` | IK **目标**关节 (deg) | **MuJoCo 跟这条** |
| `/tianji_arm/right/joint_state` | 真机 **反馈**关节 (deg) | 零点/FK、MuJoCo **初值**；read_only 下不变 |
| `/tf` | Tracker + chest + tianji 帧 | 控制与 RViz |

---

## 3. 按层：文件 ↔ 功能

### 3.1 环境与容器

| 文件 | 功能 |
|------|------|
| `scripts/ensure_docker_exec.sh` | 同步 `DexProj` 进容器 `wuji22-hand`，同步 `~/.wuji/sdk` 标定，在容器内执行命令 |
| `scripts/activate_dexproj_env.sh` | 容器内：ROS2 workspace、`colcon` 环境变量 |
| `wuji-hand-teleop/docker/docker-compose.yml` | Docker 服务定义（显示、设备等） |
| `wuji-hand-teleop/docker/Dockerfile` | 镜像构建 |
| `scripts/setup_ros_workspace.sh` | 主机/容器 ROS workspace 初始化（按需） |

容器内工作空间默认：`/home/wuji/ros2_ws`（已 install 的 `controller`、`tianji_output`、`wuji_teleop_bringup` 等）。

---

### 3.2 Launch 编排

| 文件 | 功能 |
|------|------|
| `wuji-hand-teleop/src/wuji_teleop_bringup/launch/wuji_teleop_arm.launch.py` | **仿真臂主 launch**：按参数起 OpenVR、static TF、tianji 控制器、MuJoCo、RViz、sim_viz |
| `wuji-hand-teleop/src/wuji_teleop_bringup/wuji_teleop_bringup/tf_utils.py` | 从 yaml 生成 `static_transform_publisher` 节点（胸架、wrist→tianji） |
| `wuji-hand-teleop/src/wuji_teleop_bringup/package.xml` | 依赖 `openvr_input`、`controller`、`tianji_urdf` 等 |

**Launch 参数（仿真常用）：**

| 参数 | 默认 | 仿真金标准 |
|------|------|------------|
| `arm_input` | `tracker` | `tracker` |
| `read_only` | `false` | **`true`** |
| `feedback_handshake` | `false` | **`true`** |
| `controller_config` | `tianji_output.yaml` | **`tianji_output_sim.yaml`** |
| `openvr_config` | `openvr_input.yaml` | 同左 |
| `sim_viz` | `false` | **`true`** |
| `enable_mujoco` | `false` | **`true`** |
| `enable_rviz` | `false` | **`true`** |
| `enable_tianji_model` | `false` | 通常 **false**（RViz 不播 URDF 关节模型） |

---

### 3.3 输入：OpenVR Trackers

| 文件 | 功能 |
|------|------|
| `wuji-hand-teleop/src/input_devices/openvr_input/openvr_input/openvr_input_node.py` | ROS 节点：定时读 tracker，广播 TF |
| `wuji-hand-teleop/src/input_devices/openvr_input/openvr_input/openvr_tracker_wrapper.py` | OpenVR API、腕部坐标校正、`right_arm` 无额外校正 |
| `wuji-hand-teleop/src/input_devices/openvr_input/config/openvr_input.yaml` | **Tracker 序列号**、`wrist_offset`、`publish_rate_hz`、TF parent=`world` |
| `wuji-hand-teleop/src/input_devices/openvr_input/rviz/openvr_visualization.rviz` | `enable_rviz:=true` 时加载的 RViz 布局 |
| `wuji-hand-teleop/src/input_devices/openvr_input/setup.py` | 注册可执行文件 `openvr_input` |

**OpenVR 发布的 TF 帧（parent=`world`）：**

- `chest`、`head`（胸 tracker + Z 偏移）
- `right_wrist`、`left_wrist`（若配置）
- `right_arm`、`left_arm`（若配置 SN）

---

### 3.4 静态 TF：胸架与 wrist→Tianji

| 文件 | 功能 |
|------|------|
| `wuji-hand-teleop/src/wuji_teleop_bringup/config/static_transforms.yaml` | `chest`→`right_chest_base`→`right_chest`；`right_wrist`→`tianji_right`（含四元数映射） |

**控制用的末端帧：** `tianji_right`（不是裸 `right_wrist`）。

链：`world → chest → right_chest → …` 与 `world → right_wrist → tianji_right` 在 `right_chest` 下合成。

---

### 3.5 控制核心：`tianji_arm_controller` 节点

| 文件 | 功能 |
|------|------|
| `wuji-hand-teleop/src/controller/controller/tianji_arm_node.py` | **主 ROS 节点**：TF→目标位姿→IK→发布 command/state/ee_pose/zsp |
| `wuji-hand-teleop/src/controller/controller/common.py` | 配置加载、QoS、`ControlMode` 等共用工具 |
| `wuji-hand-teleop/src/controller/setup.py` | 注册 `tianji_arm_controller`、`tianji_mujoco_viewer` 等入口 |
| `wuji-hand-teleop/src/controller/package.xml` | 依赖 `tianji_output`、`tf2_ros` 等 |

**下游库（同进程内调用，非独立 ROS 节点）：**

| 文件 | 功能 |
|------|------|
| `wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/tianji_arm_controller.py` | Marvin 连接、`move_to_matrix_direct` / `move_to_pose_direct`、IK、`read_only` 分支 |
| `wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/_internal/fx_kine.py` | SDK 运动学/IK 封装（`ik()`、`zsp_type`/`zsp_para`） |
| `wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/_internal/fx_robot.py` | SDK 通信、`send_cmd`、`subscribe` 反馈 |
| `wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/_internal/lib/libMarvinSDK.so` | Marvin 动态库（运行时加载） |
| `wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/ccs_m*.MvKDCfg` | 机型运动学配置 |

**仿真金标准 YAML：**

| 文件 | 功能 |
|------|------|
| `.../config/tianji_output_sim.yaml` | **冻结仿真配置**（与采集 `config_snapshot` 一致） |
| `.../config/tianji_output.yaml` | 当前默认；内容与 sim 对齐，注释说明金标准 |
| `.../config/tianji_output_real.yaml` | 真机试验用（`read_only: false` 等），**不是**当前 sim 金标准 |

**`tianji_arm_node.py` 内与仿真相关的逻辑要点：**

| 函数/块 | 作用 |
|---------|------|
| `_teleop_control()` | 每周期：读 `right_arm`（zsp）、读 `tianji_right`、解 IK、发布 |
| `_resolve_tracker_pose()` | `incremental` + `full_pose`：tracker 增量 → `right_pose_mat` (mm) |
| `_lookup_orientation_source_tf()` | `mapped_tf` 时姿态仍用 `tianji_right`；仅 `raw_wrist` 才拆腕帧 |
| `_current_robot_fk_matrix()` | 零点：真机 `joint_state` → SDK FK |
| `_publish_command()` / `_publish_state()` | 发布 `joint_command` / `joint_state` |
| `move_to_matrix_direct()`（controller 内） | SDK 4×4 矩阵 IK；`read_only` 时只 log `[READ_ONLY_POSE]` |

**`right_arm` 在仿真中的实际用途：**

- **不**进入 TCP 目标姿态
- 仅 `right_y_axis` → `controller.right_zsp_para[0:3]`（IK 零空间肘平面提示）

---

### 3.6 仿真可视化

| 文件 | 功能 | Launch 条件 |
|------|------|-------------|
| `wuji-hand-teleop/src/controller/controller/tianji_mujoco_viewer_node.py` | MuJoCo 窗口：初值 `joint_state`，之后跟 **`joint_command`** | `enable_mujoco:=true` |
| `wuji-hand-teleop/src/controller/controller/tianji_tracker_sim_viz_node.py` | RViz `MarkerArray`：`right_chest` / `right_wrist` / `tianji_right` / `right_arm` | `sim_viz:=true` |
| `wuji-hand-teleop/src/controller/controller/tianji_joint_state_bridge_node.py` | 把 `joint_state` 名字映射到 URDF，供 `robot_state_publisher` | `enable_tianji_model:=true`（仿真通常不开） |

**机器人模型 URDF（MuJoCo / 可选 RViz 模型）：**

- ROS 包 `tianji_urdf`：`share/tianji_urdf/urdf/right.urdf`（由 `colcon` 安装，不在 DexProj 源码树里时常从 `/home/wuji/ros2_ws/install` 引用）

---

### 3.7 诊断工具（可选，非主链路）

| 文件 | 功能 |
|------|------|
| `wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/tools/debug_arm_axis.py` | 打印 `right_chest`→`right_arm` / `tianji_right`（采集脚本会起，包内可能未 install） |
| `scripts/log_wrist_frame_axes.py` | 腕/前臂坐标轴日志 |
| `scripts/calibrate_wrist_forearm_axes.py` | 腕-前臂标定辅助 |
| `scripts/check_right_arm_coordinate_response.py` | 右臂坐标响应检查 |

---

## 4. ROS 话题与服务（右臂）

| 话题 / 服务 | 类型 | 发布者 | 订阅者 / 用途 |
|-------------|------|--------|----------------|
| `/tianji_arm/right/joint_command` | `sensor_msgs/JointState` | `tianji_arm_controller` | **MuJoCo**、rosbag、真机对比金标准 |
| `/tianji_arm/right/joint_state` | `JointState` | `tianji_arm_controller` | MuJoCo 初值、零点 FK |
| `/tianji_arm/right/right_ee_pose` | `Float64MultiArray` | 同上 | 目标 XYZABC (m+deg) 调试 |
| `/tianji_arm/right/right_zsp_para` | `Float64MultiArray` | 同上 | 当前 zsp（含 arm Y） |
| `/tianji_tracker_sim/markers` | `MarkerArray` | `tianji_tracker_sim_viz` | RViz（sim_viz） |
| `/tf`, `/tf_static` | TF | openvr + static publishers | 控制器、RViz、rosbag |
| `/tianji_arm/reset_tracker_zero` | `Trigger` srv | — | 重置增量零点 |
| `/tianji_arm/switch_mode` | `SetBool` srv | — | TELEOP ↔ INFERENCE |

控制频率：由 `control_rate_hz`（yaml 里 **100 Hz**）定时器驱动。

---

## 5. 采集与对比（DexProj/scripts）

| 文件 | 功能 |
|------|------|
| `scripts/run_tianji_sim_baseline_capture.sh` | 容器内 colcon build + launch + rosbag/CSV/配置快照/`main.log` |
| `scripts/analyze_tianji_baseline_log.sh` | 统计 `READ_ONLY_POSE`、`RIGHT_IK_FAIL` 等 |
| `scripts/tianji_sim_baseline_motion_template.txt` | 动作步骤模板（pitch/yaw/roll 小大） |
| `scripts/tianji_motion_stamp.sh` | 第二终端给 capture 打 `motion_log.txt` 时间戳 |
| `scripts/compare_sim_real_baseline.md` | 仿真 vs 真机差异说明（简版） |

**一次采集产物目录**（例：`captures/tianji_sim_baseline_20260530_160010/`）：

| 路径 | 内容 |
|------|------|
| `main.log` | launch + 控制器全量日志 |
| `config_snapshot/` | 当时 yaml、`static_transforms`、`git_state.txt` |
| `baseline/` | rosbag2：`/tf`、`joint_command`、`joint_state`、`right_ee_pose`、`right_zsp_para` |
| `topics/*.csv` | 上述话题 CSV echo |
| `observers/` | `ros2_graph.txt`、`tf_sample.log`、`rosbag_record.log` 等 |
| `motion_script.txt` / `motion_notes.txt` / `motion_log.txt` | 动作记录 |

---

## 6. 金标准配置摘要（`tianji_output_sim.yaml`）

| 键 | 仿真值 | 含义 |
|----|--------|------|
| `read_only` | `true` | 连真机取反馈，**不发**运动指令 |
| `feedback_handshake` | `true` | 启动时发一次非运动 SDK 序列，保证 `joint_state` 有流 |
| `use_pinocchio_ik` | `false` | 走 **Marvin SDK** `move_to_matrix_direct` |
| `tracker_mode` | `incremental` | 首帧对齐真机 FK，之后用 tracker 增量 |
| `tracker_orientation_input_mode` | `mapped_tf` | 位姿+姿态均来自 `tianji_right` |
| `tracker_orientation_mode` | `full_pose` | 完整 6D 姿态进 IK |
| `tracker_orientation_map_matrix` | 3×3 | 腕部旋转映射矩阵 `R_map` |
| `ik_reference_mode` | `last_success` | IK 参考角用上次成功解 |
| `robot_ip` | `192.168.8.166` | 读反馈用（即使仿真显示靠 MuJoCo） |

**日志里应出现：**

- `use pinocchio IK: False`
- `tracker orientation input mode: mapped_tf`
- `[TRACKER_ORI_SOURCE] ... position_tf=tianji_right orientation_tf=tianji_right`
- `[READ_ONLY_POSE] right_success=True`
- **不应**出现 `Set B arm joint cmd`（read_only）

---

## 7. 不在当前仿真主链路里的东西

| 文件/能力 | 说明 |
|-----------|------|
| `wujihand_node.py` / `wuji_teleop_hand.launch.py` | 灵巧手，与臂仿真分开 |
| `pico_input` / `tianji_world_output` | PICO 另一套输入/控制栈 |
| `use_pinocchio_ik: true` 分支 | `tianji_arm_node` 内 Pinocchio IK，金标准 **未用** |
| `tracker_orientation_mode: wrist_only` | 腕关节 5–7 后处理模式，金标准 **未用** |
| `tracker_orientation_input_mode: raw_wrist` | 姿态改跟裸 `right_wrist`，曾导致 ~90° 问题，**未用** |
| `enable_tianji_model:=true` | RViz 里 URDF 跟 `joint_state`，不是 MuJoCo 主视图 |
| `dexproj/session/run_session.py` | 手套+整机 session，不是本次 HTC 臂仿真 launch |

---

## 8. 真机打通时只改什么

保持 **同一套文件与算法**，仅改：

1. Launch：`read_only:=false`（并确认 yaml 里 `read_only: false` 不冲突）
2. 仍用 `tianji_output_sim.yaml`（或与其一致的 `tianji_output.yaml`）
3. 对比真机 `joint_state` 是否跟踪 sim 采集的 `joint_command`

真机采集脚本尚未入库时可仿照 `run_tianji_sim_baseline_capture.sh` 复制一份，只改 `read_only` 与输出目录名。

---

## 9. 快速文件索引（按路径排序）

```
DexProj/
├── scripts/
│   ├── ensure_docker_exec.sh          # 进容器 + 同步代码
│   ├── activate_dexproj_env.sh        # ROS 环境
│   ├── run_tianji_sim_baseline_capture.sh
│   ├── analyze_tianji_baseline_log.sh
│   ├── tianji_motion_stamp.sh
│   ├── tianji_sim_baseline_motion_template.txt
│   ├── compare_sim_real_baseline.md
│   └── tianji_sim_pipeline_reference.md   # 本文档
├── captures/tianji_sim_baseline_*/    # 采集结果
└── wuji-hand-teleop/src/
    ├── wuji_teleop_bringup/
    │   ├── launch/wuji_teleop_arm.launch.py
    │   ├── config/static_transforms.yaml
    │   └── wuji_teleop_bringup/tf_utils.py
    ├── input_devices/openvr_input/
    │   ├── config/openvr_input.yaml
    │   ├── openvr_input/openvr_input_node.py
    │   ├── openvr_input/openvr_tracker_wrapper.py
    │   └── rviz/openvr_visualization.rviz
    ├── controller/controller/
    │   ├── tianji_arm_node.py
    │   ├── tianji_mujoco_viewer_node.py
    │   ├── tianji_tracker_sim_viz_node.py
    │   ├── tianji_joint_state_bridge_node.py
    │   └── common.py
    └── output_devices/tianji_output/tianji_output/
        ├── config/tianji_output_sim.yaml    # 仿真金标准配置
        ├── config/tianji_output.yaml
        ├── tianji_arm_controller.py
        └── _internal/fx_kine.py, fx_robot.py, libMarvinSDK.so
```

---

*文档版本：与 `tianji_sim_baseline_20260530_160010` 采集及当前 `wuji_teleop_arm.launch.py` 一致。*

# 仿真金标准 → 真机输出：差异与风险梳理

在 **不改控制算法**（仍用 `tianji_output_sim.yaml` + `mapped_tf` + SDK `move_to_matrix_direct`）的前提下，只把最后一环从 `read_only` 换成真机 `send_cmd`。下面按**启动顺序**列出会碰到的问题。

对照文档：`scripts/tianji_sim_pipeline_reference.md`  
金标准数据：`captures/tianji_sim_baseline_20260530_160010/`

---

## 0. 真机 launch 与仿真只差什么

```bash
# 仿真（当前）
read_only:=true  feedback_handshake:=true
enable_mujoco:=true   # 可关

# 真机（目标）
read_only:=false
feedback_handshake:=false   # 非必须；真机路径会 clear_error + send_cmd
# 建议先关 enable_mujoco，避免干扰判断
controller_config:=.../tianji_output_sim.yaml   # 与仿真相同
```

YAML 里也要 `read_only: false`（或 launch 参数覆盖为 false）。**算法文件与仿真一致**，变的是初始化动作和 `send_cmd`。

---

## 1. 启动阶段（最容易被忽略）

### 1.1 会自动 `move_to_init`（高危）

| | 仿真 `read_only` | 真机 `read_only:=false` |
|--|------------------|-------------------------|
| 启动时臂动作 | **不动**，保持当前物理角 | **`move_to_init(wait=True)`**，双臂插值到硬编码初始角 |
| 代码 | `tianji_arm_node.py` L315-316 | L320-322 → `tianji_arm_controller.move_to_init` |

硬编码右臂 `INIT_JOINTS_RIGHT`（度）：

```text
[-50.9, -70.5, 42.6, -80.3, -140.1, -5.5, 38.9]
```

你采集时真机反馈（`joint_state` 冻结值）约：

```text
[-90.46, -90.08, 90.05, -89.93, -0.06, -0.49, 0.10]
```

**二者相差很大。** 一切真机 teleop，launch 后会先有一段 **约 3s 的双臂归零运动**（左臂也会动，见下），然后才进入 2s `tracker_start_delay`。

**风险：** 未预料的大幅运动、碰撞、与操作者站姿不一致。  
**建议（上线前必须定一条）：**

1. 上电前把机械臂 **手动摆到与 `INIT_JOINTS_*` 接近** 再 launch；或  
2. **改代码/配置跳过 `move_to_init`**（仅真机试验分支）；或  
3. 把 `INIT_JOINTS_RIGHT` 改成你现场常用姿态（与 sim 采集时一致）。

### 1.2 关节阻抗模式

真机会 `set_impedance_mode(mode='joint')`（仿真 read_only **跳过**）。  
工具参数 `_set_tool_params()`（法兰外 120mm 等）也只在非 read_only 执行。

**影响：** 真机跟随 `set_joint_cmd_pose` 的方式与 MuJoCo「瞬时到位」不同，会有 **滞后、超调、抗扰**。

### 1.3 连接与反馈

| 仿真 | 真机 |
|------|------|
| `read_only_connect_timeout: 3s`，连不上可降级 | 必须连上 `192.168.8.166`，否则抛 `ConnectionError` |
| `feedback_handshake` 只清错、不运动 | 正常 clear_error + send_cmd |

**风险：** IP/端口占用、SDK 连不上直接起不来（仿真可降级继续 MuJoCo）。

### 1.4 左臂

OpenVR 只配了右臂 tracker，但 `move_to_init` 会 **同时动左臂 A** 到 `INIT_JOINTS_LEFT`。  
若左臂有障碍或未固定，同样危险。

**建议：** 确认左臂状态；必要时单独 disable 左臂或改 init 只动 B（需改代码）。

---

## 2. 零点与「初始位置」语义

### 2.1 Tracker 零点（与仿真相同逻辑）

`tracker_start_delay`（2s）之后第一次有效 TF：

- `_tracker_zero` ← 当前 `tianji_right`
- `_robot_zero` ← **当前真机 `joint_state` 的 SDK FK**（不是 MuJoCo command）

**仿真时：** 真机停在 `[-90, -90, 90, …]`，MuJoCo 跟 `joint_command` 大幅偏离。  
**真机时：** 若刚 `move_to_init` 完，零点绑在 **INIT 姿态** 的 FK 上，不是绑在「你采集 sim 时的 -90° 姿态」。

**结论：** 换真机后 **增量零点与 sim 采集时刻的物理姿态不必一致**；只要「站定握姿 → 等 zero 初始化 → 再动 tracker」流程一致即可。  
若跳过 `move_to_init` 且人站在 sim 同姿，零点更接近采集 bundle。

### 2.2 操作者动作顺序（建议固定）

1. 机械臂到位（init 或手动姿）  
2. Launch，等 init/连接完成  
3. **戴 tracker，保持与 zero 时相同的相对姿态 ~2–3s**  
4. 看到 log：`Tracker zero initialized for right`  
5. 再小幅度试动  

---

## 3. 运行阶段：仿真「看起来对」≠ 真机一样

### 3.1 输出对象变了

| 环节 | 仿真 | 真机 |
|------|------|------|
| IK 结果 | 只写 `joint_command` + log | **`set_joint_cmd_pose(arm='B')` + `send_cmd()`** |
| `joint_state` | 冻结（read_only） | 应跟随 command（有延迟） |
| MuJoCo | 跟 command | 可关 |

真机 gold 对比：真机 `joint_state` 能否跟上 sim 录的 `joint_command`（同 tracker 动作）。

### 3.2 IK 失败时行为

SDK 路径 IK 失败 → `right_joints=None` → **本周期不更新 B 臂 cmd**，保持上一帧 SDK 指令。  
log：`[RIGHT_IK_FAIL]`（sim 采集有 1 次）。

**真机风险：** 大姿态时 **臂僵住** 或 **突然跳**（下一帧 IK 又成功）。比 MuJoCo 更明显。

### 3.3 `max_joint_step_deg`（yaml 2.5°）对 SDK 路径

`max_joint_step_deg` 在 node 里主要接在 **Pinocchio** 分支的 `_apply_joint_step_limit`。  
当前金标准 **`use_pinocchio_ik: false`** → SDK IK 结果 **不经逐步限速** 直接下发（仅有姿态侧 `tracker_orientation_max_step_deg` 12° 限制 **TCP 目标**，不限制 7 关节增量）。

**真机风险：** 首帧或大 tracker 动作时，关节命令可能 **单步变化很大**（仿真 MuJoCo 无动力学会「瞬间跟上」）。

**建议：** 真机首测用小动作；后续若抖/猛，再考虑给 SDK 路径也加 step limit（属增强，非改算法核心）。

### 3.4 阻抗 + 100Hz 指令

`control_rate_hz: 100`，每 10ms 一次 `send_cmd`。  
真机可能：发热、跟随误差、振动；与 MuJoCo 理想关节插值不同。

### 3.5 工具/动力学参数

真机启动设 tool：TCP +120mm 等。FK/IK 与 sim 一致（都用同一 SDK），但若现场未装手或质量不同，动力学行为会变（不影响 IK 几何，影响实际运动）。

### 3.6 仍存在的「肘/腕不分」

换真机 **不会自动** 变成「肘 yaw→J3、腕 yaw→J6/7」；仍是整段 TCP `full_pose` + zsp。  
这是预期限制，不是真机独有 bug。

---

## 4. 安全与运维

| 项 | 说明 |
|----|------|
| 急停 / disable | `scripts/disable_tianji_right_arm.sh`（B 臂 state=0） |
| 诊断脚本 | `diagnose_tianji_right_arm.sh`、`check_right_arm_coordinate_response.py` |
| SDK 崩溃 | 历史有 IK 段错误 -11；异常退出前勿靠近臂展 |
| 双臂 | init 与 clear_error 都作用 A+B |

---

## 5. 建议的真机首测流程（不改金标准算法）

### Phase A：只连机、不跟手（可选）

- `read_only:=true` 再确认 feedback 与 sim 相同（与现采集一致）。

### Phase B：真机输出，但抑制 init 风险

1. 机械臂 **手动** 摆到安全、与日常 teleop 接近姿态。  
2. Launch：`read_only:=false`，**同一 `tianji_output_sim.yaml`**，`enable_mujoco:=false`。  
3. **人离开臂展**，观察 init 是否大幅运动；记录 `joint_state` 是否到 INIT。  
4. 若 init 不可接受 → **先解决 init 再 teleop**（改 INIT 或跳过 init）。

### Phase C：跟手 smoke

1. Zero 后 **只动右腕平移几厘米**（先不要大 yaw/roll）。  
2. 对比：sim 同动作的 `joint_command` 趋势 vs 真机 `joint_state`。  
3. 再加大姿态；盯 `[RIGHT_IK_FAIL]`、`[JOINT_STEP_LIMIT]`（若以后接上 SDK step limit）。

### Phase D：与 bundle 对比

- 录真机 bundle（同 `run_tianji_sim_baseline_capture.sh`，改 `read_only:=false`）。  
- 对比 `topics/right_joint_command.csv`（sim）与真机 `joint_state` 或 command。

---

## 6. 问题速查表

| 现象 | 可能原因 |
|------|----------|
| 一启动臂大幅动 | `move_to_init` 到硬编码 INIT，与当前姿差很多 |
| 左臂也动 | init 双臂；与只控右无关 |
| Tracker 动了臂不动 | IK fail；或未过 start_delay；或 read_only 仍为 true |
| 臂抖/冲 | 100Hz 关节 cmd + 无阻抗仿真；SDK 路径无 joint step limit |
| 与 MuJoCo 差很多 | 正常：MuJoCo=command 理想值，真机=阻抗跟随 |
| 与 sim 采集姿不一致 | init/零点时刻物理姿不同；不是 yaml 错了 |
| _orientation 怪 | 勿改 `raw_wrist`/static TF；先确认 mapped_tf 与 sim 一致 |

---

## 7. 与仿真的「同一链路」核对清单

上线真机前逐项打勾：

- [ ] `controller_config` = `tianji_output_sim.yaml`（或与其 byte 一致的 `tianji_output.yaml`）
- [ ] `static_transforms.yaml` 与 capture `config_snapshot` 一致
- [ ] `openvr_input.yaml` tracker SN 正确
- [ ] `read_only:=false`，yaml 中 `read_only: false`
- [ ] 已决策：`move_to_init` 接受 / 跳过 / 改 INIT
- [ ] 左臂安全
- [ ] 零点流程：等 `Tracker zero initialized` 再动
- [ ] 准备 disable 脚本
- [ ] 计划录真机 bundle 与 sim `160010` 对比

---

*先理清 init 与零点，再谈跟手质量；其余参数（mapped_tf、R_map）与仿真保持一致即可。*

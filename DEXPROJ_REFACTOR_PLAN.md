# DexProj 改造计划

## 目标

在 `DexProj` 根目录下建立一套干净、可维护的遥操作与数据采集工作流，遵循以下原则：

- 后续所有新文档和新代码都直接放在 `DexProj` 下
- `wuji-hand-teleop` 和 `wuji-retargeting` 作为上游 git 子模块保留
- `TJ` 仅作为临时历史参考，不再继续扩展
- 最终目标是实现“一键启动遥操作并开始数据采集”

整体思路是：只保留 `TJ` 里真正有价值的运行时和录制能力，但不继承它里面已经废弃、混乱或历史包袱较重的链路。

## 当前仓库策略

### 保留目录

- `wuji-hand-teleop/`
  - 角色：ROS2 系统集成、设备驱动、bringup 启动层
  - 预期：尽量贴近上游，后续作为子模块维护

- `wuji-retargeting/`
  - 角色：手部 retargeting 算法层、灵巧手遥操作样例层
  - 预期：尽量贴近上游，后续作为子模块维护

### 后续删除目录

- `TJ/`
  - 当前角色：历史参考目录
  - 处理方式：等新链路完整替代后删除

### 在 DexProj 根目录新增的本地目录

建议新增以下第一方目录：

- `docs/`
  - 架构说明
  - 操作说明
  - 数据格式文档

- `dexproj/`
  - 本地 Python 包
  - 承载编排、录制、导出等核心逻辑

- `scripts/`
  - 一键启动脚本
  - 开始/停止采集脚本
  - 导出/校验脚本

- `config/`
  - 本地配置
  - 用于 glue `wuji-hand-teleop` 与 `wuji-retargeting`

- `data/`
  - 本地数据输出根目录
  - 保存 session、episode 和导出数据集

## 当前主链路假设

当前计划默认以下主链路形态：

- 机械臂与系统 bringup 主链路：`wuji-hand-teleop`
- 手部 retargeting 主链路：`wuji-retargeting`
- DexProj 本地层负责：
  - 一键拉起整套系统
  - 管理单臂/双臂运行模式
  - 管理录制 session / episode
  - 整合 arm、hand、camera 数据采集

其中，当前手部真机遥操作默认参考以下链路：

```bash
python wuji-retargeting/example/teleop_real.py --input wuji_glove --hand right --glove-sn <YOUR_SN>
```

这条链路目前是单手真机样例链路，不是现成的双手统一入口。后续 DexProj 需要在此基础上补出更正式的单手/双手运行编排。

## 机械臂遥操作路线约束

这个项目后续只保留 HTC 遥操作机械臂链路，不走 PICO 路线。

### 当前采用的机械臂链路

当前 `wuji-hand-teleop` 里与 HTC 对应的机械臂链路是：

- `arm_input:=tracker`
- 对应输入节点：`openvr_input`
- 对应设备：HTC Vive Tracker / OpenVR / SteamVR 体系

也就是说，当前计划中机械臂遥操作默认采用：

- HTC Tracker 提供位姿输入
- `openvr_input` 发布 TF / 输入数据
- `tianji_arm_controller` 消费对应输入并控制机械臂

### 不纳入当前改造范围的链路

以下链路当前不作为 DexProj 主路线的一部分：

- `pico_input`
- `pico_teleop.launch.py`
- 任何以 PICO 作为机械臂主输入源的方案

这些内容可以保留在子模块中，但 DexProj 本地改造、文档、脚本、录制链路、验收标准全部按 HTC 路线设计，不再同时维护 HTC/PICO 两套机械臂遥操作主方案。

### 对后续实现的要求

后续所有本地封装默认按以下前提设计：

- 机械臂输入设备：HTC Tracker
- 机械臂输入软件栈：OpenVR / SteamVR
- bringup 默认入口：基于 `arm_input:=tracker` 的链路
- 启动前检查默认检查 HTC / OpenVR 相关设备与配置

如果以后要重新支持 PICO，应作为单独扩展项目处理，而不是混入当前主线。

## 模式要求

后续系统必须明确保留以下模式，而不是只支持一种：

1. 单侧模式
   - `single_left`
   - `single_right`

2. 双侧模式
   - `dual`

这三个模式必须贯穿以下层面：

- bringup 启动层
- teleop 运行层
- recorder 录制层
- episode 元数据层
- dataset 导出层

### 对机械臂链路的要求

`wuji-hand-teleop` 当前已经具备单侧和双侧入口基础，后续 DexProj 需要将其收敛成统一模式配置，而不是让同事直接记多个 launch 文件。

### 对灵巧手链路的要求

当前 `wuji-retargeting` 的样例链路以单手为主。后续需要明确支持：

- 单手运行
- 双手运行

其中双手运行不一定要求直接改造现有 `example/teleop_real.py`，也可以由 DexProj 新增一个更清晰的双手入口来完成。

### 对数据采集链路的要求

新的数据采集链路必须完整支持：

- 单侧采集
- 双侧采集

并且在元数据中明确记录本次 episode 的运行模式，例如：

- `mode: single_left`
- `mode: single_right`
- `mode: dual`

## 手部双手设计约束

当前 `wuji-retargeting/example/teleop_real.py` 仍视为单手真机样例，不作为未来双手正式入口。

后续双手手部遥操作采用以下设计原则：

1. 双手运行模型
   - 采用“两条显式绑定的单手通道 + 一个双手编排器”
   - 不在现有单手样例脚本上继续堆双手参数

2. 每一侧独立绑定
   - `left_glove_sn -> left retargeter -> left hand`
   - `right_glove_sn -> right retargeter -> right hand`

3. 设备标识必须显式配置
   - 左手手套 SN
   - 右手手套 SN
   - 左手灵巧手 SN
   - 右手灵巧手 SN
   - 不依赖自动猜测左右映射作为正式运行逻辑

4. 每侧独立组件
   - glove endpoint
   - retargeter
   - hand endpoint

5. 单侧/双侧共用统一框架
   - 单手和双手只是运行模式不同
   - 不维护两套完全割裂的手部 teleop 代码

### 推荐参数与配置模型

建议后续统一支持配置文件驱动，而不是把所有 SN 都堆到命令行中。

例如：

```yaml
mode: dual

hands:
  left:
    glove_sn: "LEFT_GLOVE_SN"
    hand_sn: "LEFT_HAND_SN"
    retarget_config: "config/adaptive_analytical_wuji_glove_left.yaml"

  right:
    glove_sn: "RIGHT_GLOVE_SN"
    hand_sn: "RIGHT_HAND_SN"
    retarget_config: "config/adaptive_analytical_wuji_glove_right.yaml"
```

### 对录制元数据的要求

`meta.json` 中必须显式记录：

- `left_glove_sn`
- `right_glove_sn`
- `left_hand_sn`
- `right_hand_sn`

这样后续调试、追溯、数据清洗和导出才不会混乱。

## 操作工作流要求

我希望最终的操作体验固定为：

1. 一键拉起系统
2. 系统进入 ready 状态但暂不开始正式录制
3. 操作者通过按键或按钮开始遥操作录制
4. 操作者通过按键或按钮结束当前录制
5. 系统完成安全收尾并将 episode 落盘

### 开始/结束触发规则

优先保留 `TJ` 里已经存在的交互方式；如果某个场景无法复用，再退化为键盘方案。

根据当前 `TJ` 代码，已经存在的录制触发方式是：

- 开始录制：默认 `LB/RB`
- 结束录制：默认 `START`

这一点来自 `TJ/src/tool/collect_marvin_remote_teach_dual.py` 中的参数默认值：

- `--record-trigger-buttons=lb,rb`
- `--record-stop-buttons=start`

所以新方案默认要求如下：

1. 优先保留 `TJ` 的开始/结束触发习惯
   - 开始：`LB/RB`
   - 结束：`START`

2. 如果某条新链路不方便接入手柄按钮事件
   - 则提供键盘 fallback
   - 默认约定：`B` 开始，`E` 结束

### 推荐的最终交互策略

建议最终系统同时支持两套触发方式：

- 主方式：手柄按钮触发
  - 开始：`LB/RB`
  - 结束：`START`

- 备用方式：键盘触发
  - 开始：`B`
  - 结束：`E`

并通过配置项控制优先级，例如：

- `trigger_mode: gamepad`
- `trigger_mode: keyboard`
- `trigger_mode: both`

默认建议使用：

- `trigger_mode: both`

这样在调试和正式采集时都更灵活。

## 设备识别与启动前准备

当前链路在正式运行前，确实需要准备和确认部分设备标识信息。后续需要把这部分收敛成 DexProj 的标准准备流程，而不是靠口头传递。

### 当前需要关注的设备标识

1. Wuji Glove 序列号
   - 当前 `wuji-retargeting` 的 Wuji Glove 真机链路支持通过 `--glove-sn` 显式指定手套
   - 当只有一个手套在线时，某些场景可以自动连接
   - 当有多个手套、或需要明确左右对应时，应显式使用序列号

2. Wuji Hand 序列号
   - 当前 `wuji-hand-teleop` 通过 `wujihand_ik.yaml` 配置左右手的 `serial_number`
   - 单手模式下也建议明确填入实际序列号，未使用的一侧可置空或禁用

3. Tracker 序列号
   - OpenVR / Vive Tracker 链路依赖 tracker serial 到角色的映射
   - 例如：`chest`、`left_wrist`、`right_wrist`、`left_arm`、`right_arm`
   - 这些映射通常需要写入对应 YAML 配置

4. 相机序列号
   - 如果启用腕部 RealSense 或其他需要区分左右的相机，通常也需要按 serial number 区分左/右设备

### 设备序列号查询与左右对应确认

后续计划中必须补一套统一的设备枚举与确认流程，至少覆盖：

- 所有在线 Wuji Glove
- 所有在线 Wuji Hand
- 所有在线 HTC Tracker
- 所有在线相机（如启用）

你提到的方法是可行的，而且很适合第一次建表：

1. 先查询当前所有在线设备
2. 拔掉目标设备中的一个
3. 再查询一次
4. 比较前后差异，确认少了哪个序列号
5. 将该序列号记录为对应的左/右设备

这个方法建议正式写入文档，作为人工兜底方案。

### 推荐的更稳妥方法

建议最终采用“两阶段确认”方式：

1. 枚举阶段
   - 查询所有在线设备及其序列号
   - 将结果打印并保存到一份本地清单中

2. 对应确认阶段
   - 使用拔插法确认左右对应关系
   - 对于 HTC Tracker，可结合佩戴位置和实时姿态显示确认
   - 对于手套，可结合运行时数据流或 SDK `frame_id` 确认左右
   - 对于相机，可直接看预览画面确认左右是否反了

这样比只靠一次拔插更稳，因为可以避免“记住了序列号但写错角色”的问题。

### 后续应补的 DexProj 能力

DexProj 后续应新增统一设备检查工具，例如：

- 枚举所有在线设备
- 打印序列号、类型、当前推断角色
- 支持生成本地配置草稿
- 支持在启动前做一致性检查

目标是把“查序列号”和“确认左右对应”变成一条标准流程。

### 启动前准备事项

除了设备序列号和左右映射，当前链路在正式启动前还有一些应当明确的准备事项：

1. 手套准备
   - 连接手套
   - 在 Wuji Studio 中确认手套在线
   - 在 Wuji Studio 中完成校准
   - 确认当前使用的是正确的手套和用户校准结果

2. 灵巧手准备
   - 确认左右手硬件在线
   - 确认 `wujihand_ik.yaml` 中左右手序列号配置正确
   - 单手模式下确认未使用一侧已禁用或不参与运行

3. HTC Tracker 准备
   - 确认 SteamVR / OpenVR 正常工作
   - 确认 tracker 已配对
   - 确认 tracker serial 到角色映射正确
   - 确认佩戴方式与配置一致

4. 相机准备（如启用）
   - 确认设备在线
   - 确认 serial number 到 `left` / `right` / `head` 的映射正确
   - 确认图像画面方向和左右对应无误

5. 机器人准备
   - 确认机械臂与灵巧手可连接
   - 确认 IP、命名空间、运行模式配置正确
   - 确认系统可以进入安全初始姿态或 home 状态

6. 软件环境准备
   - source 对应 ROS2 工作区
   - 确认子模块依赖已安装
   - 确认本地 DexProj wrapper 使用的是预期环境

### 计划要求

后续改造中，必须把以下内容纳入标准流程：

- 启动前设备检查
- 设备序列号查询
- 左右对应确认
- 配置一致性检查
- ready 前置校验

并尽量把这些检查收敛成可执行脚本，而不是仅保留在文档说明中。

## 相比 TJ 的功能缺口

当前已经实现或正在实现的部分：

- `session / episode` 级别目录管理
- TJ raw 风格的落盘结构
- 单键/手柄触发编排
- 只走 HTC 的 bringup 路线
- 相机图像帧落盘，不走 mp4

当前仍未完整实现的部分：

1. 更完整的停止与收尾语义
   - 当前支持开始/结束触发与安全停止
   - 但更细的进程健康监控和异常恢复还没有做成完整产品化流程

2. 数据导出链路
   - 现在先只保证 raw 落盘
   - 后续如果需要训练格式导出，再单独补导出器

3. 异常收尾策略
   - 遥操作或数采过程中只要发生中断、报错、手动取消
   - 当前 episode 直接视为作废并自动删除
   - 只保留正常 start -> stop 完整结束的 episode

## 目标架构

### 第 1 层：上游子模块层

- `wuji-hand-teleop`
  - 负责 ROS2 节点、launch、驱动、机械臂/灵巧手控制集成

- `wuji-retargeting`
  - 负责 retargeting 算法与手部输入处理

这一层尽量少改，只有在改动足够通用、值得回馈上游时才改子模块内部。

### 第 2 层：DexProj 本地胶水层

在 `dexproj/` 下新建本地代码，负责：

- session 生命周期管理
- episode 命名与目录创建
- 元数据管理
- 同步录制
- 本地启动编排
- 模式管理（`single_left` / `single_right` / `dual`）
- 触发管理（手柄按钮 / 键盘）
- 设备检查与启动前校验
- 双手手部通道编排
- 数据导出

这一层只依赖两个子模块，不复制它们的实现。

### 第 3 层：操作入口层

在 `scripts/` 下提供简洁入口，例如：

- `scripts/bringup_teleop.sh`
- `scripts/start_collection.sh`
- `scripts/stop_collection.sh`
- `scripts/export_dataset.sh`
- `scripts/check_devices.sh`
- 后续可能合并成 `scripts/run_session.sh`

最终操作体验应该支持：

1. 启动设备和遥操作图
2. 检查 ready 状态
3. 等待操作者触发开始
4. 开始录制
5. 触发停止并安全落盘
6. 在磁盘上留下完整 episode

## 改造原则

1. 不再扩展 `TJ`
   - 只把它作为行为参考源

2. 不大规模拷贝子模块代码
   - 优先通过 wrapper、adapter、小补丁解决

3. 遥操作运行时和录制运行时分离
   - 遥操作本身应可独立运行
   - 录制逻辑应作为附加层接入

4. 尽量使用清晰、可检查的文件格式
   - `JSON` 存元数据
   - `CSV` 存表格数据
   - 图像帧序列存相机数据，不输出 `MP4`

5. 每个 episode 都要自描述
   - 后续即使脱离运行环境，也能重放、调试、导出

6. 单侧和双侧必须共享同一套主框架
   - 不允许后期演变成两套割裂的采集系统

7. 机械臂主路线只保留 HTC
   - 当前 DexProj 主线不同时维护 HTC 和 PICO 两套机械臂输入方案

## 分阶段实施计划

## Phase 0：仓库准备

目标：先把 `DexProj` 根目录结构整理干净

任务：

- 创建 `docs/`、`dexproj/`、`scripts/`、`config/`、`data/`
- 补基础架构文档和录制格式文档
- 补设备准备与序列号确认文档
- 明确机械臂只走 HTC 链路
- 确认 `wuji-hand-teleop` 与 `wuji-retargeting` 以子模块方式管理
- 明确本地 Python 环境和依赖管理方式

交付物：

- 一个结构清晰、可继续开发的根目录

## Phase 1：Bringup 包装层

目标：在 `wuji-hand-teleop` 外面包一层 DexProj 自己的一键启动入口

任务：

- 审核现有 `wuji-hand-teleop` launch 入口
- 固定机械臂输入路线为 HTC / `arm_input:=tracker`
- 确定默认模式：`single_left` / `single_right` / `dual`
- 在 `DexProj` 下补统一 wrapper 脚本和配置
- 统一环境变量、配置路径和启动参数
- 提供启动前设备检查
- 提供 ready 检查
- 提供“启动后等待触发开始”的运行态

交付物：

- 一条 DexProj 命令可稳定拉起遥操作系统
- 且支持单侧/双侧两种运行形态
- 且机械臂默认只走 HTC 链路

## Phase 2：手部 Retarget 集成

目标：将 `wuji-retargeting` 干净地并入实际运行链路

任务：

- 明确未来主用的手部输入路径
- 明确 retarget 输出如何进入实时控制链路
- 如有必要，增加很薄的一层 adapter
- 决定 retargeting 以独立进程、ROS2 节点还是 wrapper 方式运行
- 明确单手和双手的运行组织方式
- 明确手套序列号的查询与传入方式
- 明确左右灵巧手 SN 的传入与绑定方式

交付物：

- 不依赖 `TJ` 的稳定手部遥操作链路
- 支持单手与双手运行

## Phase 3：Session Recorder

目标：用新的 DexProj 实现替代 `TJ` 中真正有价值的录制能力

任务：

- 设计 session 与 episode 目录结构
- 设计 `session.json` / `episode/meta.json`
- 记录以下内容：
  - mode
  - trigger_mode
  - start_trigger
  - stop_trigger
  - 设备标识信息（如手套/手/tracker/相机 serial）
  - arm timestamps
  - arm observation state
  - arm action
  - hand actual position
  - hand target position
  - stop reason
  - task label（如需要）
- 增加 start/stop 接口
- 增加退出时安全 flush 行为
- 保证单侧/双侧数据字段兼容

交付物：

- 第一份由新链路写出的完整 episode

## Phase 4：相机录制

目标：把相机数据正式纳入每个 episode

任务：

- 明确相机列表和命名规范
- 明确每个相机的录制模式
- 保存到 episode 内部标准目录
- 在元数据中记录相机信息
- 尽可能与手/臂时间戳对齐
- 保证单侧和双侧模式下目录结构一致、字段语义一致

交付物：

- 一个同时包含 arm、hand、camera 数据的完整 episode

## Phase 5：数据集导出

目标：用本地维护的导出链路替代 `TJ` 的数据集构建脚本

任务：

- 定义目标数据集格式
- 实现 episode 完整性检查
- 实现从 episode 到目标格式的导出器
- 保证导出器兼容单侧和双侧 episode
- 只有确实需要时再做 shard merge

交付物：

- 能将新录制 episode 转成训练数据的导出工具

## Phase 6：一键工作流

目标：把前面这些能力整合成最终操作方式

任务：

- 增加一条命令，实现：
  - 拉起 teleop
  - 做启动前检查
  - 检查 ready
  - 进入等待触发开始状态
- 支持以下触发方式：
  - 手柄按钮开始/结束
  - 键盘开始/结束
- 默认保留 `TJ` 习惯：
  - `LB/RB` 开始
  - `START` 结束
- 增加键盘 fallback：
  - `B` 开始
  - `E` 结束
- 增加干净的 stop 流程
- 增加对操作者友好的日志和状态输出

交付物：

- 一键启动遥操作并开始数据采集的最终工作流

## 推荐根目录结构

```text
DexProj/
├── DEXPROJ_REFACTOR_PLAN.md
├── docs/
├── config/
├── data/
├── dexproj/
│   ├── __init__.py
│   ├── session/
│   ├── recording/
│   ├── export/
│   ├── integration/
│   └── triggers/
├── scripts/
├── wuji-hand-teleop/        # 子模块
├── wuji-retargeting/        # 子模块
└── TJ/                      # 临时历史参考，后续删除
```

## 建议的 Episode 目录结构

```text
data/
└── raw/
    └── session_YYYY_MM_DD/
        └── episode_000001/
            ├── meta.json
            ├── arm_data/
            │   ├── timestamp.csv
            │   ├── observation_state.csv
            │   └── action.csv
            ├── hand_data/
            │   ├── actual_position.csv
            │   └── target_position.csv
            └── camera_data/
                ├── head/
                ├── left_wrist/
                └── right_wrist/
```

这个结构延续了 `TJ` 里有价值的组织方式，但实现上可以完全重写得更干净。

## 删除 TJ 前的验收标准

只有以下条件全部满足后，才删除 `TJ`：

1. DexProj 自己的命令可以稳定拉起 teleop 系统
2. DexProj 自己的命令可以稳定支持 `single_left`、`single_right`、`dual`
3. DexProj 自己的命令可以完成启动前设备检查与 ready 校验
4. DexProj 自己的命令可以稳定等待触发开始，并完成开始/结束录制
5. 手柄按钮触发链路可用，或至少键盘 fallback 可用
6. 机械臂主链路只依赖 HTC 方案即可完成真实采集
7. 一个录制出的 episode 同时包含 arm、hand、camera 数据
8. 元数据足够支持调试和后续导出
9. 数据集导出链路可以处理新录制数据
10. 至少有一次真实采集任务全程不依赖 `TJ` 完成

## 近期下一步

1. 先创建根目录骨架：
   - `docs/`
   - `dexproj/`
   - `scripts/`
   - `config/`
   - `data/`

2. 再补一份简短架构说明：
   - 哪些功能留在子模块里
   - 哪些功能属于 DexProj 本地代码

3. 优先做 Phase 1：
   - 先把一键 bringup 包起来
   - 统一单侧/双侧模式配置
   - 固定 HTC 机械臂主路线
   - 统一启动前检查入口

4. 尽早做 Phase 2 和 Phase 3：
   - 手部双手编排和 recorder 是最关键的结构性缺口
   - 同时把触发逻辑和设备元数据也设计进去

5. 避免继续改 `TJ`
   - 除非只是查参考实现

## 建议路线

最干净的方向是：

- 用 `wuji-hand-teleop` 做系统 bringup 层
- 机械臂输入只保留 HTC / OpenVR 路线
- 用 `wuji-retargeting` 做手部 retargeting 层
- 在 `DexProj` 根目录下新建自己的 orchestration + recording 层
- 在 DexProj 本地层统一管理：
  - 单侧/双侧模式
  - 手柄/键盘触发方式
  - 设备识别与启动前校验
  - 双手手部通道绑定
  - session / episode 录制

这样后面结构会比较稳，也更容易在功能补齐后彻底删除 `TJ`。

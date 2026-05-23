# 控制流与数据流

## 控制流

1. `scripts/run_session.sh` 启动 DexProj 会话。
2. `dexproj.session.run_session` 读取 `config/session_htc_wuji_glove.yaml`。
3. 先做设备预检，确认 HTC/OpenVR、手套、机械臂、相机可用。
4. 进入 `ready`，等待触发。
5. 触发开始后，同时拉起：
   - `wuji-hand-teleop` bringup
   - 左/右手 `wuji-retargeting/example/teleop_real.py`
   - `camera_launch.py`（如启用相机）
6. 触发停止后统一收尾。
7. 异常或中断则删除当前 episode。

## 数据流

1. 机械臂话题写入 `arm_data/`
2. 相机图像帧写入 `camera_data/<name>/images/`，索引写入 `frames.csv`
3. 会话元数据写入 `meta.json`
4. 运行期摘要写入 `_runtime/`
5. 只保留 TJ raw 风格落盘，不转 leRobot

## 目录

- `arm_data/action.csv`
- `arm_data/observation_state.csv`
- `arm_data/timestamp.csv`
- `camera_data/head|left_wrist|right_wrist/frames.csv`
- `camera_data/head|left_wrist|right_wrist/images/`
- `_runtime/sample_info.json`
- `_runtime/remote_teach_session.json`

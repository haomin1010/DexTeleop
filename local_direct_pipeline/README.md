# Local Direct Pipeline

This folder is the local direct-conversion version for `DexTeleop-ssh-167`.

Collection stays unchanged:

- raw episodes remain under the raw capture root
- no local batch step
- no local watcher step
- no local shard merge step

This folder only handles:

- read the current raw root directly
- build one local LeRobot/GR00T-style dataset
- optionally upload that built dataset with `rsync`

## Default paths

- raw root: `data/raw_data`
- local dataset output: `data/local_lerobot_dataset`

Important:

- `convert_raw2lerobot.sh` still defaults to `data/raw_data`
- `run_session` currently records under `data/raw/session_YYYY_MM_DD/episode_xxxxxx`
- In normal usage, prefer passing `RAW_ROOT=...` explicitly instead of relying on the default raw root

## Main files

- `build_groot_lerobot_dataset.py`
- `convert_raw2lerobot.sh`
- `merge_lerobot_datasets.py`
- `run_merge_lerobot_datasets.sh`
- `append_lerobot_datasets.py`
- `run_append_lerobot_datasets.sh`

## Main commands

Build locally only:

```bash
bash local_direct_pipeline/convert_raw2lerobot.sh
```

Recommended when converting freshly recorded sessions:

```bash
RAW_ROOT=/home/user/workspace/DexProj_back_up_0602/data/raw/session_2026_06_03 \
bash local_direct_pipeline/convert_raw2lerobot.sh
```

Build locally, then upload:

```bash
RAW_ROOT=/home/user/workspace/DexProj_back_up_0602/data/raw/session_2026_06_03 \
REMOTE_HOST=h20-0 \
REMOTE_ROOT=/root/nas/dexproj/ \
bash local_direct_pipeline/convert_raw2lerobot.sh --rsync
```

Merge multiple already-built local datasets:

```bash
bash local_direct_pipeline/run_merge_lerobot_datasets.sh \
  --dataset-path /path/to/dataset_a \
  --dataset-path /path/to/dataset_b
```

Append new datasets into an existing merged dataset:

```bash
bash local_direct_pipeline/run_append_lerobot_datasets.sh \
  --target-root /path/to/existing_merged_dataset \
  --dataset-path /path/to/new_dataset_a \
  --dataset-path /path/to/new_dataset_b
```

## `merge` 和 `append`

- `merge`：把多个已经构建好的 dataset 一次性合成一个新的 dataset，适合第一次做总集合并
- `append`：往一个已经存在的 dataset 后面继续追加新的 dataset，适合增量加数据
- `append` 不是任意 dataset 都能加，要求 `robot_type`、`modality.json`、`features` 一致

常见用法：

```bash
# 重新合一个新的总数据集
bash local_direct_pipeline/run_merge_lerobot_datasets.sh \
  --dataset-path /path/to/dataset_a \
  --dataset-path /path/to/dataset_b

# 往已有总数据集继续追加
bash local_direct_pipeline/run_append_lerobot_datasets.sh \
  --target-root /path/to/existing_merged_dataset \
  --dataset-path /path/to/new_dataset
```

## Notes

- The build script is copied from the local build logic and still supports the same raw episode structure.
- Because raw capture roots can vary by workflow, recording date, mount point, or sync target, it is safer to pass `RAW_ROOT` explicitly for each conversion run.
- It builds `camera.mp4` from saved image frames during conversion.
- Final `observation.state` and `action` include:
  - arm joint
  - arm ee pose
  - hand
- If you do not pass upload parameters, the built dataset stays local and nothing is uploaded.
- Merge is full rebuild style: you explicitly pass the dataset paths to merge.
- Append is incremental for parquet/video copying, but still recomputes dataset stats after appending.

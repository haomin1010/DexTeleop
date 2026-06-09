# Local Direct Pipeline

This folder is the local direct-conversion version for `DexTeleop-ssh-167`.

Collection stays unchanged:

- raw episodes remain in `data/raw_data/episode_*`
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

Build locally, then upload:

```bash
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

## Notes

- The build script is copied from the local build logic and still supports the same raw episode structure.
- It builds `camera.mp4` from saved image frames during conversion.
- Final `observation.state` and `action` include:
  - arm joint
  - arm ee pose
  - hand
- If you do not pass upload parameters, the built dataset stays local and nothing is uploaded.
- Merge is full rebuild style: you explicitly pass the dataset paths to merge.
- Append is incremental for parquet/video copying, but still recomputes dataset stats after appending.

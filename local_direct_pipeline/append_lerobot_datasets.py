from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

from build_groot_lerobot_dataset import (
    _collect_dataset_arrays,
    _collect_relative_horizon_values,
    _read_json,
    _read_jsonl,
    _summarize_array,
    _summarize_relative_horizon_groups,
    _write_json,
    _write_jsonl,
)


def _resolve_dataset_roots(dataset_roots: list[str]) -> list[Path]:
    resolved: list[Path] = []
    seen: set[Path] = set()
    for value in dataset_roots:
        path = Path(value).expanduser().resolve()
        if not path.is_dir():
            raise NotADirectoryError(f"Dataset root is not a directory: {path}")
        if path in seen:
            continue
        seen.add(path)
        resolved.append(path)
    if not resolved:
        raise ValueError("No dataset roots were provided")
    return resolved


def _load_dataset_meta(dataset_root: Path) -> tuple[dict, list[dict], list[dict], dict, list[dict]]:
    meta_root = dataset_root / "meta"
    info = _read_json(meta_root / "info.json")
    episodes = _read_jsonl(meta_root / "episodes.jsonl")
    tasks_rows = _read_jsonl(meta_root / "tasks.jsonl")
    modality = _read_json(meta_root / "modality.json")
    source_map = _read_jsonl(meta_root / "source_map.jsonl")
    return info, episodes, tasks_rows, modality, source_map


def _source_map_lookup(source_map_rows: list[dict]) -> dict[int, dict]:
    lookup: dict[int, dict] = {}
    for row in source_map_rows:
        lookup[int(row["episode_index"])] = row
    return lookup


def _copy_video_files(
    *,
    dataset_root: Path,
    output_root: Path,
    old_episode_index: int,
    new_episode_index: int,
    old_chunk_size: int,
    new_chunk_size: int,
    video_keys: list[str],
) -> int:
    old_chunk = old_episode_index // old_chunk_size
    new_chunk = new_episode_index // new_chunk_size
    total_videos = 0
    for video_key in video_keys:
        source = (
            dataset_root
            / "videos"
            / f"chunk-{old_chunk:03d}"
            / video_key
            / f"episode_{old_episode_index:06d}.mp4"
        )
        if not source.is_file():
            raise FileNotFoundError(f"Missing video file: {source}")
        target = (
            output_root
            / "videos"
            / f"chunk-{new_chunk:03d}"
            / video_key
            / f"episode_{new_episode_index:06d}.mp4"
        )
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        total_videos += 1
    return total_videos


def _recompute_dataset_stats(
    *,
    output_root: Path,
    chunks_size: int,
    modality: dict,
    features: dict,
    robot_type: str,
    tasks_rows: list[dict],
    episodes_rows: list[dict],
    source_map_rows: list[dict],
    merged_from_rows: list[dict],
) -> dict[str, object]:
    merged_action_array, merged_observation_array, merged_timestamp_array = _collect_dataset_arrays(output_root)
    stats = {
        "action": _summarize_array(merged_action_array),
        "observation.state": _summarize_array(merged_observation_array),
        "timestamp": _summarize_array(merged_timestamp_array),
    }

    min_episode_length = min(int(row["length"]) for row in episodes_rows)
    max_relative_horizon = max(1, min(16, min_episode_length))
    delta_indices = list(range(max_relative_horizon))
    relative_horizon_parts: dict[str, list[list[np.ndarray]]] = {
        key: [[] for _ in delta_indices]
        for key in modality.get("action", {})
        if key in modality.get("state", {})
    }

    for row in episodes_rows:
        episode_index = int(row["episode_index"])
        parquet_path = (
            output_root
            / "data"
            / f"chunk-{episode_index // chunks_size:03d}"
            / f"episode_{episode_index:06d}.parquet"
        )
        episode_df = pd.read_parquet(parquet_path, columns=["observation.state", "action"])
        observation_array = np.asarray(episode_df["observation.state"].tolist(), dtype=np.float32)
        action_array = np.asarray(episode_df["action"].tolist(), dtype=np.float32)
        for key, parts in relative_horizon_parts.items():
            state_slice = modality["state"][key]
            action_slice = modality["action"][key]
            values = _collect_relative_horizon_values(
                observation_array[:, int(state_slice["start"]):int(state_slice["end"])],
                action_array[:, int(action_slice["start"]):int(action_slice["end"])],
                delta_indices,
            )
            for delta_idx in range(len(delta_indices)):
                parts[delta_idx].append(values[delta_idx])

    relative_stats = {
        key: _summarize_relative_horizon_groups(parts)
        for key, parts in relative_horizon_parts.items()
    }

    fps_values = [
        float(value["info"].get("video.fps", 0.0))
        for key, value in features.items()
        if value.get("dtype") == "video"
    ]
    avg_fps = sum(fps_values) / len(fps_values) if fps_values else 0.0
    total_frames = int(sum(int(row["length"]) for row in episodes_rows))
    total_videos = int(len(episodes_rows) * len([k for k, v in features.items() if v.get("dtype") == "video"]))

    info = {
        "codebase_version": "v2.1",
        "robot_type": robot_type,
        "total_episodes": len(episodes_rows),
        "total_frames": total_frames,
        "total_tasks": len(tasks_rows),
        "chunks_size": chunks_size,
        "fps": round(avg_fps, 6),
        "splits": {"train": f"0:{len(episodes_rows)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": features,
        "total_chunks": max(1, len({int(row['episode_index']) // chunks_size for row in episodes_rows})),
        "total_videos": total_videos,
    }

    _write_json(output_root / "meta" / "info.json", info)
    _write_json(output_root / "meta" / "stats.json", stats)
    _write_json(output_root / "meta" / "relative_stats.json", relative_stats)
    _write_json(output_root / "meta" / "modality.json", modality)
    _write_jsonl(output_root / "meta" / "tasks.jsonl", tasks_rows)
    _write_jsonl(output_root / "meta" / "episodes.jsonl", episodes_rows)
    _write_jsonl(output_root / "meta" / "source_map.jsonl", source_map_rows)
    _write_json(output_root / "meta" / "appended_from_datasets.json", {"dataset_paths": merged_from_rows})

    return {
        "output_root": str(output_root),
        "total_episodes": len(episodes_rows),
        "total_frames": total_frames,
        "total_videos": total_videos,
    }


def append_datasets(
    *,
    target_root: Path,
    dataset_paths: list[Path],
    chunks_size_override: int | None,
) -> dict[str, object]:
    import pyarrow as pa
    import pyarrow.parquet as pq

    if not target_root.is_dir():
        raise NotADirectoryError(f"Target dataset root is not a directory: {target_root}")

    target_info, target_episodes, target_tasks_rows, target_modality, target_source_map = _load_dataset_meta(target_root)
    target_robot_type = str(target_info["robot_type"])
    target_features = dict(target_info["features"])
    target_chunks_size = int(chunks_size_override or target_info["chunks_size"])
    video_keys = [key for key, value in target_features.items() if value.get("dtype") == "video"]

    task_to_index = {str(row["task"]): int(row["task_index"]) for row in target_tasks_rows}
    episodes_rows = list(target_episodes)
    source_map_rows = list(target_source_map)
    appended_from_rows = [str(path) for path in _read_json(target_root / "meta" / "appended_from_datasets.json").get("dataset_paths", [])] if (target_root / "meta" / "appended_from_datasets.json").is_file() else []

    next_episode_index = len(episodes_rows)
    total_frames = int(sum(int(row["length"]) for row in episodes_rows))
    appended_episodes = 0
    appended_videos = 0

    for dataset_root in dataset_paths:
        info, dataset_episodes, dataset_tasks_rows, modality, dataset_source_map = _load_dataset_meta(dataset_root)
        if str(info["robot_type"]) != target_robot_type:
            raise ValueError(
                f"robot_type mismatch: expected {target_robot_type}, got {info['robot_type']} from {dataset_root}"
            )
        if modality != target_modality:
            raise ValueError(f"modality.json mismatch for dataset {dataset_root}")
        if dict(info["features"]) != target_features:
            raise ValueError(f"features mismatch for dataset {dataset_root}")

        source_lookup = _source_map_lookup(dataset_source_map)
        dataset_chunk_size = int(info["chunks_size"])

        for task_row in dataset_tasks_rows:
            task_name = str(task_row["task"])
            if task_name not in task_to_index:
                new_index = len(task_to_index)
                task_to_index[task_name] = new_index
                target_tasks_rows.append({"task_index": new_index, "task": task_name})

        for episode_row in dataset_episodes:
            old_episode_index = int(episode_row["episode_index"])
            source_parquet = (
                dataset_root
                / "data"
                / f"chunk-{old_episode_index // dataset_chunk_size:03d}"
                / f"episode_{old_episode_index:06d}.parquet"
            )
            if not source_parquet.is_file():
                raise FileNotFoundError(f"Missing parquet file: {source_parquet}")

            df = pd.read_parquet(source_parquet)
            old_task_name = str(episode_row["tasks"][0])
            new_task_index = task_to_index[old_task_name]

            frame_count = len(df)
            df["episode_index"] = next_episode_index
            df["index"] = np.arange(total_frames, total_frames + frame_count, dtype=np.int64)
            if "task_index" in df.columns:
                df["task_index"] = new_task_index
            if "annotation.human.action.task_description" in df.columns:
                df["annotation.human.action.task_description"] = new_task_index

            target_chunk = next_episode_index // target_chunks_size
            target_parquet = (
                target_root
                / "data"
                / f"chunk-{target_chunk:03d}"
                / f"episode_{next_episode_index:06d}.parquet"
            )
            target_parquet.parent.mkdir(parents=True, exist_ok=True)
            table = pa.Table.from_pandas(df, preserve_index=False)
            pq.write_table(table, target_parquet)

            appended_videos += _copy_video_files(
                dataset_root=dataset_root,
                output_root=target_root,
                old_episode_index=old_episode_index,
                new_episode_index=next_episode_index,
                old_chunk_size=dataset_chunk_size,
                new_chunk_size=target_chunks_size,
                video_keys=video_keys,
            )

            episodes_rows.append(
                {
                    "episode_index": next_episode_index,
                    "episode_name": episode_row.get("episode_name", f"episode_{next_episode_index:06d}"),
                    "tasks": list(episode_row.get("tasks", [old_task_name])),
                    "length": int(episode_row["length"]),
                }
            )

            source_info = source_lookup.get(old_episode_index, {})
            source_map_rows.append(
                {
                    "episode_index": next_episode_index,
                    "dataset_file_stem": f"episode_{next_episode_index:06d}",
                    "source_episode_name": source_info.get(
                        "source_episode_name", episode_row.get("episode_name", "")
                    ),
                    "source_episode_dir": source_info.get("source_episode_dir", ""),
                    "task": source_info.get("task", old_task_name),
                    "source_dataset_root": str(dataset_root),
                    "source_dataset_episode_index": old_episode_index,
                }
            )

            total_frames += frame_count
            next_episode_index += 1
            appended_episodes += 1

        appended_from_rows.append(str(dataset_root))

    summary = _recompute_dataset_stats(
        output_root=target_root,
        chunks_size=target_chunks_size,
        modality=target_modality,
        features=target_features,
        robot_type=target_robot_type,
        tasks_rows=target_tasks_rows,
        episodes_rows=episodes_rows,
        source_map_rows=source_map_rows,
        merged_from_rows=appended_from_rows,
    )
    summary["appended_input_datasets"] = len(dataset_paths)
    summary["appended_episodes"] = appended_episodes
    summary["appended_videos"] = appended_videos
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Append one or more LeRobot datasets into an existing target dataset."
    )
    parser.add_argument("--target-root", required=True, help="Existing target dataset root to append into.")
    parser.add_argument(
        "--dataset-path",
        action="append",
        default=[],
        help="Input dataset root to append. Can be passed multiple times.",
    )
    parser.add_argument(
        "--chunks-size",
        type=int,
        default=None,
        help="Optional output chunk size override. Defaults to the target dataset chunk size.",
    )
    args = parser.parse_args()

    dataset_paths = _resolve_dataset_roots(list(args.dataset_path))
    summary = append_datasets(
        target_root=Path(args.target_root).expanduser().resolve(),
        dataset_paths=dataset_paths,
        chunks_size_override=args.chunks_size,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

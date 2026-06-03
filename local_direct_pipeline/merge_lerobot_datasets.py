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


def _ensure_output_root(output_root: Path, force: bool) -> None:
    if output_root.exists():
        if not force:
            raise FileExistsError(
                f"Output root already exists: {output_root}\n"
                "Pass --force to replace it."
            )
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)


def _load_dataset_meta(dataset_root: Path) -> tuple[dict, list[dict], dict[str, int], dict, list[dict]]:
    meta_root = dataset_root / "meta"
    info = _read_json(meta_root / "info.json")
    episodes = _read_jsonl(meta_root / "episodes.jsonl")
    tasks_rows = _read_jsonl(meta_root / "tasks.jsonl")
    tasks_map = {str(row["task"]): int(row["task_index"]) for row in tasks_rows}
    modality = _read_json(meta_root / "modality.json")
    source_map = _read_jsonl(meta_root / "source_map.jsonl")
    return info, episodes, tasks_map, modality, source_map


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


def merge_datasets(
    *,
    dataset_paths: list[Path],
    output_root: Path,
    chunks_size: int,
) -> dict[str, object]:
    import pyarrow as pa
    import pyarrow.parquet as pq

    first_info, _, _, first_modality, _ = _load_dataset_meta(dataset_paths[0])
    robot_type = str(first_info["robot_type"])
    features = dict(first_info["features"])
    video_keys = [key for key, value in features.items() if value.get("dtype") == "video"]

    task_to_new_index: dict[str, int] = {}
    merged_task_rows: list[dict] = []
    merged_episode_rows: list[dict] = []
    merged_source_map_rows: list[dict] = []

    total_frames = 0
    total_videos = 0
    next_episode_index = 0

    for dataset_root in dataset_paths:
        info, episodes_rows, _dataset_task_map, modality, source_map_rows = _load_dataset_meta(dataset_root)
        if str(info["robot_type"]) != robot_type:
            raise ValueError(
                f"robot_type mismatch: expected {robot_type}, got {info['robot_type']} from {dataset_root}"
            )
        if modality != first_modality:
            raise ValueError(f"modality.json mismatch for dataset {dataset_root}")

        source_lookup = _source_map_lookup(source_map_rows)
        dataset_chunk_size = int(info["chunks_size"])

        for episode_row in episodes_rows:
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
            if old_task_name not in task_to_new_index:
                task_to_new_index[old_task_name] = len(task_to_new_index)
                merged_task_rows.append(
                    {"task_index": task_to_new_index[old_task_name], "task": old_task_name}
                )
            new_task_index = task_to_new_index[old_task_name]

            frame_count = len(df)
            df["episode_index"] = next_episode_index
            df["index"] = np.arange(total_frames, total_frames + frame_count, dtype=np.int64)
            if "task_index" in df.columns:
                df["task_index"] = new_task_index
            if "annotation.human.action.task_description" in df.columns:
                df["annotation.human.action.task_description"] = new_task_index

            target_chunk = next_episode_index // chunks_size
            target_parquet = (
                output_root
                / "data"
                / f"chunk-{target_chunk:03d}"
                / f"episode_{next_episode_index:06d}.parquet"
            )
            target_parquet.parent.mkdir(parents=True, exist_ok=True)
            table = pa.Table.from_pandas(df, preserve_index=False)
            pq.write_table(table, target_parquet)

            total_videos += _copy_video_files(
                dataset_root=dataset_root,
                output_root=output_root,
                old_episode_index=old_episode_index,
                new_episode_index=next_episode_index,
                old_chunk_size=dataset_chunk_size,
                new_chunk_size=chunks_size,
                video_keys=video_keys,
            )

            merged_episode_rows.append(
                {
                    "episode_index": next_episode_index,
                    "episode_name": episode_row.get("episode_name", f"episode_{next_episode_index:06d}"),
                    "tasks": list(episode_row.get("tasks", [old_task_name])),
                    "length": int(episode_row["length"]),
                }
            )

            source_info = source_lookup.get(old_episode_index, {})
            merged_source_map_rows.append(
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

    merged_action_array, merged_observation_array, merged_timestamp_array = _collect_dataset_arrays(
        output_root
    )
    stats = {
        "action": _summarize_array(merged_action_array),
        "observation.state": _summarize_array(merged_observation_array),
        "timestamp": _summarize_array(merged_timestamp_array),
    }

    min_episode_length = min(int(row["length"]) for row in merged_episode_rows)
    max_relative_horizon = max(1, min(16, min_episode_length))
    delta_indices = list(range(max_relative_horizon))
    relative_horizon_parts: dict[str, list[list[np.ndarray]]] = {
        key: [[] for _ in delta_indices]
        for key in first_modality.get("action", {})
        if key in first_modality.get("state", {})
    }

    for row in merged_episode_rows:
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
            state_slice = first_modality["state"][key]
            action_slice = first_modality["action"][key]
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
    avg_fps = sum(fps_values) / len(fps_values) if fps_values else float(first_info.get("fps", 0.0))

    merged_info = {
        "codebase_version": str(first_info.get("codebase_version", "v2.1")),
        "robot_type": robot_type,
        "total_episodes": len(merged_episode_rows),
        "total_frames": total_frames,
        "total_tasks": len(merged_task_rows),
        "chunks_size": chunks_size,
        "fps": round(avg_fps, 6),
        "splits": {"train": f"0:{len(merged_episode_rows)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": features,
        "total_chunks": max(1, len({row['episode_index'] // chunks_size for row in merged_episode_rows})),
        "total_videos": total_videos,
    }

    _write_json(output_root / "meta" / "info.json", merged_info)
    _write_json(output_root / "meta" / "stats.json", stats)
    _write_json(output_root / "meta" / "relative_stats.json", relative_stats)
    _write_json(output_root / "meta" / "modality.json", first_modality)
    _write_jsonl(output_root / "meta" / "tasks.jsonl", merged_task_rows)
    _write_jsonl(output_root / "meta" / "episodes.jsonl", merged_episode_rows)
    _write_jsonl(output_root / "meta" / "source_map.jsonl", merged_source_map_rows)
    _write_json(output_root / "meta" / "merged_from_datasets.json", {"dataset_paths": [str(path) for path in dataset_paths]})

    return {
        "output_root": str(output_root),
        "total_input_datasets": len(dataset_paths),
        "total_episodes": len(merged_episode_rows),
        "total_frames": total_frames,
        "total_videos": total_videos,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Merge multiple direct-built LeRobot datasets into one final dataset."
    )
    parser.add_argument(
        "--dataset-path",
        action="append",
        default=[],
        help="Input dataset root to merge. Can be passed multiple times.",
    )
    parser.add_argument("--output-root", required=True, help="Output merged dataset root.")
    parser.add_argument(
        "--chunks-size",
        type=int,
        default=1000,
        help="Episodes per chunk in the merged output dataset.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace the output directory if it already exists.",
    )
    args = parser.parse_args()

    dataset_paths = _resolve_dataset_roots(list(args.dataset_path))
    output_root = Path(args.output_root).expanduser().resolve()
    _ensure_output_root(output_root, bool(args.force))
    (output_root / "meta").mkdir(parents=True, exist_ok=True)
    (output_root / "data").mkdir(parents=True, exist_ok=True)
    (output_root / "videos").mkdir(parents=True, exist_ok=True)

    summary = merge_datasets(
        dataset_paths=dataset_paths,
        output_root=output_root,
        chunks_size=int(args.chunks_size),
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

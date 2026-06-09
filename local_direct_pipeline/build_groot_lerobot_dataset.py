from __future__ import annotations

import argparse
import bisect
import csv
import json
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import pyarrow as pa


DEFAULT_CHUNK_SIZE = 1000
DEFAULT_ROBOT_TYPE = "marvin_dual_arm_with_dexterous_hand"
DEFAULT_CAMERA_FPS = 30.0


@dataclass(frozen=True)
class CameraSpec:
    name: str
    path: Path
    fps: float
    width: int
    height: int


@dataclass(frozen=True)
class EpisodeData:
    episode_name: str
    episode_dir: Path
    task: str
    arm_state_names: list[str]
    arm_action_names: list[str]
    arm_ee_pose_state_names: list[str]
    arm_ee_pose_action_names: list[str]
    hand_actual_names: list[str]
    hand_target_names: list[str]
    observation_rows: list[list[float]]
    action_rows: list[list[float]]
    rel_timestamps: list[float]
    camera_specs: list[CameraSpec]


def _read_numeric_csv(path: Path) -> tuple[list[str], list[list[float]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        rows = [[float(value) for value in row] for row in reader]
    return header, rows


def _read_timestamp_csv(path: Path) -> list[float]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        return [float(row["timestamp_unix"]) for row in reader]


def _read_timestamped_numeric_csv(timestamp_path: Path, data_path: Path) -> tuple[list[str], list[float], list[list[float]]]:
    names, rows = _read_numeric_csv(data_path)
    timestamps = _read_timestamp_csv(timestamp_path)
    if len(timestamps) != len(rows):
        raise RuntimeError(
            f"Timestamp/data length mismatch: {timestamp_path} has {len(timestamps)} rows, "
            f"{data_path} has {len(rows)} rows"
        )
    return names, timestamps, rows


def _read_hand_csv(path: Path) -> tuple[list[str], list[float], list[list[float]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        hand_names = fieldnames[3:]
        timestamps: list[float] = []
        values: list[list[float]] = []
        for row in reader:
            timestamps.append(float(row["recv_wall"]))
            values.append([float(row[name]) for name in hand_names])
    return hand_names, timestamps, values


def _read_optional_json(path: Path) -> dict:
    if not path.is_file():
        return {}
    return _read_json(path)


def _read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")


def _finite_row(values: list[float]) -> bool:
    return bool(values) and bool(np.all(np.isfinite(np.asarray(values, dtype=np.float32))))


def _resolve_image_path(raw_path: str, camera_dir: Path) -> Path:
    path = Path(raw_path)
    if path.is_file():
        return path
    if not path.is_absolute():
        candidate = camera_dir / path
        if candidate.is_file():
            return candidate
        candidate = camera_dir / "images" / path.name
        if candidate.is_file():
            return candidate
    candidate = camera_dir / "images" / path.name
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(f"Image listed in frames.csv is missing: {raw_path}")


def _read_frame_rows(frames_csv: Path) -> list[dict[str, str]]:
    if not frames_csv.is_file():
        return []
    with frames_csv.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def _image_shape(path: Path) -> tuple[int, int]:
    try:
        import cv2

        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is not None:
            height, width = image.shape[:2]
            return int(width), int(height)
    except ImportError:
        pass
    try:
        import imageio.v2 as imageio

        image = imageio.imread(path)
        height, width = image.shape[:2]
        return int(width), int(height)
    except ImportError:
        pass
    try:
        from PIL import Image

        with Image.open(path) as image:
            width, height = image.size
            return int(width), int(height)
    except ImportError as exc:
        raise RuntimeError("OpenCV, imageio, or Pillow is required to infer image dimensions.") from exc


def _ffmpeg_quote(path: Path) -> str:
    return str(path).replace("'", "'\\''")


def _write_video_with_ffmpeg(image_paths: list[Path], output_path: Path, fps: float) -> bool:
    if shutil.which("ffmpeg") is None:
        return False
    with tempfile.TemporaryDirectory() as tmp_dir:
        list_path = Path(tmp_dir) / "frames.txt"
        with list_path.open("w", encoding="utf-8") as handle:
            for image_path in image_paths:
                handle.write(f"file '{_ffmpeg_quote(image_path)}'\n")
        command = [
            "ffmpeg",
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-r",
            f"{float(fps):.6f}",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(list_path),
            "-an",
            "-pix_fmt",
            "yuv420p",
            "-vcodec",
            "libx264",
            str(output_path),
        ]
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        return False
    return output_path.is_file()


def _write_video_from_images(image_paths: list[Path], output_path: Path, fps: float) -> tuple[int, int]:
    if not image_paths:
        raise RuntimeError(f"No images available to build video: {output_path}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    width, height = _image_shape(image_paths[0])
    if _write_video_with_ffmpeg(image_paths, output_path, fps):
        return width, height
    try:
        import cv2

        writer = cv2.VideoWriter(
            str(output_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(fps),
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {output_path}")
        try:
            for image_path in image_paths:
                frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError(f"Failed to read image: {image_path}")
                if frame.shape[1] != width or frame.shape[0] != height:
                    frame = cv2.resize(frame, (width, height))
                writer.write(frame)
        finally:
            writer.release()
        return width, height
    except ImportError:
        pass

    try:
        import imageio.v2 as imageio

        with imageio.get_writer(output_path, fps=float(fps), codec="libx264") as writer:
            for image_path in image_paths:
                writer.append_data(imageio.imread(image_path))
        return width, height
    except ImportError as exc:
        raise RuntimeError("OpenCV or imageio is required to create camera.mp4 from images.") from exc


def _ensure_camera_video(camera_dir: Path, fps: float) -> CameraSpec | None:
    frames_csv = camera_dir / "frames.csv"
    frame_rows = _read_frame_rows(frames_csv)
    if not frame_rows:
        return None

    image_paths = [_resolve_image_path(row.get("image_path", ""), camera_dir) for row in frame_rows]
    video_path = camera_dir / "camera.mp4"
    width = int(float(frame_rows[0].get("width", 0) or 0))
    height = int(float(frame_rows[0].get("height", 0) or 0))
    if video_path.is_file():
        if width <= 0 or height <= 0:
            width, height = _image_shape(image_paths[0])
        return CameraSpec(name=camera_dir.name, path=video_path, fps=fps, width=width, height=height)

    width, height = _write_video_from_images(image_paths, video_path, fps)
    return CameraSpec(name=camera_dir.name, path=video_path, fps=fps, width=width, height=height)


def _group_name(column_name: str, modality: str) -> str:
    if column_name.startswith("left_"):
        return f"left_{modality}"
    if column_name.startswith("right_"):
        return f"right_{modality}"
    return modality


def _contiguous_slices(names: list[str], modality: str, start_offset: int = 0) -> dict[str, dict[str, int]]:
    slices: dict[str, dict[str, int]] = {}
    active_key = ""
    seen_closed: set[str] = set()
    for offset, name in enumerate(names):
        key = _group_name(name, modality)
        absolute = start_offset + offset
        if key != active_key:
            if key in seen_closed:
                raise RuntimeError(f"Columns for {key} must be contiguous")
            if active_key:
                seen_closed.add(active_key)
            slices[key] = {"start": absolute, "end": absolute}
            active_key = key
        slices[key]["end"] = absolute + 1
    return slices


def _build_modality_slices(
    arm_joint_names: list[str], arm_ee_pose_names: list[str], hand_names: list[str]
) -> dict[str, dict[str, int]]:
    slices = _contiguous_slices(arm_joint_names, "arm_joint", 0)
    ee_offset = len(arm_joint_names)
    slices.update(_contiguous_slices(arm_ee_pose_names, "arm_ee_pose", ee_offset))
    hand_offset = ee_offset + len(arm_ee_pose_names)
    slices.update(_contiguous_slices(hand_names, "hand", hand_offset))
    return slices


def _dataset_episode_stem(episode_index: int) -> str:
    return f"episode_{episode_index:06d}"


def _nearest_index(sorted_values: list[float], target: float) -> int:
    position = bisect.bisect_left(sorted_values, target)
    if position <= 0:
        return 0
    if position >= len(sorted_values):
        return len(sorted_values) - 1
    left = position - 1
    right = position
    if abs(sorted_values[right] - target) < abs(sorted_values[left] - target):
        return right
    return left


def _downsample_indices_by_time(timestamps: list[float], target_hz: float) -> list[int]:
    if target_hz <= 0.0 or len(timestamps) <= 1:
        return list(range(len(timestamps)))
    period_sec = 1.0 / float(target_hz)
    selected = [0]
    next_time = float(timestamps[0]) + period_sec
    for index, timestamp in enumerate(timestamps[1:], start=1):
        timestamp = float(timestamp)
        if timestamp + 1e-9 < next_time:
            continue
        selected.append(index)
        while next_time <= timestamp:
            next_time += period_sec
    return selected


def _fixed_size_list_array(rows: list[list[float]], width: int) -> pa.Array:
    import pyarrow as pa

    flat: list[float] = []
    for row in rows:
        flat.extend(float(value) for value in row)
    values = pa.array(flat, type=pa.float32())
    return pa.FixedSizeListArray.from_arrays(values, width)


def _summarize_array(values: np.ndarray) -> dict[str, list[float]]:
    if values.ndim == 1:
        values = values.reshape(-1, 1)
    return {
        "mean": np.mean(values, axis=0, dtype=np.float64).astype(np.float32).tolist(),
        "std": np.std(values, axis=0, dtype=np.float64).astype(np.float32).tolist(),
        "min": np.min(values, axis=0).astype(np.float32).tolist(),
        "max": np.max(values, axis=0).astype(np.float32).tolist(),
        "q01": np.quantile(values, 0.01, axis=0).astype(np.float32).tolist(),
        "q99": np.quantile(values, 0.99, axis=0).astype(np.float32).tolist(),
    }


def _collect_relative_horizon_values(
    current_state: np.ndarray,
    future_action: np.ndarray,
    delta_indices: list[int],
) -> list[np.ndarray]:
    horizon_values: list[np.ndarray] = []
    total_steps = current_state.shape[0]
    for delta in delta_indices:
        if delta < 0:
            raise ValueError("Negative delta_indices are not supported")
        if delta >= total_steps:
            raise ValueError(f"delta index {delta} exceeds available steps {total_steps}")
        base = current_state[: total_steps - delta]
        target = future_action[delta:]
        horizon_values.append((target - base).astype(np.float32))
    return horizon_values


def _summarize_relative_horizon_groups(groups: list[list[np.ndarray]]) -> dict[str, list[list[float]]]:
    keys = ("mean", "std", "min", "max", "q01", "q99")
    summary = {key: [] for key in keys}
    for group in groups:
        merged = np.concatenate(group, axis=0)
        summary["mean"].append(np.mean(merged, axis=0, dtype=np.float64).astype(np.float32).tolist())
        summary["std"].append(np.std(merged, axis=0, dtype=np.float64).astype(np.float32).tolist())
        summary["min"].append(np.min(merged, axis=0).astype(np.float32).tolist())
        summary["max"].append(np.max(merged, axis=0).astype(np.float32).tolist())
        summary["q01"].append(np.quantile(merged, 0.01, axis=0).astype(np.float32).tolist())
        summary["q99"].append(np.quantile(merged, 0.99, axis=0).astype(np.float32).tolist())
    return summary


def episode_is_complete(episode_dir: Path, default_task: str = "") -> tuple[bool, str]:
    required_paths = [
        episode_dir / "meta.json",
        episode_dir / "arm_data" / "timestamp.csv",
        episode_dir / "arm_data" / "observation_state.csv",
        episode_dir / "arm_data" / "action.csv",
        episode_dir / "arm_data" / "ee_pose_timestamp.csv",
        episode_dir / "arm_data" / "ee_pose_observation_state.csv",
        episode_dir / "arm_data" / "ee_pose_action.csv",
    ]
    for required_path in required_paths:
        if not required_path.exists():
            return False, f"missing required file: {required_path}"
    new_hand_paths = [
        episode_dir / "hand_data" / "timestamp.csv",
        episode_dir / "hand_data" / "observation_state.csv",
        episode_dir / "hand_data" / "action.csv",
    ]
    legacy_hand_paths = [
        episode_dir / "hand_data" / "actual_position.csv",
        episode_dir / "hand_data" / "target_position.csv",
    ]
    if not all(path.exists() for path in new_hand_paths) and not all(path.exists() for path in legacy_hand_paths):
        return False, "missing required hand_data files"
    try:
        meta = _read_json(episode_dir / "meta.json")
    except json.JSONDecodeError as exc:
        return False, f"invalid meta.json: {exc}"
    sample_info = _read_optional_json(episode_dir / "_runtime" / "sample_info.json")
    task = str(meta.get("task", "") or sample_info.get("task", "") or default_task).strip()
    if not task:
        return False, "empty task in meta.json"
    return True, ""


def discover_episode_dirs(source_dir: Path) -> list[Path]:
    if source_dir.name.startswith("episode_"):
        return [source_dir.resolve()]
    return sorted(path.resolve() for path in source_dir.iterdir() if path.is_dir() and path.name.startswith("episode_"))


def _discover_camera_specs(episode_dir: Path, meta: dict, sample_info: dict) -> list[CameraSpec]:
    camera_specs: list[CameraSpec] = []
    recorder_items = meta.get("camera_data", {}).get("camera_recorders", [])
    for recorder in recorder_items:
        camera_name = str(recorder.get("camera_name", "")).strip()
        if not camera_name:
            continue
        video_path = episode_dir / "camera_data" / camera_name / "camera.mp4"
        if not video_path.is_file():
            continue
        camera_specs.append(
            CameraSpec(
                name=camera_name,
                path=video_path,
                fps=float(recorder.get("camera_fps", 0.0) or 0.0),
                width=int(recorder.get("camera_width", 0) or 0),
                height=int(recorder.get("camera_height", 0) or 0),
            )
        )
    if camera_specs:
        return sorted(camera_specs, key=lambda item: item.name)

    camera_names = list(sample_info.get("camera_names", []) or meta.get("camera_names", []) or [])
    if not camera_names:
        camera_root = episode_dir / "camera_data"
        if camera_root.is_dir():
            camera_names = [path.name for path in sorted(camera_root.iterdir()) if path.is_dir()]
    fps = float(sample_info.get("camera_fps", 0.0) or sample_info.get("fps", 0.0) or DEFAULT_CAMERA_FPS)
    if fps <= 0.0 or fps > 240.0:
        fps = DEFAULT_CAMERA_FPS
    for camera_name in camera_names:
        spec = _ensure_camera_video(episode_dir / "camera_data" / str(camera_name), fps)
        if spec is not None:
            camera_specs.append(spec)
    return sorted(camera_specs, key=lambda item: item.name)


def load_episode(episode_dir: Path, target_hz: float, default_task: str = "") -> EpisodeData:
    is_complete, reason = episode_is_complete(episode_dir, default_task=default_task)
    if not is_complete:
        raise ValueError(f"Episode {episode_dir} is not complete: {reason}")

    meta = _read_json(episode_dir / "meta.json")
    sample_info = _read_optional_json(episode_dir / "_runtime" / "sample_info.json")
    task = str(meta.get("task", "") or sample_info.get("task", "") or default_task).strip()

    arm_dir = episode_dir / "arm_data"
    hand_dir = episode_dir / "hand_data"
    episode_timestamps = _read_timestamp_csv(arm_dir / "timestamp.csv")
    arm_state_names, arm_state_rows = _read_numeric_csv(arm_dir / "observation_state.csv")
    arm_action_names, arm_action_rows = _read_numeric_csv(arm_dir / "action.csv")
    arm_ee_pose_state_names, arm_ee_pose_times, arm_ee_pose_state_rows = _read_timestamped_numeric_csv(
        arm_dir / "ee_pose_timestamp.csv", arm_dir / "ee_pose_observation_state.csv"
    )
    arm_ee_pose_action_names, arm_ee_pose_action_times, arm_ee_pose_action_rows = _read_timestamped_numeric_csv(
        arm_dir / "ee_pose_timestamp.csv", arm_dir / "ee_pose_action.csv"
    )
    if (hand_dir / "timestamp.csv").is_file():
        hand_actual_names, hand_actual_times, hand_actual_rows = _read_timestamped_numeric_csv(
            hand_dir / "timestamp.csv", hand_dir / "observation_state.csv"
        )
        hand_target_names, hand_target_times, hand_target_rows = _read_timestamped_numeric_csv(
            hand_dir / "timestamp.csv", hand_dir / "action.csv"
        )
    else:
        hand_actual_names, hand_actual_times, hand_actual_rows = _read_hand_csv(hand_dir / "actual_position.csv")
        hand_target_names, hand_target_times, hand_target_rows = _read_hand_csv(hand_dir / "target_position.csv")

    if not episode_timestamps:
        raise RuntimeError(f"Episode {episode_dir.name} has no timestamps")
    if not (len(episode_timestamps) == len(arm_state_rows) == len(arm_action_rows)):
        raise RuntimeError(f"Episode {episode_dir.name} timestamp/state/action lengths do not match")

    keep_indices = _downsample_indices_by_time(episode_timestamps, target_hz)
    episode_timestamps = [episode_timestamps[index] for index in keep_indices]
    arm_state_rows = [arm_state_rows[index] for index in keep_indices]
    arm_action_rows = [arm_action_rows[index] for index in keep_indices]

    observation_rows: list[list[float]] = []
    action_rows: list[list[float]] = []
    rel_timestamps: list[float] = []
    start_time = episode_timestamps[0]
    for idx, timestamp in enumerate(episode_timestamps):
        arm_ee_pose_state_idx = _nearest_index(arm_ee_pose_times, timestamp)
        arm_ee_pose_action_idx = _nearest_index(arm_ee_pose_action_times, timestamp)
        hand_actual_idx = _nearest_index(hand_actual_times, timestamp)
        hand_target_idx = _nearest_index(hand_target_times, timestamp)
        observation_row = (
            list(arm_state_rows[idx])
            + list(arm_ee_pose_state_rows[arm_ee_pose_state_idx])
            + list(hand_actual_rows[hand_actual_idx])
        )
        action_row = (
            list(arm_action_rows[idx])
            + list(arm_ee_pose_action_rows[arm_ee_pose_action_idx])
            + list(hand_target_rows[hand_target_idx])
        )
        if not (_finite_row(observation_row) and _finite_row(action_row)):
            continue
        observation_rows.append(observation_row)
        action_rows.append(action_row)
        rel_timestamps.append(float(timestamp - start_time))
    if not rel_timestamps:
        raise RuntimeError(f"Episode {episode_dir.name} has no finite aligned rows")

    return EpisodeData(
        episode_name=episode_dir.name,
        episode_dir=episode_dir,
        task=task,
        arm_state_names=arm_state_names,
        arm_action_names=arm_action_names,
        arm_ee_pose_state_names=arm_ee_pose_state_names,
        arm_ee_pose_action_names=arm_ee_pose_action_names,
        hand_actual_names=hand_actual_names,
        hand_target_names=hand_target_names,
        observation_rows=observation_rows,
        action_rows=action_rows,
        rel_timestamps=rel_timestamps,
        camera_specs=_discover_camera_specs(episode_dir, meta, sample_info),
    )


def _collect_dataset_arrays(output_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    import pyarrow.parquet as pq

    parquet_paths = sorted((output_dir / "data").glob("chunk-*/*.parquet"))
    all_action_rows: list[list[float]] = []
    all_observation_rows: list[list[float]] = []
    all_timestamp_rows: list[float] = []
    for parquet_path in parquet_paths:
        table = pq.read_table(parquet_path, columns=["action", "observation.state", "timestamp"])
        all_action_rows.extend(table["action"].to_pylist())
        all_observation_rows.extend(table["observation.state"].to_pylist())
        all_timestamp_rows.extend(table["timestamp"].to_pylist())
    return (
        np.asarray(all_action_rows, dtype=np.float32),
        np.asarray(all_observation_rows, dtype=np.float32),
        np.asarray(all_timestamp_rows, dtype=np.float32),
    )


def _read_batch_manifest(batch_root: Path) -> dict:
    manifest_path = batch_root / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"manifest.json not found under {batch_root}")
    payload = _read_json(manifest_path)
    if not isinstance(payload, dict):
        raise ValueError(f"Invalid batch manifest in {manifest_path}")
    return payload


def _resolve_episode_dirs_from_batch(batch_root: Path) -> list[Path]:
    manifest = _read_batch_manifest(batch_root)
    episode_names = manifest.get("episode_names")
    if not isinstance(episode_names, list) or not episode_names:
        raise ValueError(f"Batch manifest missing non-empty episode_names: {batch_root / 'manifest.json'}")
    episode_dirs: list[Path] = []
    for episode_name in episode_names:
        episode_dir = (batch_root / str(episode_name)).expanduser().resolve()
        if not episode_dir.is_dir():
            raise FileNotFoundError(f"Episode referenced by manifest is missing: {episode_dir}")
        episode_dirs.append(episode_dir)
    return episode_dirs


def build_dataset(
    *,
    episode_dirs: list[Path],
    output_dir: Path,
    target_hz: float,
    chunks_size: int,
    robot_type: str,
    overwrite_output: bool,
    batch_manifest: dict | None = None,
    default_task: str = "",
) -> dict[str, object]:
    import pyarrow as pa
    import pyarrow.parquet as pq

    if not episode_dirs:
        raise ValueError("episode_dirs must not be empty")

    if output_dir.exists():
        if not overwrite_output:
            raise FileExistsError(
                f"Output dataset path already exists: {output_dir}\n"
                "Pass --overwrite-output to recreate it."
            )
        shutil.rmtree(output_dir)

    episodes_data = [
        load_episode(episode_dir, target_hz, default_task=default_task)
        for episode_dir in episode_dirs
    ]
    first_episode = episodes_data[0]
    observation_names = (
        list(first_episode.arm_state_names)
        + list(first_episode.arm_ee_pose_state_names)
        + list(first_episode.hand_actual_names)
    )
    action_names = (
        list(first_episode.arm_action_names)
        + list(first_episode.arm_ee_pose_action_names)
        + list(first_episode.hand_target_names)
    )

    for episode in episodes_data[1:]:
        current_observation_names = (
            list(episode.arm_state_names)
            + list(episode.arm_ee_pose_state_names)
            + list(episode.hand_actual_names)
        )
        current_action_names = (
            list(episode.arm_action_names)
            + list(episode.arm_ee_pose_action_names)
            + list(episode.hand_target_names)
        )
        if current_observation_names != observation_names:
            raise RuntimeError(f"{episode.episode_name} observation columns do not match the first episode")
        if current_action_names != action_names:
            raise RuntimeError(f"{episode.episode_name} action columns do not match the first episode")

    (output_dir / "meta").mkdir(parents=True, exist_ok=True)

    task_to_index: dict[str, int] = {}
    next_task_index = 0
    for task in sorted({episode.task for episode in episodes_data}):
        if task not in task_to_index:
            task_to_index[task] = next_task_index
            next_task_index += 1
    tasks = [{"task_index": idx, "task": task} for task, idx in sorted(task_to_index.items(), key=lambda item: item[1])]

    all_camera_names = sorted({camera.name for episode in episodes_data for camera in episode.camera_specs})
    camera_feature_info: dict[str, CameraSpec] = {}
    for episode in episodes_data:
        for camera in episode.camera_specs:
            camera_feature_info.setdefault(camera.name, camera)

    all_action_arrays: list[np.ndarray] = []
    all_observation_arrays: list[np.ndarray] = []
    all_timestamp_arrays: list[np.ndarray] = []
    new_episode_rows: list[dict[str, object]] = []
    source_map_rows: list[dict[str, object]] = []
    total_frames = 0
    total_videos = 0
    min_episode_length = min(len(episode.rel_timestamps) for episode in episodes_data)
    max_relative_horizon = max(1, min(16, min_episode_length))
    delta_indices = list(range(0, max_relative_horizon))
    state_slices = _build_modality_slices(
        first_episode.arm_state_names,
        first_episode.arm_ee_pose_state_names,
        first_episode.hand_actual_names,
    )
    action_slices = _build_modality_slices(
        first_episode.arm_action_names,
        first_episode.arm_ee_pose_action_names,
        first_episode.hand_target_names,
    )
    relative_horizon_parts: dict[str, list[list[np.ndarray]]] = {
        key: [[] for _ in delta_indices]
        for key in action_slices
        if key in state_slices
    }

    for offset, episode in enumerate(episodes_data):
        dataset_episode_index = offset
        chunk_index = dataset_episode_index // chunks_size
        chunk_dir = output_dir / "data" / f"chunk-{chunk_index:03d}"
        chunk_dir.mkdir(parents=True, exist_ok=True)

        observation_rows = episode.observation_rows
        action_rows = episode.action_rows
        rel_timestamps = episode.rel_timestamps
        action_array = np.asarray(action_rows, dtype=np.float32)
        observation_array = np.asarray(observation_rows, dtype=np.float32)
        timestamp_array = np.asarray(rel_timestamps, dtype=np.float32)
        num_frames = len(rel_timestamps)
        task_index_value = task_to_index[episode.task]

        frame_index = list(range(num_frames))
        episode_index = [dataset_episode_index] * num_frames
        global_frame_offset = total_frames
        index = list(range(global_frame_offset, global_frame_offset + num_frames))
        task_index = [task_index_value] * num_frames
        annotation_task = [task_index_value] * num_frames
        next_reward = [0.0] * num_frames
        next_done = [idx == num_frames - 1 for idx in range(num_frames)]

        schema = pa.schema(
            [
                ("observation.state", pa.list_(pa.float32(), len(observation_rows[0]))),
                ("action", pa.list_(pa.float32(), len(action_rows[0]))),
                ("timestamp", pa.float32()),
                ("annotation.human.action.task_description", pa.int64()),
                ("frame_index", pa.int64()),
                ("episode_index", pa.int64()),
                ("index", pa.int64()),
                ("task_index", pa.int64()),
                ("next.reward", pa.float32()),
                ("next.done", pa.bool_()),
            ]
        )
        table = pa.Table.from_arrays(
            [
                _fixed_size_list_array(observation_rows, len(observation_rows[0])),
                _fixed_size_list_array(action_rows, len(action_rows[0])),
                pa.array(rel_timestamps, type=pa.float32()),
                pa.array(annotation_task, type=pa.int64()),
                pa.array(frame_index, type=pa.int64()),
                pa.array(episode_index, type=pa.int64()),
                pa.array(index, type=pa.int64()),
                pa.array(task_index, type=pa.int64()),
                pa.array(next_reward, type=pa.float32()),
                pa.array(next_done, type=pa.bool_()),
            ],
            schema=schema,
        )
        episode_stem = _dataset_episode_stem(dataset_episode_index)
        pq.write_table(table, chunk_dir / f"{episode_stem}.parquet")

        for camera in episode.camera_specs:
            target_path = (
                output_dir
                / "videos"
                / f"chunk-{chunk_index:03d}"
                / f"observation.images.{camera.name}"
                / f"{episode_stem}.mp4"
            )
            target_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(camera.path, target_path)
            total_videos += 1

        all_action_arrays.append(action_array)
        all_observation_arrays.append(observation_array)
        all_timestamp_arrays.append(timestamp_array)
        for key, parts in relative_horizon_parts.items():
            state_slice = state_slices[key]
            action_slice = action_slices[key]
            values = _collect_relative_horizon_values(
                observation_array[:, state_slice["start"]:state_slice["end"]],
                action_array[:, action_slice["start"]:action_slice["end"]],
                delta_indices,
            )
            for delta_idx in range(len(delta_indices)):
                parts[delta_idx].append(values[delta_idx])
        new_episode_rows.append(
            {
                "episode_index": dataset_episode_index,
                "episode_name": episode.episode_name,
                "tasks": [episode.task],
                "length": num_frames,
            }
        )
        source_map_rows.append(
            {
                "episode_index": dataset_episode_index,
                "dataset_file_stem": episode_stem,
                "source_episode_name": episode.episode_name,
                "source_episode_dir": str(episode.episode_dir),
                "task": episode.task,
            }
        )
        total_frames += num_frames

    modality = {
        "state": state_slices,
        "action": action_slices,
        "video": {name: {"original_key": f"observation.images.{name}"} for name in all_camera_names},
        "annotation": {
            "human.action.task_description": {
                "original_key": "annotation.human.action.task_description"
            }
        },
    }

    features: dict[str, object] = {
        "action": {"dtype": "float32", "names": action_names, "shape": [len(action_names)]},
        "observation.state": {
            "dtype": "float32",
            "names": observation_names,
            "shape": [len(observation_names)],
        },
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
        "annotation.human.action.task_description": {"dtype": "int64", "shape": [1], "names": None},
        "next.reward": {"dtype": "float32", "shape": [1], "names": None},
        "next.done": {"dtype": "bool", "shape": [1], "names": None},
    }
    for camera_name in all_camera_names:
        camera = camera_feature_info[camera_name]
        features[f"observation.images.{camera_name}"] = {
            "dtype": "video",
            "shape": [camera.height, camera.width, 3],
            "names": ["height", "width", "channels"],
            "info": {
                "video.height": camera.height,
                "video.width": camera.width,
                "video.codec": "h264",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False,
                "video.fps": round(camera.fps, 6),
                "video.channels": 3,
                "has_audio": False,
            },
        }

    fps_values = [camera.fps for camera in camera_feature_info.values() if camera.fps > 0.0]
    avg_fps = sum(fps_values) / len(fps_values) if fps_values else 0.0
    info = {
        "codebase_version": "v2.1",
        "robot_type": robot_type,
        "total_episodes": len(new_episode_rows),
        "total_frames": total_frames,
        "total_tasks": len(tasks),
        "chunks_size": chunks_size,
        "fps": round(avg_fps, 6),
        "splits": {"train": f"0:{len(new_episode_rows)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": features,
        "total_chunks": max(1, len({row["episode_index"] // chunks_size for row in new_episode_rows})),
        "total_videos": total_videos,
    }

    full_action_array, full_observation_array, full_timestamp_array = _collect_dataset_arrays(output_dir)
    stats = {
        "action": _summarize_array(full_action_array),
        "observation.state": _summarize_array(full_observation_array),
        "timestamp": _summarize_array(full_timestamp_array),
    }
    relative_stats = {key: _summarize_relative_horizon_groups(parts) for key, parts in relative_horizon_parts.items()}

    _write_json(output_dir / "meta" / "info.json", info)
    _write_json(output_dir / "meta" / "stats.json", stats)
    _write_json(output_dir / "meta" / "relative_stats.json", relative_stats)
    _write_json(output_dir / "meta" / "modality.json", modality)
    _write_jsonl(output_dir / "meta" / "tasks.jsonl", tasks)
    _write_jsonl(output_dir / "meta" / "episodes.jsonl", new_episode_rows)
    _write_jsonl(output_dir / "meta" / "source_map.jsonl", source_map_rows)
    if batch_manifest is not None:
        _write_json(output_dir / "meta" / "batch_manifest.json", batch_manifest)

    return {
        "output_dir": str(output_dir),
        "episodes": [str(episode_dir) for episode_dir in episode_dirs],
        "episode_names": [episode_dir.name for episode_dir in episode_dirs],
        "total_episodes": len(new_episode_rows),
        "total_frames": total_frames,
        "total_videos": total_videos,
    }


def _resolve_episode_dirs(source_dir: Path, episode_names: list[str], explicit_episode_dirs: list[str]) -> list[Path]:
    if explicit_episode_dirs:
        return [Path(item).expanduser().resolve() for item in explicit_episode_dirs]

    discovered = discover_episode_dirs(source_dir)
    if not episode_names:
        return discovered

    mapping = {path.name: path for path in discovered}
    missing = [name for name in episode_names if name not in mapping]
    if missing:
        raise FileNotFoundError(f"Missing requested episodes under {source_dir}: {missing}")
    return [mapping[name] for name in episode_names]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build a GR00T-compatible LeRobot dataset from one or more complete episodes."
    )
    parser.add_argument("--source-dir", default="data", help="Data root, session root, or episode directory.")
    parser.add_argument(
        "--batch-root",
        default=None,
        help="Frozen local batch root containing manifest.json and episode symlinks/directories.",
    )
    parser.add_argument(
        "--episode-name",
        action="append",
        default=[],
        help="Specific episode name to include. Can be passed multiple times.",
    )
    parser.add_argument(
        "--episode-dir",
        action="append",
        default=[],
        help="Explicit episode directory to include. Can be passed multiple times.",
    )
    parser.add_argument("--output-dir", required=True, help="Output dataset directory.")
    parser.add_argument(
        "--overwrite-output",
        action="store_true",
        help="Replace an existing output dataset directory.",
    )
    parser.add_argument(
        "--target-hz",
        type=float,
        default=0.0,
        help="Optional target frequency for downsampling each episode before fusion. Use <=0 to keep all frames.",
    )
    parser.add_argument(
        "--chunks-size",
        type=int,
        default=DEFAULT_CHUNK_SIZE,
        help="Number of episodes per chunk directory inside the built dataset.",
    )
    parser.add_argument(
        "--robot-type",
        default=DEFAULT_ROBOT_TYPE,
        help="Robot type written into meta/info.json.",
    )
    parser.add_argument(
        "--default-task",
        default="",
        help="Task description used when an episode meta.json does not contain a task.",
    )
    args = parser.parse_args()

    batch_manifest = None
    if args.batch_root:
        batch_root = Path(args.batch_root).expanduser().resolve()
        batch_manifest = _read_batch_manifest(batch_root)
        episode_dirs = _resolve_episode_dirs_from_batch(batch_root)
    else:
        source_dir = Path(args.source_dir).expanduser().resolve()
        episode_dirs = _resolve_episode_dirs(source_dir, list(args.episode_name), list(args.episode_dir))
    summary = build_dataset(
        episode_dirs=episode_dirs,
        output_dir=Path(args.output_dir).expanduser().resolve(),
        target_hz=float(args.target_hz),
        chunks_size=int(args.chunks_size),
        robot_type=str(args.robot_type),
        overwrite_output=bool(args.overwrite_output),
        batch_manifest=batch_manifest,
        default_task=str(args.default_task),
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

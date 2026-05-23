"""TJ-style raw episode helpers for DexProj."""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TJRawEpisodePaths:
    episode_dir: Path
    runtime_dir: Path
    arm_data_dir: Path
    camera_data_dir: Path
    camera_dirs: dict[str, Path]

    @property
    def arm_action_csv(self) -> Path:
        return self.arm_data_dir / "action.csv"

    @property
    def arm_observation_csv(self) -> Path:
        return self.arm_data_dir / "observation_state.csv"

    @property
    def arm_timestamp_csv(self) -> Path:
        return self.arm_data_dir / "timestamp.csv"

    @property
    def runtime_remote_session_json(self) -> Path:
        return self.runtime_dir / "remote_teach_session.json"

    @property
    def runtime_sample_info_json(self) -> Path:
        return self.runtime_dir / "sample_info.json"

    def camera_frames_csv(self, camera: str) -> Path:
        return self.camera_dirs[camera] / "frames.csv"


class TJRawEpisodeWriter:
    def __init__(self, episode_dir: Path, camera_names: list[str] | None = None):
        camera_names = camera_names or ["head", "left_wrist", "right_wrist"]
        self.paths = TJRawEpisodePaths(
            episode_dir=episode_dir,
            runtime_dir=episode_dir / "_runtime",
            arm_data_dir=episode_dir / "arm_data",
            camera_data_dir=episode_dir / "camera_data",
            camera_dirs={name: episode_dir / "camera_data" / name for name in camera_names},
        )
        self.paths.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.paths.arm_data_dir.mkdir(parents=True, exist_ok=True)
        self.paths.camera_data_dir.mkdir(parents=True, exist_ok=True)
        for camera_dir in self.paths.camera_dirs.values():
            camera_dir.mkdir(parents=True, exist_ok=True)
            (camera_dir / "images").mkdir(parents=True, exist_ok=True)
        self._arm_timestamp_fp = None
        self._arm_obs_fp = None
        self._arm_action_fp = None
        self._camera_frame_fps: dict[str, object] = {}
        self._camera_frame_writers: dict[str, csv.writer] = {}

    def start(self, plan: dict) -> None:
        self._arm_timestamp_fp = self.paths.arm_timestamp_csv.open("w", newline="", encoding="utf-8")
        self._arm_obs_fp = self.paths.arm_observation_csv.open("w", newline="", encoding="utf-8")
        self._arm_action_fp = self.paths.arm_action_csv.open("w", newline="", encoding="utf-8")

        self._arm_timestamp_writer = csv.writer(self._arm_timestamp_fp)
        self._arm_obs_writer = csv.writer(self._arm_obs_fp)
        self._arm_action_writer = csv.writer(self._arm_action_fp)

        self._arm_timestamp_writer.writerow(["index", "timestamp_unix"])
        self._arm_obs_writer.writerow(["index", "timestamp_unix", "runtime_state", "start_trigger", "stop_trigger"])
        self._arm_action_writer.writerow(["index", "timestamp_unix", "trigger_mode", "bringup_command"])

        self._camera_frame_writers = {}
        for camera_name in self.paths.camera_dirs:
            frames_fp = self.paths.camera_frames_csv(camera_name).open("w", newline="", encoding="utf-8")
            self._camera_frame_fps[camera_name] = frames_fp
            writer = csv.writer(frames_fp)
            self._camera_frame_writers[camera_name] = writer
            writer.writerow(["index", "timestamp_unix", "timestamp_ms", "image_path"])

        sample_info = {
            "session_dir": str(self.paths.episode_dir.parent),
            "episode_index": int(self.paths.episode_dir.name.split("_")[-1]),
            "arm_data": {
                "timestamp": str(self.paths.arm_timestamp_csv),
                "observation_state": str(self.paths.arm_observation_csv),
                "action": str(self.paths.arm_action_csv),
            },
            "camera_data": {
                "camera_count": len(self.paths.camera_dirs),
                "camera_data_dir": str(self.paths.camera_data_dir),
                "camera_frame_log_paths": {
                    name: str(self.paths.camera_frames_csv(name)) for name in self.paths.camera_dirs
                },
                "camera_image_dirs": {
                    name: str(self.paths.camera_dirs[name] / "images") for name in self.paths.camera_dirs
                },
            },
            "camera_names": list(self.paths.camera_dirs.keys()),
            "fps": 1000.0,
            "sync_source": "camera",
            "state_names": [
                "right_joint_1.pos",
                "right_joint_2.pos",
                "right_joint_3.pos",
                "right_joint_4.pos",
                "right_joint_5.pos",
                "right_joint_6.pos",
                "right_joint_7.pos",
                "left_joint_1.pos",
                "left_joint_2.pos",
                "left_joint_3.pos",
                "left_joint_4.pos",
                "left_joint_5.pos",
                "left_joint_6.pos",
                "left_joint_7.pos",
            ],
            "num_frames": 0,
            "task": "",
            "hand_data": {"hand_data_enabled": False},
            "with_gripper_data": False,
        }
        self.paths.runtime_sample_info_json.write_text(json.dumps(sample_info, ensure_ascii=False, indent=2), encoding="utf-8")
        self.paths.runtime_remote_session_json.write_text(json.dumps({"plan": plan, "sample_info_path": str(self.paths.runtime_sample_info_json)}, ensure_ascii=False, indent=2), encoding="utf-8")

    def append_arm(self, index: int, snapshot: dict) -> None:
        ts = float(snapshot.get("timestamp_unix", 0.0) or 0.0)
        self._arm_timestamp_writer.writerow([index, ts])
        self._arm_obs_writer.writerow([
            index,
            ts,
            snapshot.get("runtime_state", ""),
            snapshot.get("runtime", {}).get("start_trigger", ""),
            snapshot.get("runtime", {}).get("stop_trigger", ""),
        ])
        self._arm_action_writer.writerow([
            index,
            ts,
            snapshot.get("trigger", {}).get("trigger_mode", ""),
            json.dumps(snapshot.get("bringup", {}).get("command", []), ensure_ascii=False),
        ])

    def append_camera_frame(self, camera: str, index: int, timestamp_unix: float, image_path: Path) -> None:
        writer = self._camera_frame_writers[camera]
        writer.writerow([index, f"{timestamp_unix:.9f}", f"{timestamp_unix * 1000.0:.3f}", str(image_path)])
        fp = self._camera_frame_fps.get(camera)
        if fp is not None:
            fp.flush()

    def close(self) -> None:
        for fp in [self._arm_timestamp_fp, self._arm_obs_fp, self._arm_action_fp, *self._camera_frame_fps.values()]:
            if fp is not None:
                fp.close()

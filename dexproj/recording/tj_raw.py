"""TJ-style raw episode helpers for DexProj."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TJRawEpisodePaths:
    episode_dir: Path
    runtime_dir: Path
    arm_data_dir: Path
    hand_data_dir: Path
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
    def hand_action_csv(self) -> Path:
        return self.hand_data_dir / "action.csv"

    @property
    def hand_observation_csv(self) -> Path:
        return self.hand_data_dir / "observation_state.csv"

    @property
    def hand_timestamp_csv(self) -> Path:
        return self.hand_data_dir / "timestamp.csv"

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
            hand_data_dir=episode_dir / "hand_data",
            camera_data_dir=episode_dir / "camera_data",
            camera_dirs={name: episode_dir / "camera_data" / name for name in camera_names},
        )
        self.paths.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.paths.arm_data_dir.mkdir(parents=True, exist_ok=True)
        self.paths.hand_data_dir.mkdir(parents=True, exist_ok=True)
        self.paths.camera_data_dir.mkdir(parents=True, exist_ok=True)
        for camera_dir in self.paths.camera_dirs.values():
            camera_dir.mkdir(parents=True, exist_ok=True)
            (camera_dir / "images").mkdir(parents=True, exist_ok=True)

    def start(self, plan: dict) -> None:
        sample_info = {
            "session_dir": str(self.paths.episode_dir.parent),
            "episode_index": int(self.paths.episode_dir.name.split("_")[-1]),
            "arm_data": {
                "timestamp": str(self.paths.arm_timestamp_csv),
                "observation_state": str(self.paths.arm_observation_csv),
                "action": str(self.paths.arm_action_csv),
            },
            "hand_data": {
                "hand_data_enabled": True,
                "hand_data_dir": str(self.paths.hand_data_dir),
                "timestamp": str(self.paths.hand_timestamp_csv),
                "observation_state": str(self.paths.hand_observation_csv),
                "action": str(self.paths.hand_action_csv),
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
                "left_joint_1.pos",
                "left_joint_2.pos",
                "left_joint_3.pos",
                "left_joint_4.pos",
                "left_joint_5.pos",
                "left_joint_6.pos",
                "left_joint_7.pos",
                "right_joint_1.pos",
                "right_joint_2.pos",
                "right_joint_3.pos",
                "right_joint_4.pos",
                "right_joint_5.pos",
                "right_joint_6.pos",
                "right_joint_7.pos",
            ],
            "num_frames": 0,
            "task": "",
            "with_gripper_data": False,
        }
        self.paths.runtime_sample_info_json.write_text(json.dumps(sample_info, ensure_ascii=False, indent=2), encoding="utf-8")
        self.paths.runtime_remote_session_json.write_text(json.dumps({"plan": plan, "sample_info_path": str(self.paths.runtime_sample_info_json)}, ensure_ascii=False, indent=2), encoding="utf-8")

    def close(self) -> None:
        return

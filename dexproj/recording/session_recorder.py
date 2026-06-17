"""Session recorder and artifact layout for DexProj."""

from __future__ import annotations

import json
import shutil
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass
class RecorderPaths:
    session_dir: Path
    episode_dir: Path
    meta_path: Path
    logs_dir: Path
    runtime_dir: Path


class SessionRecorder:
    def __init__(self, data_root: Path, session_mode: str, delete_on_abort: bool = True):
        self.data_root = data_root
        self.session_mode = session_mode
        self.delete_on_abort = delete_on_abort
        self.paths: RecorderPaths | None = None
        self.meta: dict = {}
        self.session_name: str | None = None
        self._started = False

    def start(self, plan: dict, start_trigger: str) -> RecorderPaths:
        now = time.localtime()
        requested_session_name = str(plan.get("session_name", "") or "").strip()
        if requested_session_name:
            if Path(requested_session_name).name != requested_session_name or requested_session_name in {".", ".."}:
                raise ValueError(
                    "session_name must be a single directory name without path separators."
                )
            self.session_name = requested_session_name
        else:
            self.session_name = f"session_{now.tm_year:04d}_{now.tm_mon:02d}_{now.tm_mday:02d}"
        session_dir = self.data_root / "raw" / self.session_name
        session_dir.mkdir(parents=True, exist_ok=True)

        episode_index = self._next_episode_index(session_dir)
        episode_name = f"episode_{episode_index:06d}"
        episode_dir = session_dir / episode_name
        episode_dir.mkdir(parents=True, exist_ok=False)

        runtime_dir = episode_dir / "_runtime"
        logs_dir = runtime_dir / "logs"
        runtime_dir.mkdir(parents=True, exist_ok=True)
        logs_dir.mkdir(parents=True, exist_ok=True)

        meta_path = episode_dir / "meta.json"
        started_unix = time.time()
        self.paths = RecorderPaths(
            session_dir=session_dir,
            episode_dir=episode_dir,
            meta_path=meta_path,
            logs_dir=logs_dir,
            runtime_dir=runtime_dir,
        )

        self.meta = {
            "mode": self.session_mode,
            "task": str(plan.get("task", "") or "").strip(),
            "session_name": self.session_name,
            "episode_name": episode_name,
            "episode_index": episode_index,
            "created_at_unix": started_unix,
            "start_trigger": start_trigger,
            "stop_trigger": None,
            "recording": {
                "data_root": str(self.data_root),
                "session_dir": str(session_dir),
                "episode_dir": str(episode_dir),
                "meta_path": str(meta_path),
                "runtime_dir": str(runtime_dir),
                "logs_dir": str(logs_dir),
            },
            "writers": {
                "logs": [],
            },
            "artifacts": {
                "logs": {},
            },
            "bringup": plan.get("bringup", {}),
            "hand_teleop": plan.get("hand_teleop", {}),
            "openvr_config": plan.get("openvr_config", ""),
            "trigger": plan.get("trigger", {}),
            "runtime": {
                "state": "running",
                "started_unix": started_unix,
                "stopped_unix": None,
            },
        }
        self._flush_meta()
        self._started = True
        return self.paths

    def register_writer(self, section: str, name: str, path: Path) -> None:
        writers = self.meta.setdefault("writers", {})
        writers.setdefault(section, []).append({"name": name, "path": str(path)})
        self._flush_meta()

    def register_log_artifact(self, name: str, path: Path) -> None:
        self._register_artifact("logs", name, path)

    def stop(self, stop_trigger: str) -> None:
        if self.paths is None:
            raise RuntimeError("SessionRecorder.stop() called before start().")

        self.meta["stop_trigger"] = stop_trigger
        runtime = self.meta.setdefault("runtime", {})
        runtime["state"] = "stopped"
        runtime["stopped_unix"] = time.time()
        self._flush_meta()

    def abort(self, reason: str) -> None:
        if self.paths is None:
            return
        if self.delete_on_abort:
            shutil.rmtree(self.paths.episode_dir, ignore_errors=True)
            self.paths = None
            self.meta = {}
            return
        self.meta["stop_trigger"] = f"abort:{reason}"
        runtime = self.meta.setdefault("runtime", {})
        runtime["state"] = "aborted"
        runtime["aborted_unix"] = time.time()
        runtime["abort_reason"] = reason
        self._flush_meta()

    def _register_artifact(self, section: str, name: str, path: Path) -> None:
        artifacts = self.meta.setdefault("artifacts", {})
        scoped = artifacts.setdefault(section, {})
        scoped[name] = str(path)
        self._flush_meta()

    def _flush_meta(self) -> None:
        if self.paths is None:
            return
        self.paths.meta_path.write_text(
            json.dumps(self.meta, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    @staticmethod
    def _next_episode_index(session_dir: Path) -> int:
        max_index = -1
        for child in session_dir.iterdir():
            if not child.is_dir():
                continue
            name = child.name
            if not name.startswith("episode_"):
                continue
            suffix = name.removeprefix("episode_")
            if suffix.isdigit():
                max_index = max(max_index, int(suffix))
        return max_index + 1

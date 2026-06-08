"""DexProj unified session runner."""

from __future__ import annotations

import argparse
import json
import os
import select
import shlex
import sys
import subprocess
import termios
import time
import tty
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from dexproj.check_devices.cli import collect_report, evaluate_report
from dexproj.hand_teleop.config import HandChannelConfig, HandTeleopConfig
from dexproj.integration.bringup import BringupConfig
from dexproj.recording.ros_writers import FloatArrayCsvPairRecorder, JointCsvPairRecorder, RosImageFrameRecorder
from dexproj.recording.session_recorder import SessionRecorder
from dexproj.recording.tj_raw import TJRawEpisodeWriter
from dexproj.session.runtime_support import (
    CombinedTrigger,
    InputsGamepadTrigger,
    KeyboardTrigger,
    ManagedProcessGroup,
    ManagedProcessSpec,
    PeriodicStatusWriter,
)

VALID_TRIGGER_MODES = {"gamepad", "keyboard", "both"}
VALID_RUNTIME_STATES = {"initialized", "ready", "running", "stopped"}
VALID_SESSION_MODES = {"single_left", "single_right", "dual"}
ARM_READY_SERVICE = "/tianji_arm/get_mode"
ARM_SWITCH_MODE_SERVICE = "/tianji_arm/switch_mode"
ARM_START_TELEOP_SERVICE = "/tianji_arm/start_teleop"
ARM_STOP_TELEOP_SERVICE = "/tianji_arm/stop_teleop"
ARM_READY_TIMEOUT_SEC = 30.0
ARM_LOG_FAILURE_MARKERS = (
    "[ERROR] [tianji_arm_controller",
    "ModuleNotFoundError:",
    "ConnectionError:",
)

DEFAULT_CAMERA_TOPIC_SPECS = {
    "head_usb": {"name": "head", "topic": "/stereo/left/compressed", "schema": "compressed_image", "reliability": "best_effort"},
    "head_realsense": {"name": "head", "topic": "/cam_head/color/image_raw", "schema": "image", "reliability": "best_effort"},
    "left_wrist": {
        "name": "left_wrist",
        "topic": "/cam_left_wrist/color/image_raw",
        "fallback_topics": ["/cam_left_wrist/color/image_rect_raw"],
        "schema": "image",
        "reliability": "best_effort",
    },
    "right_wrist": {
        "name": "right_wrist",
        "topic": "/cam_right_wrist/color/image_raw",
        "fallback_topics": ["/cam_right_wrist/color/image_rect_raw"],
        "schema": "image",
        "reliability": "best_effort",
    },
}


@dataclass
class TriggerConfig:
    trigger_mode: str = "both"
    gamepad_start: list[str] | None = None
    gamepad_stop: list[str] | None = None
    keyboard_start: str = "B"
    keyboard_stop: str = "E"

    def __post_init__(self) -> None:
        if self.trigger_mode not in VALID_TRIGGER_MODES:
            raise ValueError(f"Unsupported trigger_mode: {self.trigger_mode}")
        if self.gamepad_start is None:
            self.gamepad_start = ["lb", "rb"]
        if self.gamepad_stop is None:
            self.gamepad_stop = ["start"]


@dataclass
class SessionConfig:
    mode: str
    bringup_config: str
    hand_teleop_config: str
    openvr_config: str
    camera_config: str
    enable_camera: bool
    trigger: TriggerConfig

    @classmethod
    def from_yaml(cls, path: Path) -> "SessionConfig":
        if yaml is None:
            raise RuntimeError("PyYAML is required to load session config files.")
        with path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"Invalid session config: {path}")

        trigger_raw = raw.get("trigger", {})
        if not isinstance(trigger_raw, dict):
            trigger_raw = {}

        return cls(
            mode=str(raw.get("mode", "dual")),
            bringup_config=str(raw.get("bringup_config", "config/bringup_htc.yaml")),
            hand_teleop_config=str(raw.get("hand_teleop_config", "config/hand_teleop_wuji_glove.yaml")),
            openvr_config=str(raw.get("openvr_config", "config/htc_openvr_tracker.yaml")),
            camera_config=str(raw.get("camera_config", "config/camera_config.yaml")),
            enable_camera=bool(raw.get("enable_camera", False)),
            trigger=TriggerConfig(
                trigger_mode=str(trigger_raw.get("trigger_mode", "both")),
                gamepad_start=list(trigger_raw.get("gamepad_start", ["lb", "rb"])),
                gamepad_stop=list(trigger_raw.get("gamepad_stop", ["start"])),
                keyboard_start=str(trigger_raw.get("keyboard_start", "B")),
                keyboard_stop=str(trigger_raw.get("keyboard_stop", "E")),
            ),
        )


class SessionRuntime:
    def __init__(self, plan: dict):
        self.plan = plan
        self.state = "initialized"
        self.recorder = SessionRecorder(Path("data").resolve(), session_mode=str(plan["mode"]), delete_on_abort=True)
        self.status_writer: PeriodicStatusWriter | None = None
        self.process_group = ManagedProcessGroup()
        self.tj_raw_writer: TJRawEpisodeWriter | None = None
        self.joint_writers: list[JointCsvPairRecorder] = []
        self.camera_writers: list[RosImageFrameRecorder] = []
        self.active_episode_dir: Path | None = None
        self.camera_topic_specs = _resolve_camera_topic_specs(plan)
        self._ensure_valid_state()

    def _ensure_valid_state(self) -> None:
        if self.state not in VALID_RUNTIME_STATES:
            raise ValueError(f"Invalid runtime state: {self.state}")

    def enter_ready(self) -> None:
        self.state = "ready"
        self.plan["runtime_state"] = self.state
        self._ensure_valid_state()
        print(f"[session] state -> {self.state}")

    def enter_running(self, trigger: str) -> None:
        self.state = "running"
        self.plan["runtime_state"] = self.state
        self._ensure_valid_state()
        self.plan.setdefault("runtime", {})["start_trigger"] = trigger
        self.plan["runtime"]["started_unix"] = time.time()
        print(f"[session] state -> {self.state} (trigger={trigger})")
        print("[session] episode recording controls: S=start episode, D=stop episode")

    def start_episode(self, trigger: str) -> None:
        if self.state != "running":
            raise RuntimeError("Episode recording can only start while session is running.")
        if self.active_episode_dir is not None:
            print(f"[session] episode already recording: {self.active_episode_dir}")
            return
        paths = self.recorder.start(self.plan, trigger)
        self.plan["recording"] = {
            "session_dir": str(paths.session_dir),
            "episode_dir": str(paths.episode_dir),
            "meta_path": str(paths.meta_path),
            "runtime_dir": str(paths.runtime_dir),
            "logs_dir": str(paths.logs_dir),
        }
        self.tj_raw_writer = TJRawEpisodeWriter(paths.episode_dir, camera_names=[spec["name"] for spec in self.camera_topic_specs])
        self.tj_raw_writer.start(self.plan)
        self._attach_writers(paths.episode_dir)
        self.status_writer = PeriodicStatusWriter(Path(self.plan["recording"]["runtime_dir"]) / "runtime_status.ndjson")
        self.status_writer.start(self._build_status_snapshot)
        self.active_episode_dir = paths.episode_dir
        print(f"[session] episode dir: {paths.episode_dir}")

    def stop_episode(self, trigger: str) -> None:
        if self.active_episode_dir is None:
            print("[session] no active episode recording")
            return
        if self.status_writer is not None:
            self.status_writer.stop()
            self.status_writer = None
        self._write_camera_diagnostics(self.active_episode_dir)
        for recorder in self.joint_writers:
            recorder.stop()
        self.joint_writers = []
        for recorder in self.camera_writers:
            recorder.stop()
        self.camera_writers = []
        if self.tj_raw_writer is not None:
            self.tj_raw_writer.close()
            self.tj_raw_writer = None
        self.recorder.stop(trigger)
        print(f"[session] episode stopped: {self.active_episode_dir}")
        self.active_episode_dir = None

    def enter_stopped(self, trigger: str) -> None:
        self.state = "stopped"
        self.plan["runtime_state"] = self.state
        self._ensure_valid_state()
        self.plan.setdefault("runtime", {})["stop_trigger"] = trigger
        self.plan["runtime"]["stopped_unix"] = time.time()
        if self.active_episode_dir is not None:
            self.stop_episode(trigger)
        self.process_group.stop_all()
        print(f"[session] state -> {self.state} (trigger={trigger})")

    def abort(self, reason: str) -> None:
        try:
            if self.status_writer is not None:
                self.status_writer.stop()
        except BaseException:
            pass
        try:
            self.process_group.stop_all()
        except BaseException:
            pass
        try:
            self._fallback_disable_tianji()
        except BaseException:
            pass
        for recorder in self.joint_writers:
            with suppress(BaseException):
                recorder.stop()
        for recorder in self.camera_writers:
            with suppress(BaseException):
                recorder.stop()
        with suppress(BaseException):
            if self.tj_raw_writer is not None:
                self.tj_raw_writer.close()
        if self.active_episode_dir is not None:
            with suppress(BaseException):
                self.recorder.abort(reason)
        self.state = "stopped"
        self.plan["runtime_state"] = self.state
        self.plan.setdefault("runtime", {})["abort_reason"] = reason
        print(f"[session] aborted: {reason}")

    def _fallback_disable_tianji(self) -> None:
        if not self.plan.get("bringup", {}).get("enable_arm", False):
            return
        script = Path("scripts/tianji_disable_arms.sh").resolve()
        if not script.exists():
            print(f"[session] Tianji fallback disable script missing: {script}")
            return
        try:
            completed = subprocess.run(
                [str(script)],
                cwd=Path(".").resolve(),
                check=False,
                timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            print("[session] Tianji fallback disable timed out")
            return
        if completed.returncode != 0:
            print(f"[session] Tianji fallback disable exited {completed.returncode}")

    def _attach_writers(self, episode_dir: Path) -> None:
        runtime_dir = episode_dir / "_runtime"
        logs_dir = runtime_dir / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)

        for recorder in self._build_joint_recorders(episode_dir):
            recorder.start()
            self.joint_writers.append(recorder)
            self.recorder.register_writer("data", f"{recorder.name}_timestamp", recorder.timestamp_path)
            self.recorder.register_writer("data", f"{recorder.name}_observation_state", recorder.observation_path)
            self.recorder.register_writer("data", f"{recorder.name}_action", recorder.action_path)

        camera_root = episode_dir / "camera_data"
        for spec in self.camera_topic_specs:
            camera_dir = camera_root / spec["name"]
            recorder = RosImageFrameRecorder(
                name=spec["name"],
                topic=spec["topic"],
                fallback_topics=list(spec.get("fallback_topics", [])),
                image_dir=camera_dir / "images",
                frames_csv_path=camera_dir / "frames.csv",
                schema=spec["schema"],
                reliability=str(spec.get("reliability", "best_effort")),
            )
            recorder.start()
            self.camera_writers.append(recorder)
            self.recorder.register_writer("logs", f"{spec['name']}_frames", camera_dir / "frames.csv")

        sample_info_path = episode_dir / "_runtime" / "sample_info.json"
        self.recorder.register_log_artifact("sample_info", sample_info_path)

    def _build_joint_recorders(self, episode_dir: Path) -> list[JointCsvPairRecorder]:
        hand_input = str(self.plan.get("bringup", {}).get("hand_input", "none"))
        if hand_input == "manus":
            hand_state_topics = {
                "left": "/left_hand/joint_states",
                "right": "/right_hand/joint_states",
            }
            hand_action_topics = {
                "left": "/wuji_hand/left/joint_command",
                "right": "/wuji_hand/right/joint_command",
            }
        else:
            hand_state_topics = {
                "left": "/wuji_hand/left/joint_state",
                "right": "/wuji_hand/right/joint_state",
            }
            hand_action_topics = {
                "left": "/wuji_hand/left/joint_command",
                "right": "/wuji_hand/right/joint_command",
            }

        return [
            JointCsvPairRecorder(
                name="arm",
                state_topics={
                    "left": "/tianji_arm/left/joint_state",
                    "right": "/tianji_arm/right/joint_state",
                },
                action_topics={
                    "left": "/tianji_arm/left/joint_command",
                    "right": "/tianji_arm/right/joint_command",
                },
                timestamp_path=episode_dir / "arm_data" / "timestamp.csv",
                observation_path=episode_dir / "arm_data" / "observation_state.csv",
                action_path=episode_dir / "arm_data" / "action.csv",
                columns_by_side={
                    "left": [f"left_joint_{index}.pos" for index in range(1, 8)],
                    "right": [f"right_joint_{index}.pos" for index in range(1, 8)],
                },
                freeze_missing_sides=True,
            ),
            FloatArrayCsvPairRecorder(
                name="arm_ee_pose",
                state_topics={
                    "left": "/tianji_arm/left/left_ee_pose",
                    "right": "/tianji_arm/right/right_ee_pose",
                },
                action_topics={
                    "left": "/tianji_arm/left/left_ee_pose",
                    "right": "/tianji_arm/right/right_ee_pose",
                },
                timestamp_path=episode_dir / "arm_data" / "ee_pose_timestamp.csv",
                observation_path=episode_dir / "arm_data" / "ee_pose_observation_state.csv",
                action_path=episode_dir / "arm_data" / "ee_pose_action.csv",
                columns_by_side={
                    "left": [f"left_ee_pose_{name}" for name in ("x", "y", "z", "a", "b", "c")],
                    "right": [f"right_ee_pose_{name}" for name in ("x", "y", "z", "a", "b", "c")],
                },
                freeze_missing_sides=True,
            ),
            JointCsvPairRecorder(
                name="hand",
                state_topics=hand_state_topics,
                action_topics=hand_action_topics,
                timestamp_path=episode_dir / "hand_data" / "timestamp.csv",
                observation_path=episode_dir / "hand_data" / "observation_state.csv",
                action_path=episode_dir / "hand_data" / "action.csv",
                columns_by_side={
                    "left": [f"left_finger{finger}_joint{joint}" for finger in range(1, 6) for joint in range(1, 5)],
                    "right": [f"right_finger{finger}_joint{joint}" for finger in range(1, 6) for joint in range(1, 5)],
                },
                freeze_missing_sides=True,
            ),
        ]

    def _build_status_snapshot(self) -> dict:
        return {
            "runtime_state": self.plan.get("runtime_state"),
            "runtime": self.plan.get("runtime", {}),
            "recording": self.plan.get("recording", {}),
            "bringup": self.plan.get("bringup", {}),
            "trigger": self.plan.get("trigger", {}),
            "processes": self.process_group.snapshot(),
            "cameras": [recorder.snapshot() for recorder in self.camera_writers],
        }

    def _write_camera_diagnostics(self, episode_dir: Path) -> None:
        diagnostics = {
            "generated_at_unix": time.time(),
            "processes": self.process_group.snapshot(),
            "cameras": [recorder.snapshot() for recorder in self.camera_writers],
        }
        output_path = episode_dir / "_runtime" / "camera_recording_diagnostics.json"
        output_path.write_text(json.dumps(diagnostics, ensure_ascii=False, indent=2), encoding="utf-8")

        missing = [
            camera["name"]
            for camera in diagnostics["cameras"]
            if int(camera.get("frame_count", 0) or 0) <= 0
        ]
        if missing:
            print(
                "[session] warning: no camera frames recorded for "
                + ", ".join(str(name) for name in missing)
                + f" (see {output_path})"
            )


def _load_camera_config_dict(camera_config_path: str) -> dict:
    if yaml is None:
        return {}
    path = Path(camera_config_path).expanduser().resolve()
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return raw if isinstance(raw, dict) else {}


def _resolve_camera_topic_specs(plan: dict) -> list[dict]:
    camera_plan = plan.get("camera", {})
    camera_config_path = str(camera_plan.get("config", "config/camera_config.yaml"))
    config = _load_camera_config_dict(camera_config_path)
    cameras = config.get("cameras", {}) if isinstance(config.get("cameras", {}), dict) else {}

    specs: list[dict] = []
    for key in ("head", "left_wrist", "right_wrist"):
        cam_cfg = cameras.get(key, {})
        if not isinstance(cam_cfg, dict) or not bool(cam_cfg.get("enabled", False)):
            continue
        cam_type = str(cam_cfg.get("type", "") or "").strip().lower()
        if key == "head":
            spec_key = "head_realsense" if cam_type in {"d435i", "d405"} else "head_usb"
            spec = DEFAULT_CAMERA_TOPIC_SPECS[spec_key]
            device_path = str(cam_cfg.get("video_device", "") or "").strip()
            if spec_key == "head_usb" and device_path and not Path(device_path).exists():
                print(f"[session] warning: head camera device missing, skipping recording: {device_path}")
                continue
        else:
            spec = DEFAULT_CAMERA_TOPIC_SPECS[key]
        specs.append(dict(spec))
    return specs


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="DexProj unified session runner.")
    parser.add_argument("--config", default="config/session_htc_wuji_glove.yaml", help="Session config file.")
    parser.add_argument("--task", default="", help="Task name written into each recorded episode meta.json.")
    parser.add_argument("--dry-run", action="store_true", help="Print resolved commands without executing.")
    parser.add_argument("--skip-preflight", action="store_true", help="Skip device preflight checks.")
    return parser


def _build_trigger(trigger_cfg: TriggerConfig):
    if trigger_cfg.trigger_mode == "keyboard":
        return KeyboardTrigger(trigger_cfg.keyboard_start, trigger_cfg.keyboard_stop)
    if trigger_cfg.trigger_mode == "gamepad":
        return InputsGamepadTrigger(trigger_cfg.gamepad_start, trigger_cfg.gamepad_stop)
    return CombinedTrigger(
        InputsGamepadTrigger(trigger_cfg.gamepad_start, trigger_cfg.gamepad_stop),
        KeyboardTrigger(trigger_cfg.keyboard_start, trigger_cfg.keyboard_stop),
    )


def _load_plan(
    config: SessionConfig,
    bringup_command: list[str],
    hand_cfg: HandTeleopConfig,
    bringup_cfg: BringupConfig,
    task: str = "",
) -> dict:
    if config.mode not in VALID_SESSION_MODES:
        raise ValueError(f"Unsupported session mode: {config.mode}")

    def _serialize_channel(channel: HandChannelConfig | None) -> dict | None:
        if channel is None:
            return None
        return {
            "side": channel.side,
            "glove_sn": channel.glove_sn,
            "hand_sn": channel.hand_sn,
            "glove_device_name": channel.glove_device_name,
            "retarget_config": channel.retarget_config,
        }

    return {
        "mode": config.mode,
        "task": str(task).strip(),
        "bringup": {
            "config": config.bringup_config,
            "command": bringup_command,
            "hand_input": bringup_cfg.hand_input,
            "arm_input": bringup_cfg.arm_input,
            "enable_arm": bringup_cfg.enable_arm,
            "arm_controller_config": bringup_cfg.arm_controller_config,
            "arm_dry_run": bringup_cfg.arm_dry_run,
            "arm_read_only": bringup_cfg.arm_read_only,
            "arm_feedback_handshake": bringup_cfg.arm_feedback_handshake,
            "arm_sdk_executor_enable": bringup_cfg.arm_sdk_executor_enable,
            "arm_sim_viz": bringup_cfg.arm_sim_viz,
        },
        "hand_teleop": {
            "config": config.hand_teleop_config,
            "mode": hand_cfg.mode,
            "left": _serialize_channel(hand_cfg.left),
            "right": _serialize_channel(hand_cfg.right),
        },
        "openvr_config": config.openvr_config,
        "camera": {
            "enabled": bool(config.enable_camera),
            "config": config.camera_config,
        },
        "trigger": {
            "trigger_mode": config.trigger.trigger_mode,
            "gamepad_start": list(config.trigger.gamepad_start or []),
            "gamepad_stop": list(config.trigger.gamepad_stop or []),
            "keyboard_start": config.trigger.keyboard_start,
            "keyboard_stop": config.trigger.keyboard_stop,
        },
        "runtime_state": "initialized",
        "runtime": {},
    }


def _build_camera_process(plan: dict) -> ManagedProcessSpec:
    camera_plan = plan.get("camera", {})
    if not camera_plan.get("enabled", False):
        raise ValueError("Camera process requested without enable_camera=true")
    camera_config = str(camera_plan.get("config", "config/camera_config.yaml"))
    camera_cfg = _load_camera_config_dict(camera_config)
    cameras = camera_cfg.get("cameras", {}) if isinstance(camera_cfg.get("cameras", {}), dict) else {}
    head_cfg = cameras.get("head", {}) if isinstance(cameras.get("head", {}), dict) else {}
    head_type = str(head_cfg.get("type", "usb") or "usb").strip().lower()
    enable_head = bool(head_cfg.get("enabled", False))
    head_device = str(head_cfg.get("video_device", "") or "").strip()
    if head_type == "usb" and enable_head and head_device and not Path(head_device).exists():
        print(f"[session] warning: head camera device missing, launching camera stack without head: {head_device}")
        enable_head = False
    return ManagedProcessSpec(
        name="camera",
        command=[
            "ros2",
            "launch",
            "camera",
            "camera_launch.py",
            f"config_file:={camera_config}",
            f"enable_head:={'true' if enable_head else 'false'}",
            "enable_pico:=false",
        ],
        cwd=Path(".").resolve(),
        stdout_path=Path("data").resolve() / "logs" / "camera.stdout.log",
        stderr_path=Path("data").resolve() / "logs" / "camera.stderr.log",
    )


def _build_hand_processes(plan: dict) -> list[ManagedProcessSpec]:
    hand_plan = plan.get("hand_teleop", {})
    mode = str(hand_plan.get("mode", "dual"))
    bringup_cfg_path = str(plan.get("bringup", {}).get("config", "config/bringup_htc.yaml"))
    hand_cfg_path = str(hand_plan.get("config", "config/hand_teleop_wuji_glove.yaml"))
    command = [
        str(Path("scripts/bringup_teleop.sh").resolve()),
        "--config",
        str(Path(bringup_cfg_path).expanduser().resolve()),
        "--hand-teleop-config",
        str(Path(hand_cfg_path).expanduser().resolve()),
        "--hand-only",
        "--skip-preflight",
        "--mode",
        mode,
    ]

    return [
        ManagedProcessSpec(
            name="hand_bringup",
            command=command,
            cwd=Path(".").resolve(),
            stdout_path=Path("data").resolve() / "logs" / "hand_bringup.stdout.log",
            stderr_path=Path("data").resolve() / "logs" / "hand_bringup.stderr.log",
        )
    ]


def _start_processes(plan: dict, dry_run: bool) -> ManagedProcessGroup:
    group = ManagedProcessGroup(_build_process_specs(plan), dry_run=dry_run)
    group.start_all()
    return group


def _wait_for_arm_ready(group: ManagedProcessGroup, timeout_sec: float = ARM_READY_TIMEOUT_SEC) -> None:
    """Wait until the Tianji node has finished startup and exposed its services."""
    deadline = time.monotonic() + timeout_sec
    warned_failures: set[str] = set()
    bringup_stdout_pos = 0
    print(f"[session] waiting for Tianji arm ready service: {ARM_READY_SERVICE}")
    while time.monotonic() < deadline:
        bringup_stdout_path = None
        for handle in group.handles:
            if handle.spec.name == "bringup":
                bringup_stdout_path = handle.spec.stdout_path
            code = handle.process.poll()
            if code is None or code == 0:
                continue
            failure = f"{handle.spec.name} exited with code {code}"
            if handle.spec.name == "bringup":
                raise RuntimeError(failure)
            if failure not in warned_failures:
                warned_failures.add(failure)
                print(f"[session] warning: {failure}; continuing to wait for Tianji arm")

        if bringup_stdout_path is not None:
            try:
                with bringup_stdout_path.open("r", encoding="utf-8", errors="replace") as log:
                    log.seek(bringup_stdout_pos)
                    chunk = log.read()
                    bringup_stdout_pos = log.tell()
            except OSError:
                chunk = ""
            if any(marker in chunk for marker in ARM_LOG_FAILURE_MARKERS):
                raise RuntimeError(
                    "tianji_arm_controller failed during bringup: "
                    f"{_summarize_tianji_log_failure(chunk)}"
                )

        try:
            completed = subprocess.run(
                ["ros2", "service", "list"],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            completed = None

        if completed is not None and completed.returncode == 0:
            services = {line.strip() for line in completed.stdout.splitlines()}
            if ARM_READY_SERVICE in services:
                print("[session] Tianji arm controller ready")
                return

        time.sleep(0.5)

    raise TimeoutError(
        f"Tianji arm controller did not expose {ARM_READY_SERVICE} "
        f"within {timeout_sec:.0f}s"
    )


def _set_arm_teleop_enabled(enabled: bool) -> None:
    # tianji_arm_node uses SetBool(True)=INFERENCE, SetBool(False)=TELEOP.
    data = "false" if enabled else "true"
    mode = "TELEOP" if enabled else "INFERENCE"
    completed = subprocess.run(
        [
            "ros2",
            "service",
            "call",
            ARM_SWITCH_MODE_SERVICE,
            "std_srvs/srv/SetBool",
            f"{{data: {data}}}",
        ],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=5.0,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(f"failed to switch Tianji arm to {mode}: {detail}")
    print(f"[session] Tianji arm mode -> {mode}")


def _list_ros_services() -> set[str]:
    completed = subprocess.run(
        ["ros2", "service", "list"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=2.0,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(f"failed to list ros2 services: {detail}")
    return {line.strip() for line in completed.stdout.splitlines() if line.strip()}


def _call_trigger_service(service_name: str) -> None:
    completed = subprocess.run(
        ["ros2", "service", "call", service_name, "std_srvs/srv/Trigger", "{}"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=5.0,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(f"failed to call {service_name}: {detail}")
    print(f"[session] called {service_name}")


def _arm_keyboard_gate_service_if_present(enabled: bool) -> None:
    service_name = ARM_START_TELEOP_SERVICE if enabled else ARM_STOP_TELEOP_SERVICE
    try:
        services = _list_ros_services()
    except RuntimeError as exc:
        print(f"[session] warning: {exc}")
        return
    if service_name not in services:
        return
    _call_trigger_service(service_name)


def _wait_for_runtime_control_events(runtime: SessionRuntime, trigger_cfg: TriggerConfig) -> str:
    stop_event = None
    if trigger_cfg.trigger_mode in {"gamepad", "both"}:
        stop_event = _start_gamepad_stop_waiter(trigger_cfg)
        if stop_event.get("done") is None:
            stop_event = None

    keyboard_enabled = trigger_cfg.trigger_mode in {"keyboard", "both"}
    if not keyboard_enabled:
        if stop_event is None:
            raise RuntimeError("No runtime stop trigger is available.")
        while True:
            if stop_event["done"].wait(0.1):
                return str(stop_event.get("trigger", "gamepad"))

    if not sys.stdin.isatty():
        raise RuntimeError("Keyboard episode controls require an interactive TTY.")

    print("Press S to start episode, D to stop episode, E to stop session...", flush=True)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            if stop_event is not None and stop_event["done"].is_set():
                return str(stop_event.get("trigger", "gamepad"))
            ready, _, _ = select.select([fd], [], [], 0.1)
            if not ready:
                continue
            value = os.read(fd, 1).decode("utf-8", errors="ignore")
            if not value:
                continue
            key = value.upper()
            if key == "S":
                runtime.start_episode("S")
            elif key == "D":
                runtime.stop_episode("D")
            elif key == str(trigger_cfg.keyboard_stop).strip().upper():
                print(f"[trigger] keyboard -> {key}")
                return key
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _start_gamepad_stop_waiter(trigger_cfg: TriggerConfig) -> dict:
    try:
        gamepad = InputsGamepadTrigger(list(trigger_cfg.gamepad_start or []), list(trigger_cfg.gamepad_stop or []))
    except Exception as exc:
        print(f"[session] warning: gamepad stop trigger unavailable: {exc}")
        return {"done": None, "trigger": None}

    state = {"done": None, "trigger": None}
    from threading import Event, Thread

    done = Event()
    state["done"] = done

    def wait_stop() -> None:
        try:
            state["trigger"] = gamepad.wait_for_stop()
            done.set()
        except Exception as exc:
            print(f"[session] warning: gamepad stop trigger unavailable: {exc}")

    Thread(target=wait_stop, daemon=True).start()
    return state


def _summarize_tianji_log_failure(chunk: str) -> str:
    lines = [line.strip() for line in chunk.splitlines() if "tianji_arm_controller" in line or "Error:" in line]
    if not lines:
        return "see data/logs/bringup.stdout.log"
    return " | ".join(lines[-4:])


def _build_process_specs(plan: dict) -> list[ManagedProcessSpec]:
    logs_dir = Path("data").resolve() / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    specs = [
        ManagedProcessSpec(
            name="bringup",
            command=plan["bringup"]["command"],
            cwd=Path(".").resolve(),
            stdout_path=logs_dir / "bringup.stdout.log",
            stderr_path=logs_dir / "bringup.stderr.log",
        )
    ]
    if plan.get("camera", {}).get("enabled", False):
        specs.append(_build_camera_process(plan))
    if str(plan.get("bringup", {}).get("hand_input", "none")) == "none":
        specs.extend(_build_hand_processes(plan))
    return specs


def _print_dry_run(plan: dict) -> None:
    for spec in _build_process_specs(plan):
        command = " ".join(shlex.quote(token) for token in spec.command)
        print(f"[dry-run] {spec.name}: {command}")


def _resolve_configs(config_path: Path) -> tuple[SessionConfig, BringupConfig, HandTeleopConfig, list[str]]:
    session_cfg = SessionConfig.from_yaml(config_path)
    bringup_cfg = BringupConfig.from_yaml(Path(session_cfg.bringup_config).expanduser().resolve())
    hand_cfg = HandTeleopConfig.from_yaml(Path(session_cfg.hand_teleop_config).expanduser().resolve())
    bringup_cfg.hand_input = "none"
    bringup_command = [
        str(Path("scripts/bringup_teleop.sh").resolve()),
        "--config",
        str(Path(session_cfg.bringup_config).expanduser().resolve()),
        "--hand-teleop-config",
        str(Path(session_cfg.hand_teleop_config).expanduser().resolve()),
        "--hand-input",
        "none",
        "--openvr-config",
        str(Path(session_cfg.openvr_config).expanduser().resolve()),
        "--skip-preflight",
    ]
    return session_cfg, bringup_cfg, hand_cfg, bringup_command


def main() -> int:
    args = build_arg_parser().parse_args()
    config_path = Path(args.config).expanduser().resolve()
    session_cfg, bringup_cfg, hand_cfg, bringup_command = _resolve_configs(config_path)

    if not args.skip_preflight:
        preflight_report = collect_report(Path(session_cfg.openvr_config).expanduser().resolve())
        ok, issues = evaluate_report(preflight_report)
        if not ok:
            for issue in issues:
                print(f"[preflight] {issue}")
            print("[preflight] Use `python3 -m dexproj.check_devices` for details.")
            return 2

    plan = _load_plan(session_cfg, bringup_command, hand_cfg, bringup_cfg, task=args.task)
    if args.dry_run:
        _print_dry_run(plan)
        return 0

    runtime = SessionRuntime(plan)
    try:
        runtime.enter_ready()
        trigger = _build_trigger(session_cfg.trigger)
        start_trigger = trigger.wait_for_start()
        runtime.process_group = _start_processes(plan, dry_run=args.dry_run)
        if plan.get("bringup", {}).get("enable_arm", False):
            _wait_for_arm_ready(runtime.process_group)
        runtime.enter_running(start_trigger)
        if plan.get("bringup", {}).get("enable_arm", False):
            _set_arm_teleop_enabled(True)
            _arm_keyboard_gate_service_if_present(True)
        stop_trigger = _wait_for_runtime_control_events(runtime, session_cfg.trigger)
        if plan.get("bringup", {}).get("enable_arm", False):
            _arm_keyboard_gate_service_if_present(False)
        runtime.enter_stopped(stop_trigger)
        return 0
    except KeyboardInterrupt:
        runtime.abort("keyboard_interrupt")
        return 130
    except Exception as exc:
        runtime.abort(f"exception:{exc.__class__.__name__}")
        raise


if __name__ == "__main__":
    raise SystemExit(main())

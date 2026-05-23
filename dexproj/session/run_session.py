"""DexProj unified session runner."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from dexproj.check_devices.cli import collect_report, evaluate_report
from dexproj.hand_teleop.config import HandChannelConfig, HandTeleopConfig
from dexproj.integration.bringup import BringupConfig, resolve_launch_command
from dexproj.recording.ros_writers import RosImageFrameRecorder, RosTopicRecorder
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

ARM_TOPIC_SPECS = [
    {"name": "tianji_left_joint_state", "topic": "/tianji_arm/left/joint_state", "schema": "float_array"},
    {"name": "tianji_right_joint_state", "topic": "/tianji_arm/right/joint_state", "schema": "float_array"},
    {"name": "wujihand_left_joint_state", "topic": "/wuji_hand/left/joint_state", "schema": "float_array"},
    {"name": "wujihand_right_joint_state", "topic": "/wuji_hand/right/joint_state", "schema": "float_array"},
]

CAMERA_TOPIC_SPECS = [
    {"name": "head", "topic": "/cam_head/color/image_raw", "schema": "image"},
    {"name": "left_wrist", "topic": "/cam_left_wrist/color/image_rect_raw", "schema": "image"},
    {"name": "right_wrist", "topic": "/cam_right_wrist/color/image_rect_raw", "schema": "image"},
]


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
        self.arm_topic_writers: list[RosTopicRecorder] = []
        self.camera_writers: list[RosImageFrameRecorder] = []
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
        paths = self.recorder.start(self.plan, trigger)
        self.plan["recording"] = {
            "session_dir": str(paths.session_dir),
            "episode_dir": str(paths.episode_dir),
            "meta_path": str(paths.meta_path),
            "runtime_dir": str(paths.runtime_dir),
            "logs_dir": str(paths.logs_dir),
        }
        self.tj_raw_writer = TJRawEpisodeWriter(paths.episode_dir, camera_names=[spec["name"] for spec in CAMERA_TOPIC_SPECS])
        self.tj_raw_writer.start(self.plan)
        self._attach_writers(paths.episode_dir)
        self.status_writer = PeriodicStatusWriter(Path(self.plan["recording"]["runtime_dir"]) / "runtime_status.ndjson")
        self.status_writer.start(self._build_status_snapshot)
        print(f"[session] state -> {self.state} (trigger={trigger})")
        print(f"[session] episode dir: {paths.episode_dir}")

    def enter_stopped(self, trigger: str) -> None:
        self.state = "stopped"
        self.plan["runtime_state"] = self.state
        self._ensure_valid_state()
        self.plan.setdefault("runtime", {})["stop_trigger"] = trigger
        self.plan["runtime"]["stopped_unix"] = time.time()
        if self.status_writer is not None:
            self.status_writer.stop()
        self.process_group.stop_all()
        for recorder in self.arm_topic_writers:
            recorder.stop()
        for recorder in self.camera_writers:
            recorder.stop()
        if self.tj_raw_writer is not None:
            self.tj_raw_writer.append_arm(0, self._build_status_snapshot())
            self._append_camera_index_markers()
            self.tj_raw_writer.close()
        self.recorder.stop(trigger)
        print(f"[session] state -> {self.state} (trigger={trigger})")

    def abort(self, reason: str) -> None:
        with suppress(Exception):
            if self.status_writer is not None:
                self.status_writer.stop()
        with suppress(Exception):
            self.process_group.stop_all()
        for recorder in self.arm_topic_writers:
            with suppress(Exception):
                recorder.stop()
        for recorder in self.camera_writers:
            with suppress(Exception):
                recorder.stop()
        with suppress(Exception):
            if self.tj_raw_writer is not None:
                self.tj_raw_writer.close()
        with suppress(Exception):
            self.recorder.abort(reason)
        self.state = "stopped"
        self.plan["runtime_state"] = self.state
        self.plan.setdefault("runtime", {})["abort_reason"] = reason
        print(f"[session] aborted: {reason}")

    def _attach_writers(self, episode_dir: Path) -> None:
        runtime_dir = episode_dir / "_runtime"
        logs_dir = runtime_dir / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)

        for spec in ARM_TOPIC_SPECS:
            path = logs_dir / f"{spec['name']}.ndjson"
            recorder = RosTopicRecorder(
                name=spec["name"],
                topic=spec["topic"],
                path=path,
                schema=spec["schema"],
            )
            recorder.start()
            self.arm_topic_writers.append(recorder)
            self.recorder.register_writer("logs", spec["name"], path)

        camera_root = episode_dir / "camera_data"
        for spec in CAMERA_TOPIC_SPECS:
            camera_dir = camera_root / spec["name"]
            recorder = RosImageFrameRecorder(
                name=spec["name"],
                topic=spec["topic"],
                image_dir=camera_dir / "images",
                frames_csv_path=camera_dir / "frames.csv",
                schema=spec["schema"],
            )
            recorder.start()
            self.camera_writers.append(recorder)
            self.recorder.register_writer("logs", f"{spec['name']}_frames", camera_dir / "frames.csv")

        sample_info_path = episode_dir / "_runtime" / "sample_info.json"
        self.recorder.register_log_artifact("sample_info", sample_info_path)

    def _append_camera_index_markers(self) -> None:
        if self.tj_raw_writer is None:
            return
        now = time.time()
        for camera_name in [spec["name"] for spec in CAMERA_TOPIC_SPECS]:
            self.tj_raw_writer.append_camera_frame(camera_name, 0, now, self._camera_csv_path(camera_name))

    def _camera_csv_path(self, camera_name: str) -> Path:
        assert self.recorder.paths is not None
        return self.recorder.paths.episode_dir / "camera_data" / camera_name / "frames.csv"

    def _build_status_snapshot(self) -> dict:
        return {
            "runtime_state": self.plan.get("runtime_state"),
            "runtime": self.plan.get("runtime", {}),
            "recording": self.plan.get("recording", {}),
            "bringup": self.plan.get("bringup", {}),
            "trigger": self.plan.get("trigger", {}),
        }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="DexProj unified session runner.")
    parser.add_argument("--config", default="config/session_htc_wuji_glove.yaml", help="Session config file.")
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


def _load_plan(config: SessionConfig, bringup_command: list[str], hand_cfg: HandTeleopConfig) -> dict:
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
        "bringup": {
            "config": config.bringup_config,
            "command": bringup_command,
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
    return ManagedProcessSpec(
        name="camera",
        command=[
            "ros2",
            "launch",
            "camera",
            "camera_launch.py",
            f"config_file:={camera_config}",
            "enable_head:=true",
            "enable_pico:=false",
        ],
        cwd=Path(".").resolve(),
        stdout_path=Path("data").resolve() / "logs" / "camera.stdout.log",
        stderr_path=Path("data").resolve() / "logs" / "camera.stderr.log",
    )


def _build_hand_processes(plan: dict) -> list[ManagedProcessSpec]:
    specs: list[ManagedProcessSpec] = []
    hand_plan = plan.get("hand_teleop", {})
    mode = str(hand_plan.get("mode", "dual"))
    base_cmd = ["python3", "wuji-retargeting/example/teleop_real.py", "--input", "wuji_glove"]
    if mode in {"single_left", "single_right"}:
        channel = hand_plan.get("left" if mode == "single_left" else "right")
        if not channel:
            raise ValueError(f"Missing hand config for mode {mode}")
        side = channel["side"]
        cmd = base_cmd + ["--hand", side, "--glove-sn", channel["glove_sn"]]
        specs.append(
            ManagedProcessSpec(
                name=f"hand_{side}",
                command=cmd,
                cwd=Path(".").resolve(),
                stdout_path=Path("data").resolve() / "logs" / f"hand_{side}.stdout.log",
                stderr_path=Path("data").resolve() / "logs" / f"hand_{side}.stderr.log",
            )
        )
        return specs

    for side in ("left", "right"):
        channel = hand_plan.get(side)
        if not channel:
            raise ValueError(f"Missing {side} hand config for dual mode")
        cmd = base_cmd + ["--hand", side, "--glove-sn", channel["glove_sn"]]
        specs.append(
            ManagedProcessSpec(
                name=f"hand_{side}",
                command=cmd,
                cwd=Path(".").resolve(),
                stdout_path=Path("data").resolve() / "logs" / f"hand_{side}.stdout.log",
                stderr_path=Path("data").resolve() / "logs" / f"hand_{side}.stderr.log",
            )
        )
    return specs


def _start_processes(plan: dict, dry_run: bool) -> ManagedProcessGroup:
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
    specs.extend(_build_hand_processes(plan))
    group = ManagedProcessGroup(specs, dry_run=dry_run)
    group.start_all()
    return group


def _resolve_configs(config_path: Path) -> tuple[SessionConfig, BringupConfig, HandTeleopConfig, list[str]]:
    session_cfg = SessionConfig.from_yaml(config_path)
    bringup_cfg = BringupConfig.from_yaml(Path(session_cfg.bringup_config).expanduser().resolve())
    hand_cfg = HandTeleopConfig.from_yaml(Path(session_cfg.hand_teleop_config).expanduser().resolve())
    bringup_command = resolve_launch_command(bringup_cfg)
    return session_cfg, bringup_cfg, hand_cfg, bringup_command


def main() -> int:
    args = build_arg_parser().parse_args()
    config_path = Path(args.config).expanduser().resolve()
    session_cfg, _, hand_cfg, bringup_command = _resolve_configs(config_path)

    if not args.skip_preflight:
        preflight_report = collect_report(Path(session_cfg.openvr_config).expanduser().resolve())
        ok, issues = evaluate_report(preflight_report)
        if not ok:
            for issue in issues:
                print(f"[preflight] {issue}")
            print("[preflight] Use `python3 -m dexproj.check_devices` for details.")
            return 2

    plan = _load_plan(session_cfg, bringup_command, hand_cfg)
    runtime = SessionRuntime(plan)
    try:
        runtime.enter_ready()
        trigger = _build_trigger(session_cfg.trigger)
        start_trigger = trigger.wait_for_start()
        runtime.process_group = _start_processes(plan, dry_run=args.dry_run)
        runtime.enter_running(start_trigger)
        stop_trigger = trigger.wait_for_stop()
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

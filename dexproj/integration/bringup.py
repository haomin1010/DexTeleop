"""DexProj HTC bringup command builder.

This module provides a stable local entry point that maps DexProj modes onto
the current `wuji-hand-teleop` launch files. The first version focuses on:

- HTC/OpenVR as the only arm-input route
- single_left / single_right / dual modes
- optional camera bringup
- dry-run command generation
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

from dexproj.check_devices.cli import collect_report, evaluate_report

VALID_MODES = {"single_left", "single_right", "dual"}


@dataclass
class BringupConfig:
    mode: str = "dual"
    enable_camera: bool = False
    enable_rviz: bool = False
    hand_input: str = "manus"
    arm_input: str = "tracker"

    @classmethod
    def from_yaml(cls, path: Path) -> "BringupConfig":
        if yaml is None:
            raise RuntimeError("PyYAML is required to load bringup config files.")
        with path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"Invalid bringup config: {path}")
        return cls(
            mode=str(raw.get("mode", "dual")),
            enable_camera=bool(raw.get("enable_camera", False)),
            enable_rviz=bool(raw.get("enable_rviz", False)),
            hand_input=str(raw.get("hand_input", "manus")),
            arm_input=str(raw.get("arm_input", "tracker")),
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="DexProj HTC teleop bringup wrapper.")
    parser.add_argument(
        "--config",
        default="config/bringup_htc.yaml",
        help="DexProj bringup config file.",
    )
    parser.add_argument(
        "--mode",
        choices=sorted(VALID_MODES),
        default=None,
        help="Override operating mode from config.",
    )
    parser.add_argument(
        "--camera",
        action="store_true",
        help="Enable camera launch even if config disables it.",
    )
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="Enable RViz even if config disables it.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved ros2 launch command without executing it.",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Skip DexProj device preflight checks before launch.",
    )
    return parser


def resolve_launch_command(config: BringupConfig) -> list[str]:
    if config.arm_input != "tracker":
        raise ValueError(
            f"DexProj only supports HTC/OpenVR arm input for now, got {config.arm_input!r}."
        )
    if config.mode not in VALID_MODES:
        raise ValueError(f"Unsupported mode: {config.mode}")

    launch_file: str
    extra_args: list[str] = []

    if config.mode == "dual":
        launch_file = "wuji_teleop_camera.launch.py" if config.enable_camera else "wuji_teleop.launch.py"
        extra_args.extend(
            [
                f"hand_input:={config.hand_input}",
                f"arm_input:={config.arm_input}",
            ]
        )
        if config.enable_camera:
            extra_args.append("enable_camera:=true")
    else:
        launch_file = "wuji_teleop_camera.launch.py" if config.enable_camera else "wuji_teleop_single.launch.py"
        side = "left" if config.mode == "single_left" else "right"
        extra_args.extend(
            [
                f"side:={side}",
                f"hand_input:={config.hand_input}",
                f"arm_input:={config.arm_input}",
            ]
        )
        if config.enable_camera:
            extra_args.append("enable_camera:=true")

    extra_args.append(f"enable_rviz:={'true' if config.enable_rviz else 'false'}")

    return [
        "ros2",
        "launch",
        "wuji_teleop_bringup",
        launch_file,
        *extra_args,
    ]


def main() -> int:
    args = build_arg_parser().parse_args()
    config_path = Path(args.config).expanduser().resolve()
    config = BringupConfig.from_yaml(config_path)

    if args.mode is not None:
        config.mode = args.mode
    if args.camera:
        config.enable_camera = True
    if args.rviz:
        config.enable_rviz = True

    if not args.skip_preflight:
        preflight_report = collect_report(Path("config/htc_openvr_tracker.yaml").resolve())
        ok, issues = evaluate_report(preflight_report)
        if not ok:
            for issue in issues:
                print(f"[preflight] {issue}")
            print("[preflight] Use `python3 -m dexproj.check_devices` for details.")
            return 2

    command = resolve_launch_command(config)
    if args.dry_run:
        print(" ".join(shlex.quote(token) for token in command))
        return 0

    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


if __name__ == "__main__":
    raise SystemExit(main())

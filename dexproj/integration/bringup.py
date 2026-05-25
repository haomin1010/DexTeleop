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
from dexproj.hand_teleop.config import HandTeleopConfig

VALID_MODES = {"single_left", "single_right", "dual"}


@dataclass
class BringupConfig:
    mode: str = "dual"
    enable_camera: bool = False
    enable_rviz: bool = False
    enable_arm: bool = True
    hand_input: str = "wuji_glove"
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
            enable_arm=bool(raw.get("enable_arm", True)),
            hand_input=str(raw.get("hand_input", "wuji_glove")),
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
        "--hand-teleop-config",
        default="config/hand_teleop_wuji_glove.yaml",
        help="DexProj hand teleop config file.",
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
        "--hand-only",
        action="store_true",
        help="Disable arm/tracker bringup and launch hand nodes only.",
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
    if config.enable_arm and config.arm_input != "tracker":
        raise ValueError(
            f"DexProj only supports HTC/OpenVR arm input for now, got {config.arm_input!r}."
        )
    if config.mode not in VALID_MODES:
        raise ValueError(f"Unsupported mode: {config.mode}")
    if not config.enable_arm and config.enable_camera:
        raise ValueError("Hand-only bringup does not support camera launch yet.")

    launch_file: str
    extra_args: list[str] = []

    if not config.enable_arm:
        if config.mode == "dual":
            launch_file = "wuji_teleop_hand.launch.py"
            extra_args.append(f"hand_input:={config.hand_input}")
        else:
            launch_file = "wuji_teleop_single.launch.py"
            side = "left" if config.mode == "single_left" else "right"
            extra_args.extend(
                [
                    f"side:={side}",
                    f"hand_input:={config.hand_input}",
                    f"arm_input:={config.arm_input}",
                    "enable_arm:=false",
                ]
            )
    elif config.mode == "dual":
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

    if config.enable_arm:
        extra_args.append(f"enable_rviz:={'true' if config.enable_rviz else 'false'}")

    return [
        "ros2",
        "launch",
        "wuji_teleop_bringup",
        launch_file,
        *extra_args,
    ]


def apply_hand_teleop_overrides(command: list[str], hand_cfg: HandTeleopConfig, run_mode: str) -> list[str]:
    updated = list(command)

    if hand_cfg.left is not None:
        updated.extend(
            [
                f"left_serial:={hand_cfg.left.hand_sn}",
                f"left_glove_sn:={hand_cfg.left.glove_sn}",
                f"left_device_name:={hand_cfg.left.glove_device_name}",
            ]
        )
        if hand_cfg.left.retarget_config:
            updated.append(f"left_retarget_config:={hand_cfg.left.retarget_config}")
    else:
        updated.append("include_left_hand:=false")

    if hand_cfg.right is not None:
        updated.extend(
            [
                f"right_serial:={hand_cfg.right.hand_sn}",
                f"right_glove_sn:={hand_cfg.right.glove_sn}",
                f"right_device_name:={hand_cfg.right.glove_device_name}",
            ]
        )
        if hand_cfg.right.retarget_config:
            updated.append(f"right_retarget_config:={hand_cfg.right.retarget_config}")
    else:
        updated.append("include_right_hand:=false")

    if run_mode == "single_left":
        updated.append("include_right_hand:=false")
    elif run_mode == "single_right":
        updated.append("include_left_hand:=false")

    return updated


def main() -> int:
    args = build_arg_parser().parse_args()
    config_path = Path(args.config).expanduser().resolve()
    config = BringupConfig.from_yaml(config_path)
    hand_cfg = HandTeleopConfig.from_yaml(Path(args.hand_teleop_config).expanduser().resolve())

    if args.mode is not None:
        config.mode = args.mode
    if args.camera:
        config.enable_camera = True
    if args.rviz:
        config.enable_rviz = True
    if args.hand_only:
        config.enable_arm = False

    if config.enable_arm and not args.skip_preflight:
        preflight_report = collect_report(Path("config/htc_openvr_tracker.yaml").resolve())
        ok, issues = evaluate_report(preflight_report)
        if not ok:
            for issue in issues:
                print(f"[preflight] {issue}")
            print("[preflight] Use `python3 -m dexproj.check_devices` for details.")
            return 2

    command = apply_hand_teleop_overrides(resolve_launch_command(config), hand_cfg, config.mode)
    if args.dry_run:
        print(" ".join(shlex.quote(token) for token in command))
        return 0

    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


if __name__ == "__main__":
    raise SystemExit(main())

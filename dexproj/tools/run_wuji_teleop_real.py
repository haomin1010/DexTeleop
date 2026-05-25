"""Run the known-good Wuji teleop_real.py path for configured hands."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from dexproj.hand_teleop.config import HandChannelConfig, HandTeleopConfig


ROOT = Path(__file__).resolve().parents[2]
RETARGETING_ROOT = ROOT / "wuji-retargeting"
EXAMPLE_DIR = ROOT / "wuji-retargeting" / "example"
DEFAULT_CONDA_ENV = os.environ.get(
    "DEXPROJ_HAND_CONDA_ENV",
    "dexproj" if Path("/.dockerenv").exists() else "lhm-wuji",
)


def _build_shell_command(channel: HandChannelConfig, conda_env: str) -> str:
    config_path = (ROOT / channel.retarget_config).resolve()
    conda_candidates = [
        Path.home() / "miniconda3" / "etc" / "profile.d" / "conda.sh",
        Path("/home/wuji/miniconda3/etc/profile.d/conda.sh"),
        Path("/opt/miniconda3/etc/profile.d/conda.sh"),
    ]
    conda_sh = next((path for path in conda_candidates if path.exists()), None)
    if conda_sh is None:
        raise RuntimeError(f"Cannot find conda.sh for activating {conda_env}.")

    args = [
        "python",
        "teleop_real.py",
        "--input",
        "wuji_glove",
        "--hand",
        channel.side,
        "--hand-sn",
        channel.hand_sn,
        "--glove-sn",
        channel.glove_sn,
        "--device-name",
        channel.glove_device_name,
        "--config",
        str(config_path),
    ]
    quoted_args = " ".join(subprocess.list2cmdline([arg]) for arg in args)
    return (
        f"source {subprocess.list2cmdline([str(conda_sh)])} && "
        f"conda activate {subprocess.list2cmdline([conda_env])} && "
        f"export PYTHONPATH={subprocess.list2cmdline([str(RETARGETING_ROOT)])}:${{PYTHONPATH:-}} && "
        f"cd {subprocess.list2cmdline([str(EXAMPLE_DIR)])} && "
        f"exec {quoted_args}"
    )


def _start(channel: HandChannelConfig, conda_env: str) -> subprocess.Popen:
    command = _build_shell_command(channel, conda_env)
    print(
        f"[dexproj] starting {channel.side} teleop_real: "
        f"hand={channel.hand_sn} glove={channel.glove_sn} device={channel.glove_device_name}",
        flush=True,
    )
    return subprocess.Popen(["bash", "-lc", command], start_new_session=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Wuji teleop_real.py for hand-only control")
    parser.add_argument(
        "--hand-teleop-config",
        default="config/hand_teleop_wuji_glove.yaml",
        help="DexProj hand teleop config file.",
    )
    parser.add_argument("--conda-env", default=DEFAULT_CONDA_ENV)
    parser.add_argument("--hand", choices=("left", "right", "both"), default="both")
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=float(os.environ.get("DEXPROJ_HAND_STARTUP_DELAY", "0.0")),
        help="Seconds to wait between starting left/right teleop_real processes.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print commands without starting hardware.")
    args = parser.parse_args(argv)

    config = HandTeleopConfig.from_yaml((ROOT / args.hand_teleop_config).resolve())
    channels = [channel for channel in (config.left, config.right) if channel is not None]
    if args.hand in {"left", "right"}:
        channels = [channel for channel in channels if channel.side == args.hand]
    elif config.mode == "single_left":
        channels = [channel for channel in channels if channel.side == "left"]
    elif config.mode == "single_right":
        channels = [channel for channel in channels if channel.side == "right"]

    processes: list[subprocess.Popen] = []
    try:
        for channel in channels:
            if args.dry_run:
                print(_build_shell_command(channel, args.conda_env))
                continue
            processes.append(_start(channel, args.conda_env))
            if len(channels) > 1 and channel is not channels[-1] and args.startup_delay > 0:
                time.sleep(args.startup_delay)
        if args.dry_run:
            return 0
        if not processes:
            raise RuntimeError("No hand channels configured.")

        while processes:
            for process in list(processes):
                code = process.poll()
                if code is not None:
                    processes.remove(process)
                    if code != 0:
                        return code
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for process in processes:
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Configuration models for DexProj hand teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

VALID_HAND_MODES = {"single_left", "single_right", "dual"}
VALID_HAND_BACKENDS = {"ros2", "teleop_real", "py"}


@dataclass
class HandChannelConfig:
    side: str
    glove_sn: str = ""
    hand_sn: str = ""
    glove_device_name: str = "glove"
    retarget_config: str = ""


@dataclass
class HandTeleopConfig:
    mode: str
    backend: str
    auto_discover_glove_sn: bool
    auto_discover_hand_sn: bool
    left: HandChannelConfig | None
    right: HandChannelConfig | None

    @classmethod
    def from_yaml(cls, path: Path) -> "HandTeleopConfig":
        if yaml is None:
            raise RuntimeError("PyYAML is required to load hand teleop config files.")
        with path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"Invalid hand teleop config: {path}")

        mode = str(raw.get("mode", "dual"))
        if mode not in VALID_HAND_MODES:
            raise ValueError(f"Unsupported hand teleop mode: {mode}")
        backend = str(raw.get("backend", "ros2"))
        if backend not in VALID_HAND_BACKENDS:
            raise ValueError(f"Unsupported hand teleop backend: {backend}")

        hands = raw.get("hands", {})
        if not isinstance(hands, dict):
            hands = {}

        def _channel(side: str) -> HandChannelConfig | None:
            section = hands.get(side)
            if not isinstance(section, dict):
                return None
            return HandChannelConfig(
                side=side,
                glove_sn=str(section.get("glove_sn", "")),
                hand_sn=str(section.get("hand_sn", "")),
                glove_device_name=str(section.get("glove_device_name", "glove")),
                retarget_config=str(section.get("retarget_config", "")),
            )

        config = cls(
            mode=mode,
            backend="teleop_real" if backend == "py" else backend,
            auto_discover_glove_sn=bool(raw.get("auto_discover_glove_sn", False)),
            auto_discover_hand_sn=bool(raw.get("auto_discover_hand_sn", False)),
            left=_channel("left"),
            right=_channel("right"),
        )
        config.validate()
        return config

    def validate(self) -> None:
        if self.mode == "single_left":
            if self.left is None:
                raise ValueError("single_left mode requires left hand channel config.")
        elif self.mode == "single_right":
            if self.right is None:
                raise ValueError("single_right mode requires right hand channel config.")
        elif self.mode == "dual":
            if self.left is None or self.right is None:
                raise ValueError("dual mode requires both left and right hand channel configs.")

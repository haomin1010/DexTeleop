"""CLI helper for listing connected Wuji glove serial numbers."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dexproj.check_devices.scanners import WujiGloveScanner, WujiHandScanner

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="List connected Wuji glove serial numbers.")
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON output.",
    )
    parser.add_argument(
        "--update-config",
        metavar="PATH",
        help="Write discovered glove serial numbers into a hand teleop config file.",
    )
    parser.add_argument(
        "--left",
        help="Left glove serial number to write instead of auto-detecting it.",
    )
    parser.add_argument(
        "--right",
        help="Right glove serial number to write instead of auto-detecting it.",
    )
    return parser


def _infer_sides(found: list[str]) -> dict[str, str]:
    mapping: dict[str, str] = {}
    for serial in found:
        if serial.startswith("WG1J") and "left" not in mapping:
            mapping["left"] = serial
        elif serial.startswith("WG1K") and "right" not in mapping:
            mapping["right"] = serial

    remaining = [serial for serial in found if serial not in mapping.values()]
    if "left" not in mapping and remaining:
        mapping["left"] = remaining.pop(0)
    if "right" not in mapping and remaining:
        mapping["right"] = remaining.pop(0)
    return mapping


def _infer_hand_sides(found: list[str], raw: dict) -> dict[str, str]:
    found_set = set(found)
    hands = raw.get("hands", {})
    if not isinstance(hands, dict):
        hands = {}

    mapping: dict[str, str] = {}
    for side in ("left", "right"):
        section = hands.get(side)
        if not isinstance(section, dict):
            continue
        serial = str(section.get("hand_sn", "")).strip()
        if serial in found_set:
            mapping[side] = serial

    remaining = sorted(serial for serial in found if serial not in mapping.values())
    for side in ("left", "right"):
        if side not in mapping and remaining:
            mapping[side] = remaining.pop(0)
    return mapping


def _update_config_fields(path: Path, mapping: dict[str, str], field: str) -> None:
    if yaml is None:
        raise RuntimeError("PyYAML is required to update hand teleop config files.")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else {}
    if not isinstance(raw, dict):
        raw = {}
    hands = raw.setdefault("hands", {})
    if not isinstance(hands, dict):
        hands = {}
        raw["hands"] = hands
    for side, serial in mapping.items():
        if not serial:
            continue
        section = hands.setdefault(side, {})
        if not isinstance(section, dict):
            section = {}
            hands[side] = section
        section[field] = serial
    path.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")


def ensure_retargeting_assets(workspace_root: Path) -> None:
    urdf_dir = workspace_root / "wuji-retargeting/wuji_retargeting/wuji_hand_description/urdf"
    missing = [name for name in ("left.urdf", "right.urdf") if not (urdf_dir / name).is_file()]
    if not missing:
        return
    raise RuntimeError(
        "Missing wuji_hand_description URDF files required for hand teleop: "
        f"{', '.join(missing)}. "
        "Initialize submodule: "
        "cd wuji-retargeting && git submodule update --init wuji_retargeting/wuji_hand_description"
    )


def auto_update_config_if_enabled(path: Path) -> dict[str, dict[str, str]]:
    """Refresh glove/hand serial numbers in *path* when auto_discover flags are enabled."""
    updates: dict[str, dict[str, str]] = {}
    if yaml is None or not path.exists():
        return updates

    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        return updates

    if raw.get("auto_discover_glove_sn", False):
        glove_mapping = _infer_sides(WujiGloveScanner.scan().found)
        if glove_mapping:
            _update_config_fields(path, glove_mapping, "glove_sn")
            updates["glove"] = glove_mapping

    if raw.get("auto_discover_hand_sn", False):
        hand_mapping = _infer_hand_sides(WujiHandScanner.scan().found, raw)
        if hand_mapping:
            _update_config_fields(path, hand_mapping, "hand_sn")
            updates["hand"] = hand_mapping

    return updates


def main() -> int:
    args = build_arg_parser().parse_args()
    result = WujiGloveScanner.scan()
    mapping = _infer_sides(result.found)
    if args.left:
        mapping["left"] = args.left
    if args.right:
        mapping["right"] = args.right

    if args.update_config:
        config_path = Path(args.update_config).expanduser()
        updates = auto_update_config_if_enabled(config_path)
        if not updates:
            glove_mapping = mapping
            if glove_mapping:
                _update_config_fields(config_path, glove_mapping, "glove_sn")
                updates["glove"] = glove_mapping
        for kind, side_mapping in updates.items():
            label = "glove" if kind == "glove" else "hand"
            for side in ("left", "right"):
                if side in side_mapping:
                    print(f"{side} {label}: {side_mapping[side]}")
        print(f"updated: {config_path}")
        return 0

    if args.json:
        print(
            json.dumps(
                {"found": result.found, "sides": mapping, "notes": result.notes},
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    if result.found:
        for serial in result.found:
            print(serial)
    else:
        print("No Wuji glove serial numbers found.")

    for note in result.notes:
        print(f"note: {note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

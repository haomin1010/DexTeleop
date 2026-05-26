"""CLI helper for listing connected Wuji glove serial numbers."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dexproj.check_devices.scanners import WujiGloveScanner

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


def _update_config(path: Path, mapping: dict[str, str]) -> None:
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
        section["glove_sn"] = serial
    path.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")


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
        _update_config(config_path, mapping)
        for side in ("left", "right"):
            if side in mapping:
                print(f"{side}: {mapping[side]}")
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

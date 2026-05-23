"""CLI and reusable helpers for DexProj device checks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .scanners import OpenVRTrackerScanner, WujiGloveScanner, WujiHandScanner


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check DexProj hardware dependencies.")
    parser.add_argument(
        "--openvr-config",
        default="config/htc_openvr_tracker.yaml",
        help="Local DexProj HTC tracker config file.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON output.",
    )
    return parser


def _human_report(report: dict) -> str:
    lines: list[str] = []

    hands = report["wuji_hands"]
    gloves = report["wuji_gloves"]
    trackers = report["htc_trackers"]

    lines.append("Wuji Hands:")
    lines.append(f"  found: {hands['found'] or '[]'}")
    for note in hands["notes"]:
        lines.append(f"  note: {note}")

    lines.append("Wuji Gloves:")
    lines.append(f"  found: {gloves['found'] or '[]'}")
    for note in gloves["notes"]:
        lines.append(f"  note: {note}")

    lines.append("HTC Trackers:")
    lines.append(f"  found: {trackers['found'] or '[]'}")
    if trackers["expected"]:
        lines.append(f"  expected mapping: {trackers['expected']}")
    if trackers["missing_roles"]:
        lines.append(f"  missing roles: {trackers['missing_roles']}")
    for note in trackers["notes"]:
        lines.append(f"  note: {note}")

    return "\n".join(lines)


def collect_report(openvr_config: Path) -> dict:
    hands = WujiHandScanner.scan()
    gloves = WujiGloveScanner.scan()
    trackers = OpenVRTrackerScanner.scan(openvr_config)

    return {
        "wuji_hands": {
            "found": hands.found,
            "notes": hands.notes,
        },
        "wuji_gloves": {
            "found": gloves.found,
            "notes": gloves.notes,
        },
        "htc_trackers": {
            "found": trackers.found,
            "expected": trackers.expected,
            "missing_roles": trackers.missing_roles,
            "notes": trackers.notes,
        },
    }


def evaluate_report(report: dict) -> tuple[bool, list[str]]:
    issues: list[str] = []
    trackers = report.get("htc_trackers", {})
    missing_roles = trackers.get("missing_roles", [])
    if missing_roles:
        issues.append(f"Missing HTC tracker roles: {missing_roles}")

    expected_tracker_roles = trackers.get("expected", {})
    if not expected_tracker_roles:
        issues.append("No expected HTC tracker mapping configured.")

    return len(issues) == 0, issues


def main() -> int:
    args = build_arg_parser().parse_args()
    openvr_config = Path(args.openvr_config).expanduser().resolve()
    report = collect_report(openvr_config)

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(_human_report(report))
    return 0

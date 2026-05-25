"""CLI helper for listing connected Wuji glove serial numbers."""

from __future__ import annotations

import argparse
import json

from dexproj.check_devices.scanners import WujiGloveScanner


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="List connected Wuji glove serial numbers.")
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON output.",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    result = WujiGloveScanner.scan()

    if args.json:
        print(json.dumps({"found": result.found, "notes": result.notes}, ensure_ascii=False, indent=2))
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

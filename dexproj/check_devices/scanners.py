"""Minimal device scanners for DexProj.

This module intentionally starts small and depends only on standard
system utilities when possible, so we can use it before the full ROS2
runtime is brought up.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - optional dependency in early bootstrap
    yaml = None


def _run_command(args: list[str], timeout: float = 5.0) -> tuple[bool, str]:
    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False, ""

    if result.returncode != 0:
        return False, result.stdout or result.stderr
    return True, result.stdout


def _load_yaml(path: Path) -> dict[str, Any]:
    if yaml is None or not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else {}


@dataclass
class ScanResult:
    found: list[str] = field(default_factory=list)
    expected: dict[str, str] = field(default_factory=dict)
    missing_roles: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


class WujiHandScanner:
    USB_VID = "0483"
    USB_PID = "2000"

    @classmethod
    def scan(cls) -> ScanResult:
        ok, output = _run_command(["lsusb", "-v", "-d", f"{cls.USB_VID}:{cls.USB_PID}"])
        result = ScanResult()
        if not ok:
            result.notes.append("`lsusb` unavailable or no Wuji Hand device response.")
            return result

        serials: set[str] = set()
        for line in output.splitlines():
            if "iSerial" not in line:
                continue
            parts = line.strip().split()
            if len(parts) >= 3 and parts[-1] != "0":
                serials.add(parts[-1])
        result.found = sorted(serials)
        if not serials:
            result.notes.append("No Wuji Hand serials found via USB scan.")
        return result


class OpenVRTrackerScanner:
    @classmethod
    def scan(cls, config_path: Path | None = None) -> ScanResult:
        result = ScanResult()
        if config_path is not None:
            config = _load_yaml(config_path)
            tracker_serials = config.get("tracker_serials", {})
            if isinstance(tracker_serials, dict):
                result.expected = {
                    str(role): str(serial)
                    for role, serial in tracker_serials.items()
                    if str(serial).strip()
                }

        python_bin = shutil.which("python3") or shutil.which("python")
        if python_bin is None:
            result.notes.append("No python interpreter found for OpenVR scan.")
            result.missing_roles = sorted(result.expected.keys())
            return result

        one_liner = (
            "import openvr; "
            "openvr.init(openvr.VRApplication_Other); "
            "vr=openvr.VRSystem(); "
            "serials=[]; "
            "[(serials.append(vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String))) "
            "for i in range(64) if vr.getTrackedDeviceClass(i)==openvr.TrackedDeviceClass_GenericTracker and vr.isTrackedDeviceConnected(i)]; "
            "openvr.shutdown(); "
            "print('\\n'.join(serials))"
        )
        ok, output = _run_command([python_bin, "-c", one_liner], timeout=8.0)
        if not ok:
            result.notes.append("OpenVR tracker scan failed. Check SteamVR/OpenVR environment.")
            result.missing_roles = sorted(result.expected.keys())
            return result

        result.found = sorted({line.strip() for line in output.splitlines() if line.strip()})
        if result.expected:
            found_set = set(result.found)
            result.missing_roles = sorted(
                role for role, serial in result.expected.items() if serial not in found_set
            )
        return result


class WujiGloveScanner:
    @classmethod
    def scan(cls) -> ScanResult:
        result = ScanResult()
        python_bin = shutil.which("python3") or shutil.which("python")
        if python_bin is None:
            result.notes.append("No python interpreter found for Wuji Glove scan.")
            return result

        one_liner = (
            "import json; "
            "from wuji_sdk import SdkManager; "
            "mgr=SdkManager.instance(); "
            "devices=[]; "
            "enum=getattr(mgr, 'list_devices', None); "
            "devices = enum() if callable(enum) else []; "
            "out=[]; "
            "for d in devices: "
            "  sn=getattr(d, 'sn', None) or getattr(d, 'serial_number', None) or str(d); "
            "  out.append(str(sn)); "
            "print(json.dumps(out))"
        )
        ok, output = _run_command([python_bin, "-c", one_liner], timeout=8.0)
        if not ok:
            result.notes.append("Wuji SDK glove scan unavailable. Fallback to manual SN recording.")
            return result

        try:
            data = json.loads(output.strip() or "[]")
        except json.JSONDecodeError:
            result.notes.append("Wuji SDK glove scan returned unparsable output.")
            return result
        if isinstance(data, list):
            result.found = sorted(str(item) for item in data if str(item).strip())
        return result


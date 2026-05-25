"""Minimal device scanners for DexProj.

This module intentionally starts small and depends only on standard
system utilities when possible, so we can use it before the full ROS2
runtime is brought up.
"""

from __future__ import annotations

import json
import re
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


def _coerce_serial_value(value: Any) -> str | None:
    visited: set[int] = set()
    return _coerce_serial_value_inner(value, visited)


def _coerce_serial_value_inner(value: Any, visited: set[int]) -> str | None:
    obj_id = id(value)
    if obj_id in visited:
        return None
    visited.add(obj_id)

    if callable(value):
        try:
            value = value()
        except Exception:
            return None
        return _coerce_serial_value_inner(value, visited)
    if value is None:
        return None
    if isinstance(value, dict):
        for key in ("sn", "serial_number", "serial", "id", "value", "text", "name"):
            if key in value:
                serial = _coerce_serial_value_inner(value[key], visited)
                if serial:
                    return serial
        return None
    for attr in ("sn", "serial_number", "serial", "id", "value", "text", "name", "raw"):
        if hasattr(value, attr):
            try:
                nested = getattr(value, attr)
            except Exception:
                continue
            serial = _coerce_serial_value_inner(nested, visited)
            if serial:
                return serial
    text = str(value).strip()
    if not text:
        return None
    if text.startswith("<built-in method "):
        return None
    if text.startswith("<builtins.") and text.endswith(">"):
        return None
    return text


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

        script = """
import json
import inspect

payload = {"found": [], "notes": []}

from wuji_sdk import SdkManager


def extract_serial(value, visited=None):
    if visited is None:
        visited = set()
    obj_id = id(value)
    if obj_id in visited:
        return None
    visited.add(obj_id)

    if callable(value):
        try:
            if len(inspect.signature(value).parameters) == 0:
                return extract_serial(value(), visited)
        except Exception:
            try:
                return extract_serial(value(), visited)
            except Exception:
                return None

    if value is None:
        return None

    if isinstance(value, str):
        text = value.strip()
        if text and not text.startswith("<builtins.") and not text.startswith("<built-in method "):
            return text
        return None

    if isinstance(value, dict):
        for key in ("sn", "serial_number", "serial", "id", "value", "text", "name", "raw"):
            if key in value:
                serial = extract_serial(value[key], visited)
                if serial:
                    return serial
        return None

    for attr in ("sn", "serial_number", "serial", "id", "value", "text", "name", "raw"):
        if hasattr(value, attr):
            try:
                serial = extract_serial(getattr(value, attr), visited)
            except Exception:
                continue
            if serial:
                return serial

    try:
        text = str(value).strip()
    except Exception:
        return None
    if text and not text.startswith("<builtins.") and not text.startswith("<built-in method "):
        return text
    return None


mgr = SdkManager.instance()
api_names = [
    "list_devices",
    "devices",
    "get_devices",
    "enumerate_devices",
    "scan_devices",
    "discover_devices",
]
devices = []
used_api = None

for name in api_names:
    fn = getattr(mgr, name, None)
    if not callable(fn):
        continue
    try:
        value = fn()
        if value is None:
            continue
        if not isinstance(value, (list, tuple, set)):
            try:
                value = list(value)
            except TypeError:
                value = [value]
        devices = list(value)
        used_api = name
        break
    except Exception as exc:
        payload["notes"].append(f"{name}() failed: {exc!r}")

if used_api is not None:
    payload["notes"].append(f"enumeration api: {used_api}")

for device in devices:
    if isinstance(device, dict):
        sn = (
            device.get("sn")
            or device.get("serial_number")
            or device.get("serial")
            or device.get("id")
        )
    else:
        sn = (
            getattr(device, "sn", None)
            or getattr(device, "serial_number", None)
            or getattr(device, "serial", None)
            or getattr(device, "id", None)
        )
    sn = extract_serial(sn)
    if sn:
        payload["found"].append(str(sn))

if payload["found"]:
    payload["found"] = sorted(set(payload["found"]))
    print(json.dumps(payload))
    raise SystemExit(0)

connect_targets = []
for device_name in ("glove", "left_glove", "right_glove"):
    for method_name in ("auto_connect", "connect"):
        connect_targets.append((method_name, device_name))

for method_name, device_name in connect_targets:
    fn = getattr(mgr, method_name, None)
    if not callable(fn):
        continue
    try:
        if method_name == "auto_connect":
            device = fn(device_name=device_name)
        else:
            continue

        if device is None:
            payload["notes"].append(
                f"{method_name}(device_name={device_name!r}) returned None"
            )
            continue

        sn = (
            getattr(device, "sn", None)
            or getattr(device, "serial_number", None)
            or getattr(device, "serial", None)
        )
        if not sn:
            info = getattr(device, "info", None)
            if isinstance(info, dict):
                sn = info.get("sn") or info.get("serial_number") or info.get("serial")
        sn = extract_serial(sn)

        if sn:
            payload["found"].append(str(sn))
            payload["notes"].append(
                f"connected via {method_name}(device_name={device_name!r})"
            )
            break

        debug_attrs = []
        for attr in ("sn", "serial_number", "serial", "id", "value", "text", "name", "raw"):
            if hasattr(device, attr):
                debug_attrs.append(attr)
        payload["notes"].append(
            f"{method_name}(device_name={device_name!r}) connected but serial not exposed; "
            f"attrs={debug_attrs}; repr={device!r}"
        )
    except Exception as exc:
        payload["notes"].append(f"{method_name}(device_name={device_name!r}) failed: {exc!r}")

    if payload["found"]:
        break

payload["found"] = sorted(set(payload["found"]))
if not payload["found"] and not payload["notes"]:
    payload["notes"].append(
        "No compatible Wuji SDK scan/connect API found on SdkManager.instance()."
    )

print(json.dumps(payload))
"""
        ok, output = _run_command([python_bin, "-c", script], timeout=8.0)
        if not ok:
            message = (output or "").strip()
            if message:
                result.notes.append(f"Wuji SDK glove scan failed: {message}")
            result.notes.append("Fallback to manual SN recording.")
            return result

        try:
            data = json.loads(output.strip() or "{}")
        except json.JSONDecodeError:
            result.notes.append("Wuji SDK glove scan returned unparsable output.")
            return result
        if isinstance(data, dict):
            found = data.get("found", [])
            notes = data.get("notes", [])
            if isinstance(found, list):
                cleaned = []
                for item in found:
                    serial = _coerce_serial_value(item)
                    if serial:
                        cleaned.append(serial)
                result.found = sorted(cleaned)
            if isinstance(notes, list):
                result.notes.extend(str(item) for item in notes if str(item).strip())
        elif isinstance(data, list):
            cleaned = []
            for item in data:
                serial = _coerce_serial_value(item)
                if serial:
                    cleaned.append(serial)
            result.found = sorted(cleaned)

        if not result.found:
            extracted = cls._extract_serials_from_notes(result.notes)
            if extracted:
                result.found = extracted
                result.notes.append("Extracted glove serial numbers from Wuji SDK connection error.")

        if not result.found and not result.notes:
            result.notes.append("Wuji SDK did not report any connected glove.")
        return result

    @staticmethod
    def _extract_serials_from_notes(notes: list[str]) -> list[str]:
        serials: set[str] = set()
        for note in notes:
            for match in re.findall(r"\bWG[A-Z0-9]+\b", note):
                serials.add(match)
        return sorted(serials)

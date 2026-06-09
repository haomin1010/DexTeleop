"""Discover RealSense serial numbers and update camera config."""

from __future__ import annotations

from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

WRIST_PLACEHOLDER_SERIALS = {
    "",
    "LEFT_WRIST_REALSENSE_SN",
    "RIGHT_WRIST_REALSENSE_SN",
    "YOUR_LEFT_WRIST_CAM_SERIAL",
    "YOUR_RIGHT_WRIST_CAM_SERIAL",
}
REALSENSE_VENDOR = "8086"
D405_PRODUCT = "0b5b"
D435_PRODUCTS = {"0b3a", "0b07"}


def _usb_identity(video_name: str) -> tuple[str, str, str]:
    device = Path(f"/sys/class/video4linux/{video_name}/device").resolve()
    for parent in [device, *device.parents]:
        vendor_path = parent / "idVendor"
        product_path = parent / "idProduct"
        if vendor_path.is_file() and product_path.is_file():
            return (
                parent.name,
                vendor_path.read_text().strip().lower(),
                product_path.read_text().strip().lower(),
            )
    return "", "", ""


def discover_realsense_serials() -> list[dict[str, str]]:
    """Return connected RealSense cameras with serial, product, and video node."""
    discovered: dict[str, dict[str, str]] = {}
    for index_path in sorted(Path("/sys/class/video4linux").glob("video*/index")):
        if index_path.read_text().strip() != "0":
            continue
        video_name = index_path.parent.name
        serial_path = index_path.parent / "device" / "serial"
        if not serial_path.is_file():
            continue
        serial = serial_path.read_text().strip()
        if not serial or serial in discovered:
            continue
        usb_path, vendor, product = _usb_identity(video_name)
        if vendor != REALSENSE_VENDOR:
            continue
        discovered[serial] = {
            "serial": serial,
            "product": product,
            "video": video_name,
            "usb": usb_path,
        }
    return [discovered[key] for key in sorted(discovered)]


def _is_placeholder_serial(value: str) -> bool:
    return str(value or "").strip() in WRIST_PLACEHOLDER_SERIALS


def _assign_wrist_serials(cameras: dict) -> dict[str, str]:
    d405_serials = [
        item["serial"]
        for item in discover_realsense_serials()
        if item.get("product") == D405_PRODUCT
    ]
    mapping: dict[str, str] = {}
    for side in ("left_wrist", "right_wrist"):
        cam_cfg = cameras.get(side, {})
        if not isinstance(cam_cfg, dict) or not bool(cam_cfg.get("enabled", False)):
            continue
        current = str(cam_cfg.get("serial_number", "")).strip()
        if current and not _is_placeholder_serial(current):
            mapping[side] = current
    remaining = [serial for serial in d405_serials if serial not in mapping.values()]
    for side in ("left_wrist", "right_wrist"):
        cam_cfg = cameras.get(side, {})
        if not isinstance(cam_cfg, dict) or not bool(cam_cfg.get("enabled", False)):
            continue
        if side in mapping:
            continue
        if not bool(cam_cfg.get("auto_discover_serial", False)) and not _is_placeholder_serial(
            str(cam_cfg.get("serial_number", "")).strip()
        ):
            continue
        if not remaining:
            break
        mapping[side] = remaining.pop(0)
    return mapping


def auto_update_config_if_enabled(path: Path) -> dict[str, str]:
    if yaml is None or not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        return {}
    cameras = raw.get("cameras", {})
    if not isinstance(cameras, dict):
        return {}
    mapping = _assign_wrist_serials(cameras)
    if not mapping:
        return {}
    changed = False
    for side, serial in mapping.items():
        section = cameras.get(side, {})
        if not isinstance(section, dict):
            continue
        if str(section.get("serial_number", "")).strip() == serial:
            continue
        section["serial_number"] = serial
        changed = True
    if changed:
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(raw, handle, sort_keys=False, allow_unicode=True)
    return mapping

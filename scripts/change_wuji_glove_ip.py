#!/usr/bin/env python3
"""Change a Wuji Glove network IP through wuji_sdk.

Run this from the native dexproj conda environment:

    source ~/miniconda3/etc/profile.d/conda.sh
    conda activate dexproj
    python scripts/change_wuji_glove_ip.py --address 192.168.1.100:50001 --new-ip 192.168.2.150
"""

from __future__ import annotations

import argparse
import ipaddress
import socket
import sys
from typing import Any


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Change Wuji Glove IP via wuji_sdk.")
    parser.add_argument(
        "--address",
        default="192.168.1.100:50001",
        help="Current glove address, usually IP:port.",
    )
    parser.add_argument(
        "--new-ip",
        default="192.168.2.150",
        help="New static IP to write to the glove.",
    )
    parser.add_argument(
        "--new-port",
        type=int,
        default=50001,
        help="New glove data port.",
    )
    parser.add_argument(
        "--device-name",
        default="wuji_glove",
        help="SDK device name. Keep 'wuji_glove' for the semantic glove API.",
    )
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=3000,
        help="SDK connect timeout in milliseconds.",
    )
    parser.add_argument(
        "--retry-count",
        type=int,
        default=1,
        help="SDK connect retry count.",
    )
    parser.add_argument(
        "--skip-free-check",
        action="store_true",
        help="Skip checking whether the new IP already responds.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Connect and print current settings without writing changes.",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="Scan and print discovered devices before connecting.",
    )
    return parser.parse_args()


def _tcp_probe(ip: str, port: int, timeout_s: float = 1.0) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _ping_probe(ip: str, timeout_s: float = 1.0) -> bool:
    # Avoid shelling out; a TCP check is more meaningful for the glove port.
    # Use common ports only as a weak "is there something there" signal.
    for port in (50001, 22, 80, 443):
        if _tcp_probe(ip, port, timeout_s=timeout_s):
            return True
    return False


def _split_ip_port(address: str) -> tuple[str, int | None]:
    host, sep, port_text = address.rpartition(":")
    if not sep:
        return address, None
    try:
        return host, int(port_text)
    except ValueError:
        return address, None


def _get_resource_value(resource: Any) -> Any:
    try:
        return resource.get()
    except Exception as exc:  # noqa: BLE001 - SDK raises extension exceptions.
        return f"<read failed: {exc}>"


def _print_connect_help(address: str) -> None:
    old_ip, _ = _split_ip_port(address)
    try:
        old_network = ipaddress.ip_network(f"{old_ip}/24", strict=False)
    except ValueError:
        old_network = None

    print("[change_wuji_glove_ip] connect failed, so the glove IP was NOT changed.", file=sys.stderr)
    print("[change_wuji_glove_ip] Scan can see UDP discovery, but SDK connect still needs the old IP to be reachable.", file=sys.stderr)

    if old_network is None:
        return

    helper_ip = None
    for last_octet in (104, 150, 200):
        candidate = ipaddress.ip_address(f"{old_network.network_address + last_octet}")
        if candidate in old_network and str(candidate) != old_ip:
            helper_ip = str(candidate)
            break
    if helper_ip is None:
        helper_ip = str(next(ip for ip in old_network.hosts() if str(ip) != old_ip))
    print("[change_wuji_glove_ip] Temporarily add a PC IP on the glove's old subnet, then rerun:", file=sys.stderr)
    print(f"  sudo ip addr add {helper_ip}/24 dev enp129s0", file=sys.stderr)
    print(f"  ping -c 2 {old_ip}", file=sys.stderr)
    print(
        "  python scripts/change_wuji_glove_ip.py "
        f"--scan --address {address} --new-ip 192.168.2.150",
        file=sys.stderr,
    )
    print("[change_wuji_glove_ip] After the glove moves to the new IP, remove the temporary PC IP:", file=sys.stderr)
    print(f"  sudo ip addr del {helper_ip}/24 dev enp129s0", file=sys.stderr)


def main() -> int:
    args = _parse_args()

    try:
        ipaddress.ip_address(args.new_ip)
    except ValueError as exc:
        print(f"[change_wuji_glove_ip] invalid --new-ip: {exc}", file=sys.stderr)
        return 2

    try:
        from wuji_sdk import ConnectOptions, SdkManager
    except ImportError as exc:
        print(
            "[change_wuji_glove_ip] wuji_sdk is not importable. Activate native env first:\n"
            "  source ~/miniconda3/etc/profile.d/conda.sh\n"
            "  conda activate dexproj",
            file=sys.stderr,
        )
        print(f"[change_wuji_glove_ip] import error: {exc}", file=sys.stderr)
        return 2

    manager = SdkManager.instance()

    if args.scan:
        print("[change_wuji_glove_ip] scanning devices...")
        for dev in manager.scan():
            print(
                "  "
                f"sn={getattr(dev, 'sn', '<unknown>')} "
                f"address={getattr(dev, 'address', '<unknown>')} "
                f"ip={getattr(dev, 'ip', '<unknown>')} "
                f"port={getattr(dev, 'port', '<unknown>')} "
                f"transport={getattr(dev, 'transport_type', '<unknown>')}"
            )

    if not args.skip_free_check and _ping_probe(args.new_ip):
        print(
            f"[change_wuji_glove_ip] new IP {args.new_ip} appears to be in use. "
            "Choose another IP or pass --skip-free-check if you are sure.",
            file=sys.stderr,
        )
        return 1

    options = ConnectOptions(timeout_ms=args.timeout_ms, retry_count=args.retry_count)
    print(f"[change_wuji_glove_ip] connecting to {args.address} as {args.device_name}...")
    try:
        glove = manager.connect(address=args.address, device_name=args.device_name, options=options)
    except Exception as exc:  # noqa: BLE001 - SDK raises extension exceptions.
        print(f"[change_wuji_glove_ip] SDK connect error: {exc}", file=sys.stderr)
        _print_connect_help(args.address)
        return 1

    current_ip = _get_resource_value(glove.ip())
    current_port = _get_resource_value(glove.port())
    print(f"[change_wuji_glove_ip] current glove IP: {current_ip}")
    print(f"[change_wuji_glove_ip] current glove port: {current_port}")

    if args.dry_run:
        print("[change_wuji_glove_ip] dry run: no changes written.")
        return 0

    print(f"[change_wuji_glove_ip] setting glove IP to {args.new_ip}...")
    glove.ip().set(args.new_ip)

    print(f"[change_wuji_glove_ip] setting glove port to {args.new_port}...")
    glove.port().set(args.new_port)

    print("[change_wuji_glove_ip] flushing device params to persistent storage...")
    glove.save_params()

    print("[change_wuji_glove_ip] write complete.")
    print("[change_wuji_glove_ip] If the glove disconnects, rescan/connect at the new address:")
    print(f"  {args.new_ip}:{args.new_port}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

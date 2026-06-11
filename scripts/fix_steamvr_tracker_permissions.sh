#!/usr/bin/env bash
set -euo pipefail

USER_NAME="${SUDO_USER:-${USER:-user}}"
RULE_FILE="/etc/udev/rules.d/99-steamvr-tracker-local.rules"

if [ "$(id -u)" -ne 0 ]; then
  exec sudo "$0" "$@"
fi

cat > "$RULE_FILE" <<'EOF'
# Local SteamVR / Vive Tracker permission override.
# Valve USB dongles, trackers, base stations, and SteamVR HID interfaces.
SUBSYSTEM=="usb", ATTR{idVendor}=="28de", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="28de", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usbmisc", ATTRS{idVendor}=="28de", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="28de", MODE="0666", TAG+="uaccess"

# HTC Vive devices.
SUBSYSTEM=="usb", ATTR{idVendor}=="0bb4", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0bb4", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usbmisc", ATTRS{idVendor}=="0bb4", MODE="0666", TAG+="uaccess"
EOF

udevadm control --reload-rules
udevadm trigger

for path in /dev/hidraw* /dev/usb/hiddev*; do
  [ -e "$path" ] || continue
  if udevadm info -a -n "$path" 2>/dev/null | grep -qi 'ATTRS{idVendor}=="28de"'; then
    chmod a+rw "$path" || true
    setfacl -m "u:${USER_NAME}:rw" "$path" 2>/dev/null || true
    echo "allowed $path"
  fi
done

echo "Installed $RULE_FILE"
echo "Unplug/replug all SteamVR dongles/trackers, then restart SteamVR."

"""OpenVR Tracker wrapper for ROS2 integration.

This module provides a wrapper around OpenVR to:
1. Connect to OpenVR system
2. Detect trackers by serial number
3. Get pose data with coordinate frame corrections
"""

import time
from typing import Dict, Optional
import numpy as np

try:
    import openvr
    OPENVR_AVAILABLE = True
except ImportError:
    OPENVR_AVAILABLE = False


def pose_matrix_to_numpy(pose_matrix) -> np.ndarray:
    """Convert OpenVR HmdMatrix34_t to numpy 4x4 transformation matrix.

    Args:
        pose_matrix: OpenVR HmdMatrix34_t structure

    Returns:
        np.ndarray: 4x4 transformation matrix
    """
    mat = np.eye(4, dtype=np.float32)
    for i in range(3):
        for j in range(3):
            mat[i, j] = pose_matrix.m[i][j]
        mat[i, 3] = pose_matrix.m[i][3]
    return mat


class OpenVRTrackerWrapper:
    """Wrapper for OpenVR Tracker data acquisition and coordinate correction.

    This class handles:
    - OpenVR system initialization
    - Tracker detection by serial number
    - Pose acquisition with coordinate frame corrections

    Args:
        tracker_serials: Dictionary mapping roles to tracker serial numbers
                        e.g., {'chest': 'LHR-xxx', 'right_wrist': 'LHR-yyy', 'left_wrist': 'LHR-zzz'}
        wrist_offset: Optional offset [x, y, z] in tracker's local frame (meters).
                     Applied before coordinate correction to map tracker position to wrist.

    Example:
        >>> wrapper = OpenVRTrackerWrapper({
        ...     'chest': 'LHR-12345678',
        ...     'right_wrist': 'LHR-87654321',
        ...     'left_wrist': 'LHR-11223344'
        ... }, wrist_offset=[0.0, 0.0, -0.05])
        >>> poses = wrapper.get_poses()
        >>> print(poses['chest'].shape)  # (4, 4)
    """

    # Pre-computed correction matrices:
    # - Chest: Rx(90°, flip xz) @ Rz(180°)
    # - Right wrist: Ry(90°) @ Rz(-90°) @ Ry(180°)
    # - Left wrist: Ry(90°) @ Rz(90°) @ Ry(180°)
    _CHEST_CORRECTION = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    _RIGHT_WRIST_CORRECTION = np.array([
        [0, 0, -1, 0],
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    _LEFT_WRIST_CORRECTION = np.array([
        [0, 0, -1, 0],
        [-1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)

    def __init__(self, tracker_serials: Dict[str, str], wrist_offset: Optional[list] = None):
        ## YOU HAVE TO RUN STEAMVR FIRST
        self.tracker_serials = tracker_serials
        self._wrist_offset = np.array(wrist_offset, dtype=np.float32) if wrist_offset else None
        self._vr_system: Optional[openvr.VRSystem] = None
        self._is_connected = False
        self._detected_trackers: Dict[int, str] = {}  # device_index -> role
        self._rediscover_interval_sec = 2.0
        self._log_interval_sec = 5.0
        self._last_rediscover_time = 0.0
        self._last_missing_log_time = 0.0
        self._last_invalid_log_time = 0.0
        self._active_tracking_universe = None

        # Initialize connection
        self._connect()

    def _connect(self):
        """Establish connection to OpenVR system."""
        try:
            print("Initializing OpenVR...")
            openvr.init(openvr.VRApplication_Other)
            self._vr_system = openvr.VRSystem()
            self._is_connected = True

            # Detect trackers by serial number
            self._detect_trackers_by_serial()

            print(f"Successfully connected to OpenVR!")
            print(f"Detected trackers: {self._detected_trackers}")

        except Exception as e:
            print(f"Failed to connect to OpenVR: {e}")
            self._is_connected = False
            raise

    def _get_device_serial(self, device_index: int) -> Optional[str]:
        """Get the serial number of a tracked device."""
        try:
            serial = self._vr_system.getStringTrackedDeviceProperty(
                device_index,
                openvr.Prop_SerialNumber_String
            )
            return serial
        except Exception:
            return None

    def _detect_trackers_by_serial(self, verbose: bool = True):
        """Detect trackers by serial number and map them to roles."""
        if self._vr_system is None:
            return

        serial_to_index = {}

        # Scan all devices and build serial -> index mapping
        for device_index in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self._vr_system.getTrackedDeviceClass(device_index)

            if device_class == openvr.TrackedDeviceClass_GenericTracker:
                serial = self._get_device_serial(device_index)
                if serial:
                    serial_to_index[serial] = device_index
                    if verbose:
                        print(f"Found tracker: {serial} at index {device_index}")

        detected_trackers = {}

        # Map configured serial numbers to roles
        for role, serial in self.tracker_serials.items():
            if serial in serial_to_index:
                detected_trackers[serial_to_index[serial]] = role
                if verbose:
                    print(f"Mapped {serial} -> {role}")
            else:
                if verbose:
                    print(f"Warning: Tracker {serial} ({role}) not found!")

        mapping_changed = detected_trackers != self._detected_trackers
        self._detected_trackers = detected_trackers
        self._last_rediscover_time = time.monotonic()
        if not verbose and mapping_changed:
            print(f"Updated OpenVR tracker mapping: {self._detected_trackers}")

    def _missing_roles(self) -> list[str]:
        detected_roles = set(self._detected_trackers.values())
        return [role for role in self.tracker_serials if role not in detected_roles]

    def _rediscover_trackers_if_due(self, force: bool = False):
        """Re-scan OpenVR device indexes after SteamVR changes tracking state."""
        now = time.monotonic()
        if not force and now - self._last_rediscover_time < self._rediscover_interval_sec:
            return
        self._detect_trackers_by_serial(verbose=False)

    def _tracking_universe_candidates(self) -> list[tuple[int, str]]:
        candidates = [(openvr.TrackingUniverseStanding, "Standing")]
        raw_universe = getattr(openvr, "TrackingUniverseRawAndUncalibrated", None)
        if raw_universe is not None:
            candidates.append((raw_universe, "RawAndUncalibrated"))
        candidates.append((openvr.TrackingUniverseSeated, "Seated"))
        return candidates

    def _get_tracker_pose(self, device_index: int) -> Optional[np.ndarray]:
        """Get raw pose matrix for a specific tracker device."""
        if self._vr_system is None:
            return None

        try:
            for tracking_universe, universe_name in self._tracking_universe_candidates():
                poses = self._vr_system.getDeviceToAbsoluteTrackingPose(
                    tracking_universe,
                    0,
                    openvr.k_unMaxTrackedDeviceCount
                )

                if device_index >= len(poses):
                    continue

                pose = poses[device_index]

                if not pose.bPoseIsValid:
                    continue

                if universe_name != self._active_tracking_universe:
                    print(f"Using OpenVR tracking universe: {universe_name}")
                    self._active_tracking_universe = universe_name
                return pose_matrix_to_numpy(pose.mDeviceToAbsoluteTracking)

            return None

        except Exception as e:
            print(f"Error getting pose for device {device_index}: {e}")
            return None

    def get_poses(self) -> Dict[str, Optional[np.ndarray]]:
        """Get corrected pose data for all detected trackers.

        Returns:
            dict: Dictionary containing corrected 4x4 transformation matrices:
                - 'chest': chest tracker pose (corrected)
                - 'right_wrist': right wrist pose (corrected)
                - 'left_wrist': left wrist pose (corrected)
                Values are None if tracker not detected or pose invalid.
        """
        if not self._is_connected or self._vr_system is None:
            raise RuntimeError("OpenVR system is not connected")

        missing_roles = self._missing_roles()
        if missing_roles:
            self._rediscover_trackers_if_due()
            missing_roles = self._missing_roles()
            now = time.monotonic()
            if missing_roles and now - self._last_missing_log_time >= self._log_interval_sec:
                print(f"Waiting for configured OpenVR trackers: {', '.join(missing_roles)}")
                self._last_missing_log_time = now

        poses = {role: None for role in self.tracker_serials}
        invalid_roles = []

        # Get raw poses for all detected trackers
        for device_index, role in self._detected_trackers.items():
            raw_pose = self._get_tracker_pose(device_index)

            if raw_pose is not None:
                # Apply wrist offset before coordinate correction (for wrist trackers only)
                if role in ('right_wrist', 'left_wrist') and self._wrist_offset is not None:
                    # Convert local offset to world frame and apply
                    rotation = raw_pose[:3, :3]
                    world_offset = rotation @ self._wrist_offset
                    raw_pose[:3, 3] += world_offset

                # Apply coordinate correction based on role
                if role == 'chest':
                    corrected_pose = raw_pose @ self._CHEST_CORRECTION
                elif role == 'right_wrist':
                    corrected_pose = raw_pose @ self._RIGHT_WRIST_CORRECTION
                elif role == 'left_wrist':
                    corrected_pose = raw_pose @ self._LEFT_WRIST_CORRECTION
                elif role in ('left_arm', 'right_arm'):
                    # Upper arm tracker: no coordinate correction, only used for direction vector projection
                    corrected_pose = raw_pose
                else:
                    corrected_pose = raw_pose

                poses[role] = corrected_pose
            else:
                poses[role] = None

                invalid_roles.append(role)

        if invalid_roles:
            self._rediscover_trackers_if_due()
            now = time.monotonic()
            if now - self._last_invalid_log_time >= self._log_interval_sec:
                print(f"Waiting for valid OpenVR poses: {', '.join(sorted(invalid_roles))}")
                self._last_invalid_log_time = now

        return poses

    def is_connected(self) -> bool:
        """Check if OpenVR system is connected."""
        return self._is_connected and self._vr_system is not None

    def list_available_trackers(self) -> Dict[str, int]:
        """List all available trackers with their serial numbers."""
        if not self._is_connected:
            raise RuntimeError("OpenVR not connected")

        trackers = {}
        for device_index in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self._vr_system.getTrackedDeviceClass(device_index)
            if device_class == openvr.TrackedDeviceClass_GenericTracker:
                serial = self._get_device_serial(device_index)
                if serial:
                    trackers[serial] = device_index

        return trackers

    def shutdown(self):
        """Shutdown OpenVR connection."""
        if self._is_connected:
            try:
                openvr.shutdown()
                print("OpenVR connection closed.")
            except Exception as e:
                print(f"Error during shutdown: {e}")
            finally:
                self._vr_system = None
                self._is_connected = False

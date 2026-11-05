"""Core data contracts and immutable message models for the GCS pipeline.

All models adhere to Section 4 of the architectural specification,
using lightweight Python dataclasses with slots for maximum throughput.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any

import numpy as np


class HotspotStatus(StrEnum):
    """Lifecycle status of a detected thermal hotspot."""

    CANDIDATE = "CANDIDATE"
    CONFIRMED = "CONFIRMED"
    DISMISSED = "DISMISSED"


class EventLevel(StrEnum):
    """Severity level for system and operational events."""

    INFO = "INFO"
    WARN = "WARN"
    CRITICAL = "CRITICAL"


class FlightMode(StrEnum):
    """Drone autopilot and control operational modes."""

    MANUAL = "MANUAL"
    TERRAIN_FOLLOW = "TERRAIN_FOLLOW"
    RTL = "RTL"


@dataclass(slots=True)
class DronePose:
    """Synchronized spatial pose snapshot for sensor frame geotagging."""

    lat: float
    lon: float
    alt_agl: float
    heading_deg: float


@dataclass(slots=True)
class Telemetry:
    """Full drone status packet emitted by the link interface at ~10 Hz."""

    timestamp: float
    lat: float
    lon: float
    alt_msl: float
    alt_agl: float
    heading_deg: float
    ground_speed: float
    vertical_speed: float
    battery_pct: float
    battery_v: float
    gps_fix: str  # "NO_FIX", "2D", "3D"
    gps_sats: int
    mode: str
    armed: bool
    link_quality_pct: float


@dataclass(slots=True)
class ThermalFrame:
    """Thermal array frame (e.g. MLX90640 32x24) with calibrated temperature array."""

    timestamp: float
    data: np.ndarray  # float32 shape (24, 32) in degrees Celsius
    sensor_fov_deg: tuple[float, float]  # (horizontal_fov, vertical_fov), e.g. (55.0, 35.0)
    drone_pose: DronePose


@dataclass(slots=True)
class VideoFrame:
    """RGB top-down or gimbal camera frame."""

    timestamp: float
    image: np.ndarray  # uint8 shape (H, W, 3) in RGB order


@dataclass(slots=True)
class ControlInput:
    """Normalized operator flight command in range [-1.0, 1.0] and mode flags."""

    forward: float = 0.0  # -1.0 (backward) to 1.0 (forward)
    right: float = 0.0  # -1.0 (left) to 1.0 (right)
    up: float = 0.0  # -1.0 (descend) to 1.0 (climb)
    yaw_rate: float = 0.0  # -1.0 (yaw left) to 1.0 (yaw right)
    toggle_terrain_follow: bool = False
    rtl: bool = False
    mark_hotspot: bool = False


@dataclass(slots=True)
class Hotspot:
    """Persistent ground-referenced thermal target identified by the detector."""

    id: str
    lat: float
    lon: float
    peak_temp_c: float
    mean_temp_c: float
    area_px: int
    first_seen: float
    last_seen: float
    detections: int
    confidence: float  # 0.0 to 1.0
    status: HotspotStatus = HotspotStatus.CANDIDATE
    note: str = ""
    thumbnail: np.ndarray | None = None  # optional (24, 32) float32 snapshot

    def to_dict(self) -> dict[str, Any]:
        """Serialize hotspot metadata to dictionary (excluding raw ndarray)."""
        return {
            "id": self.id,
            "lat": self.lat,
            "lon": self.lon,
            "peak_temp_c": round(self.peak_temp_c, 2),
            "mean_temp_c": round(self.mean_temp_c, 2),
            "area_px": self.area_px,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
            "detections": self.detections,
            "confidence": round(self.confidence, 3),
            "status": self.status.value,
            "note": self.note,
        }


@dataclass(slots=True)
class Event:
    """System message displayed in the operator log and saved to mission recording."""

    timestamp: float
    level: EventLevel
    message: str

"""Central configuration system with typed dataclasses and TOML overrides.

No hardcoded magic numbers exist outside this module.
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path


@dataclass(slots=True)
class GeoConfig:
    """Geographic anchor and coordinate system parameters."""

    # Reference origin: Believable Alpine SAR search area (Lauterbrunnen Valley)
    origin_lat: float = 46.5920
    origin_lon: float = 7.9080
    origin_alt_msl: float = 850.0  # meters above sea level


@dataclass(slots=True)
class WorldConfig:
    """Simulated terrain and ground environment specifications."""

    width_m: float = 400.0  # East-West size
    height_m: float = 400.0  # North-South size
    grid_resolution_m: float = 1.0  # 1 meter per terrain cell
    max_terrain_elevation_m: float = 60.0
    base_ground_temp_c: float = 16.0
    ground_temp_variation_c: float = 4.0
    rock_patch_min_temp_c: float = 28.0
    rock_patch_max_temp_c: float = 34.0
    default_seed: int = 42


@dataclass(slots=True)
class DroneConfig:
    """Drone flight dynamics, limits, and battery endurance parameters."""

    max_horizontal_speed_mps: float = 8.0  # m/s
    max_vertical_speed_mps: float = 3.0  # m/s
    max_yaw_rate_degps: float = 90.0  # deg/s
    accel_time_constant_s: float = 0.4  # smooth velocity response

    # Safety limits
    min_safe_agl_m: float = 3.0  # soft floor warning
    max_safe_agl_m: float = 40.0  # ceiling limit
    default_target_agl_m: float = 8.0
    geofence_margin_m: float = 20.0  # margin inside 400x400m boundary

    # Battery model (~20 minute flight duration)
    battery_capacity_mah: float = 5000.0
    battery_nominal_voltage: float = 14.8  # 4S LiPo
    battery_empty_voltage: float = 13.2
    battery_full_voltage: float = 16.8
    battery_warn_pct: float = 25.0  # warning threshold
    battery_rtl_pct: float = 15.0  # emergency RTL threshold
    hover_current_draw_a: float = 15.0  # gives ~20 min hover

    # Sensor noise
    gps_noise_std_m: float = 1.2
    compass_noise_std_deg: float = 1.5


@dataclass(slots=True)
class ThermalConfig:
    """MLX90640 32x24 thermal array sensor characteristics."""

    width_px: int = 32
    height_px: int = 24
    fov_h_deg: float = 55.0
    fov_v_deg: float = 35.0
    frame_rate_hz: float = 8.0
    noise_std_c: float = 0.25
    edge_noise_factor: float = 1.6  # slightly noisier outer pixels
    shutter_interval_s: float = 45.0  # periodic NUC calibration cycle


@dataclass(slots=True)
class VideoConfig:
    """Down-facing contextual RGB camera characteristics."""

    width_px: int = 640
    height_px: int = 480
    fov_h_deg: float = 70.0
    fov_v_deg: float = 52.5
    frame_rate_hz: float = 20.0


@dataclass(slots=True)
class DetectorConfig:
    """Hotspot detection, thresholding and validation criteria."""

    mad_multiplier_k: float = 4.0
    min_delta_c: float = 6.0
    absolute_floor_c: float = 24.0
    min_blob_area_px: int = 1
    max_blob_area_px: int = 18  # reject massive hot rock slopes
    human_temp_min_c: float = 26.0
    human_temp_max_c: float = 37.0


@dataclass(slots=True)
class TrackerConfig:
    """Spatial-temporal tracking and confirmation rules."""

    merge_radius_m: float = 3.0
    confirm_detections: int = 3
    confirm_min_interval_s: float = 2.0


@dataclass(slots=True)
class UIConfig:
    """UI layout, theming and presentation settings."""

    target_fps: int = 30
    presented_by: str = "SAR Payload Evaluation Team"
    palette_default: str = "ironbow"  # "ironbow", "inferno", "grayscale"
    high_dpi_scaling: bool = True


@dataclass(slots=True)
class AppConfig:
    """Master application configuration tree."""

    geo: GeoConfig = field(default_factory=GeoConfig)
    world: WorldConfig = field(default_factory=WorldConfig)
    drone: DroneConfig = field(default_factory=DroneConfig)
    thermal: ThermalConfig = field(default_factory=ThermalConfig)
    video: VideoConfig = field(default_factory=VideoConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    ui: UIConfig = field(default_factory=UIConfig)


def load_config(toml_path: str | Path | None = None) -> AppConfig:
    """Load configuration with optional TOML file overrides."""
    config = AppConfig()
    if toml_path is None:
        toml_path = Path("config.toml")
    else:
        toml_path = Path(toml_path)

    if toml_path.exists():
        try:
            with open(toml_path, "rb") as f:
                data = tomllib.load(f)

            if "geo" in data:
                for k, v in data["geo"].items():
                    if hasattr(config.geo, k):
                        setattr(config.geo, k, v)
            if "world" in data:
                for k, v in data["world"].items():
                    if hasattr(config.world, k):
                        setattr(config.world, k, v)
            if "drone" in data:
                for k, v in data["drone"].items():
                    if hasattr(config.drone, k):
                        setattr(config.drone, k, v)
            if "thermal" in data:
                for k, v in data["thermal"].items():
                    if hasattr(config.thermal, k):
                        setattr(config.thermal, k, v)
            if "video" in data:
                for k, v in data["video"].items():
                    if hasattr(config.video, k):
                        setattr(config.video, k, v)
            if "detector" in data:
                for k, v in data["detector"].items():
                    if hasattr(config.detector, k):
                        setattr(config.detector, k, v)
            if "tracker" in data:
                for k, v in data["tracker"].items():
                    if hasattr(config.tracker, k):
                        setattr(config.tracker, k, v)
            if "ui" in data:
                for k, v in data["ui"].items():
                    if hasattr(config.ui, k):
                        setattr(config.ui, k, v)
        except Exception as e:
            # Fall back to defaults on parse error
            print(f"[Config] Warning: Failed to parse {toml_path}: {e}")

    return config

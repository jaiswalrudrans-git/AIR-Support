"""Unit tests for models, config loader, and event bus."""

import numpy as np

from gcs.config import load_config
from gcs.core.bus import AppBus
from gcs.core.models import (
    ControlInput,
    DronePose,
    Event,
    EventLevel,
    Hotspot,
    HotspotStatus,
    Telemetry,
    ThermalFrame,
)


def test_models_instantiation_and_serialization() -> None:
    """Verify core models instantiate correctly with slotted dataclasses."""
    pose = DronePose(lat=46.592, lon=7.908, alt_agl=10.0, heading_deg=90.0)
    assert pose.lat == 46.592

    telem = Telemetry(
        timestamp=100.0,
        lat=46.592,
        lon=7.908,
        alt_msl=860.0,
        alt_agl=10.0,
        heading_deg=90.0,
        ground_speed=5.0,
        vertical_speed=0.0,
        battery_pct=88.5,
        battery_v=15.8,
        gps_fix="3D",
        gps_sats=14,
        mode="MANUAL",
        armed=True,
        link_quality_pct=98.0,
    )
    assert telem.gps_sats == 14

    data = np.full((24, 32), 22.0, dtype=np.float32)
    thermal = ThermalFrame(
        timestamp=100.0,
        data=data,
        sensor_fov_deg=(55.0, 35.0),
        drone_pose=pose,
    )
    assert thermal.data.shape == (24, 32)

    hotspot = Hotspot(
        id="H-001",
        lat=46.5921,
        lon=7.9082,
        peak_temp_c=34.2,
        mean_temp_c=31.5,
        area_px=3,
        first_seen=10.0,
        last_seen=12.0,
        detections=4,
        confidence=0.88,
        status=HotspotStatus.CONFIRMED,
    )
    d = hotspot.to_dict()
    assert d["id"] == "H-001"
    assert d["status"] == "CONFIRMED"

    ctrl = ControlInput(forward=0.5, right=-0.2)
    assert ctrl.forward == 0.5

    event = Event(timestamp=100.0, level=EventLevel.INFO, message="Link online")
    assert event.level == EventLevel.INFO


def test_bus_signal_dispatch() -> None:
    """Verify AppBus can emit and connect Qt signals."""
    bus = AppBus()
    received = []

    def on_event(ev: Event) -> None:
        received.append(ev.message)

    bus.event_emitted.connect(on_event)
    bus.event_emitted.emit(Event(timestamp=1.0, level=EventLevel.INFO, message="Bus test OK"))
    assert received == ["Bus test OK"]


def test_config_loader_defaults() -> None:
    """Verify config loads default values without exceptions."""
    cfg = load_config()
    assert cfg.thermal.width_px == 32
    assert cfg.thermal.height_px == 24
    assert cfg.thermal.fov_h_deg == 55.0
    assert cfg.drone.default_target_agl_m == 8.0

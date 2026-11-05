"""Global mission state container holding tracking data, trail, and mission stats."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

from gcs.core.models import (
    FlightMode,
    Hotspot,
    HotspotStatus,
    Telemetry,
    ThermalFrame,
)


@dataclass
class MissionState:
    """Consolidated runtime state of the active mission."""

    mode: FlightMode = FlightMode.MANUAL
    armed: bool = True
    latest_telemetry: Telemetry | None = None
    latest_thermal: ThermalFrame | None = None

    # Persistent spatial tracking
    hotspots: dict[str, Hotspot] = field(default_factory=dict)
    selected_hotspot_id: str | None = None

    # Flight path trail: list of (east, north, alt_agl, timestamp)
    trail: list[tuple[float, float, float, float]] = field(default_factory=list)

    # Mission statistics
    mission_start_time: float = field(default_factory=time.time)
    min_agl_m: float = 999.0
    max_agl_m: float = 0.0
    total_distance_m: float = 0.0

    # Recording state
    is_recording: bool = False
    recording_dir: Path | None = None

    def update_telemetry(self, telem: Telemetry, east: float, north: float) -> None:
        """Update telemetry snapshot, flight trail, and altitude extremes."""
        self.latest_telemetry = telem

        # Track altitude limits
        if telem.alt_agl < self.min_agl_m:
            self.min_agl_m = telem.alt_agl
        if telem.alt_agl > self.max_agl_m:
            self.max_agl_m = telem.alt_agl

        # Track trail and distance
        if self.trail:
            last_e, last_n, _, _ = self.trail[-1]
            dx = east - last_e
            dy = north - last_n
            step = (dx * dx + dy * dy) ** 0.5
            if step >= 1.0:  # append every 1 meter movement
                self.trail.append((east, north, telem.alt_agl, telem.timestamp))
                self.total_distance_m += step
        else:
            self.trail.append((east, north, telem.alt_agl, telem.timestamp))

    def get_confirmed_hotspots(self) -> list[Hotspot]:
        """Return all confirmed hotspots."""
        return [h for h in self.hotspots.values() if h.status == HotspotStatus.CONFIRMED]

    def reset_for_scenario(self) -> None:
        """Clear dynamic flight trail and hotspots for new scenario."""
        self.hotspots.clear()
        self.selected_hotspot_id = None
        self.trail.clear()
        self.min_agl_m = 999.0
        self.max_agl_m = 0.0
        self.total_distance_m = 0.0
        self.mission_start_time = time.time()

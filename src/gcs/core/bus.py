"""Central Qt-signal event bus for decoupled communication across threads.

UI widgets and background links subscribe to signals without tight coupling.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal


class AppBus(QObject):
    """Singleton-friendly signal bus connecting UI, Link, Detector, and Storage."""

    # Real-time sensor and telemetry streams
    telemetry_updated = Signal(object)  # Telemetry
    thermal_updated = Signal(object)  # ThermalFrame
    video_updated = Signal(object)  # VideoFrame

    # Hotspot detection and tracking lifecycle
    hotspot_detected = Signal(object)  # Hotspot (new candidate)
    hotspot_updated = Signal(object)  # Hotspot (confirmed or updated stats)
    hotspot_dismissed = Signal(str)  # Hotspot id
    hotspot_selected = Signal(str)  # Hotspot id (for cross-view linking)

    # Operational events and link state
    event_emitted = Signal(object)  # Event
    link_status_changed = Signal(str)  # Link status message
    flight_mode_changed = Signal(str)  # Mode name ("MANUAL", etc.)

    # Outbound operator commands
    control_command = Signal(object)  # ControlInput
    command_toggle_terrain_follow = Signal()
    command_rtl = Signal()
    command_mark_hotspot = Signal()

    # System and mission control
    recording_state_changed = Signal(bool)  # is_recording
    request_recording_toggle = Signal()
    request_scenario_reset = Signal(str)  # scenario name
    request_survey_pattern = Signal()  # autonomous lawnmower toggle
    request_emergency_stop = Signal()


# Global default bus instance
_BUS_INSTANCE: AppBus | None = None


def get_bus() -> AppBus:
    """Retrieve or create the global application event bus."""
    global _BUS_INSTANCE
    if _BUS_INSTANCE is None:
        _BUS_INSTANCE = AppBus()
    return _BUS_INSTANCE

"""Abstract DroneLink interface forming the strict Hardware Seam of the GCS.

Per Section 3 of the specification, the UI, detector, tracker, and storage subsystems
communicate exclusively through this interface. They never import simulator modules.
"""

from __future__ import annotations

from abc import abstractmethod

from PySide6.QtCore import QObject, Signal

from gcs.core.models import ControlInput, Telemetry, ThermalFrame, VideoFrame


class DroneLink(QObject):
    """Abstract communication link between GCS and drone hardware or simulator."""

    # Real-time telemetry and imagery signals emitted to the application bus
    telemetry_received = Signal(Telemetry)
    thermal_received = Signal(ThermalFrame)
    video_received = Signal(VideoFrame)
    link_status_changed = Signal(str)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)

    @abstractmethod
    def connect_link(self) -> bool:
        """Establish physical serial/radio connection or start simulation engine."""
        raise NotImplementedError

    @abstractmethod
    def disconnect_link(self) -> None:
        """Close connection or stop simulation engine."""
        raise NotImplementedError

    @abstractmethod
    def send_control(self, control: ControlInput) -> None:
        """Transmit operator flight commands down the link."""
        raise NotImplementedError

    @abstractmethod
    def is_connected(self) -> bool:
        """Check whether telemetry carrier is active."""
        raise NotImplementedError

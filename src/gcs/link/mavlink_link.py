"""MAVLink hardware link implementation stub for Pixhawk / ArduPilot / PX4 autopilots.

This module documents how a real flight controller connects via pymavlink over
a telemetry radio (e.g. SiK 915MHz/433MHz or UDP/TCP Wi-Fi link).
"""

from __future__ import annotations

from PySide6.QtCore import QObject

from gcs.core.models import ControlInput
from gcs.link.base import DroneLink


class MavlinkLink(DroneLink):
    """Production MAVLink adapter interfacing Pixhawk / ArduPilot / PX4 flight controllers."""

    def __init__(
        self,
        connection_str: str = "udpin:0.0.0.0:14550",
        baudrate: int = 57600,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._connection_str = connection_str
        self._baudrate = baudrate
        self._is_connected = False

    def connect_link(self) -> bool:
        """Connect to MAVLink endpoint using pymavlink.mavutil.mavlink_connection."""
        raise NotImplementedError(
            "MavlinkLink is a hardware seam stub. To connect physical drone: "
            "1. Install pymavlink: pip install pymavlink\n"
            "2. Open mavutil.mavlink_connection(self._connection_str, baud=self._baudrate)\n"
            "3. Spawn a background listener thread parsing ATTITUDE, GLOBAL_POSITION_INT, "
            "SYS_STATUS, and HEARTBEAT messages.\n"
            "4. Emit self.telemetry_received.emit(telemetry) at 10 Hz."
        )

    def disconnect_link(self) -> None:
        """Terminate MAVLink session and close socket/serial port."""
        self._is_connected = False
        self.link_status_changed.emit("MAVLink Disconnected")

    def send_control(self, control: ControlInput) -> None:
        """Translate ControlInput into MAVLink MANUAL_CONTROL or SET_POSITION_TARGET_LOCAL_NED."""
        raise NotImplementedError(
            "MAVLink control requires active flight controller link. "
            "Send mav.manual_control_send() or SET_ATTITUDE_TARGET commands."
        )

    def is_connected(self) -> bool:
        return self._is_connected

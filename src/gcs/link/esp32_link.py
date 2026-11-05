"""ESP32 payload receiver stub for MLX90640 thermal array and GPS over UDP or USB-UART.

This module documents how the real ESP32 hardware payload streams calibrated
32x24 float32 thermal arrays and NMEA GPS sentences into the GCS.
"""

from __future__ import annotations

from PySide6.QtCore import QObject

from gcs.core.models import ControlInput
from gcs.link.base import DroneLink


class Esp32Link(DroneLink):
    """Payload receiver interfacing the ESP32 + MLX90640 thermal sensor array."""

    def __init__(
        self,
        port: str = "COM3",
        baudrate: int = 921600,
        udp_port: int = 5005,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._port = port
        self._baudrate = baudrate
        self._udp_port = udp_port
        self._is_connected = False

    def connect_link(self) -> bool:
        """Connect to ESP32 payload via USB serial or UDP Wi-Fi socket."""
        raise NotImplementedError(
            "Esp32Link is a hardware seam stub. To connect physical thermal payload:\n"
            "1. Open UDP socket listening on udp_port or open pyserial at 921600 baud.\n"
            "2. ESP32 firmware transmits packet: [HEADER 0xAA 0x55] [TIMESTAMP 4B] "
            "[GPS LAT, LON, ALT 12B] [RAW PIXELS 32x24 float16/float32 = 1536B] [CRC16 2B].\n"
            "3. Deserialize frame into ThermalFrame with calibrated degrees Celsius.\n"
            "4. Emit self.thermal_received.emit(thermal_frame) at ~8 Hz."
        )

    def disconnect_link(self) -> None:
        """Close payload receiver port."""
        self._is_connected = False
        self.link_status_changed.emit("ESP32 Payload Disconnected")

    def send_control(self, control: ControlInput) -> None:
        """The ESP32 payload link is a receive-only sensor downlink."""
        pass

    def is_connected(self) -> bool:
        return self._is_connected

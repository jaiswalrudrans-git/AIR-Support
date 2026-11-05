"""Gamepad/joystick flight controller interface stub.

Implements the interface contract for future USB/Bluetooth RC flight controllers
(e.g. FrSky Taranis, Radiomaster, or standard Xbox/PlayStation gamepads).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from gcs.core.models import ControlInput


class BaseGamepad(ABC):
    """Abstract interface contract for joystick and RC transmitter devices."""

    @abstractmethod
    def connect(self, device_id: int = 0) -> bool:
        """Connect to physical gamepad device."""
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect and release device handle."""
        raise NotImplementedError

    @abstractmethod
    def poll(self) -> ControlInput:
        """Sample stick positions and return normalized ControlInput."""
        raise NotImplementedError


class GamepadStub(BaseGamepad):
    """Stub implementation documenting future PyGame/SDL gamepad integration."""

    def __init__(self) -> None:
        self._connected = False

    def connect(self, device_id: int = 0) -> bool:
        """Simulate hardware probe."""
        # Hardware not yet connected in offline simulation phase
        self._connected = False
        return False

    def disconnect(self) -> None:
        self._connected = False

    def poll(self) -> ControlInput:
        """Return zero input when no physical gamepad is attached."""
        return ControlInput()

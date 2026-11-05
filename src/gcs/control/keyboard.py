"""Keyboard input processor translating operator key presses into normalized flight controls."""

from __future__ import annotations

from PySide6.QtCore import Qt

from gcs.core.models import ControlInput


class KeyboardController:
    """Tracks active keyboard state and generates continuous ControlInput frames."""

    def __init__(self) -> None:
        self._keys_down: set[int] = set()

    def handle_key_press(self, key: int) -> None:
        """Record key down event."""
        self._keys_down.add(key)

    def handle_key_release(self, key: int) -> None:
        """Record key up event."""
        self._keys_down.discard(key)

    def clear(self) -> None:
        """Clear all pressed keys (e.g. when window loses focus)."""
        self._keys_down.clear()

    def get_control_input(self) -> ControlInput:
        """Synthesize current flight commands from held movement keys."""
        fwd = 0.0
        right = 0.0
        up = 0.0
        yaw = 0.0

        # Longitudinal: W / S
        if Qt.Key.Key_W in self._keys_down:
            fwd += 1.0
        if Qt.Key.Key_S in self._keys_down:
            fwd -= 1.0

        # Lateral: A / D
        if Qt.Key.Key_D in self._keys_down:
            right += 1.0
        if Qt.Key.Key_A in self._keys_down:
            right -= 1.0

        # Vertical: R / F or Space / Shift
        if Qt.Key.Key_R in self._keys_down or Qt.Key.Key_Space in self._keys_down:
            up += 1.0
        if Qt.Key.Key_F in self._keys_down or Qt.Key.Key_Shift in self._keys_down:
            up -= 1.0

        # Yaw: Q / E
        if Qt.Key.Key_E in self._keys_down:
            yaw += 1.0
        if Qt.Key.Key_Q in self._keys_down:
            yaw -= 1.0

        return ControlInput(
            forward=max(-1.0, min(1.0, fwd)),
            right=max(-1.0, min(1.0, right)),
            up=max(-1.0, min(1.0, up)),
            yaw_rate=max(-1.0, min(1.0, yaw)),
        )

"""Smoke test verifying module import and basic window construction."""

import os

from PySide6.QtWidgets import QApplication

from gcs.app import MainWindow

# Ensure headless execution in CI / automated test runs
os.environ["QT_QPA_PLATFORM"] = "offscreen"


def test_main_window_title(qapp: QApplication) -> None:
    """Window initializes with correct title and minimum dimensions."""
    window = MainWindow()
    assert window.windowTitle() == "Thermal SAR Ground Station"
    assert window.width() >= 960
    assert window.height() >= 600
    window.close()

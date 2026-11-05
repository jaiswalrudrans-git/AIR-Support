"""Main application entry point for Thermal SAR Ground Station."""

import sys

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget


class MainWindow(QMainWindow):
    """Main ground control station window."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Thermal SAR Ground Station")
        self.resize(1280, 800)
        self.setMinimumSize(960, 600)

        # Apply dark background
        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor("#121417"))
        palette.setColor(QPalette.ColorRole.WindowText, QColor("#E0E6ED"))
        self.setPalette(palette)

        central = QWidget(self)
        central.setStyleSheet("background-color: #121417;")
        self.setCentralWidget(central)


def main(argv: list[str] | None = None) -> int:
    """Run the GCS desktop application."""
    if argv is None:
        argv = sys.argv

    # Support high DPI displays
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication.instance()
    if app is None:
        app = QApplication(argv)

    window = MainWindow()
    window.show()

    # If run in non-interactive verification mode
    if "--smoke-test" in argv:
        window.close()
        return 0

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

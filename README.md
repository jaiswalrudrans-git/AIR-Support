# Thermal SAR Ground Control Station (GCS)

Desktop Ground Control Station (GCS) built with PySide6 (Qt 6) for a low-cost Search and Rescue (SAR) thermal drone payload.

## Quick Tasks

| Task | Command |
|---|---|
| **Run GCS** | `.\.venv\Scripts\python -m gcs` |
| **Run Tests** | `.\.venv\Scripts\pytest` |
| **Run Linter** | `.\.venv\Scripts\ruff check .` |
| **Format Code** | `.\.venv\Scripts\ruff format .` |
| **Type Check** | `.\.venv\Scripts\mypy src` |
| **Build Executable**| `.\.venv\Scripts\pyinstaller --noconfirm --onedir --windowed --name "ThermalSAR-GCS" src/gcs/app.py` |

## Documentation
- [Decisions Log](docs/DECISIONS.md)
- [System Architecture](docs/ARCHITECTURE.md)
- [Hardware Seam Specification](docs/HARDWARE_SEAM.md)
- [External Panel Demo Script](docs/DEMO_SCRIPT.md)

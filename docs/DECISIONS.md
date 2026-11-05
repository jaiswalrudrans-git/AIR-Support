# Technical Decisions Log

This document records key engineering choices, trade-offs, and design rationale made during the development of the Thermal SAR Ground Control Station.

---

### DEC-001: Packaging and Execution Entry Points
- **Context:** Need standard invocation via `python -m gcs`, entry point `src/gcs/app.py:main`, and console script `thermal-gcs`.
- **Decision:** Use `src/` layout with `src/gcs/__main__.py` delegating to `src/gcs/app.py:main`. Configure `pyproject.toml` with `src` in `setuptools.packages.find` and `pythonpath = ["src"]` in pytest.
- **Rationale:** Ensures clean import hygiene without polluting the workspace root with top-level package namespace collisions.

### DEC-002: Graphics Rendering Approach (Pure QPainter)
- **Context:** Need high-performance, dark avionics-styled custom views (moving map, artificial horizon gauges, HUD overlays) running offline at smooth 30 FPS without web browser dependencies.
- **Decision:** Implement map rendering, gauges, and HUD overlays using native Qt `QPainter` widgets with double-buffering. Avoid external heavy web or WebEngine stacks.
- **Rationale:** Guarantees sub-second startup, deterministic sub-millisecond paint latency, clean dark-theme aesthetic consistency, and zero runtime network requirements.

### DEC-003: Coordinate Frames and Map Projection
- **Context:** The drone flies in a 400x400m search area. Calculations for footprint projection, geotagging, and trail logging need to be fast and numerically stable.
- **Decision:** Use an East-North-Up (ENU) tangent-plane approximation anchored at a configurable WGS84 origin (default: mountain valley 46.5000° N, 8.0000° E).
- **Rationale:** Over a 400m area, flat-Earth ENU tangent projection error is below 1 mm, while allowing fast vector arithmetic in standard SI meters.

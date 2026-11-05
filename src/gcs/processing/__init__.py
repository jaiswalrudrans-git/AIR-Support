"""Thermal image processing, anomaly detection, geotagging, and spatial tracking."""

from gcs.processing.colormap import apply_colormap, get_lut
from gcs.processing.detector import RawDetection, ThermalDetector
from gcs.processing.geotag import geotag_detection
from gcs.processing.tracker import HotspotTracker

__all__ = [
    "HotspotTracker",
    "RawDetection",
    "ThermalDetector",
    "apply_colormap",
    "geotag_detection",
    "get_lut",
]

"""Spatiotemporal hotspot tracking, multi-pass confirmation, and clustering."""

from __future__ import annotations

import math

from gcs.config import GeoConfig, TrackerConfig
from gcs.core.geo import enu_to_latlon
from gcs.core.models import Hotspot, HotspotStatus, ThermalFrame
from gcs.processing.detector import RawDetection
from gcs.processing.geotag import geotag_detection


class HotspotTracker:
    """Fuses multi-frame thermal anomalies into ground-referenced persistent hotspots."""

    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self.hotspots: dict[str, Hotspot] = {}
        self._metric_coords: dict[str, tuple[float, float]] = {}  # id -> (east, north)
        self._next_id_counter = 1

    def process_detection(
        self,
        detection: RawDetection,
        frame: ThermalFrame,
        geo_config: GeoConfig,
    ) -> tuple[Hotspot, bool, bool]:
        """Incorporate a detected anomaly into the spatial tracker.

        Returns:
            (hotspot, is_new_hotspot, just_confirmed)
        """
        target_lat, target_lon, ge, gn = geotag_detection(detection, frame, geo_config)

        # 1. Search for nearest existing hotspot within merge radius
        best_match_id: str | None = None
        min_distance_m = float("inf")

        for h_id, (he, hn) in self._metric_coords.items():
            dist = math.hypot(ge - he, gn - hn)
            if dist < min_distance_m:
                min_distance_m = dist
                if dist <= self.config.merge_radius_m:
                    best_match_id = h_id

        # 2. Existing hotspot match -> Merge and update running averages
        if best_match_id is not None:
            hotspot = self.hotspots[best_match_id]
            he, hn = self._metric_coords[best_match_id]

            # Running weighted centroid update
            n_hits = hotspot.detections
            new_he = (he * n_hits + ge) / (n_hits + 1)
            new_hn = (hn * n_hits + gn) / (n_hits + 1)
            self._metric_coords[best_match_id] = (new_he, new_hn)

            new_lat, new_lon, _ = enu_to_latlon(
                east=new_he,
                north=new_hn,
                up=0.0,
                origin_lat=geo_config.origin_lat,
                origin_lon=geo_config.origin_lon,
                origin_alt_msl=geo_config.origin_alt_msl,
            )

            hotspot.lat = new_lat
            hotspot.lon = new_lon
            hotspot.peak_temp_c = max(hotspot.peak_temp_c, detection.peak_temp_c)
            hotspot.mean_temp_c = (hotspot.mean_temp_c * n_hits + detection.mean_temp_c) / (
                n_hits + 1
            )
            hotspot.area_px = max(hotspot.area_px, detection.area_px)
            hotspot.detections += 1
            hotspot.last_seen = frame.timestamp
            hotspot.confidence = min(0.99, hotspot.confidence + 0.08)

            just_confirmed = False
            # Multi-pass confirmation criteria (>=3 detections spanning >=2 seconds)
            time_span_s = frame.timestamp - hotspot.first_seen
            if (
                hotspot.status == HotspotStatus.CANDIDATE
                and hotspot.detections >= self.config.confirm_detections
                and time_span_s >= self.config.confirm_min_interval_s
            ):
                hotspot.status = HotspotStatus.CONFIRMED
                just_confirmed = True

            return hotspot, False, just_confirmed

        # 3. No match -> Instantiate new CANDIDATE hotspot
        h_id = f"H-{self._next_id_counter:03d}"
        self._next_id_counter += 1

        new_hotspot = Hotspot(
            id=h_id,
            lat=target_lat,
            lon=target_lon,
            peak_temp_c=detection.peak_temp_c,
            mean_temp_c=detection.mean_temp_c,
            area_px=detection.area_px,
            first_seen=frame.timestamp,
            last_seen=frame.timestamp,
            detections=1,
            confidence=detection.confidence,
            status=HotspotStatus.CANDIDATE,
            thumbnail=detection.thumbnail,
        )

        self.hotspots[h_id] = new_hotspot
        self._metric_coords[h_id] = (ge, gn)
        return new_hotspot, True, False

    def confirm_hotspot(self, hotspot_id: str) -> bool:
        """Manually promote candidate hotspot to CONFIRMED."""
        if hotspot_id in self.hotspots:
            self.hotspots[hotspot_id].status = HotspotStatus.CONFIRMED
            return True
        return False

    def dismiss_hotspot(self, hotspot_id: str) -> bool:
        """Mark hotspot as DISMISSED."""
        if hotspot_id in self.hotspots:
            self.hotspots[hotspot_id].status = HotspotStatus.DISMISSED
            return True
        return False

    def add_note(self, hotspot_id: str, note: str) -> bool:
        """Attach operator field observation note to hotspot."""
        if hotspot_id in self.hotspots:
            self.hotspots[hotspot_id].note = note
            return True
        return False

    def get_hotspot_metric_position(self, hotspot_id: str) -> tuple[float, float] | None:
        """Retrieve local ENU ground coordinates (east, north) for hotspot."""
        return self._metric_coords.get(hotspot_id)

    def clear(self) -> None:
        """Clear all hotspots and reset counter."""
        self.hotspots.clear()
        self._metric_coords.clear()
        self._next_id_counter = 1

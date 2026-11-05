"""Thermal anomaly detector using median/MAD background estimation and connected components."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import label

from gcs.config import DetectorConfig
from gcs.core.models import ThermalFrame


@dataclass(slots=True)
class RawDetection:
    """Single-frame 2D pixel anomaly detected before spatial geotagging."""

    px: float
    py: float
    peak_temp_c: float
    mean_temp_c: float
    area_px: int
    confidence: float
    thumbnail: np.ndarray  # cropped (24, 32) snapshot


class ThermalDetector:
    """Robust hotspot detector running on calibrated float32 thermal arrays."""

    def __init__(self, config: DetectorConfig | None = None) -> None:
        self.config = config or DetectorConfig()
        # 8-connectivity neighborhood
        self._struct = np.ones((3, 3), dtype=int)

    def process_frame(self, frame: ThermalFrame) -> list[RawDetection]:
        """Detect candidate warm anomalies in a single thermal frame.

        Algorithm:
        1. Robust background estimation via Median and Median Absolute Deviation (MAD).
        2. Dynamic thresholding: max(median + k*MAD, median + min_delta_c, floor_c).
        3. 8-connected component labeling and area filtering (reject massive hot rocks).
        4. Multi-criteria confidence scoring based on human temperature profile (26-37°C).
        """
        data = frame.data
        bg_median = float(np.median(data))
        mad = float(np.median(np.abs(data - bg_median)))
        mad = max(0.25, mad)

        # Dynamic thresholding
        thresh = max(
            bg_median + self.config.mad_multiplier_k * mad,
            bg_median + self.config.min_delta_c,
            self.config.absolute_floor_c,
        )

        binary_mask = data >= thresh
        if not np.any(binary_mask):
            return []

        labeled_array, num_features = label(binary_mask, structure=self._struct)
        detections: list[RawDetection] = []

        for feat_idx in range(1, num_features + 1):
            mask = labeled_array == feat_idx
            area_px = int(np.sum(mask))

            # Area filtering: reject tiny noise blips or massive hot rock faces
            if area_px < self.config.min_blob_area_px or area_px > self.config.max_blob_area_px:
                continue

            blob_temps = data[mask]
            peak_temp = float(np.max(blob_temps))
            mean_temp = float(np.mean(blob_temps))

            # Weighted centroid
            weights = np.maximum(0.1, blob_temps - thresh)
            y_indices, x_indices = np.where(mask)
            total_w = float(np.sum(weights))
            cx = float(np.sum(x_indices * weights) / total_w)
            cy = float(np.sum(y_indices * weights) / total_w)

            # Confidence scoring
            # 1. Thermal contrast above background (0 to 1)
            contrast_score = min(1.0, (peak_temp - bg_median) / 12.0)

            # 2. Human biological temperature profile fit (26°C to 37°C)
            if self.config.human_temp_min_c <= peak_temp <= self.config.human_temp_max_c:
                temp_fit = 1.0
            elif peak_temp > self.config.human_temp_max_c:
                # Strong penalty for extreme heat (car engines at 52C or fires at 80C)
                temp_fit = float(np.exp(-0.06 * (peak_temp - self.config.human_temp_max_c)))
            else:
                # Cooler temperatures
                temp_fit = float(np.exp(-0.25 * (self.config.human_temp_min_c - peak_temp)))

            # 3. Compact spatial size fit (1 to 5 pixels is ideal for 8m AGL)
            if 1 <= area_px <= 4:
                size_score = 1.0
            elif area_px <= 8:
                size_score = 0.8
            else:
                size_score = 0.5

            confidence = 0.45 * temp_fit + 0.35 * contrast_score + 0.20 * size_score
            confidence = max(0.1, min(0.99, confidence))

            detections.append(
                RawDetection(
                    px=cx,
                    py=cy,
                    peak_temp_c=peak_temp,
                    mean_temp_c=mean_temp,
                    area_px=area_px,
                    confidence=confidence,
                    thumbnail=data.copy(),
                )
            )

        return detections

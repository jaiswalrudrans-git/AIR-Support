"""Thermal pseudo-color palettes and 256-entry lookup tables (LUTs).

Provides calibrated Ironbow, Inferno, and Grayscale palettes for thermal visualisation.
"""

from __future__ import annotations

import numpy as np


def _create_ironbow_lut() -> np.ndarray:
    """Generate authentic 256-entry Ironbow (FLIR-style) RGB lookup table."""
    # Key color control points: (position 0-255, R, G, B)
    points = [
        (0, 0, 0, 0),  # Black
        (35, 20, 0, 80),  # Dark purple
        (85, 30, 20, 160),  # Blue / indigo
        (140, 180, 20, 120),  # Magenta / violet
        (190, 240, 90, 20),  # Deep orange
        (225, 250, 220, 50),  # Golden yellow
        (255, 255, 255, 255),  # Pure white
    ]

    lut = np.zeros((256, 3), dtype=np.uint8)
    for i in range(len(points) - 1):
        idx0, r0, g0, b0 = points[i]
        idx1, r1, g1, b1 = points[i + 1]
        span = idx1 - idx0
        for step in range(span):
            idx = idx0 + step
            t = step / span
            lut[idx, 0] = int(r0 + t * (r1 - r0))
            lut[idx, 1] = int(g0 + t * (g1 - g0))
            lut[idx, 2] = int(b0 + t * (b1 - b0))

    lut[255] = [255, 255, 255]
    return lut


def _create_inferno_lut() -> np.ndarray:
    """Generate 256-entry Inferno perceptually uniform RGB lookup table."""
    points = [
        (0, 0, 0, 4),
        (40, 30, 12, 70),
        (90, 85, 15, 109),
        (145, 160, 42, 80),
        (195, 225, 100, 30),
        (235, 250, 180, 50),
        (255, 252, 255, 164),
    ]

    lut = np.zeros((256, 3), dtype=np.uint8)
    for i in range(len(points) - 1):
        idx0, r0, g0, b0 = points[i]
        idx1, r1, g1, b1 = points[i + 1]
        span = idx1 - idx0
        for step in range(span):
            idx = idx0 + step
            t = step / span
            lut[idx, 0] = int(r0 + t * (r1 - r0))
            lut[idx, 1] = int(g0 + t * (g1 - g0))
            lut[idx, 2] = int(b0 + t * (b1 - b0))

    lut[255] = [252, 255, 164]
    return lut


def _create_grayscale_lut() -> np.ndarray:
    """Generate 256-entry linear Grayscale (White-Hot) lookup table."""
    indices = np.arange(256, dtype=np.uint8)
    return np.column_stack([indices, indices, indices])


# Precomputed lookup tables
PALETTES: dict[str, np.ndarray] = {
    "ironbow": _create_ironbow_lut(),
    "inferno": _create_inferno_lut(),
    "grayscale": _create_grayscale_lut(),
}


def get_lut(palette_name: str = "ironbow") -> np.ndarray:
    """Retrieve 256x3 uint8 RGB colormap LUT by name."""
    return PALETTES.get(palette_name.lower(), PALETTES["ironbow"])


def apply_colormap(
    data: np.ndarray,
    vmin: float | None = None,
    vmax: float | None = None,
    palette_name: str = "ironbow",
) -> np.ndarray:
    """Map a 2D float32 temperature array to an (H, W, 3) RGB uint8 image."""
    if vmin is None:
        vmin = float(data.min())
    if vmax is None:
        vmax = float(data.max())

    # Avoid zero division
    span = vmax - vmin
    if span < 0.1:
        span = 0.1

    normalized = np.clip((data - vmin) / span * 255.0, 0.0, 255.0).astype(np.uint8)
    lut = get_lut(palette_name)
    return lut[normalized]

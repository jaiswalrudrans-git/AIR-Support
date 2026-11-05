"""Simulated 400x400m terrain world with elevation, hillshade, and thermal fields.

Uses pure NumPy layered gradient/value noise to create realistic terrain
without external terrain dependencies.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def _smooth_noise_2d(
    width: int,
    height: int,
    grid_size: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """Generate smooth 2D value noise using bicubic Hermite interpolation."""
    gw = math.ceil(width / grid_size) + 1
    gh = math.ceil(height / grid_size) + 1
    coarse = rng.uniform(0.0, 1.0, (gh, gw)).astype(np.float32)

    # Pixel coordinate grids
    y = np.linspace(0, height / grid_size, height, endpoint=False, dtype=np.float32)
    x = np.linspace(0, width / grid_size, width, endpoint=False, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)

    x0 = np.floor(xx).astype(int)
    y0 = np.floor(yy).astype(int)
    x1 = x0 + 1
    y1 = y0 + 1

    fx = xx - x0
    fy = yy - y0

    # Quintic Hermite interpolant: 6t^5 - 15t^4 + 10t^3
    sx = fx * fx * fx * (fx * (fx * 6.0 - 15.0) + 10.0)
    sy = fy * fy * fy * (fy * (fy * 6.0 - 15.0) + 10.0)

    top = (1.0 - sx) * coarse[y0, x0] + sx * coarse[y0, x1]
    bottom = (1.0 - sx) * coarse[y1, x0] + sx * coarse[y1, x1]
    return (1.0 - sy) * top + sy * bottom


def generate_heightmap(
    width_px: int = 400,
    height_px: int = 400,
    max_elevation_m: float = 60.0,
    seed: int = 42,
) -> np.ndarray:
    """Generate multi-octave layered terrain elevation in range [0, max_elevation_m]."""
    rng = np.random.default_rng(seed)

    # Octaves: broad valley/ridges down to small boulders
    octaves = [
        (120, 0.55),
        (60, 0.25),
        (25, 0.12),
        (10, 0.08),
    ]

    total = np.zeros((height_px, width_px), dtype=np.float32)
    norm = 0.0
    for grid_size, weight in octaves:
        total += weight * _smooth_noise_2d(width_px, height_px, grid_size, rng)
        norm += weight

    total /= norm
    # Normalize to [0.0, 1.0] and scale
    min_v, max_v = total.min(), total.max()
    if max_v > min_v:
        total = (total - min_v) / (max_v - min_v)

    return total * max_elevation_m


def compute_hillshade(
    heightmap: np.ndarray,
    cell_size_m: float = 1.0,
    sun_azimuth_deg: float = 315.0,  # North-West
    sun_altitude_deg: float = 45.0,
) -> np.ndarray:
    """Compute 8-bit grayscale hillshade from heightmap using USGS illumination."""
    # Central difference gradient
    dz_dx = (np.roll(heightmap, -1, axis=1) - np.roll(heightmap, 1, axis=1)) / (2.0 * cell_size_m)
    dz_dy = (np.roll(heightmap, -1, axis=0) - np.roll(heightmap, 1, axis=0)) / (2.0 * cell_size_m)

    slope_rad = np.arctan(np.sqrt(dz_dx**2 + dz_dy**2))
    aspect_rad = np.arctan2(dz_dy, -dz_dx)

    zenith_rad = math.radians(90.0 - sun_altitude_deg)
    azimuth_rad = math.radians(360.0 - sun_azimuth_deg + 90.0)

    shaded = 255.0 * (
        (math.cos(zenith_rad) * np.cos(slope_rad))
        + (math.sin(zenith_rad) * np.sin(slope_rad) * np.cos(azimuth_rad - aspect_rad))
    )
    return np.clip(shaded, 0.0, 255.0).astype(np.uint8)


@dataclass
class World:
    """Continuous 400x400m simulated world with terrain and thermal dynamics."""

    width_m: float = 400.0
    height_m: float = 400.0
    res_m: float = 1.0
    seed: int = 42
    base_temp_c: float = 16.0
    temp_variation_c: float = 4.0

    def __post_init__(self) -> None:
        self.nx = int(self.width_m / self.res_m)
        self.ny = int(self.height_m / self.res_m)
        self.half_w = self.width_m / 2.0
        self.half_h = self.height_m / 2.0

        # Elevation and hillshade
        self.heightmap = generate_heightmap(self.nx, self.ny, seed=self.seed)
        self.hillshade = compute_hillshade(self.heightmap, self.res_m)

        # Baseline ground temperature field
        rng = np.random.default_rng(self.seed + 100)
        temp_noise = _smooth_noise_2d(self.nx, self.ny, 50, rng)
        # Higher elevations and shaded aspects are slightly cooler
        elev_factor = (self.heightmap / self.heightmap.max()) * -2.5
        self.base_temperature = (
            self.base_temp_c
            + (temp_noise - 0.5) * self.temp_variation_c
            + elev_factor
        ).astype(np.float32)

        # Active dynamic temperature field (combines ground + targets)
        self.temperature_field = self.base_temperature.copy()

    def get_elevation(self, east: float, north: float) -> float:
        """Sample terrain elevation (meters) at local metric coordinates (east, north)."""
        col = (east + self.half_w) / self.res_m
        row = (self.half_h - north) / self.res_m

        col = max(0.0, min(self.nx - 1.001, col))
        row = max(0.0, min(self.ny - 1.001, row))

        c0, r0 = int(col), int(row)
        c1, r1 = min(c0 + 1, self.nx - 1), min(r0 + 1, self.ny - 1)
        fc, fr = col - c0, row - r0

        v00 = self.heightmap[r0, c0]
        v01 = self.heightmap[r0, c1]
        v10 = self.heightmap[r1, c0]
        v11 = self.heightmap[r1, c1]

        top = (1.0 - fc) * v00 + fc * v01
        bot = (1.0 - fc) * v10 + fc * v11
        return float((1.0 - fr) * top + fr * bot)

    def reset_temperature_field(self) -> None:
        """Reset thermal field to base ground temperature before painting targets."""
        np.copyto(self.temperature_field, self.base_temperature)

    def paint_thermal_blob(
        self,
        center_east: float,
        center_north: float,
        peak_temp_c: float,
        radius_m: float,
        asymmetry_ratio: float = 1.0,
        angle_deg: float = 0.0,
    ) -> None:
        """Paint a gaussian/elliptical thermal anomaly into the ground temperature field."""
        r_px = int(math.ceil(radius_m * 2.5 / self.res_m))
        col_c = int((center_east + self.half_w) / self.res_m)
        row_c = int((self.half_h - center_north) / self.res_m)

        c_min, c_max = max(0, col_c - r_px), min(self.nx, col_c + r_px + 1)
        r_min, r_max = max(0, row_c - r_px), min(self.ny, row_c + r_px + 1)

        if c_max <= c_min or r_max <= r_min:
            return

        ys = (self.half_h - (np.arange(r_min, r_max) * self.res_m)) - center_north
        xs = ((np.arange(c_min, c_max) * self.res_m) - self.half_w) - center_east
        xx, yy = np.meshgrid(xs, ys)

        # Rotate coordinates if asymmetric
        rad = math.radians(angle_deg)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        xr = xx * cos_a + yy * sin_a
        yr = -xx * sin_a + yy * cos_a

        dist_sq = (xr / radius_m) ** 2 + (yr / (radius_m * asymmetry_ratio)) ** 2
        heat_mask = np.exp(-0.5 * dist_sq)

        patch = self.temperature_field[r_min:r_max, c_min:c_max]
        # Thermal emission combines with ground: heat source replaces cooler ground
        new_patch = np.maximum(patch, patch + (peak_temp_c - patch) * heat_mask)
        self.temperature_field[r_min:r_max, c_min:c_max] = new_patch

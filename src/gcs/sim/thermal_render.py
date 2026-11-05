"""Physical MLX90640 thermal sensor renderer with pixel footprint projection and noise."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from gcs.config import ThermalConfig
from gcs.core.models import DronePose, ThermalFrame
from gcs.sim.world import World


class ThermalRenderer:
    """Simulates 32x24 microbolometer sensor optics, footprint integration, and noise."""

    def __init__(self, config: ThermalConfig | None = None, seed: int = 42) -> None:
        self.config = config or ThermalConfig()
        self.rng = np.random.default_rng(seed)

        # Precompute normalized sensor grid [-1.0, 1.0]
        # Image coordinates: px in [0, 31], py in [0, 23]
        px = np.arange(self.config.width_px, dtype=np.float32)
        py = np.arange(self.config.height_px, dtype=np.float32)
        self.px_grid, self.py_grid = np.meshgrid(px, py)

        cx = (self.config.width_px - 1.0) / 2.0
        cy = (self.config.height_px - 1.0) / 2.0
        self.norm_x = (self.px_grid - cx) / (self.config.width_px / 2.0)
        self.norm_y = (cy - self.py_grid) / (self.config.height_px / 2.0)

        # Precalculate angular tangents
        self.tan_half_h = math.tan(math.radians(self.config.fov_h_deg / 2.0))
        self.tan_half_v = math.tan(math.radians(self.config.fov_v_deg / 2.0))

        # Edge noise mask (thermal sensors have higher noise toward perimeter)
        r_sq = self.norm_x**2 + self.norm_y**2
        self.noise_mask = (1.0 + (self.config.edge_noise_factor - 1.0) * (r_sq / 2.0)).astype(
            np.float32
        )

        # Shutter calibration state
        self._last_shutter_time: float = 0.0

    def render(
        self,
        drone_east: float,
        drone_north: float,
        alt_agl: float,
        heading_deg: float,
        world: World,
        sim_time_s: float,
        pose: DronePose,
        targets: list[Any] | None = None,
    ) -> ThermalFrame | None:
        """Render a 32x24 thermal frame from the world ground temperature field and targets.

        Returns None during periodic shutter calibration cycles (dropped frame blip).
        """
        # Periodic shutter recalibration (1 frame drop every 45s)
        if sim_time_s - self._last_shutter_time >= self.config.shutter_interval_s:
            self._last_shutter_time = sim_time_s
            return None  # mechanical shutter closed for NUC calibration

        h = max(1.0, alt_agl)
        heading_rad = math.radians(heading_deg)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        # Body-frame ground displacements from drone nadir (meters)
        d_fwd = h * self.tan_half_v * self.norm_y
        d_right = h * self.tan_half_h * self.norm_x

        # Ground sample coordinates rotated into world ENU frame
        sample_e = drone_east + d_fwd * sin_h + d_right * cos_h
        sample_n = drone_north + d_fwd * cos_h - d_right * sin_h

        # Convert to world temperature grid coordinates
        cols = (sample_e + world.half_w) / world.res_m
        rows = (world.half_h - sample_n) / world.res_m

        cols = np.clip(cols, 0.0, world.nx - 1.001)
        rows = np.clip(rows, 0.0, world.ny - 1.001)

        c0 = cols.astype(int)
        r0 = rows.astype(int)
        c1 = np.minimum(c0 + 1, world.nx - 1)
        r1 = np.minimum(r0 + 1, world.ny - 1)

        fc = cols - c0
        fr = rows - r0

        v00 = world.base_temperature[r0, c0]
        v01 = world.base_temperature[r0, c1]
        v10 = world.base_temperature[r1, c0]
        v11 = world.base_temperature[r1, c1]

        top = (1.0 - fc) * v00 + fc * v01
        bot = (1.0 - fc) * v10 + fc * v11
        sampled_temp = ((1.0 - fr) * top + fr * bot).copy()

        # High-fidelity continuous footprint integration for targets
        if targets is not None:
            # Physical spatial blur factor: at higher AGL, small targets smear over larger pixels
            # Reference altitude is 8m (standard search height)
            blur_factor = (8.0 / h) ** 1.35 if h > 8.0 else 1.0

            for tgt in targets:
                te, tn = tgt.get_current_position(sim_time_s)
                de = sample_e - te
                dn = sample_n - tn

                rad_m = tgt.radius_m
                asym = getattr(tgt, "asymmetry_ratio", 1.0)
                angle = math.radians(getattr(tgt, "angle_deg", 0.0))

                cos_a, sin_a = math.cos(angle), math.sin(angle)
                dr = de * cos_a + dn * sin_a
                df = -de * sin_a + dn * cos_a

                dist_sq = (dr / rad_m) ** 2 + (df / (rad_m * asym)) ** 2
                heat_contrib = (tgt.peak_temp_c - sampled_temp) * np.exp(-0.5 * dist_sq)
                sampled_temp = np.maximum(
                    sampled_temp,
                    sampled_temp + heat_contrib * blur_factor,
                )

        # Multi-pixel spatial averaging for high-altitude smearing
        footprint_w = 2.0 * h * self.tan_half_h / self.config.width_px
        if footprint_w > 1.2:
            kernel_size = min(5, int(math.ceil(footprint_w / world.res_m)))
            if kernel_size % 2 == 0:
                kernel_size += 1
            from scipy.ndimage import uniform_filter

            sampled_temp = uniform_filter(sampled_temp, size=kernel_size)

        # Add Gaussian microbolometer sensor noise
        raw_noise = self.rng.normal(
            0.0,
            self.config.noise_std_c,
            (self.config.height_px, self.config.width_px),
        ).astype(np.float32)

        noisy_frame = sampled_temp + (raw_noise * self.noise_mask)

        return ThermalFrame(
            timestamp=sim_time_s,
            data=noisy_frame.astype(np.float32),
            sensor_fov_deg=(self.config.fov_h_deg, self.config.fov_v_deg),
            drone_pose=pose,
        )

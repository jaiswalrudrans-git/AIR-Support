"""Down-facing contextual RGB visual camera renderer with terrain texturing and targets."""

from __future__ import annotations

import math

import cv2
import numpy as np

from gcs.config import VideoConfig
from gcs.core.models import VideoFrame
from gcs.sim.scenario import GroundTruthTarget
from gcs.sim.world import World


class VideoRenderer:
    """Renders top-down 640x480 RGB visual camera stream matching flight optics."""

    def __init__(self, config: VideoConfig | None = None) -> None:
        self.config = config or VideoConfig()
        self.night_vision: bool = False
        self._prev_frame: np.ndarray | None = None

        # Precompute coordinate grids
        px = np.arange(self.config.width_px, dtype=np.float32)
        py = np.arange(self.config.height_px, dtype=np.float32)
        self.px_grid, self.py_grid = np.meshgrid(px, py)

        cx = (self.config.width_px - 1.0) / 2.0
        cy = (self.config.height_px - 1.0) / 2.0
        self.norm_x = (self.px_grid - cx) / (self.config.width_px / 2.0)
        self.norm_y = (cy - self.py_grid) / (self.config.height_px / 2.0)

        self.tan_half_h = math.tan(math.radians(self.config.fov_h_deg / 2.0))
        self.tan_half_v = math.tan(math.radians(self.config.fov_v_deg / 2.0))

    def render(
        self,
        drone_east: float,
        drone_north: float,
        alt_agl: float,
        heading_deg: float,
        world: World,
        targets: list[GroundTruthTarget],
        sim_time_s: float,
    ) -> VideoFrame:
        """Render 640x480 RGB top-down visual camera view."""
        h = max(2.0, alt_agl)
        heading_rad = math.radians(heading_deg)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        # Ground sample coordinates
        d_fwd = h * self.tan_half_v * self.norm_y
        d_right = h * self.tan_half_h * self.norm_x

        sample_e = drone_east + d_fwd * sin_h + d_right * cos_h
        sample_n = drone_north + d_fwd * cos_h - d_right * sin_h

        # World indices
        cols = np.clip((sample_e + world.half_w) / world.res_m, 0, world.nx - 1).astype(int)
        rows = np.clip((world.half_h - sample_n) / world.res_m, 0, world.ny - 1).astype(int)

        # Sample terrain elevation and hillshade
        elev = world.heightmap[rows, cols]
        shade = world.hillshade[rows, cols].astype(np.float32) / 255.0

        # Terrain texturing: grass at lower elev, scree/rock at higher elev
        elev_norm = np.clip(elev / 50.0, 0.0, 1.0)

        # Base natural colors (RGB)
        # Low elev: grassland [75, 115, 60], high elev: alpine rock [130, 135, 140]
        r = (75 * (1.0 - elev_norm) + 130 * elev_norm) * (0.4 + 0.6 * shade)
        g = (115 * (1.0 - elev_norm) + 135 * elev_norm) * (0.4 + 0.6 * shade)
        b = (60 * (1.0 - elev_norm) + 140 * elev_norm) * (0.4 + 0.6 * shade)

        rgb = np.dstack([r, g, b]).clip(0, 255).astype(np.uint8)

        # Render visible targets onto visual camera image
        # Sensor footprint half-spans on ground
        span_fwd = h * self.tan_half_v
        span_right = h * self.tan_half_h

        for tgt in targets:
            te, tn = tgt.get_current_position(sim_time_s)
            dx = te - drone_east
            dy = tn - drone_north

            # Transform into camera body coordinates
            tgt_fwd = dx * sin_h + dy * cos_h
            tgt_right = dx * cos_h - dy * sin_h

            if abs(tgt_fwd) <= span_fwd * 1.1 and abs(tgt_right) <= span_right * 1.1:
                # Target is inside or near optical FOV
                cx_img = self.config.width_px / 2.0
                cy_img = self.config.height_px / 2.0
                img_x = int(cx_img + (tgt_right / span_right) * cx_img)
                img_y = int(cy_img - (tgt_fwd / span_fwd) * cy_img)

                px_radius = max(2, int(tgt.radius_m / (2.0 * span_right / self.config.width_px)))

                if tgt.is_human:
                    # Blue jacket / dark trousers top-down figure
                    cv2.circle(rgb, (img_x, img_y), max(2, px_radius), (35, 65, 140), -1)
                    head_r = max(1, px_radius // 2)
                    cv2.circle(rgb, (img_x, img_y - head_r), head_r, (210, 180, 150), -1)
                elif "Vehicle" in tgt.name:
                    # Parked vehicle outline
                    w_box = max(4, int(2.0 * px_radius))
                    h_box = max(8, int(4.0 * px_radius))
                    cv2.rectangle(
                        rgb,
                        (img_x - w_box // 2, img_y - h_box // 2),
                        (img_x + w_box // 2, img_y + h_box // 2),
                        (50, 55, 60),
                        -1,
                    )
                elif "Campfire" in tgt.name:
                    cv2.circle(rgb, (img_x, img_y), max(2, px_radius), (40, 40, 40), -1)
                    cv2.circle(rgb, (img_x, img_y), max(1, px_radius // 2), (230, 110, 20), -1)
                elif "Rock" in tgt.name:
                    cv2.circle(rgb, (img_x, img_y), max(3, px_radius), (90, 95, 100), -1)

        # Optional Night Vision mode (phosphor green tint + noise)
        if self.night_vision:
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            # Night vision phosphor green: mostly G channel
            nv_g = np.clip(gray.astype(np.float32) * 1.6, 0, 255).astype(np.uint8)
            nv_r = (nv_g * 0.15).astype(np.uint8)
            nv_b = (nv_g * 0.25).astype(np.uint8)
            rgb = np.dstack([nv_r, nv_g, nv_b])

        # Mild temporal motion blur
        if self._prev_frame is not None and self._prev_frame.shape == rgb.shape:
            rgb = cv2.addWeighted(rgb, 0.88, self._prev_frame, 0.12, 0)
        self._prev_frame = rgb.copy()

        return VideoFrame(timestamp=sim_time_s, image=rgb)

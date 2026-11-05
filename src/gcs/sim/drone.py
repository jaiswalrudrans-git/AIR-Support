"""Simulated physical drone flight dynamics, battery depletion, and noisy telemetry."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from gcs.config import DroneConfig, GeoConfig
from gcs.control.modes import ModeController, RTLPhase
from gcs.core.geo import enu_to_latlon
from gcs.core.models import ControlInput, Event, Telemetry
from gcs.sim.world import World


@dataclass
class DroneSim:
    """Rigid body 6-DOF kinematic drone simulation with battery and sensor noise models."""

    config: DroneConfig = field(default_factory=DroneConfig)
    geo_config: GeoConfig = field(default_factory=GeoConfig)

    # Position in local ENU frame (meters)
    east: float = 0.0
    north: float = 0.0
    alt_msl: float = 858.0  # origin_alt (850) + initial 8m AGL

    # Velocity in world ENU frame (m/s)
    vel_east: float = 0.0
    vel_north: float = 0.0
    vel_up: float = 0.0

    # Attitude
    heading_deg: float = 0.0  # 0 deg = North, 90 deg = East

    # Battery state
    battery_used_mah: float = 0.0
    battery_pct: float = 100.0
    battery_v: float = 16.8

    # Status
    sim_time_s: float = 0.0
    armed: bool = True
    link_loss: bool = False  # simulated link drop toggle

    # Random number generator for wind and sensor noise
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(1234))

    def step(
        self,
        dt: float,
        operator_cmd: ControlInput,
        world: World,
        mode_ctrl: ModeController,
    ) -> tuple[Telemetry, list[Event]]:
        """Advance physical simulation by dt seconds and return telemetry snapshot."""
        self.sim_time_s += dt

        # Sample terrain elevation below current position
        ground_elev = world.get_elevation(self.east, self.north)
        current_agl = self.alt_msl - ground_elev

        # Mode controller arbitration and safety checks
        effective_cmd, events = mode_ctrl.check_safety_and_arbitrate(
            current_east=self.east,
            current_north=self.north,
            current_msl=self.alt_msl,
            current_agl=current_agl,
            battery_pct=self.battery_pct,
            operator_cmd=operator_cmd,
            sim_time=self.sim_time_s,
            world=world,
            vel_east=self.vel_east,
            vel_north=self.vel_north,
        )

        # Check disarm condition after landing in RTL
        if mode_ctrl.rtl_phase == RTLPhase.DISARMED:
            self.armed = False
            self.vel_east = 0.0
            self.vel_north = 0.0
            self.vel_up = 0.0
        else:
            # Update heading from yaw command
            yaw_rate_cmd = effective_cmd.yaw_rate * self.config.max_yaw_rate_degps
            self.heading_deg = (self.heading_deg + yaw_rate_cmd * dt) % 360.0

            # Convert body-frame flight inputs (forward, right) to world-frame velocities
            heading_rad = math.radians(self.heading_deg)
            cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)

            body_fwd_v = effective_cmd.forward * self.config.max_horizontal_speed_mps
            body_right_v = effective_cmd.right * self.config.max_horizontal_speed_mps

            target_vel_east = body_fwd_v * sin_h + body_right_v * cos_h
            target_vel_north = body_fwd_v * cos_h - body_right_v * sin_h
            target_vel_up = effective_cmd.up * self.config.max_vertical_speed_mps

            # Exponential velocity smoothing (acceleration limit)
            alpha = min(1.0, dt / max(0.05, self.config.accel_time_constant_s))
            self.vel_east += (target_vel_east - self.vel_east) * alpha
            self.vel_north += (target_vel_north - self.vel_north) * alpha
            self.vel_up += (target_vel_up - self.vel_up) * alpha

            # Mild atmospheric turbulence (wind gust noise)
            gust_e = float(self.rng.normal(0.0, 0.08))
            gust_n = float(self.rng.normal(0.0, 0.08))

            # Integrate position
            self.east += (self.vel_east + gust_e) * dt
            self.north += (self.vel_north + gust_n) * dt
            self.alt_msl += self.vel_up * dt

            # Enforce hard ground collision floor
            new_ground_elev = world.get_elevation(self.east, self.north)
            if self.alt_msl < new_ground_elev + 0.1:
                self.alt_msl = new_ground_elev + 0.1
                self.vel_up = max(0.0, self.vel_up)

        # Battery model integration
        speed_h = math.hypot(self.vel_east, self.vel_north)
        speed_factor = (speed_h / self.config.max_horizontal_speed_mps) ** 2
        # ~15A hover current + up to 6A additional aerodynamic drag at high speed
        current_amps = self.config.hover_current_draw_a + speed_factor * 6.0
        if not self.armed:
            current_amps = 0.5  # idle electronics

        mah_drawn = (current_amps * 1000.0) * (dt / 3600.0)
        self.battery_used_mah += mah_drawn
        self.battery_pct = max(
            0.0,
            100.0 * (1.0 - self.battery_used_mah / self.config.battery_capacity_mah),
        )
        self.battery_v = (
            self.config.battery_empty_voltage
            + (self.config.battery_full_voltage - self.config.battery_empty_voltage)
            * (self.battery_pct / 100.0)
        )

        # Synthesize noisy GPS and compass sensor telemetry
        gps_err_e = float(self.rng.normal(0.0, self.config.gps_noise_std_m))
        gps_err_n = float(self.rng.normal(0.0, self.config.gps_noise_std_m))
        noisy_lat, noisy_lon, _ = enu_to_latlon(
            self.east + gps_err_e,
            self.north + gps_err_n,
            self.alt_msl,
            self.geo_config.origin_lat,
            self.geo_config.origin_lon,
            self.geo_config.origin_alt_msl,
        )

        compass_err = float(self.rng.normal(0.0, self.config.compass_noise_std_deg))
        noisy_heading = (self.heading_deg + compass_err) % 360.0

        current_elev = world.get_elevation(self.east, self.north)
        alt_agl = max(0.0, self.alt_msl - current_elev)
        ground_speed = math.hypot(self.vel_east, self.vel_north)

        link_quality = 0.0 if self.link_loss else 98.5

        telemetry = Telemetry(
            timestamp=self.sim_time_s,
            lat=noisy_lat,
            lon=noisy_lon,
            alt_msl=self.alt_msl,
            alt_agl=alt_agl,
            heading_deg=noisy_heading,
            ground_speed=ground_speed,
            vertical_speed=self.vel_up,
            battery_pct=self.battery_pct,
            battery_v=self.battery_v,
            gps_fix="3D",
            gps_sats=14,
            mode=mode_ctrl.current_mode.value,
            armed=self.armed,
            link_quality_pct=link_quality,
        )

        return telemetry, events

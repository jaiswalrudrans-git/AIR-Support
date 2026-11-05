"""Autopilot flight modes, terrain-following guidance, and safety monitoring."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

from gcs.config import DroneConfig
from gcs.core.models import ControlInput, Event, EventLevel, FlightMode


class RTLPhase(StrEnum):
    """Sequential phases of the Return-to-Launch state machine."""

    CLIMB_SAFE = "CLIMB_SAFE"
    CRUISE_HOME = "CRUISE_HOME"
    DESCEND_LAND = "DESCEND_LAND"
    DISARMED = "DISARMED"


@dataclass
class ModeController:
    """Manages flight mode state transitions, terrain following, and safety geofencing."""

    config: DroneConfig = field(default_factory=DroneConfig)
    current_mode: FlightMode = FlightMode.MANUAL
    target_agl_m: float = 8.0
    rtl_phase: RTLPhase = RTLPhase.CLIMB_SAFE
    home_east: float = 0.0
    home_north: float = 0.0
    home_elev_m: float = 0.0

    # Safety event latches (prevent spamming events every tick)
    _warned_low_battery: bool = False
    _warned_crit_battery: bool = False
    _warned_geofence: bool = False
    _warned_floor: bool = False

    def request_mode(self, new_mode: FlightMode, current_elev: float) -> list[Event]:
        """Request operator transition between modes."""
        events: list[Event] = []
        if new_mode == self.current_mode:
            return events

        if new_mode == FlightMode.RTL:
            self.rtl_phase = RTLPhase.CLIMB_SAFE
            events.append(
                Event(
                    timestamp=0.0,
                    level=EventLevel.INFO,
                    message="Engaged RTL: climbing to safe transit altitude",
                )
            )
        elif new_mode == FlightMode.TERRAIN_FOLLOW:
            events.append(
                Event(
                    timestamp=0.0,
                    level=EventLevel.INFO,
                    message=f"Engaged TERRAIN_FOLLOW: holding {self.target_agl_m:.1f}m AGL",
                )
            )
        else:
            events.append(
                Event(
                    timestamp=0.0,
                    level=EventLevel.INFO,
                    message="Manual flight mode active",
                )
            )

        self.current_mode = new_mode
        return events

    def adjust_target_agl(self, delta_m: float) -> float:
        """Increment or decrement terrain-following target altitude within safety bounds."""
        self.target_agl_m = max(
            self.config.min_safe_agl_m + 1.0,
            min(self.config.max_safe_agl_m - 5.0, self.target_agl_m + delta_m),
        )
        return self.target_agl_m

    def check_safety_and_arbitrate(
        self,
        current_east: float,
        current_north: float,
        current_msl: float,
        current_agl: float,
        battery_pct: float,
        operator_cmd: ControlInput,
        sim_time: float,
        world: Any = None,
        vel_east: float = 0.0,
        vel_north: float = 0.0,
    ) -> tuple[ControlInput, list[Event]]:
        """Run safety monitoring and override control inputs if safety boundaries violated."""
        events: list[Event] = []
        cmd = ControlInput(
            forward=operator_cmd.forward,
            right=operator_cmd.right,
            up=operator_cmd.up,
            yaw_rate=operator_cmd.yaw_rate,
        )

        # 1. Battery safety monitoring
        if battery_pct <= self.config.battery_rtl_pct:
            if not self._warned_crit_battery:
                self._warned_crit_battery = True
                events.append(
                    Event(
                        timestamp=sim_time,
                        level=EventLevel.CRITICAL,
                        message=f"CRITICAL BATTERY ({battery_pct:.1f}%): Automatic RTL engaged",
                    )
                )
                self.current_mode = FlightMode.RTL
                self.rtl_phase = RTLPhase.CLIMB_SAFE
        elif battery_pct <= self.config.battery_warn_pct:
            if not self._warned_low_battery:
                self._warned_low_battery = True
                events.append(
                    Event(
                        timestamp=sim_time,
                        level=EventLevel.WARN,
                        message=f"Low Battery Warning: {battery_pct:.1f}% remaining",
                    )
                )

        # 2. Geofence monitoring (400x400m world -> ±200m boundaries)
        max_dist = max(abs(current_east), abs(current_north))
        fence_limit = 200.0 - self.config.geofence_margin_m  # 180m
        breach_limit = 195.0

        if max_dist > breach_limit:
            if self.current_mode != FlightMode.RTL:
                events.append(
                    Event(
                        timestamp=sim_time,
                        level=EventLevel.CRITICAL,
                        message="Geofence breach (>195m): Emergency RTL triggered",
                    )
                )
                self.current_mode = FlightMode.RTL
                self.rtl_phase = RTLPhase.CLIMB_SAFE
        elif max_dist > fence_limit and not self._warned_geofence:
            self._warned_geofence = True
            events.append(
                Event(
                    timestamp=sim_time,
                    level=EventLevel.WARN,
                    message="Approaching 180m geofence safety boundary",
                )
            )
        elif max_dist < fence_limit - 10.0:
            self._warned_geofence = False

        # 3. Altitude floor and ceiling safety
        if current_agl < self.config.min_safe_agl_m:
            if not self._warned_floor:
                self._warned_floor = True
                events.append(
                    Event(
                        timestamp=sim_time,
                        level=EventLevel.WARN,
                        message=f"Terrain proximity warning: {current_agl:.1f}m AGL",
                    )
                )
            # Soft floor: prevent commanded descent
            if cmd.up < 0.0:
                cmd.up = 0.0
        elif current_agl > self.config.min_safe_agl_m + 1.0:
            self._warned_floor = False

        if current_agl > self.config.max_safe_agl_m:
            # Ceiling: prevent commanded ascent
            if cmd.up > 0.0:
                cmd.up = 0.0

        # 4. Mode-specific control arbitration
        if self.current_mode == FlightMode.TERRAIN_FOLLOW:
            # In terrain-following mode, speed is governed to safe cruise (~3.6 m/s)
            # to keep climb/descent demands within rotor dynamics (DEC-004)
            safe_terrain_ratio = 0.45
            cmd.forward *= safe_terrain_ratio
            cmd.right *= safe_terrain_ratio

            # Anticipatory elevation lookahead (0.5s)
            current_ground = current_msl - current_agl
            lookahead_s = 0.5
            look_e = current_east + vel_east * lookahead_s
            look_n = current_north + vel_north * lookahead_s

            if world is not None:
                ground_ahead = float(world.get_elevation(look_e, look_n))
            else:
                ground_ahead = current_ground

            target_msl = ground_ahead + self.target_agl_m
            error_msl = target_msl - current_msl

            # High-gain altitude tracking
            kp_agl = 1.5
            cmd.up = float(max(-1.0, min(1.0, error_msl * kp_agl)))

        elif self.current_mode == FlightMode.RTL:
            cmd = self._compute_rtl_command(
                current_east, current_north, current_msl, current_agl
            )

        return cmd, events

    def _compute_rtl_command(
        self,
        current_east: float,
        current_north: float,
        current_msl: float,
        current_agl: float,
    ) -> ControlInput:
        """Execute the Return-to-Launch state machine."""
        safe_cruise_msl = self.home_elev_m + 15.0
        cmd = ControlInput()

        d_east = self.home_east - current_east
        d_north = self.home_north - current_north
        dist_home = math.hypot(d_east, d_north)

        if self.rtl_phase == RTLPhase.CLIMB_SAFE:
            if current_msl < safe_cruise_msl - 0.5:
                cmd.up = 0.8
                cmd.forward = 0.0
                cmd.right = 0.0
            else:
                self.rtl_phase = RTLPhase.CRUISE_HOME

        if self.rtl_phase == RTLPhase.CRUISE_HOME:
            if dist_home > 2.0:
                # Steer home
                angle_to_home = math.atan2(d_east, d_north)
                cmd.forward = math.cos(angle_to_home)
                cmd.right = math.sin(angle_to_home)
                # Maintain safe altitude
                cmd.up = math.tanh(0.5 * (safe_cruise_msl - current_msl))
            else:
                self.rtl_phase = RTLPhase.DESCEND_LAND

        if self.rtl_phase == RTLPhase.DESCEND_LAND:
            # Small position correction over home pad
            cmd.forward = math.tanh(0.3 * d_north)
            cmd.right = math.tanh(0.3 * d_east)
            if current_agl > 0.4:
                cmd.up = -0.5  # descend at steady rate
            else:
                cmd.up = 0.0
                self.rtl_phase = RTLPhase.DISARMED

        if self.rtl_phase == RTLPhase.DISARMED:
            cmd.forward = 0.0
            cmd.right = 0.0
            cmd.up = 0.0

        return cmd

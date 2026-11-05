"""Integrated physics and sensor simulation link implementing DroneLink interface."""

from __future__ import annotations

from PySide6.QtCore import QObject, QTimer

from gcs.config import AppConfig
from gcs.control.modes import ModeController
from gcs.core.models import ControlInput, DronePose
from gcs.link.base import DroneLink
from gcs.sim.drone import DroneSim
from gcs.sim.scenario import ScenarioType, build_scenario_world
from gcs.sim.thermal_render import ThermalRenderer
from gcs.sim.video_render import VideoRenderer


class SimLink(DroneLink):
    """High-fidelity simulation link delivering real-time telemetry and imagery streams."""

    def __init__(self, config: AppConfig | None = None, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.config = config or AppConfig()

        # Build world and targets
        self.world, self.targets = build_scenario_world(ScenarioType.DEMO)

        # Initialize physical drone and controllers
        self.mode_ctrl = ModeController(
            config=self.config.drone,
            target_agl_m=self.config.drone.default_target_agl_m,
        )
        self.drone = DroneSim(config=self.config.drone, geo_config=self.config.geo)

        # Sensor renderers
        self.thermal_renderer = ThermalRenderer(self.config.thermal)
        self.video_renderer = VideoRenderer(self.config.video)

        # Operator command latch
        self._current_cmd = ControlInput()
        self._is_connected = False
        self._is_paused = False

        # Timing accumulators for multirate streams (10 Hz telem, 8 Hz thermal, 20 Hz video)
        self._telem_accumulator = 0.0
        self._thermal_accumulator = 0.0
        self._telem_period = 0.10  # 10 Hz
        self._thermal_period = 1.0 / self.config.thermal.frame_rate_hz  # ~8 Hz (0.125s)

        # Simulation loop timer (20 Hz -> 50ms tick)
        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._simulation_step)

    def connect_link(self) -> bool:
        """Start the simulation clock."""
        self._is_connected = True
        self._timer.start()
        self.link_status_changed.emit("Connected (Simulation)")
        return True

    def disconnect_link(self) -> None:
        """Stop simulation timer."""
        self._timer.stop()
        self._is_connected = False
        self.link_status_changed.emit("Disconnected")

    def send_control(self, control: ControlInput) -> None:
        """Receive flight command from keyboard/autopilot."""
        self._current_cmd = control

    def is_connected(self) -> bool:
        return self._is_connected

    def set_paused(self, paused: bool) -> None:
        """Pause / resume physics update."""
        self._is_paused = paused

    def switch_scenario(self, scenario_type: ScenarioType) -> None:
        """Switch active scenario (DEMO, NIGHT, HOT_AFTERNOON, RANDOM)."""
        self.world, self.targets = build_scenario_world(scenario_type)
        # Reset drone position to home
        self.drone.east = 0.0
        self.drone.north = 0.0
        home_elev = self.world.get_elevation(0.0, 0.0)
        self.drone.alt_msl = home_elev + self.config.drone.default_target_agl_m
        self.drone.battery_used_mah = 0.0
        self.drone.battery_pct = 100.0
        self.drone.armed = True

    def _simulation_step(self) -> None:
        """Execute one simulation cycle (50ms)."""
        if not self._is_connected or self._is_paused:
            return

        dt = 0.05
        # Step physics and modes
        telem, events = self.drone.step(dt, self._current_cmd, self.world, self.mode_ctrl)

        # Construct synchronous pose snapshot for geotagging
        pose = DronePose(
            lat=telem.lat,
            lon=telem.lon,
            alt_agl=telem.alt_agl,
            heading_deg=telem.heading_deg,
        )

        # 1. Telemetry stream at 10 Hz
        self._telem_accumulator += dt
        if self._telem_accumulator >= self._telem_period:
            self._telem_accumulator -= self._telem_period
            self.telemetry_received.emit(telem)

        # 2. Thermal frame stream at 8 Hz
        self._thermal_accumulator += dt
        if self._thermal_accumulator >= self._thermal_period:
            self._thermal_accumulator -= self._thermal_period
            thermal_frame = self.thermal_renderer.render(
                drone_east=self.drone.east,
                drone_north=self.drone.north,
                alt_agl=telem.alt_agl,
                heading_deg=telem.heading_deg,
                world=self.world,
                sim_time_s=self.drone.sim_time_s,
                pose=pose,
                targets=self.targets,
            )
            if thermal_frame is not None:
                self.thermal_received.emit(thermal_frame)

        # 3. Visual camera stream at 20 Hz
        video_frame = self.video_renderer.render(
            drone_east=self.drone.east,
            drone_north=self.drone.north,
            alt_agl=telem.alt_agl,
            heading_deg=telem.heading_deg,
            world=self.world,
            targets=self.targets,
            sim_time_s=self.drone.sim_time_s,
        )
        self.video_received.emit(video_frame)

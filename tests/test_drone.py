"""Unit and integration tests for drone dynamics, motion, and battery drain."""

from gcs.config import DroneConfig, GeoConfig
from gcs.control.modes import ModeController
from gcs.core.models import ControlInput
from gcs.sim.drone import DroneSim
from gcs.sim.world import World


def test_scripted_motion_and_yaw() -> None:
    """Scripted forward and yaw controls produce correct movement vectors."""
    cfg = DroneConfig()
    geo = GeoConfig()
    world = World(seed=42)
    mode_ctrl = ModeController(config=cfg)

    drone = DroneSim(config=cfg, geo_config=geo, east=0.0, north=0.0, alt_msl=858.0)

    # 1. Fly forward (North) for 5 seconds (dt=0.1s -> 50 steps)
    cmd_fwd = ControlInput(forward=1.0)
    for _ in range(50):
        drone.step(0.1, cmd_fwd, world, mode_ctrl)

    # After 5s forward at up to 8m/s, drone should have moved significantly North (>25m)
    assert drone.north > 25.0
    assert abs(drone.east) < 5.0  # minimal lateral drift

    # 2. Command 90 degree yaw right (clockwise toward East)
    cmd_yaw = ControlInput(yaw_rate=1.0)
    for _ in range(10):  # 1s at 90 deg/s
        drone.step(0.1, cmd_yaw, world, mode_ctrl)

    # Heading should be ~90 degrees
    assert 80.0 < drone.heading_deg < 100.0


def test_battery_depletion_model() -> None:
    """Battery drains predictably during flight."""
    cfg = DroneConfig()
    geo = GeoConfig()
    world = World(seed=42)
    mode_ctrl = ModeController(config=cfg)

    drone = DroneSim(config=cfg, geo_config=geo)
    initial_pct = drone.battery_pct

    # Hover for 60 seconds
    cmd_hover = ControlInput()
    for _ in range(600):  # 60s at dt=0.1s
        drone.step(0.1, cmd_hover, world, mode_ctrl)

    # In 1 minute of ~20 min endurance, ~4.5% to 6% should have been consumed
    consumed = initial_pct - drone.battery_pct
    assert 3.5 < consumed < 7.0
    assert drone.battery_v < 16.8

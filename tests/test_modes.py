"""Integration tests verifying terrain-following, RTL return accuracy, and safety triggers."""

import math

from gcs.config import DroneConfig, GeoConfig
from gcs.control.modes import ModeController, RTLPhase
from gcs.core.models import ControlInput, FlightMode
from gcs.sim.drone import DroneSim
from gcs.sim.world import World


def test_terrain_follow_holds_agl_over_hilly_flight() -> None:
    """Terrain-follow holds target AGL within +/-1m over a 200m flight across hilly terrain."""
    cfg = DroneConfig()
    geo = GeoConfig()
    world = World(seed=42)
    target_agl = 8.0

    mode_ctrl = ModeController(
        config=cfg,
        current_mode=FlightMode.TERRAIN_FOLLOW,
        target_agl_m=target_agl,
    )
    initial_elev = world.get_elevation(0.0, -100.0)
    drone = DroneSim(
        config=cfg,
        geo_config=geo,
        east=0.0,
        north=-100.0,
        alt_msl=initial_elev + target_agl,
    )

    # Command forward flight North for 200m across hills (from north=-100 to north=+100)
    cmd_fwd = ControlInput(forward=1.0)
    agl_history: list[float] = []

    # Fly until north >= 100m (dt=0.05s for high control loop fidelity)
    steps = 0
    while drone.north < 100.0 and steps < 2000:
        telem, _ = drone.step(0.05, cmd_fwd, world, mode_ctrl)
        ground_elev = world.get_elevation(drone.east, drone.north)
        actual_agl = drone.alt_msl - ground_elev
        # Exclude initial 2 seconds of acceleration settling
        if steps > 40:
            agl_history.append(actual_agl)
        steps += 1

    # Distance flown is 200m
    assert drone.north >= 100.0
    assert len(agl_history) > 100

    min_recorded_agl = min(agl_history)
    max_recorded_agl = max(agl_history)

    # Acceptance criterion: holds AGL within +/- 1m of target (8.0m -> range [7.0, 9.0])
    assert min_recorded_agl >= target_agl - 1.0, (
        f"AGL dropped to {min_recorded_agl:.2f}m (< {target_agl - 1.0}m)"
    )
    assert max_recorded_agl <= target_agl + 1.0, (
        f"AGL rose to {max_recorded_agl:.2f}m (> {target_agl + 1.0}m)"
    )


def test_rtl_returns_within_3m_of_home() -> None:
    """RTL mode flies back from distant position and lands within 3m of home."""
    cfg = DroneConfig()
    geo = GeoConfig()
    world = World(seed=42)

    home_e, home_n = 0.0, 0.0
    home_elev = world.get_elevation(home_e, home_n)

    mode_ctrl = ModeController(
        config=cfg,
        current_mode=FlightMode.RTL,
        home_east=home_e,
        home_north=home_n,
        home_elev_m=home_elev,
    )

    # Place drone 120m away to the North-East
    drone = DroneSim(
        config=cfg,
        geo_config=geo,
        east=90.0,
        north=80.0,
        alt_msl=world.get_elevation(90.0, 80.0) + 12.0,
    )

    # Step simulation under RTL until disarmed / landed
    cmd_zero = ControlInput()
    for _ in range(1200):  # max 60s at dt=0.05s
        drone.step(0.05, cmd_zero, world, mode_ctrl)
        if mode_ctrl.rtl_phase == RTLPhase.DISARMED:
            break

    # Check landing distance from home
    dist_from_home = math.hypot(drone.east - home_e, drone.north - home_n)
    # Acceptance criterion: returns within 3m of home
    assert dist_from_home < 3.0, (
        f"RTL final position error was {dist_from_home:.2f}m (expected < 3.0m)"
    )


def test_low_battery_triggers_automatic_rtl() -> None:
    """When battery falls below 15%, safety supervisor engages automatic RTL."""
    cfg = DroneConfig()
    geo = GeoConfig()
    world = World(seed=42)

    mode_ctrl = ModeController(config=cfg, current_mode=FlightMode.MANUAL)
    drone = DroneSim(config=cfg, geo_config=geo, east=50.0, north=50.0)

    # Force battery to 15.5% (just above critical threshold)
    drone.battery_pct = 15.5
    drone.battery_used_mah = cfg.battery_capacity_mah * (1.0 - 0.155)

    assert mode_ctrl.current_mode == FlightMode.MANUAL

    # Run for a few seconds so battery crosses below 15.0%
    cmd_fwd = ControlInput(forward=0.5)
    critical_event_seen = False
    for _ in range(100):
        _, events = drone.step(0.1, cmd_fwd, world, mode_ctrl)
        for ev in events:
            if "CRITICAL BATTERY" in ev.message:
                critical_event_seen = True

    # Acceptance criterion: mode changed to RTL and critical warning emitted
    assert mode_ctrl.current_mode == FlightMode.RTL
    assert critical_event_seen

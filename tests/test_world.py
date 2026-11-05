"""Unit tests for terrain world generation, elevation sampling, and scenario targets."""

import numpy as np

from gcs.sim.scenario import (
    ScenarioType,
    build_scenario_world,
    get_default_targets,
)
from gcs.sim.world import World


def test_heightmap_and_hillshade_generation() -> None:
    """Terrain heightmap stays within physical bounds and hillshade is valid 8-bit image."""
    world = World(width_m=400.0, height_m=400.0, res_m=1.0, seed=42)

    assert world.heightmap.shape == (400, 400)
    assert 0.0 <= world.heightmap.min()
    assert world.heightmap.max() <= 60.0

    assert world.hillshade.shape == (400, 400)
    assert world.hillshade.dtype == np.uint8
    assert world.hillshade.min() < 100  # contains shaded ravines
    assert world.hillshade.max() > 180  # contains illuminated ridges


def test_elevation_sampling_continuity() -> None:
    """Bilinear elevation sampling yields smooth, bounded heights."""
    world = World(width_m=400.0, height_m=400.0, res_m=1.0, seed=42)

    elev_center = world.get_elevation(0.0, 0.0)
    assert 0.0 <= elev_center <= 60.0

    # Test small step continuity (no discontinuous jumps)
    elev_step = world.get_elevation(0.1, 0.1)
    assert abs(elev_step - elev_center) < 0.5


def test_scenario_ground_truth_targets() -> None:
    """Verify standard demo scenario has 3 casualties and 5 decoys."""
    world, targets = build_scenario_world(ScenarioType.DEMO, seed=42)

    human_targets = [t for t in targets if t.is_human]
    decoy_targets = [t for t in targets if not t.is_human]

    assert len(human_targets) == 3
    assert len(decoy_targets) == 5

    # Check that exposed human casualty is painted at (30.0, 50.0) with ~34 C
    exposed = next(t for t in human_targets if t.id == "T-HUMAN-01")
    col = int(exposed.east + 200)
    row = int(200 - exposed.north)
    local_temp = world.temperature_field[row - 1 : row + 2, col - 1 : col + 2].max()
    assert abs(local_temp - exposed.peak_temp_c) < 1.0


def test_moving_animal_target() -> None:
    """Moving animal changes position over time."""
    targets = get_default_targets()
    animal = next(t for t in targets if t.is_moving)

    pos_t0 = animal.get_current_position(0.0)
    pos_t10 = animal.get_current_position(10.0)
    assert pos_t0 != pos_t10

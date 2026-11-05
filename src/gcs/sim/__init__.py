"""Simulated environment, terrain models, and physical drone dynamics."""

from gcs.sim.scenario import (
    GroundTruthTarget,
    ScenarioType,
    build_scenario_world,
    get_default_targets,
)
from gcs.sim.world import World, compute_hillshade, generate_heightmap

__all__ = [
    "GroundTruthTarget",
    "ScenarioType",
    "World",
    "build_scenario_world",
    "compute_hillshade",
    "generate_heightmap",
    "get_default_targets",
]

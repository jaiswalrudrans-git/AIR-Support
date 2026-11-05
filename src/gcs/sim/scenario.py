"""Mission scenarios, ground-truth targets, and environment presets.

Defines 3 human casualties and 5 environmental decoys according to Section 5,
providing a rigorous ground-truth evaluation standard for detection algorithms.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from gcs.sim.world import World


class ScenarioType(StrEnum):
    """Preset operational environmental conditions."""

    DEMO = "Demo Scenario"
    NIGHT = "Night Mission"
    HOT_AFTERNOON = "Hot Afternoon"
    RANDOM = "Random Scenario"


@dataclass(slots=True)
class GroundTruthTarget:
    """Documented reference target placed in the simulation world."""

    id: str
    name: str
    east: float
    north: float
    peak_temp_c: float
    radius_m: float
    is_human: bool
    is_moving: bool = False
    asymmetry_ratio: float = 1.0
    angle_deg: float = 0.0
    notes: str = ""

    def get_current_position(self, sim_time_s: float) -> tuple[float, float]:
        """Compute target position at given simulation time (for moving animal)."""
        if not self.is_moving:
            return self.east, self.north

        # Animal moves along a gentle wandering path (speed ~0.3 m/s)
        dt = sim_time_s
        d_east = 12.0 * np.sin(0.04 * dt)
        d_north = 8.0 * np.cos(0.03 * dt)
        return self.east + float(d_east), self.north + float(d_north)


def get_default_targets() -> list[GroundTruthTarget]:
    """Define the 3 human casualties and 5 decoys per project specification."""
    return [
        # --- 3 Human Casualties ---
        GroundTruthTarget(
            id="T-HUMAN-01",
            name="Casualty (Exposed)",
            east=30.0,
            north=50.0,
            peak_temp_c=34.0,
            radius_m=0.8,
            is_human=True,
            asymmetry_ratio=2.2,
            angle_deg=35.0,
            notes="Fully exposed person lying supine on grassland. High contrast.",
        ),
        GroundTruthTarget(
            id="T-HUMAN-02",
            name="Casualty (Partly Concealed)",
            east=-40.0,
            north=70.0,
            peak_temp_c=27.2,
            radius_m=0.45,
            is_human=True,
            asymmetry_ratio=1.4,
            angle_deg=110.0,
            notes="Casualty partly under foliage canopy. Reduced thermal aperture.",
        ),
        GroundTruthTarget(
            id="T-HUMAN-03",
            name="Casualty (Beside Boulder)",
            east=60.0,
            north=-45.0,
            peak_temp_c=31.5,
            radius_m=0.7,
            is_human=True,
            asymmetry_ratio=1.9,
            angle_deg=-20.0,
            notes="Casualty resting adjacent to rock outcrop in gully.",
        ),
        # --- 5 Decoys / Clutter ---
        GroundTruthTarget(
            id="T-DECOY-01",
            name="Warm Rock Outcrop Alpha",
            east=-70.0,
            north=-30.0,
            peak_temp_c=29.5,
            radius_m=1.8,
            is_human=False,
            notes="Sun-exposed granite boulder. Irregular boundary, diffuse thermal edges.",
        ),
        GroundTruthTarget(
            id="T-DECOY-02",
            name="Warm Rock Outcrop Bravo",
            east=85.0,
            north=80.0,
            peak_temp_c=30.8,
            radius_m=1.4,
            is_human=False,
            notes="Dark schist rock retaining solar heat.",
        ),
        GroundTruthTarget(
            id="T-DECOY-03",
            name="Wildlife (Moving Deer/Dog)",
            east=-20.0,
            north=-80.0,
            peak_temp_c=33.2,
            radius_m=0.45,
            is_human=False,
            is_moving=True,
            notes="Small animal slowly traversing terrain (~0.3 m/s).",
        ),
        GroundTruthTarget(
            id="T-DECOY-04",
            name="Vehicle Engine Block",
            east=-90.0,
            north=35.0,
            peak_temp_c=52.0,
            radius_m=1.3,
            is_human=False,
            asymmetry_ratio=1.8,
            notes="Parked utility vehicle engine hood. High peak temperature (>50C).",
        ),
        GroundTruthTarget(
            id="T-DECOY-05",
            name="Campfire Embers",
            east=15.0,
            north=-90.0,
            peak_temp_c=82.0,
            radius_m=0.35,
            is_human=False,
            notes="Residual burning embers. Very hot, concentrated single-pixel signature.",
        ),
    ]


def build_scenario_world(
    scenario_type: ScenarioType = ScenarioType.DEMO,
    seed: int | None = None,
) -> tuple[World, list[GroundTruthTarget]]:
    """Build and populate a world instance according to selected scenario."""
    if seed is None:
        if scenario_type == ScenarioType.RANDOM:
            seed = int(np.random.default_rng().integers(1, 10000))
        else:
            seed = 42

    # Environmental presets
    if scenario_type == ScenarioType.NIGHT:
        base_temp = 8.0
        var_temp = 2.5
    elif scenario_type == ScenarioType.HOT_AFTERNOON:
        base_temp = 24.0
        var_temp = 6.0
    else:
        base_temp = 16.0
        var_temp = 4.0

    world = World(seed=seed, base_temp_c=base_temp, temp_variation_c=var_temp)

    # Add sun-warmed rocky slopes / patches
    rng = np.random.default_rng(seed + 50)
    num_rock_patches = 8 if scenario_type != ScenarioType.NIGHT else 2
    for _ in range(num_rock_patches):
        rx = float(rng.uniform(-160.0, 160.0))
        ry = float(rng.uniform(-160.0, 160.0))
        rtemp = float(rng.uniform(28.0, 34.0)) if scenario_type != ScenarioType.NIGHT else 14.0
        rrad = float(rng.uniform(3.0, 8.0))
        world.paint_thermal_blob(rx, ry, peak_temp_c=rtemp, radius_m=rrad)

    # Retrieve targets
    targets = get_default_targets()

    # In hot afternoon, adjust targets' peak temperature slightly
    if scenario_type == ScenarioType.HOT_AFTERNOON:
        for t in targets:
            if not t.is_human and "Rock" in t.name:
                t.peak_temp_c += 4.0

    # Paint static targets into the baseline world temperature field
    for t in targets:
        if not t.is_moving:
            world.paint_thermal_blob(
                center_east=t.east,
                center_north=t.north,
                peak_temp_c=t.peak_temp_c,
                radius_m=t.radius_m,
                asymmetry_ratio=t.asymmetry_ratio,
                angle_deg=t.angle_deg,
            )

    return world, targets

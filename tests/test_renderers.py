"""Unit and physical verification tests for thermal and video sensor renderers."""

import numpy as np

from gcs.core.models import DronePose
from gcs.processing.colormap import apply_colormap, get_lut
from gcs.sim.scenario import ScenarioType, build_scenario_world
from gcs.sim.thermal_render import ThermalRenderer
from gcs.sim.video_render import VideoRenderer


def test_thermal_rendering_physics_at_8m_and_20m() -> None:
    """Prove exposed person is a 1-4 pixel warm blob at 8m AGL and smears at 20m AGL."""
    world, targets = build_scenario_world(ScenarioType.DEMO, seed=42)
    renderer = ThermalRenderer(seed=42)

    exposed_target = next(t for t in targets if t.id == "T-HUMAN-01")
    target_e = exposed_target.east  # 30.0m
    target_n = exposed_target.north  # 50.0m

    # 1. Render at 8m AGL directly above target
    pose_8m = DronePose(lat=46.592, lon=7.908, alt_agl=8.0, heading_deg=0.0)
    frame_8m = renderer.render(
        drone_east=target_e,
        drone_north=target_n,
        alt_agl=8.0,
        heading_deg=0.0,
        world=world,
        sim_time_s=1.0,
        pose=pose_8m,
        targets=targets,
    )
    assert frame_8m is not None
    assert frame_8m.data.shape == (24, 32)

    # Center pixel should see the warm person (~34 C peak)
    bg_median = float(np.median(frame_8m.data))
    peak_8m = float(np.max(frame_8m.data))
    assert peak_8m > bg_median + 6.0  # strong thermal contrast
    assert peak_8m >= 28.0

    # Count warm blob pixels (above bg + 6 C)
    warm_pixels_8m = int(np.sum(frame_8m.data >= bg_median + 6.0))
    # Acceptance criterion: visible 1 to 4 pixel warm blob
    assert 1 <= warm_pixels_8m <= 4, (
        f"Expected 1-4 warm pixels at 8m AGL, got {warm_pixels_8m}"
    )

    # 2. Render at 20m AGL directly above same target
    pose_20m = DronePose(lat=46.592, lon=7.908, alt_agl=20.0, heading_deg=0.0)
    frame_20m = renderer.render(
        drone_east=target_e,
        drone_north=target_n,
        alt_agl=20.0,
        heading_deg=0.0,
        world=world,
        sim_time_s=2.0,
        pose=pose_20m,
        targets=targets,
    )
    assert frame_20m is not None
    peak_20m = float(np.max(frame_20m.data))

    # Smearing: at 20m AGL, pixel ground footprint is ~0.65m x 0.53m (4x larger area),
    # so peak temperature is diluted significantly compared to 8m AGL
    assert peak_20m < peak_8m - 2.0, (
        f"Peak at 20m ({peak_20m:.1f} C) should be noticeably lower than at 8m ({peak_8m:.1f} C)"
    )


def test_thermal_shutter_calibration_blip() -> None:
    """Microbolometer shutter calibration drops 1 frame at 45s."""
    world, _ = build_scenario_world(ScenarioType.DEMO)
    renderer = ThermalRenderer()
    pose = DronePose(lat=46.592, lon=7.908, alt_agl=8.0, heading_deg=0.0)

    # Normal frame at 10s
    f1 = renderer.render(0.0, 0.0, 8.0, 0.0, world, 10.0, pose)
    assert f1 is not None

    # Shutter recalibration frame at 45s (returns None)
    f_shutter = renderer.render(0.0, 0.0, 8.0, 0.0, world, 45.0, pose)
    assert f_shutter is None


def test_video_render_and_night_vision() -> None:
    """Video renderer generates 640x480 RGB frames in day and night modes."""
    world, targets = build_scenario_world(ScenarioType.DEMO)
    renderer = VideoRenderer()

    # Day mode
    day_frame = renderer.render(0.0, 0.0, 10.0, 45.0, world, targets, 1.0)
    assert day_frame.image.shape == (480, 640, 3)
    assert day_frame.image.dtype == np.uint8

    # Night vision mode
    renderer.night_vision = True
    night_frame = renderer.render(0.0, 0.0, 10.0, 45.0, world, targets, 2.0)
    assert night_frame.image.shape == (480, 640, 3)
    # Green channel should dominate in night vision
    mean_g = float(night_frame.image[:, :, 1].mean())
    mean_r = float(night_frame.image[:, :, 0].mean())
    assert mean_g > mean_r * 2.0


def test_colormaps_lut_shapes() -> None:
    """Colormaps return 256x3 LUTs and apply cleanly to 2D float arrays."""
    for name in ["ironbow", "inferno", "grayscale"]:
        lut = get_lut(name)
        assert lut.shape == (256, 3)
        assert lut.dtype == np.uint8

    data = np.linspace(15.0, 35.0, 768, dtype=np.float32).reshape(24, 32)
    colored = apply_colormap(data, 15.0, 35.0, "ironbow")
    assert colored.shape == (24, 32, 3)
    assert colored.dtype == np.uint8

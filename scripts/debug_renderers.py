"""Debug script saving rendered thermal and visual sensor frames to disk."""

from pathlib import Path

import cv2

from gcs.core.models import DronePose
from gcs.processing.colormap import apply_colormap
from gcs.sim.scenario import ScenarioType, build_scenario_world
from gcs.sim.thermal_render import ThermalRenderer
from gcs.sim.video_render import VideoRenderer


def main() -> None:
    output_dir = Path("docs/screenshots")
    output_dir.mkdir(parents=True, exist_ok=True)

    world, targets = build_scenario_world(ScenarioType.DEMO, seed=42)
    thermal_rnd = ThermalRenderer(seed=42)
    video_rnd = VideoRenderer()

    # Target: exposed person at (30.0, 50.0)
    target = next(t for t in targets if t.id == "T-HUMAN-01")

    # 1. Thermal frame at 8m AGL
    pose_8m = DronePose(lat=46.592, lon=7.908, alt_agl=8.0, heading_deg=0.0)
    frame_8m = thermal_rnd.render(
        target.east, target.north, 8.0, 0.0, world, 1.0, pose_8m, targets=targets
    )
    assert frame_8m is not None
    img_8m_color = apply_colormap(frame_8m.data, palette_name="ironbow")
    # Upscale 10x for visual inspection
    img_8m_large = cv2.resize(img_8m_color, (320, 240), interpolation=cv2.INTER_NEAREST)
    bgr_8m = cv2.cvtColor(img_8m_large, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(output_dir / "debug_thermal_8m.png"), bgr_8m)

    # 2. Thermal frame at 20m AGL (showing smearing)
    pose_20m = DronePose(lat=46.592, lon=7.908, alt_agl=20.0, heading_deg=0.0)
    frame_20m = thermal_rnd.render(
        target.east, target.north, 20.0, 0.0, world, 2.0, pose_20m, targets=targets
    )
    assert frame_20m is not None
    img_20m_color = apply_colormap(frame_20m.data, palette_name="ironbow")
    img_20m_large = cv2.resize(img_20m_color, (320, 240), interpolation=cv2.INTER_NEAREST)
    bgr_20m = cv2.cvtColor(img_20m_large, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(output_dir / "debug_thermal_20m.png"), bgr_20m)

    # 3. Video frame at 8m AGL
    v_frame = video_rnd.render(target.east, target.north, 8.0, 0.0, world, targets, 1.0)
    bgr_v = cv2.cvtColor(v_frame.image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(output_dir / "debug_video_8m.png"), bgr_v)

    print(f"Rendered sensor frames saved to {output_dir.resolve()}:")
    print(" - debug_thermal_8m.png (Exposed casualty: distinct 1-4 pixel hotspot)")
    print(" - debug_thermal_20m.png (High altitude: sensor footprint smearing)")
    print(" - debug_video_8m.png (640x480 RGB visual camera)")


if __name__ == "__main__":
    main()

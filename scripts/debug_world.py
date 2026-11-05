"""Debug script to inspect and save terrain heightmap, hillshade, and temperature PNGs."""

from pathlib import Path

import cv2
import numpy as np

from gcs.sim.scenario import ScenarioType, build_scenario_world


def main() -> None:
    output_dir = Path("docs/screenshots")
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Generating simulated world for DEMO scenario...")
    world, targets = build_scenario_world(ScenarioType.DEMO, seed=42)

    # 1. Heightmap PNG (normalized 0-255)
    h_norm = (
        (world.heightmap - world.heightmap.min())
        / (world.heightmap.max() - world.heightmap.min())
        * 255.0
    ).astype(np.uint8)
    cv2.imwrite(str(output_dir / "debug_heightmap.png"), h_norm)

    # 2. Hillshade PNG
    cv2.imwrite(str(output_dir / "debug_hillshade.png"), world.hillshade)

    # 3. Ground Temperature PNG (with COLORMAP_INFERNO)
    t_min, t_max = world.temperature_field.min(), world.temperature_field.max()
    t_norm = ((world.temperature_field - t_min) / (t_max - t_min) * 255.0).astype(np.uint8)
    t_color = cv2.applyColorMap(t_norm, cv2.COLORMAP_INFERNO)
    cv2.imwrite(str(output_dir / "debug_temperature.png"), t_color)

    print(f"Successfully saved PNGs to {output_dir.resolve()}:")
    print(" - debug_heightmap.png")
    print(" - debug_hillshade.png")
    print(" - debug_temperature.png")

    print("\n" + "=" * 76)
    print("GROUND TRUTH TARGETS (DEMO SCENARIO)")
    print("=" * 76)
    header = (
        f"{'ID':<12} | {'Name':<26} | {'East':>7} | {'North':>7} | {'Peak °C':>7} | {'Human'}"
    )
    print(header)
    print("-" * 76)
    for t in targets:
        row = (
            f"{t.id:<12} | {t.name:<26} | {t.east:>7.1f} | {t.north:>7.1f} | "
            f"{t.peak_temp_c:>7.1f} | {str(t.is_human):<5}"
        )
        print(row)
    print("=" * 76)


if __name__ == "__main__":
    main()

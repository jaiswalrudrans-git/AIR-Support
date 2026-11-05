"""Integration test executing an automated lawnmower survey grid at 8m AGL.

Validates the complete thermal pipeline:
World -> Sensor Render -> Detection -> Geotagging -> Spatiotemporal Tracking.
"""

from __future__ import annotations

import math

from gcs.config import AppConfig
from gcs.core.geo import enu_to_latlon
from gcs.core.models import DronePose, HotspotStatus
from gcs.processing.detector import ThermalDetector
from gcs.processing.tracker import HotspotTracker
from gcs.sim.scenario import ScenarioType, build_scenario_world
from gcs.sim.thermal_render import ThermalRenderer


def test_automated_lawnmower_survey_pipeline() -> None:
    """Execute automated back-and-forth survey grid at 8m AGL.

    Acceptance Criteria:
    - All exposed casualties (T-HUMAN-01, T-HUMAN-03) confirmed.
    - Partly covered casualty (T-HUMAN-02) detected in at least one pass.
    - Mean position error of confirmed human casualties against ground truth < 5m.
    - Precision/Recall report generated.
    """
    config = AppConfig()
    world, targets = build_scenario_world(ScenarioType.DEMO, seed=42)
    renderer = ThermalRenderer(config.thermal, seed=42)
    detector = ThermalDetector(config.detector)
    tracker = HotspotTracker(config.tracker)

    # Lawnmower survey grid at 8m AGL
    # Waypoints covering casualties and decoys across target operational lanes
    survey_lines_n = [-45.0, -30.0, 35.0, 50.0, 70.0]
    dt_step_s = 0.25  # 4 Hz sampling during survey sweep

    sim_time = 0.0

    # Execute 3-pass survey to satisfy multi-pass temporal confirmation criteria
    for pass_num in range(3):
        for line_idx, north_coord in enumerate(survey_lines_n):
            # Alternating direction (East -> West, then West -> East)
            if (line_idx + pass_num) % 2 == 0:
                east_coords = list(range(-60, 75, 3))
            else:
                east_coords = list(range(75, -60, -3))

            for east_coord in east_coords:
                sim_time += dt_step_s
                drone_e = float(east_coord)
                drone_n = float(north_coord)
                alt_agl = 8.0
                heading_deg = 90.0 if (line_idx + pass_num) % 2 == 0 else 270.0

                lat, lon, _ = enu_to_latlon(
                    drone_e,
                    drone_n,
                    alt_agl,
                    config.geo.origin_lat,
                    config.geo.origin_lon,
                    config.geo.origin_alt_msl,
                )
                pose = DronePose(
                    lat=lat,
                    lon=lon,
                    alt_agl=alt_agl,
                    heading_deg=heading_deg,
                )

                # 1. Physical sensor render
                frame = renderer.render(
                    drone_east=drone_e,
                    drone_north=drone_n,
                    alt_agl=alt_agl,
                    heading_deg=heading_deg,
                    world=world,
                    sim_time_s=sim_time,
                    pose=pose,
                    targets=targets,
                )
                if frame is None:
                    continue

                # 2. Anomaly detection
                detections = detector.process_frame(frame)

                # 3. Spatial geotagging and temporal tracking
                for det in detections:
                    tracker.process_detection(det, frame, config.geo)

    # --- Pipeline Evaluation Against Ground Truth ---
    human_targets = [t for t in targets if t.is_human]
    confirmed_hotspots = [
        h for h in tracker.hotspots.values() if h.status == HotspotStatus.CONFIRMED
    ]
    all_hotspots = list(tracker.hotspots.values())

    # Match confirmed hotspots to human ground truth targets
    matched_humans: set[str] = set()
    position_errors_m: list[float] = []

    for h in confirmed_hotspots:
        he, hn = tracker.get_hotspot_metric_position(h.id) or (0.0, 0.0)
        # Find closest human target
        for ht in human_targets:
            dist = math.hypot(he - ht.east, hn - ht.north)
            if dist < 5.0:  # within 5m match radius
                matched_humans.add(ht.id)
                position_errors_m.append(dist)
                break

    # Check partly covered casualty in candidate or confirmed list
    partly_covered = next(t for t in human_targets if t.id == "T-HUMAN-02")
    partly_covered_found = False
    for h in all_hotspots:
        he, hn = tracker.get_hotspot_metric_position(h.id) or (0.0, 0.0)
        if math.hypot(he - partly_covered.east, hn - partly_covered.north) < 6.0:
            partly_covered_found = True
            break

    # Calculate metrics
    true_positives = len(matched_humans)
    false_positives = len(confirmed_hotspots) - true_positives

    precision = true_positives / max(1, true_positives + false_positives)
    recall = true_positives / max(1, len(human_targets))
    f1 = 2.0 * precision * recall / max(0.01, precision + recall)

    mean_pos_error = (
        sum(position_errors_m) / len(position_errors_m) if position_errors_m else 0.0
    )

    # Print Precision / Recall report
    print("\n" + "=" * 65)
    print("THERMAL SEARCH-AND-RESCUE DETECTION PIPELINE REPORT")
    print("=" * 65)
    print(f"Total Hotspots Tracked:      {len(all_hotspots)}")
    print(f"Confirmed Hotspots:          {len(confirmed_hotspots)}")
    print(f"Human Casualties Matched:    {true_positives} / {len(human_targets)}")
    print(f"Partly Covered Found:        {partly_covered_found}")
    print(f"Mean Position Error:         {mean_pos_error:.2f} m")
    print(f"Detection Precision:         {precision:.1%}")
    print(f"Detection Recall:            {recall:.1%}")
    print(f"Detection F1-Score:          {f1:.2f}")
    print("=" * 65)

    # Acceptance Criteria Verification:
    # 1. Exposed casualties found
    assert "T-HUMAN-01" in matched_humans, "Exposed casualty T-HUMAN-01 was not confirmed"
    assert "T-HUMAN-03" in matched_humans, "Casualty beside boulder T-HUMAN-03 was not confirmed"

    # 2. Partly covered casualty detected in at least one pass
    assert partly_covered_found, (
        "Partly covered casualty T-HUMAN-02 was not detected in any survey pass"
    )

    # 3. Mean position error strictly under 5m
    assert mean_pos_error < 5.0, (
        f"Mean position error ({mean_pos_error:.2f}m) exceeds 5.0m acceptance threshold"
    )

    # 4. Decoys produce bounded false hotspots (<= 3 confirmed decoys e.g. campfire/engine)
    assert false_positives <= 3, f"False positive hotspots ({false_positives}) exceeded threshold"

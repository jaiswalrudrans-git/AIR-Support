"""Unit tests for geographic math, coordinate transforms, and footprint projection."""

import math

from gcs.core.geo import (
    calculate_footprint_dimensions,
    enu_to_latlon,
    latlon_to_enu,
    project_pixel_to_ground,
)


def test_enu_latlon_roundtrip_precision() -> None:
    """Prove round-trip lat/lon <-> ENU error is strictly under 0.01 m."""
    origin_lat = 46.5920
    origin_lon = 7.9080
    origin_alt = 850.0

    # Test test points across the 400x400m operating region
    test_offsets = [
        (0.0, 0.0, 0.0),
        (150.0, -180.0, 25.0),
        (-200.0, 200.0, -10.0),
        (50.5, 75.3, 12.0),
        (-120.0, -140.0, 45.0),
    ]

    for east_in, north_in, up_in in test_offsets:
        lat, lon, alt = enu_to_latlon(east_in, north_in, up_in, origin_lat, origin_lon, origin_alt)
        east_out, north_out, up_out = latlon_to_enu(
            lat, lon, alt, origin_lat, origin_lon, origin_alt
        )

        error_east = abs(east_out - east_in)
        error_north = abs(north_out - north_in)
        error_up = abs(up_out - up_in)
        total_3d_error = math.sqrt(error_east**2 + error_north**2 + error_up**2)

        # Requirement: strictly under 0.01 m
        assert total_3d_error < 0.01, (
            f"Round-trip error {total_3d_error:.6f}m exceeds 0.01m tolerance"
        )


def test_center_pixel_projects_exactly_below_drone() -> None:
    """Prove pixel at image centre projects exactly below the drone."""
    drone_east = 50.0
    drone_north = -75.0
    altitude_agl = 10.0
    heading_deg = 45.0  # arbitrary heading

    # Sensor: 32 columns x 24 rows
    img_w, img_h = 32, 24
    center_px = 16.0
    center_py = 12.0

    gx, gy = project_pixel_to_ground(
        px=center_px,
        py=center_py,
        img_width=img_w,
        img_height=img_h,
        altitude_agl=altitude_agl,
        heading_deg=heading_deg,
        fov_h_deg=55.0,
        fov_v_deg=35.0,
        drone_east=drone_east,
        drone_north=drone_north,
    )

    assert abs(gx - drone_east) < 1e-6
    assert abs(gy - drone_north) < 1e-6


def test_footprint_width_at_10m_with_55_deg_lens() -> None:
    """Footprint width at 10 m with 55 deg lens is ~10.4 m, giving ~0.33 m per pixel."""
    altitude_agl = 10.0
    fov_h_deg = 55.0
    fov_v_deg = 35.0

    width_m, height_m = calculate_footprint_dimensions(altitude_agl, fov_h_deg, fov_v_deg)

    # Theoretical: 2 * 10 * tan(27.5 deg) ~= 10.411 m
    assert 10.35 < width_m < 10.45, f"Footprint width was {width_m:.3f}m, expected ~10.4m"

    # Per pixel width for 32 columns: ~0.325 m (about 0.33 m)
    pixel_width_m = width_m / 32.0
    assert 0.32 < pixel_width_m < 0.34, f"Pixel width was {pixel_width_m:.3f}m, expected ~0.33m"

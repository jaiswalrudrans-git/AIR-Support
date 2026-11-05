"""Geographic transformations, local ENU projection, and footprint geometry.

Provides geodesy calculations anchored at a local tangent frame and flat-ground
pinhole projection for downward-looking thermal and visual sensors.
"""

from __future__ import annotations

import math

# WGS84 Ellipsoid constants
WGS84_A = 6378137.0  # semi-major axis (meters)
WGS84_F = 1.0 / 298.257223563  # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)  # first eccentricity squared


def _radii_of_curvature(lat_deg: float) -> tuple[float, float]:
    """Compute meridian (M) and prime vertical (N) radii of curvature at latitude."""
    lat_rad = math.radians(lat_deg)
    sin_lat = math.sin(lat_rad)
    w = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    n = WGS84_A / w
    m = WGS84_A * (1.0 - WGS84_E2) / (w * w * w)
    return m, n


def latlon_to_enu(
    lat: float,
    lon: float,
    alt_msl: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt_msl: float,
) -> tuple[float, float, float]:
    """Convert WGS84 (lat, lon, alt_msl) to local East-North-Up (meters).

    Uses a local tangent plane approximation with WGS84 curvature radii,
    giving sub-millimeter precision over local search areas (< 20 km).
    """
    m, n = _radii_of_curvature(origin_lat)
    d_lat_rad = math.radians(lat - origin_lat)
    d_lon_rad = math.radians(lon - origin_lon)
    lat_rad = math.radians(origin_lat)

    north = d_lat_rad * m
    east = d_lon_rad * n * math.cos(lat_rad)
    up = alt_msl - origin_alt_msl
    return east, north, up


def enu_to_latlon(
    east: float,
    north: float,
    up: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt_msl: float,
) -> tuple[float, float, float]:
    """Convert local East-North-Up (meters) to WGS84 (lat, lon, alt_msl)."""
    m, n = _radii_of_curvature(origin_lat)
    lat_rad = math.radians(origin_lat)

    d_lat_deg = math.degrees(north / m)
    d_lon_deg = math.degrees(east / (n * math.cos(lat_rad)))

    lat = origin_lat + d_lat_deg
    lon = origin_lon + d_lon_deg
    alt_msl = origin_alt_msl + up
    return lat, lon, alt_msl


def calculate_footprint_dimensions(
    altitude_agl: float,
    fov_h_deg: float,
    fov_v_deg: float,
) -> tuple[float, float]:
    """Calculate ground footprint width and height in meters for a nadir sensor.

    Width corresponds to horizontal FOV, height corresponds to vertical FOV.
    """
    if altitude_agl <= 0.0:
        return 0.0, 0.0

    width_m = 2.0 * altitude_agl * math.tan(math.radians(fov_h_deg / 2.0))
    height_m = 2.0 * altitude_agl * math.tan(math.radians(fov_v_deg / 2.0))
    return width_m, height_m


def project_pixel_to_ground(
    px: float,
    py: float,
    img_width: int,
    img_height: int,
    altitude_agl: float,
    heading_deg: float,
    fov_h_deg: float,
    fov_v_deg: float,
    drone_east: float = 0.0,
    drone_north: float = 0.0,
) -> tuple[float, float]:
    """Project a 2D sensor pixel to ground ENU coordinates (meters).

    Assumptions:
    - Downward-looking (nadir) gimbal or fixed mount.
    - Flat ground approximation relative to drone altitude AGL.
    - Image coordinate system: (0, 0) is top-left.
    - Drone body orientation: top of frame (py=0) aligns with drone heading (forward),
      right of frame (px=img_width) aligns with drone starboard (right).

    Returns:
        (east, north) coordinate on ground in local metric frame.
    """
    if altitude_agl <= 0.0:
        return drone_east, drone_north

    # Normalized sensor coordinates relative to center [-1.0, 1.0]
    center_x = img_width / 2.0
    center_y = img_height / 2.0

    norm_x = (px - center_x) / center_x
    norm_y = (center_y - py) / center_y  # invert so +Y is forward (top of image)

    # Angular displacement from optical axis
    tan_half_h = math.tan(math.radians(fov_h_deg / 2.0))
    tan_half_v = math.tan(math.radians(fov_v_deg / 2.0))

    # Body frame offsets (forward, right)
    d_forward = altitude_agl * (norm_y * tan_half_v)
    d_right = altitude_agl * (norm_x * tan_half_h)

    # Rotate by drone heading (degrees clockwise from North)
    heading_rad = math.radians(heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)

    # Delta East and North
    delta_east = d_forward * sin_h + d_right * cos_h
    delta_north = d_forward * cos_h - d_right * sin_h

    ground_east = drone_east + delta_east
    ground_north = drone_north + delta_north
    return ground_east, ground_north

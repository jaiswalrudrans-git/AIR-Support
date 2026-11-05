"""Spatial geotagging projecting thermal pixel centroids to WGS84 geographic coordinates."""

from __future__ import annotations

from gcs.config import GeoConfig
from gcs.core.geo import enu_to_latlon, latlon_to_enu, project_pixel_to_ground
from gcs.core.models import ThermalFrame
from gcs.processing.detector import RawDetection


def geotag_detection(
    detection: RawDetection,
    frame: ThermalFrame,
    geo_config: GeoConfig,
) -> tuple[float, float, float, float]:
    """Project a 2D pixel anomaly into ground East-North-Up (m) and WGS84 (lat, lon).

    Returns:
        (lat, lon, ground_east, ground_north)
    """
    pose = frame.drone_pose

    # Convert drone GPS to local metric frame
    drone_e, drone_n, _ = latlon_to_enu(
        pose.lat,
        pose.lon,
        pose.alt_agl + geo_config.origin_alt_msl,
        geo_config.origin_lat,
        geo_config.origin_lon,
        geo_config.origin_alt_msl,
    )

    fov_h, fov_v = frame.sensor_fov_deg

    # Ray-cast pixel to ground plane relative to drone pose
    ground_e, ground_n = project_pixel_to_ground(
        px=detection.px,
        py=detection.py,
        img_width=frame.data.shape[1],  # 32
        img_height=frame.data.shape[0],  # 24
        altitude_agl=pose.alt_agl,
        heading_deg=pose.heading_deg,
        fov_h_deg=fov_h,
        fov_v_deg=fov_v,
        drone_east=drone_e,
        drone_north=drone_n,
    )

    # Convert ground metric position back to WGS84 latitude and longitude
    target_lat, target_lon, _ = enu_to_latlon(
        east=ground_e,
        north=ground_n,
        up=0.0,
        origin_lat=geo_config.origin_lat,
        origin_lon=geo_config.origin_lon,
        origin_alt_msl=geo_config.origin_alt_msl,
    )

    return target_lat, target_lon, ground_e, ground_n

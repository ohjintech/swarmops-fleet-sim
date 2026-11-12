"""Flat-earth helpers. Plenty accurate over the few-km areas a swarm operates in."""

from __future__ import annotations

import math

M_PER_DEG_LAT = 111_320.0


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Return (lat, lon) moved north/east by the given metres."""
    dlat = north_m / M_PER_DEG_LAT
    dlon = east_m / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def delta_m(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    """Return (north_m, east_m) from point 1 to point 2."""
    north = (lat2 - lat1) * M_PER_DEG_LAT
    east = (lon2 - lon1) * M_PER_DEG_LAT * math.cos(math.radians((lat1 + lat2) / 2))
    return north, east


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north, east = delta_m(lat1, lon1, lat2, lon2)
    return math.hypot(north, east)


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north, east = delta_m(lat1, lon1, lat2, lon2)
    return (math.degrees(math.atan2(east, north)) + 360.0) % 360.0

from __future__ import annotations

import os
from dataclasses import dataclass


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


@dataclass(frozen=True)
class Config:
    mqtt_host: str = _env("MQTT_HOST", "localhost")
    mqtt_port: int = int(_env("MQTT_PORT", "1883"))
    mqtt_username: str | None = os.environ.get("MQTT_USERNAME")
    mqtt_password: str | None = os.environ.get("MQTT_PASSWORD")

    fleet_id: str = _env("FLEET_ID", "fleet-1")
    drone_count: int = int(_env("DRONE_COUNT", "10"))
    # Default launch site: Crissy Field, San Francisco.
    home_lat: float = float(_env("HOME_LAT", "37.8037"))
    home_lon: float = float(_env("HOME_LON", "-122.4650"))
    pad_spacing_m: float = float(_env("PAD_SPACING_M", "6"))

    physics_hz: float = float(_env("PHYSICS_HZ", "20"))
    telemetry_hz: float = float(_env("TELEMETRY_HZ", "2"))

    link_latency_ms: float = float(_env("LINK_LATENCY_MS", "60"))
    link_jitter_ms: float = float(_env("LINK_JITTER_MS", "40"))
    link_loss_pct: float = float(_env("LINK_LOSS_PCT", "1"))

    chaos_host: str = _env("CHAOS_HOST", "0.0.0.0")
    chaos_port: int = int(_env("CHAOS_PORT", "8090"))

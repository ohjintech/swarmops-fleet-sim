"""Chaos HTTP API: inject faults and reshape the fleet while it runs.

This is a test harness, not part of the "drone" surface. The backend never calls it.
"""

from __future__ import annotations

import time

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
from swarmops.drone.v1 import drone_pb2 as pb

from .drone import SimDrone
from .fleet import Fleet


class LinkUpdate(BaseModel):
    up: bool


class BatteryUpdate(BaseModel):
    pct: float = Field(ge=0, le=100)


class GlobalLink(BaseModel):
    latency_ms: float | None = Field(default=None, ge=0)
    jitter_ms: float | None = Field(default=None, ge=0)
    loss_pct: float | None = Field(default=None, ge=0, le=100)


class Spawn(BaseModel):
    count: int = Field(ge=1, le=1000)


def create_app(fleet: Fleet) -> FastAPI:
    app = FastAPI(title="swarmops fleet-sim chaos API")

    def get(drone_id: str) -> SimDrone:
        d = fleet.drones.get(drone_id)
        if d is None:
            raise HTTPException(404, f"unknown drone {drone_id}")
        return d

    @app.get("/healthz")
    def healthz() -> dict:
        return {"ok": True, "drones": len(fleet.drones)}

    @app.get("/drones")
    def list_drones() -> list[dict]:
        return [
            {
                "drone_id": d.drone_id,
                "state": pb.FlightState.Name(d.state),
                "battery_pct": round(d.battery_pct, 1),
                "alt_m": round(d.alt, 1),
                "link_up": d.link_up,
            }
            for d in fleet.drones.values()
        ]

    @app.post("/drones/{drone_id}/link")
    def set_link(drone_id: str, body: LinkUpdate) -> dict:
        get(drone_id).link_up = body.up
        return {"ok": True}

    @app.post("/drones/{drone_id}/battery")
    def set_battery(drone_id: str, body: BatteryUpdate) -> dict:
        get(drone_id).battery_pct = body.pct
        return {"ok": True}

    @app.post("/drones/{drone_id}/fault")
    def fault(drone_id: str) -> dict:
        d = get(drone_id)
        d.inject_fault(time.time())
        return {"ok": True}

    @app.post("/drones/{drone_id}/reset")
    def reset(drone_id: str) -> dict:
        get(drone_id).reset()
        return {"ok": True}

    @app.post("/link")
    def set_global_link(body: GlobalLink) -> dict:
        for k, v in body.model_dump(exclude_none=True).items():
            setattr(fleet.link, k, v)
        return {"ok": True, "link": vars(fleet.link)}

    @app.post("/spawn")
    def spawn(body: Spawn) -> dict:
        return {"spawned": fleet.spawn(body.count)}

    return app

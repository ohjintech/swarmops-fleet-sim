"""Pure drone model: no I/O, no clocks. The fleet runner feeds it time and commands.

Behaviour deliberately mirrors a real autopilot:
  * commands can be rejected, are acknowledged, and can be preempted;
  * the drone owns its own failsafes (link loss -> RTL, low battery -> RTL / land),
    so the backend is never the only safety net.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from google.protobuf.timestamp_pb2 import Timestamp
from swarmops.drone.v1 import drone_pb2 as pb

from . import geo

FS = pb.FlightState
AIRBORNE = {
    FS.FLIGHT_STATE_TAKING_OFF,
    FS.FLIGHT_STATE_HOLDING,
    FS.FLIGHT_STATE_EN_ROUTE,
    FS.FLIGHT_STATE_RETURNING,
    FS.FLIGHT_STATE_LANDING,
}

ARRIVE_RADIUS_M = 1.0
RTL_MIN_ALT_M = 30.0
BATTERY_WARN_PCT = 25.0
BATTERY_RTL_PCT = 15.0
BATTERY_LAND_PCT = 5.0
MIN_TAKEOFF_BATTERY_PCT = 30.0


def _ts(now: float) -> Timestamp:
    t = Timestamp()
    t.FromNanoseconds(int(now * 1e9))
    return t


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float


@dataclass
class SimDrone:
    drone_id: str
    fleet_id: str
    home_lat: float
    home_lon: float

    cruise_speed_mps: float = 12.0
    climb_rate_mps: float = 3.0
    descent_rate_mps: float = 2.0
    link_failsafe_s: float = 10.0

    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    heading_deg: float = 0.0
    ground_speed_mps: float = 0.0
    climb_rate: float = 0.0
    battery_pct: float = 100.0
    state: int = FS.FLIGHT_STATE_LANDED

    route: list[Waypoint] = field(default_factory=list)
    route_speed_mps: float = 0.0
    target_alt: float = 0.0
    active_command_id: str = ""

    link_up: bool = True
    link_down_for_s: float = 0.0
    _battery_warned: bool = False

    # (kind, message) pairs for the runner to publish; kind is "ack" or "event".
    outbox: list[tuple[str, object]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.lat == 0.0 and self.lon == 0.0:
            self.lat, self.lon = self.home_lat, self.home_lon

    @property
    def airborne(self) -> bool:
        return self.state in AIRBORNE

    # ------------------------------------------------------------------ commands

    def handle_command(self, cmd: pb.Command, now: float) -> None:
        if cmd.HasField("expires_at") and cmd.expires_at.ToNanoseconds() < now * 1e9:
            self._ack(cmd.command_id, pb.ACK_STATUS_REJECTED, now, "command expired")
            return
        if self.state == FS.FLIGHT_STATE_FAULT:
            self._ack(cmd.command_id, pb.ACK_STATUS_REJECTED, now, "drone in fault state")
            return

        kind = cmd.WhichOneof("kind")
        reason = self._check(kind)
        if reason:
            self._ack(cmd.command_id, pb.ACK_STATUS_REJECTED, now, reason)
            return

        self._preempt(now)
        self.active_command_id = cmd.command_id
        self._ack(cmd.command_id, pb.ACK_STATUS_ACCEPTED, now)

        if kind == "takeoff":
            self.target_alt = cmd.takeoff.alt_m or RTL_MIN_ALT_M
            self.state = FS.FLIGHT_STATE_TAKING_OFF
        elif kind == "go_to":
            t = cmd.go_to.target
            self._set_route([Waypoint(t.lat, t.lon, t.alt_m or self.alt)], cmd.go_to.speed_mps)
            self.state = FS.FLIGHT_STATE_EN_ROUTE
        elif kind == "follow_route":
            wps = [Waypoint(w.lat, w.lon, w.alt_m or self.alt) for w in cmd.follow_route.waypoints]
            self._set_route(wps, cmd.follow_route.speed_mps)
            self.state = FS.FLIGHT_STATE_EN_ROUTE
        elif kind == "hold":
            self.route.clear()
            self.state = FS.FLIGHT_STATE_HOLDING
            self._complete(now)
        elif kind == "return_to_launch":
            self._start_rtl()
        elif kind == "land":
            self.route.clear()
            self.state = FS.FLIGHT_STATE_LANDING

    def _check(self, kind: str | None) -> str:
        if kind is None:
            return "empty command"
        if kind == "takeoff":
            if self.state != FS.FLIGHT_STATE_LANDED:
                return "already airborne"
            if self.battery_pct < MIN_TAKEOFF_BATTERY_PCT:
                return f"battery below {MIN_TAKEOFF_BATTERY_PCT:.0f}%"
            return ""
        if not self.airborne:
            return "drone is landed"
        if kind == "follow_route":
            return ""  # empty routes are caught by the backend
        if kind in ("go_to", "hold") and self.state == FS.FLIGHT_STATE_LANDING:
            return "drone is landing"
        return ""

    def _set_route(self, wps: list[Waypoint], speed: float) -> None:
        self.route = wps
        self.route_speed_mps = speed or self.cruise_speed_mps

    def _start_rtl(self) -> None:
        rtl_alt = max(self.alt, RTL_MIN_ALT_M)
        self._set_route(
            [Waypoint(self.lat, self.lon, rtl_alt), Waypoint(self.home_lat, self.home_lon, rtl_alt)],
            self.cruise_speed_mps,
        )
        self.state = FS.FLIGHT_STATE_RETURNING

    # ------------------------------------------------------------------ physics

    def step(self, dt: float, now: float) -> None:
        if self.link_up:
            self.link_down_for_s = 0.0
        else:
            self.link_down_for_s += dt

        self._drain_battery(dt)
        self._failsafes(now)

        if self.state == FS.FLIGHT_STATE_TAKING_OFF:
            self._move_vertical(self.target_alt, dt)
            if abs(self.alt - self.target_alt) < 0.1:
                self.state = FS.FLIGHT_STATE_HOLDING
                self._complete(now)
        elif self.state in (FS.FLIGHT_STATE_EN_ROUTE, FS.FLIGHT_STATE_RETURNING):
            self._follow_route(dt, now)
        elif self.state in (FS.FLIGHT_STATE_LANDING, FS.FLIGHT_STATE_FAULT):
            rate = self.descent_rate_mps * (3 if self.state == FS.FLIGHT_STATE_FAULT else 1)
            self.ground_speed_mps = 0.0
            self._move_vertical(0.0, dt, rate)
            if self.alt <= 0.0 and self.state == FS.FLIGHT_STATE_LANDING:
                self.state = FS.FLIGHT_STATE_LANDED
                self._complete(now)
        else:
            self.ground_speed_mps = 0.0
            self.climb_rate = 0.0

    def _follow_route(self, dt: float, now: float) -> None:
        if not self.route:
            self._route_done(now)
            return
        wp = self.route[0]
        dist = geo.distance_m(self.lat, self.lon, wp.lat, wp.lon)
        self._move_vertical(wp.alt, dt)
        if dist <= ARRIVE_RADIUS_M and abs(self.alt - wp.alt) < 0.5:
            self.route.pop(0)
            if not self.route:
                self._route_done(now)
            return
        step = min(self.route_speed_mps * dt, dist)
        if dist > 0:
            self.heading_deg = geo.bearing_deg(self.lat, self.lon, wp.lat, wp.lon)
            north, east = geo.delta_m(self.lat, self.lon, wp.lat, wp.lon)
            self.lat, self.lon = geo.offset(self.lat, self.lon, north / dist * step, east / dist * step)
        self.ground_speed_mps = step / dt if dt else 0.0

    def _route_done(self, now: float) -> None:
        self.ground_speed_mps = 0.0
        if self.state == FS.FLIGHT_STATE_RETURNING:
            self.state = FS.FLIGHT_STATE_LANDING  # RTL command completes on touchdown
        else:
            self.state = FS.FLIGHT_STATE_HOLDING
            self._complete(now)

    def _move_vertical(self, target: float, dt: float, rate: float | None = None) -> None:
        diff = target - self.alt
        if diff == 0:
            self.climb_rate = 0.0
            return
        r = rate if rate is not None else (self.climb_rate_mps if diff > 0 else self.descent_rate_mps)
        dz = math.copysign(min(abs(diff), r * dt), diff)
        self.alt = max(0.0, self.alt + dz)
        self.climb_rate = dz / dt if dt else 0.0

    def _drain_battery(self, dt: float) -> None:
        if not self.airborne:
            return
        # hover cost + speed cost + climb cost, in %/s (roughly 20-25 min flights)
        drain = 0.07 + 0.002 * self.ground_speed_mps + 0.01 * max(self.climb_rate, 0.0)
        self.battery_pct = max(0.0, self.battery_pct - drain * dt)

    def _failsafes(self, now: float) -> None:
        if not self.airborne or self.state == FS.FLIGHT_STATE_LANDING:
            return
        if self.battery_pct <= BATTERY_LAND_PCT:
            self._failsafe(now, "FAILSAFE_LAND", "battery critical, landing in place")
            self.route.clear()
            self.state = FS.FLIGHT_STATE_LANDING
            return
        if self.state == FS.FLIGHT_STATE_RETURNING:
            return
        if self.battery_pct <= BATTERY_RTL_PCT:
            self._failsafe(now, "FAILSAFE_RTL", "battery low, returning to launch")
            self._start_rtl()
        elif self.link_down_for_s >= self.link_failsafe_s:
            self._failsafe(now, "FAILSAFE_RTL", f"link lost for {self.link_down_for_s:.0f}s, returning to launch")
            self._start_rtl()
        elif self.battery_pct <= BATTERY_WARN_PCT and not self._battery_warned:
            self._battery_warned = True
            self._event(now, pb.SEVERITY_WARNING, "BATTERY_LOW", f"battery at {self.battery_pct:.0f}%")

    def _failsafe(self, now: float, code: str, msg: str) -> None:
        self._preempt(now, reason=msg)
        self._event(now, pb.SEVERITY_CRITICAL, code, msg)

    # ------------------------------------------------------------------ chaos hooks

    def inject_fault(self, now: float, code: str = "MOTOR_FAULT") -> None:
        self._preempt(now, reason="fault")
        self.route.clear()
        self.state = FS.FLIGHT_STATE_FAULT
        self._event(now, pb.SEVERITY_CRITICAL, code, "simulated hardware fault")

    def reset(self) -> None:
        self.lat, self.lon, self.alt = self.home_lat, self.home_lon, 0.0
        self.state = FS.FLIGHT_STATE_LANDED
        self.battery_pct = 100.0
        self.route.clear()
        self.active_command_id = ""
        self.link_up = True
        self._battery_warned = False

    # ------------------------------------------------------------------ outputs

    def _preempt(self, now: float, reason: str = "preempted by newer command") -> None:
        if self.active_command_id:
            self._ack(self.active_command_id, pb.ACK_STATUS_FAILED, now, reason)
            self.active_command_id = ""

    def _complete(self, now: float) -> None:
        if self.active_command_id:
            self._ack(self.active_command_id, pb.ACK_STATUS_COMPLETED, now)
            self.active_command_id = ""

    def _ack(self, command_id: str, status: int, now: float, reason: str = "") -> None:
        self.outbox.append(
            ("ack", pb.CommandAck(command_id=command_id, drone_id=self.drone_id, ts=_ts(now), status=status, reason=reason))
        )

    def _event(self, now: float, severity: int, code: str, message: str) -> None:
        self.outbox.append(
            ("event", pb.DroneEvent(drone_id=self.drone_id, ts=_ts(now), severity=severity, code=code, message=message))
        )

    def telemetry(self, seq: int, now: float, link_quality: float) -> pb.Telemetry:
        return pb.Telemetry(
            drone_id=self.drone_id,
            fleet_id=self.fleet_id,
            ts=_ts(now),
            seq=seq,
            position=pb.Position(lat=self.lat, lon=self.lon, alt_m=self.alt),
            home=pb.Position(lat=self.home_lat, lon=self.home_lon, alt_m=0.0),
            heading_deg=self.heading_deg,
            ground_speed_mps=self.ground_speed_mps,
            climb_rate_mps=self.climb_rate,
            battery_pct=self.battery_pct,
            state=self.state,
            link_quality=link_quality,
            active_command_id=self.active_command_id,
        )

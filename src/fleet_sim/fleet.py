"""Runs the drones and speaks MQTT. This is the only module that touches the network."""

from __future__ import annotations

import asyncio
import logging
import time

import aiomqtt
from swarmops.drone.v1 import drone_pb2 as pb

from . import geo
from .config import Config
from .drone import SimDrone
from .link import LinkModel

log = logging.getLogger(__name__)

PAD_COLUMNS = 10


def topic(fleet_id: str, drone_id: str, kind: str) -> str:
    return f"swarm/v1/{fleet_id}/{drone_id}/{kind}"


class Fleet:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.link = LinkModel(cfg.link_latency_ms, cfg.link_jitter_ms, cfg.link_loss_pct)
        self.drones: dict[str, SimDrone] = {}
        self._seq: dict[str, int] = {}
        self._client: aiomqtt.Client | None = None
        self._pending: set[asyncio.Task] = set()
        # QoS 1 messages a drone produced while its link was down; flushed on reconnect.
        self._backlog: dict[str, list[tuple[str, bytes]]] = {}
        self.spawn(cfg.drone_count)

    # ------------------------------------------------------------------ fleet management

    def spawn(self, count: int) -> list[str]:
        """Add drones on a grid of launch pads (PAD_COLUMNS wide) north-east of the home point."""
        ids = []
        start = len(self.drones)
        for i in range(start, start + count):
            row, col = divmod(i, PAD_COLUMNS)
            lat, lon = geo.offset(
                self.cfg.home_lat, self.cfg.home_lon, row * self.cfg.pad_spacing_m, col * self.cfg.pad_spacing_m
            )
            drone_id = f"d-{i + 1:03d}"
            self.drones[drone_id] = SimDrone(drone_id=drone_id, fleet_id=self.cfg.fleet_id, home_lat=lat, home_lon=lon)
            self._seq[drone_id] = 0
            ids.append(drone_id)
        return ids

    # ------------------------------------------------------------------ main loop

    async def run(self) -> None:
        while True:
            try:
                async with aiomqtt.Client(
                    hostname=self.cfg.mqtt_host,
                    port=self.cfg.mqtt_port,
                    username=self.cfg.mqtt_username,
                    password=self.cfg.mqtt_password,
                    identifier=f"fleet-sim-{self.cfg.fleet_id}",
                ) as client:
                    self._client = client
                    await client.subscribe(f"swarm/v1/{self.cfg.fleet_id}/+/cmd", qos=1)
                    log.info("connected to mqtt://%s:%s, %d drones", self.cfg.mqtt_host, self.cfg.mqtt_port, len(self.drones))
                    async with asyncio.TaskGroup() as tg:
                        tg.create_task(self._physics_loop())
                        tg.create_task(self._telemetry_loop())
                        tg.create_task(self._command_loop(client))
            except* aiomqtt.MqttError as eg:
                self._client = None
                log.warning("mqtt connection lost (%s); retrying in 3s", eg.exceptions[0])
                await asyncio.sleep(3)

    async def _physics_loop(self) -> None:
        dt = 1.0 / self.cfg.physics_hz
        last = time.monotonic()
        while True:
            await asyncio.sleep(dt)
            now_mono = time.monotonic()
            elapsed, last = now_mono - last, now_mono
            now = time.time()
            for d in self.drones.values():
                d.step(elapsed, now)
                self._flush_outbox(d)
                if d.link_up and self._backlog.get(d.drone_id):
                    for t, payload in self._backlog.pop(d.drone_id):
                        self._send(d, t, payload, qos=1)

    async def _telemetry_loop(self) -> None:
        period = 1.0 / self.cfg.telemetry_hz
        while True:
            now = time.time()
            for d in self.drones.values():
                self._seq[d.drone_id] += 1
                msg = d.telemetry(self._seq[d.drone_id], now, self.link.quality(d.link_up))
                self._send(d, topic(d.fleet_id, d.drone_id, "telemetry"), msg.SerializeToString(), qos=0)
            await asyncio.sleep(period)

    async def _command_loop(self, client: aiomqtt.Client) -> None:
        async for message in client.messages:
            parts = message.topic.value.split("/")
            if len(parts) != 5:
                continue
            drone = self.drones.get(parts[3])
            if drone is None:
                continue
            try:
                cmd = pb.Command.FromString(bytes(message.payload))
            except Exception:  # noqa: BLE001 - malformed payloads are dropped like radio noise
                log.warning("dropping malformed command for %s", drone.drone_id)
                continue
            if not self.link.delivers(drone.link_up, qos=1):
                continue  # link down: the command expires before the drone could hear it
            self._later(self.link.delay_s(), self._deliver_command(drone, cmd))

    async def _deliver_command(self, drone: SimDrone, cmd: pb.Command) -> None:
        drone.handle_command(cmd, time.time())
        self._flush_outbox(drone)

    # ------------------------------------------------------------------ outbound

    def _flush_outbox(self, d: SimDrone) -> None:
        while d.outbox:
            kind, msg = d.outbox.pop(0)
            self._send(d, topic(d.fleet_id, d.drone_id, kind), msg.SerializeToString(), qos=1)

    def _send(self, d: SimDrone, t: str, payload: bytes, qos: int) -> None:
        if qos >= 1 and not d.link_up:
            self._backlog.setdefault(d.drone_id, []).append((t, payload))
            return
        if self._client is None or not self.link.delivers(d.link_up, qos):
            return
        self._later(self.link.delay_s(), self._publish(t, payload, qos))

    async def _publish(self, t: str, payload: bytes, qos: int) -> None:
        client = self._client
        if client is None:
            return
        try:
            await client.publish(t, payload=payload, qos=qos)
        except aiomqtt.MqttError:
            pass  # reconnect loop handles it

    def _later(self, delay: float, coro) -> None:
        async def runner() -> None:
            await asyncio.sleep(delay)
            await coro

        task = asyncio.create_task(runner())
        self._pending.add(task)
        task.add_done_callback(self._pending.discard)

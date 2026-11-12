# swarmops-fleet-sim

A simulated drone fleet. **It stands in for hardware.** The backend only ever sees it through
MQTT, using the protocol defined in `swarmops-contracts` (`swarmops.drone.v1`). Replacing this
repo with a real radio or LTE edge gateway should need **no backend changes**.

## What each drone does

- **Flight states:** `LANDED → TAKING_OFF → HOLDING ⇄ EN_ROUTE → RETURNING → LANDING`, plus `FAULT`
- **Motion:** flat-earth kinematics (12 m/s cruise, 3 m/s climb, 2 m/s descent) and a battery model (~20 min flights)
- **Commands:** validates and acknowledges each one (`ACCEPTED` / `REJECTED` / `COMPLETED` / `FAILED`), drops expired commands, and preempts the active command when a new one arrives
- **Failsafes it owns itself:**
  - link lost for 10 s → return to launch,
  - battery ≤ 15% → return to launch,
  - battery ≤ 5% → land in place
- **Radio:** every message in both directions passes through a link model with latency, jitter and loss

## Run

```bash
uv sync
MQTT_HOST=localhost DRONE_COUNT=10 uv run fleet-sim
uv run pytest
```

Usually you'll run it through `swarmops-backend`'s dev compose stack instead.

| Env var | Default | |
|---|---|---|
| `MQTT_HOST` / `MQTT_PORT` | `localhost` / `1883` | broker |
| `MQTT_USERNAME` / `MQTT_PASSWORD` | unset | |
| `FLEET_ID` | `fleet-1` | |
| `DRONE_COUNT` | `10` | |
| `HOME_LAT` / `HOME_LON` | Crissy Field, SF | launch pads are laid out in a grid from here |
| `TELEMETRY_HZ` | `2` | per drone |
| `LINK_LATENCY_MS` / `LINK_JITTER_MS` / `LINK_LOSS_PCT` | `60` / `40` / `1` | |
| `CHAOS_PORT` | `8090` | |

## Chaos API (`:8090`)

```bash
curl localhost:8090/drones
curl -XPOST localhost:8090/drones/d-003/link    -d '{"up": false}' -H 'content-type: application/json'
curl -XPOST localhost:8090/drones/d-004/battery -d '{"pct": 16}'   -H 'content-type: application/json'
curl -XPOST localhost:8090/drones/d-005/fault
curl -XPOST localhost:8090/link  -d '{"loss_pct": 20}' -H 'content-type: application/json'
curl -XPOST localhost:8090/spawn -d '{"count": 50}'    -H 'content-type: application/json'
```

Interactive docs are at `http://localhost:8090/docs`.

## Roadmap
- PX4 SITL bridge (MAVSDK → MQTT) for a few high-fidelity drones
- Wind field, GPS drift, sensor noise
- Multiple processes, one per ~500 drones, for large swarms

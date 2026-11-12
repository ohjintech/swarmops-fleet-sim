from swarmops.drone.v1 import drone_pb2 as pb

from fleet_sim import geo
from fleet_sim.drone import SimDrone

FS = pb.FlightState
HOME = (37.8037, -122.4650)


def make() -> SimDrone:
    return SimDrone(drone_id="d-001", fleet_id="f", home_lat=HOME[0], home_lon=HOME[1])


def run(d: SimDrone, seconds: float, dt: float = 0.05, t0: float = 1_000.0) -> float:
    t = t0
    for _ in range(int(seconds / dt)):
        t += dt
        d.step(dt, t)
    return t


def acks(d: SimDrone) -> list[tuple[str, int]]:
    out = [(m.command_id, m.status) for k, m in d.outbox if k == "ack"]
    d.outbox.clear()
    return out


def test_takeoff_reaches_altitude_and_completes():
    d = make()
    d.handle_command(pb.Command(command_id="c1", takeoff=pb.Takeoff(alt_m=20)), 1_000)
    run(d, 10)
    assert d.state == FS.FLIGHT_STATE_HOLDING
    assert abs(d.alt - 20) < 0.2
    assert acks(d) == [("c1", pb.ACK_STATUS_ACCEPTED), ("c1", pb.ACK_STATUS_COMPLETED)]


def test_goto_when_landed_is_rejected():
    d = make()
    d.handle_command(pb.Command(command_id="c1", go_to=pb.GoTo(target=pb.Position(lat=1, lon=1))), 1_000)
    assert acks(d) == [("c1", pb.ACK_STATUS_REJECTED)]


def test_goto_arrives_at_target():
    d = make()
    d.handle_command(pb.Command(command_id="c1", takeoff=pb.Takeoff(alt_m=20)), 1_000)
    run(d, 10)
    lat, lon = geo.offset(*HOME, 100, 50)
    d.handle_command(pb.Command(command_id="c2", go_to=pb.GoTo(target=pb.Position(lat=lat, lon=lon))), 1_010)
    run(d, 20)
    assert d.state == FS.FLIGHT_STATE_HOLDING
    assert geo.distance_m(d.lat, d.lon, lat, lon) < 1.5
    assert ("c2", pb.ACK_STATUS_COMPLETED) in acks(d)


def test_new_command_preempts_active_one():
    d = make()
    d.handle_command(pb.Command(command_id="c1", takeoff=pb.Takeoff(alt_m=50)), 1_000)
    run(d, 2)
    d.handle_command(pb.Command(command_id="c2", land=pb.Land()), 1_002)
    assert ("c1", pb.ACK_STATUS_FAILED) in acks(d)


def test_rtl_lands_at_home():
    d = make()
    d.handle_command(pb.Command(command_id="c1", takeoff=pb.Takeoff(alt_m=20)), 1_000)
    run(d, 10)
    lat, lon = geo.offset(*HOME, 200, 0)
    d.handle_command(pb.Command(command_id="c2", go_to=pb.GoTo(target=pb.Position(lat=lat, lon=lon))), 1_010)
    run(d, 30)
    d.handle_command(pb.Command(command_id="c3", return_to_launch=pb.ReturnToLaunch()), 1_040)
    run(d, 60)
    assert d.state == FS.FLIGHT_STATE_LANDED
    assert geo.distance_m(d.lat, d.lon, *HOME) < 1.5
    assert ("c3", pb.ACK_STATUS_COMPLETED) in acks(d)


def test_link_loss_triggers_rtl_failsafe():
    d = make()
    d.handle_command(pb.Command(command_id="c1", takeoff=pb.Takeoff(alt_m=20)), 1_000)
    run(d, 10)
    d.outbox.clear()
    d.link_up = False
    run(d, d.link_failsafe_s + 1)
    assert d.state in (FS.FLIGHT_STATE_RETURNING, FS.FLIGHT_STATE_LANDING)
    events = [m.code for k, m in d.outbox if k == "event"]
    assert "FAILSAFE_RTL" in events


def test_low_battery_blocks_takeoff():
    d = make()
    d.battery_pct = 10
    d.handle_command(pb.Command(command_id="c1", takeoff=pb.Takeoff(alt_m=20)), 1_000)
    assert acks(d) == [("c1", pb.ACK_STATUS_REJECTED)]


def test_expired_command_is_rejected():
    d = make()
    cmd = pb.Command(command_id="c1", takeoff=pb.Takeoff(alt_m=20))
    cmd.expires_at.FromSeconds(500)
    d.handle_command(cmd, 1_000)
    assert acks(d) == [("c1", pb.ACK_STATUS_REJECTED)]
    assert d.state == FS.FLIGHT_STATE_LANDED

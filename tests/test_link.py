from fleet_sim.link import LinkModel


def test_link_down_delivers_nothing():
    link = LinkModel(loss_pct=0)
    assert not link.delivers(False, qos=0)
    assert not link.delivers(False, qos=1)


def test_random_loss_only_drops_qos0():
    link = LinkModel(loss_pct=100)
    assert not link.delivers(True, qos=0)
    assert link.delivers(True, qos=1)

"""Radio link model applied to every message in both directions."""

from __future__ import annotations

import random
from dataclasses import dataclass


@dataclass
class LinkModel:
    latency_ms: float = 60.0
    jitter_ms: float = 40.0
    loss_pct: float = 1.0

    def delivers(self, link_up: bool, qos: int = 0) -> bool:
        """Whether a message gets through right now.

        QoS 0 (telemetry) is fire-and-forget, so random loss drops it. QoS 1 (commands,
        acks, events) is retransmitted by MQTT, so loss only adds delay; while the link
        is down the sender holds it (see Fleet._backlog).
        """
        if not link_up:
            return False
        return qos >= 1 or random.random() * 100.0 >= self.loss_pct

    def delay_s(self) -> float:
        return max(0.0, self.latency_ms + random.uniform(-self.jitter_ms, self.jitter_ms)) / 1000.0

    def quality(self, link_up: bool) -> float:
        if not link_up:
            return 0.0
        return max(0.0, 1.0 - self.loss_pct / 100.0 - self.latency_ms / 2000.0)

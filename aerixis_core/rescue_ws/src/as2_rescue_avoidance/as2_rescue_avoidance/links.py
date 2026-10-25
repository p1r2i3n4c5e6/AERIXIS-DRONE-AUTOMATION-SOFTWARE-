"""Mesh link models between drones (simulation only).

Each drone broadcasts its own estimated state (position, velocity, GNSS sigma) at the link's rate.
Every receiver gets each message independently, after the link delay, unless it is lost; it keeps
the newest message per sender. What a drone knows about a peer is therefore late and noisy, as in
the air.

  base_station  the main link: IP network of the base station, 500 m - 1 km (mesh_bridge
                transport:=udp, up to 20 Hz)
  edge          base-station network near the edge of its range: fewer, later messages
  lora          long-range fallback (mesh_bridge transport:=serial): about 1 message per second
"""
from dataclasses import dataclass
import heapq


@dataclass(frozen=True)
class LinkProfile:
    name: str
    rate_hz: float      # broadcasts per second per drone
    latency_s: float    # fixed part of the delay
    jitter_s: float     # extra delay, uniform 0..jitter
    loss: float         # probability a message to one receiver is lost


BASE_STATION = LinkProfile('base_station', 10.0, 0.05, 0.03, 0.02)
EDGE = LinkProfile('edge', 2.0, 0.15, 0.15, 0.30)
LORA = LinkProfile('lora', 1.0, 0.40, 0.30, 0.10)
PROFILES = {p.name: p for p in (BASE_STATION, EDGE, LORA)}


class Mesh:
    """Message delivery for one episode. `send` at broadcast time, `deliver` every step."""

    def __init__(self, profile, count, rng):
        self.profile, self.rng = profile, rng
        self.period = 1.0 / profile.rate_hz
        # Drones broadcast at their own phase, not all at once.
        self.next_send = [rng.uniform(0.0, self.period) for _ in range(count)]
        self.queue = []          # (arrival time, seq, receiver, sender, message)
        self.seq = 0
        self.latest = [dict() for _ in range(count)]   # receiver -> {sender: message}

    def due(self, sender, now):
        if now + 1e-9 >= self.next_send[sender]:
            self.next_send[sender] += self.period
            return True
        return False

    def send(self, sender, message, now):
        for receiver in range(len(self.latest)):
            if receiver == sender or self.rng.random() < self.profile.loss:
                continue
            arrival = now + self.profile.latency_s + self.rng.uniform(0.0, self.profile.jitter_s)
            self.seq += 1
            heapq.heappush(self.queue, (arrival, self.seq, receiver, sender, message))

    def deliver(self, now):
        while self.queue and self.queue[0][0] <= now:
            _, _, receiver, sender, message = heapq.heappop(self.queue)
            old = self.latest[receiver].get(sender)
            if old is None or message['stamp'] > old['stamp']:   # never go back to an older state
                self.latest[receiver][sender] = message

"""BGR8-only transport helpers; no cv_bridge binary/NumPy ABI dependency."""
import threading
import numpy as np


def decode_bgr8(message):
    if message.encoding != 'bgr8' or message.width <= 0 or message.height <= 0:
        raise ValueError('Expected a nonempty bgr8 image')
    if message.step < message.width * 3 or len(message.data) != message.height * message.step:
        raise ValueError('Invalid image stride or buffer size')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    return rows[:, :message.width * 3].reshape(message.height, message.width, 3).copy()


class LatestFrame:
    """Bounded single-frame cache; each decoded frame is consumed at most once."""
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.sequence = 0
        self.received = None
        self.stamp = None

    def put(self, frame, received, stamp):
        with self.lock:
            self.sequence += 1
            self.frame, self.received, self.stamp = frame, received, stamp

    def clear(self):
        with self.lock:
            self.frame = self.received = self.stamp = None

    def take(self, previous_sequence, now, max_age):
        with self.lock:
            if (self.frame is None or self.sequence == previous_sequence or
                    now - self.received > max_age or now < self.received):
                return None
            return self.sequence, self.frame, self.stamp

    def age(self, now):
        with self.lock:
            return None if self.received is None else max(0.0, now - self.received)

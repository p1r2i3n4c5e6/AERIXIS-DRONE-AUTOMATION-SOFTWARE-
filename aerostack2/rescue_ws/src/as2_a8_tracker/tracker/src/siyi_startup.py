"""One startup centering transaction, with CRC-checked SIYI feedback."""
import socket
import struct
import time


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def packet(command, payload=b'', sequence=0):
    body = b'\x55\x66\x01' + struct.pack('<HHB', len(payload), sequence & 0xffff, command) + payload
    return body + struct.pack('<H', crc16(body))


def replies(data):
    """Yield complete valid acknowledgement packets from one UDP datagram."""
    while len(data) >= 10:
        start = data.find(b'\x55\x66')
        if start < 0:
            return
        data = data[start:]
        if len(data) < 10:
            return
        size = struct.unpack_from('<H', data, 3)[0] + 10
        if len(data) < size:
            return
        frame, data = data[:size], data[size:]
        if crc16(frame[:-2]) != struct.unpack_from('<H', frame, size-2)[0]:
            continue
        if frame[2] & 2:
            yield frame[7], struct.unpack_from('<H', frame, 5)[0], frame[8:-2]


def center_and_verify(sock, address, timeout=8.0, clock=time.monotonic, sleep=time.sleep):
    """No tracking rates during centering; fail closed if feedback is unavailable.

    Requires center ACK and three fresh, stationary yaw/pitch samples near zero.
    The camera base must remain stationary during this startup check.

    Replies are matched by command and time, not by sequence number: A8 mini firmware answers
    with its own counter (Pi 5, 2026-10-23: centre sent as 2 came back as 36, attitude queries
    100..102 came back as 1415..1417), so matching on our number never verified and tracking
    never started. A centre ACK counts only after our centre request. Only one attitude query is
    in flight, so its reply is unambiguous; unanswered after 0.5 s it is dropped and replies are
    ignored for another 0.5 s, so a late reply is never taken for a fresh one.
    """
    started = clock()
    seq = 1
    sock.sendto(packet(7, b'\x00\x00', seq), address)
    center_seq = 2
    sock.sendto(packet(8, b'\x01', center_seq), address)
    acknowledged = False
    stable = 0
    pending = None            # send time of the one attitude query in flight
    quiet_until = started     # after a lost reply: ignore replies until then
    next_query = started
    seq = 2
    try:
        while clock()-started < timeout:
            now = clock()
            if pending is not None and now-pending > .5:
                pending, quiet_until = None, now+.5
            if pending is None and now >= next_query and now >= quiet_until:
                seq = (seq+1) & 0xffff
                sock.sendto(packet(13, sequence=seq), address)
                pending, next_query = now, now+.1
            # Bound reads so unrelated traffic cannot prevent timeout checks.
            for _ in range(16):
                try:
                    data, sender = sock.recvfrom(4096)
                except (BlockingIOError, socket.timeout):
                    break
                if sender != address:
                    continue
                for command, response_seq, payload in replies(data):
                    if command == 8 and len(payload) == 1:
                        if payload[0] != 1:
                            raise RuntimeError('SIYI rejected startup centering')
                        acknowledged = True
                    elif command == 13 and len(payload) == 12:
                        if pending is None:
                            continue          # no query of ours in flight: late or not ours
                        sent_at, pending = pending, None
                        if clock()-sent_at > .5:
                            continue
                        yaw,pitch,roll,yaw_rate,pitch_rate,roll_rate = [v/10. for v in struct.unpack('<6h',payload)]
                        yaw = (yaw+180) % 360-180
                        aligned = abs(yaw)<=3 and abs(pitch)<=3 and abs(yaw_rate)<=3 and abs(pitch_rate)<=3
                        stable = stable+1 if aligned and acknowledged else 0
                        if stable >= 3:
                            return {'yaw':yaw,'pitch':pitch,'seconds':clock()-started,
                                    'sequence':(seq+3)&0xffff}
            sleep(.01)
        raise RuntimeError('Startup center not verified: need SIYI center ACK and stable attitude near zero; tracking was not started')
    finally:
        # Do not leave a rotation request active, even on timeout/Ctrl+C.
        for i in range(3):
            try:
                sock.sendto(packet(7,b'\x00\x00',(seq+i+1)&0xffff),address)
            except OSError:
                pass

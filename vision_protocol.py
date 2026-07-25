import struct

HEADER = b"\xAA\x55"
MAX_BODY_LENGTH = 64


class MessageType:
    START = 0x01
    RESET = 0x02
    STOP = 0x03
    TURN_DONE = 0x04

    TARGET_LOCKED = 0x81
    LINE_DATA = 0x82
    TURN_DECISION = 0x83
    VISION_HOLD = 0x84
    STATUS = 0x85


DIRECTION_LEFT = 0
DIRECTION_RIGHT = 1

HOLD_NO_TARGET = 1
HOLD_DIRECTION_UNCERTAIN = 2
HOLD_FIXED_ROUTE_MISSING = 3
HOLD_LINE_LOST = 4


def encode_frame(message_type, payload=b""):
    payload = bytes(payload)
    body_length = 1 + len(payload)
    if not 1 <= body_length <= MAX_BODY_LENGTH:
        raise ValueError("payload is too large")
    frame_without_checksum = HEADER + bytes((body_length, message_type)) + payload
    checksum = sum(frame_without_checksum) & 0xFF
    return frame_without_checksum + bytes((checksum,))


def encode_target_locked(target_number):
    return encode_frame(MessageType.TARGET_LOCKED, struct.pack("<B", target_number))


def encode_line_data(valid, error, angle, center_x_norm):
    center = max(0, min(1000, round(center_x_norm * 1000)))
    payload = struct.pack("<BhhH", 1 if valid else 0, round(error), round(angle * 100), center)
    return encode_frame(MessageType.LINE_DATA, payload)


def encode_turn_decision(
    direction,
    target_number=0,
    intersection_index=0,
    confidence=0,
):
    if direction == "LEFT":
        value = DIRECTION_LEFT
    elif direction == "RIGHT":
        value = DIRECTION_RIGHT
    else:
        raise ValueError("direction must be LEFT or RIGHT")
    payload = struct.pack(
        "<BBBB",
        value,
        max(0, min(255, target_number)),
        max(0, min(255, intersection_index)),
        max(0, min(100, confidence)),
    )
    return encode_frame(MessageType.TURN_DECISION, payload)


def encode_vision_hold(reason):
    return encode_frame(MessageType.VISION_HOLD, struct.pack("<B", reason))


def encode_status(state_code, detail=0):
    return encode_frame(MessageType.STATUS, struct.pack("<BB", state_code, detail))


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def clear(self):
        self.buffer.clear()

    def feed(self, data):
        if data:
            self.buffer.extend(data)
        frames = []

        while True:
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                if self.buffer and self.buffer[-1] == HEADER[0]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if header_index > 0:
                del self.buffer[:header_index]
            if len(self.buffer) < 4:
                break

            body_length = self.buffer[2]
            if not 1 <= body_length <= MAX_BODY_LENGTH:
                del self.buffer[0]
                continue
            frame_length = body_length + 4
            if len(self.buffer) < frame_length:
                break

            candidate = self.buffer[:frame_length]
            expected_checksum = sum(candidate[:-1]) & 0xFF
            if candidate[-1] != expected_checksum:
                del self.buffer[0]
                continue

            message_type = candidate[3]
            payload = bytes(candidate[4:-1])
            frames.append((message_type, payload))
            del self.buffer[:frame_length]

        return frames

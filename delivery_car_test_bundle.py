"""
delivery_car_test_bundle.py
Single-file test bundle for MaixCam delivery car vision pipeline.

Includes all dependent modules inline so it runs standalone on MaixCam
without needing other .py files on the device.

Touch buttons at screen bottom: [RESET] [START] [STOP] [TURN_DONE]
"""

from collections import Counter, deque
import math
import struct

from maix import app, camera, display, image, nn, time, touchscreen

# ============================================================================
# vision_protocol.py — UART frame encoding (send side only)
# ============================================================================

HEADER = b"\xAA\x55"
MAX_BODY_LENGTH = 64

HOLD_NO_TARGET = 1
HOLD_DIRECTION_UNCERTAIN = 2
HOLD_FIXED_ROUTE_MISSING = 3
HOLD_LINE_LOST = 4

DIRECTION_LEFT = 1
DIRECTION_RIGHT = 2


def encode_frame(message_type, payload=b""):
    payload = bytes(payload)
    body_length = 1 + len(payload)
    if not 1 <= body_length <= MAX_BODY_LENGTH:
        raise ValueError("payload is too large")
    frame_without_checksum = HEADER + bytes((body_length, message_type)) + payload
    checksum = sum(frame_without_checksum) & 0xFF
    return frame_without_checksum + bytes((checksum,))


def encode_target_locked(target_number):
    return encode_frame(0x81, struct.pack("<B", target_number))


def encode_line_data(valid, error, angle, center_x_norm):
    center = max(0, min(1000, round(center_x_norm * 1000)))
    payload = struct.pack("<BhhH", 1 if valid else 0, round(error), round(angle * 100), center)
    return encode_frame(0x82, payload)


def encode_turn_decision(direction, target_number=0, intersection_index=0, confidence=0):
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
    return encode_frame(0x83, payload)


def encode_vision_hold(reason):
    return encode_frame(0x84, struct.pack("<B", reason))


def encode_status(state_code, detail=0):
    return encode_frame(0x85, struct.pack("<BB", state_code, detail))


# ============================================================================
# delivery_logic.py — state machine, target locker, direction voter
# ============================================================================

LEFT = "LEFT"
RIGHT = "RIGHT"
UNKNOWN = "UNKNOWN"

CAPTURE_TARGET = "CAPTURE_TARGET"
WAIT_START = "WAIT_START"
FOLLOW_LINE = "FOLLOW_LINE"
DECIDE_DIRECTION = "DECIDE_DIRECTION"
WAIT_TURN_DONE = "WAIT_TURN_DONE"


def _iou(first, second):
    left = max(first["x"], second["x"])
    top = max(first["y"], second["y"])
    right = min(first["x"] + first["w"], second["x"] + second["w"])
    bottom = min(first["y"] + first["h"], second["y"] + second["h"])
    intersection = max(0, right - left) * max(0, bottom - top)
    if intersection == 0:
        return 0.0
    union = first["w"] * first["h"] + second["w"] * second["h"] - intersection
    return intersection / union if union else 0.0


def _center_distance(first, second):
    dx = first["cx_norm"] - second["cx_norm"]
    dy = first["cy_norm"] - second["cy_norm"]
    return (dx * dx + dy * dy) ** 0.5


def deduplicate_detections(detections, min_score=0.45, iou_threshold=0.55, center_threshold=0.035):
    candidates = []
    for detection in detections:
        if detection.get("number") not in range(1, 9):
            continue
        if detection.get("score", 0.0) < min_score:
            continue
        if detection.get("w", 0) <= 0 or detection.get("h", 0) <= 0:
            continue
        if not 0.0 <= detection.get("cx_norm", -1.0) <= 1.0:
            continue
        if not 0.0 <= detection.get("cy_norm", -1.0) <= 1.0:
            continue
        candidates.append(detection)

    kept = []
    for candidate in sorted(candidates, key=lambda item: item["score"], reverse=True):
        duplicate = False
        for accepted in kept:
            if candidate["number"] != accepted["number"]:
                continue
            if _iou(candidate, accepted) >= iou_threshold:
                duplicate = True
                break
            if _center_distance(candidate, accepted) <= center_threshold:
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
    return kept


def _same_row(detection, row, max_vertical_gap):
    center_y = sum(item["cy_norm"] for item in row) / len(row)
    gap_limit = max_vertical_gap
    heights = [item.get("h_norm") for item in row + [detection] if item.get("h_norm")]
    if heights:
        gap_limit = max(gap_limit, 0.65 * sum(heights) / len(heights))
    return abs(detection["cy_norm"] - center_y) <= gap_limit


def select_digit_row(detections, min_score=0.45, max_vertical_gap=0.09, row_y_range=None):
    candidates = deduplicate_detections(detections, min_score=min_score)
    if row_y_range is not None:
        y_min, y_max = row_y_range
        candidates = [item for item in candidates if y_min <= item["cy_norm"] <= y_max]
    if not candidates:
        return []

    rows = []
    for detection in sorted(candidates, key=lambda item: item["cy_norm"]):
        matching_rows = [row for row in rows if _same_row(detection, row, max_vertical_gap)]
        if matching_rows:
            row = min(
                matching_rows,
                key=lambda items: abs(
                    detection["cy_norm"]
                    - sum(item["cy_norm"] for item in items) / len(items)
                ),
            )
            row.append(detection)
        else:
            rows.append([detection])

    def row_rank(row):
        count = len(row)
        expected_count_bonus = 0.6 if 2 <= count <= 4 else 0.0
        confidence = sum(item["score"] for item in row)
        return expected_count_bonus + confidence, count

    selected = max(rows, key=row_rank)
    return sorted(selected, key=lambda item: item["cx_norm"])


class TargetLocker:
    def __init__(self, window_size=7, min_votes=5, min_score=0.55,
                 roi=(0.20, 0.10, 0.80, 0.90), min_vote_margin=2):
        self.window_size = window_size
        self.min_votes = min_votes
        self.min_score = min_score
        self.roi = roi
        self.min_vote_margin = min_vote_margin
        self.samples = deque(maxlen=window_size)
        self.locked_target = None

    def reset(self):
        self.samples.clear()
        self.locked_target = None

    def update(self, detections):
        if self.locked_target is not None:
            return self.locked_target

        x_min, y_min, x_max, y_max = self.roi
        candidates = [
            item
            for item in deduplicate_detections(detections, min_score=self.min_score)
            if x_min <= item["cx_norm"] <= x_max and y_min <= item["cy_norm"] <= y_max
        ]
        numbers = {item["number"] for item in candidates}
        if len(candidates) == 1 and len(numbers) == 1:
            self.samples.append(candidates[0]["number"])
        else:
            self.samples.append(None)

        counts = Counter(number for number in self.samples if number is not None)
        if not counts:
            return None
        ranked = counts.most_common(2)
        winner, winner_votes = ranked[0]
        runner_up_votes = ranked[1][1] if len(ranked) > 1 else 0
        if winner_votes >= self.min_votes and winner_votes - runner_up_votes >= self.min_vote_margin:
            self.locked_target = winner
        return self.locked_target


class DirectionVoter:
    def __init__(self, window_size=8, min_samples=4, dead_zone=0.07,
                 min_winner_score=2.2, min_score_margin=0.8, min_detection_score=0.45):
        self.samples = deque(maxlen=window_size)
        self.min_samples = min_samples
        self.dead_zone = dead_zone
        self.min_winner_score = min_winner_score
        self.min_score_margin = min_score_margin
        self.min_detection_score = min_detection_score

    def clear(self):
        self.samples.clear()

    def add(self, detections, target_number, road_center_x_norm):
        row = select_digit_row(detections, min_score=self.min_detection_score)
        targets = [item for item in row if item["number"] == target_number]
        if not targets or road_center_x_norm is None:
            self.samples.append(None)
            return UNKNOWN

        target = max(targets, key=lambda item: item["score"])
        offset = target["cx_norm"] - road_center_x_norm
        if abs(offset) <= self.dead_zone:
            self.samples.append(None)
            return UNKNOWN

        direction = LEFT if offset < 0 else RIGHT
        self.samples.append((direction, target["score"]))
        return direction

    def scores(self):
        scores = {LEFT: 0.0, RIGHT: 0.0}
        counts = {LEFT: 0, RIGHT: 0}
        for sample in self.samples:
            if sample is None:
                continue
            direction, score = sample
            scores[direction] += score
            counts[direction] += 1
        return scores, counts

    def decision(self):
        scores, counts = self.scores()
        valid_samples = counts[LEFT] + counts[RIGHT]
        if valid_samples < self.min_samples:
            return UNKNOWN
        winner = LEFT if scores[LEFT] > scores[RIGHT] else RIGHT
        loser = RIGHT if winner == LEFT else LEFT
        if scores[winner] < self.min_winner_score:
            return UNKNOWN
        if scores[winner] - scores[loser] < self.min_score_margin:
            return UNKNOWN
        return winner


def fixed_route_direction(target_number, route_index, routes):
    route = routes.get(target_number)
    if route is None or route_index < 0 or route_index >= len(route):
        return UNKNOWN
    direction = route[route_index]
    return direction if direction in (LEFT, RIGHT) else UNKNOWN


class DeliveryStateMachine:
    def __init__(self, clear_frames_required=3):
        self.clear_frames_required = clear_frames_required
        self.reset()

    def reset(self):
        self.state = CAPTURE_TARGET
        self.target_number = None
        self.crossroad_armed = True
        self.waiting_for_clear = False
        self.clear_frames = 0
        self.last_direction = UNKNOWN

    def lock_target(self, target_number):
        if self.state != CAPTURE_TARGET or target_number not in range(1, 9):
            return False
        self.target_number = target_number
        self.state = WAIT_START
        return True

    def start(self):
        if self.state != WAIT_START or self.target_number is None:
            return False
        self.state = FOLLOW_LINE
        self.crossroad_armed = True
        self.waiting_for_clear = False
        self.clear_frames = 0
        return True

    def stop(self):
        self.state = WAIT_START if self.target_number is not None else CAPTURE_TARGET
        self.crossroad_armed = True
        self.waiting_for_clear = False
        self.clear_frames = 0
        self.last_direction = UNKNOWN

    def trigger_crossroad(self):
        if self.state != FOLLOW_LINE or not self.crossroad_armed:
            return False
        self.crossroad_armed = False
        self.state = DECIDE_DIRECTION
        return True

    def lock_direction(self, direction):
        if self.state != DECIDE_DIRECTION or direction not in (LEFT, RIGHT):
            return False
        self.last_direction = direction
        self.state = WAIT_TURN_DONE
        return True

    def turn_done(self):
        if self.state != WAIT_TURN_DONE:
            return False
        self.state = FOLLOW_LINE
        self.waiting_for_clear = True
        self.clear_frames = 0
        return True

    def update_crossroad_visibility(self, present):
        if not self.waiting_for_clear:
            return self.crossroad_armed
        if present:
            self.clear_frames = 0
            return False
        self.clear_frames += 1
        if self.clear_frames >= self.clear_frames_required:
            self.waiting_for_clear = False
            self.crossroad_armed = True
        return self.crossroad_armed


# ============================================================================
# Shared utilities — clamp, line_threshold, color thresholds
# ============================================================================

def _clamp(value, low, high):
    return max(low, min(high, value))


LINE_COLOR = "black"

BLACK_L_MAX = 30
BLACK_THRESHOLD = [[0, BLACK_L_MAX, -128, 127, -128, 127]]
RED_THRESHOLD = [[0, 80, 40, 80, 0, 80]]


def line_threshold(line_color):
    if line_color == "black":
        return BLACK_THRESHOLD
    if line_color == "red":
        return RED_THRESHOLD
    raise ValueError("line_color must be 'black' or 'red'")


# ============================================================================
# track_line.py — line tracker (blob detection + line fitting)
# ============================================================================

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_CENTER_X = IMAGE_WIDTH // 2
TARGET_Y = IMAGE_HEIGHT - 20

TL_MIN_PIXELS = 25
TL_MIN_AREA = 25
TL_MAX_BLOB_WIDTH_RATIO = 0.70
TL_FILTER_ALPHA = 0.35

ROIS = [
    [80, 105, 160, 35],
    [60, 155, 200, 35],
]


def _choose_blob(blobs, expected_x, roi_width):
    best = None
    best_score = -1000000
    max_blob_width = int(roi_width * TL_MAX_BLOB_WIDTH_RATIO)

    for blob in blobs:
        if blob.pixels() < TL_MIN_PIXELS or blob.w() > max_blob_width:
            continue
        score = blob.pixels() - abs(blob.cx() - expected_x) * 0.8
        if score > best_score:
            best = blob
            best_score = score
    return best


def _detect_line(img, expected_x, threshold):
    points = []
    for roi in ROIS:
        blobs = img.find_blobs(
            threshold,
            roi=roi,
            area_threshold=TL_MIN_AREA,
            pixels_threshold=TL_MIN_PIXELS,
            x_stride=2,
            y_stride=1,
        )
        blob = _choose_blob(blobs, expected_x, roi[2])
        if blob is not None:
            points.append((blob.cx(), roi[1] + roi[3] // 2, blob))
    return points


def _fit_line(points):
    if not points:
        return None
    if len(points) == 1:
        x, y, _ = points[0]
        return int(x), 0.0, int(x), y

    sum_y = sum(point[1] for point in points)
    sum_x = sum(point[0] for point in points)
    sum_yy = sum(point[1] * point[1] for point in points)
    sum_xy = sum(point[0] * point[1] for point in points)
    count = len(points)
    denominator = count * sum_yy - sum_y * sum_y
    if denominator == 0:
        x = int(sum_x / count)
        return x, 0.0, x, points[0][1]

    slope = (count * sum_xy - sum_x * sum_y) / denominator
    intercept = (sum_x - slope * sum_y) / count
    line_top_y = points[0][1]
    x_top = int(_clamp(intercept + slope * line_top_y, 0, IMAGE_WIDTH - 1))
    x_bottom = int(_clamp(intercept + slope * TARGET_Y, 0, IMAGE_WIDTH - 1))
    angle = math.degrees(math.atan(slope))
    return x_bottom, angle, x_top, line_top_y


class LineTracker:
    def __init__(self, line_color=LINE_COLOR, filter_alpha=TL_FILTER_ALPHA):
        self.line_color = line_color
        self.threshold = line_threshold(line_color)
        self.filter_alpha = filter_alpha
        self.reset()

    def reset(self):
        self.last_x_bottom = IMAGE_CENTER_X
        self.filtered_error = 0.0
        self.filtered_angle = 0.0
        self.filter_initialized = False

    def process(self, img):
        points = _detect_line(img, self.last_x_bottom, self.threshold)
        fitted = _fit_line(points)
        if fitted is None:
            return {
                "valid": False,
                "error": self.filtered_error,
                "angle": self.filtered_angle,
                "x_bottom": self.last_x_bottom,
                "center_x_norm": self.last_x_bottom / IMAGE_WIDTH,
                "decision_center_x_norm": self.last_x_bottom / IMAGE_WIDTH,
                "x_top": self.last_x_bottom,
                "line_top_y": ROIS[0][1],
                "points": points,
            }

        x_bottom, angle, x_top, line_top_y = fitted
        error = x_bottom - IMAGE_CENTER_X
        if not self.filter_initialized:
            self.filtered_error = float(error)
            self.filtered_angle = float(angle)
            self.filter_initialized = True
        else:
            self.filtered_error = (
                self.filter_alpha * error
                + (1.0 - self.filter_alpha) * self.filtered_error
            )
            self.filtered_angle = (
                self.filter_alpha * angle
                + (1.0 - self.filter_alpha) * self.filtered_angle
            )
        self.last_x_bottom = x_bottom

        return {
            "valid": True,
            "error": self.filtered_error,
            "angle": self.filtered_angle,
            "x_bottom": x_bottom,
            "center_x_norm": x_bottom / IMAGE_WIDTH,
            "decision_center_x_norm": x_top / IMAGE_WIDTH,
            "x_top": x_top,
            "line_top_y": line_top_y,
            "points": points,
        }


# ============================================================================
# crossroad_detect_v2.py — blob-based crossroad detector
# ============================================================================

IMAGE_CENTER_Y = IMAGE_HEIGHT // 2

CV2_MIN_PIXELS = 20
CV2_MIN_AREA = 20
CV2_MAX_BLOB_WIDTH_RATIO = 0.70
CV2_MAX_BLOB_HEIGHT_RATIO = 0.70
CV2_FILTER_ALPHA = 0.4

MAX_V_SLOPE = 0.577
MAX_H_SLOPE = 0.577
MIN_CROSS_ANGLE = 45.0
MAX_CROSS_CENTER_DIST = 120.0
CROSS_CONFIRM_FRAMES = 3
CROSS_LOST_FRAMES = 3

V_ROI_TOP = [110, 50, 100, 30]
V_ROI_BOTTOM = [80, 160, 160, 40]
H_ROI_LEFT = [80, 80, 30, 80]
H_ROI_RIGHT = [210, 80, 30, 80]


def _choose_blob_v(blobs, expected_x, roi_width):
    best = None
    best_score = -1000000
    max_blob_width = int(roi_width * CV2_MAX_BLOB_WIDTH_RATIO)

    for blob in blobs:
        if blob.pixels() < CV2_MIN_PIXELS:
            continue
        if blob.w() > max_blob_width:
            continue
        distance = abs(blob.cx() - expected_x)
        score = blob.pixels() - distance * 0.8
        if score > best_score:
            best = blob
            best_score = score
    return best


def _choose_blob_h(blobs, expected_y, roi_height):
    best = None
    best_score = -1000000
    max_blob_height = int(roi_height * CV2_MAX_BLOB_HEIGHT_RATIO)

    for blob in blobs:
        if blob.pixels() < CV2_MIN_PIXELS:
            continue
        if blob.h() > max_blob_height:
            continue
        distance = abs(blob.cy() - expected_y)
        score = blob.pixels() - distance * 0.8
        if score > best_score:
            best = blob
            best_score = score
    return best


def _detect_vertical_line(img, expected_x, threshold=BLACK_THRESHOLD):
    top_blobs = img.find_blobs(
        threshold, roi=V_ROI_TOP,
        area_threshold=CV2_MIN_AREA, pixels_threshold=CV2_MIN_PIXELS,
        x_stride=2, y_stride=1,
    )
    top_blob = _choose_blob_v(top_blobs, expected_x, V_ROI_TOP[2])

    bottom_blobs = img.find_blobs(
        threshold, roi=V_ROI_BOTTOM,
        area_threshold=CV2_MIN_AREA, pixels_threshold=CV2_MIN_PIXELS,
        x_stride=2, y_stride=1,
    )
    bottom_blob = _choose_blob_v(bottom_blobs, expected_x, V_ROI_BOTTOM[2])

    if top_blob is None or bottom_blob is None:
        return None

    x_top = top_blob.cx()
    y_top = V_ROI_TOP[1] + V_ROI_TOP[3] // 2
    x_bottom = bottom_blob.cx()
    y_bottom = V_ROI_BOTTOM[1] + V_ROI_BOTTOM[3] // 2

    dy = y_bottom - y_top
    if abs(dy) < 1:
        return None
    slope = (x_bottom - x_top) / dy
    if abs(slope) > MAX_V_SLOPE:
        return None

    return (x_top, y_top, x_bottom, y_bottom, top_blob, bottom_blob)


def _detect_horizontal_line(img, expected_y, threshold=BLACK_THRESHOLD):
    left_blobs = img.find_blobs(
        threshold, roi=H_ROI_LEFT,
        area_threshold=CV2_MIN_AREA, pixels_threshold=CV2_MIN_PIXELS,
        x_stride=1, y_stride=2,
    )
    left_blob = _choose_blob_h(left_blobs, expected_y, H_ROI_LEFT[3])

    right_blobs = img.find_blobs(
        threshold, roi=H_ROI_RIGHT,
        area_threshold=CV2_MIN_AREA, pixels_threshold=CV2_MIN_PIXELS,
        x_stride=1, y_stride=2,
    )
    right_blob = _choose_blob_h(right_blobs, expected_y, H_ROI_RIGHT[3])

    if left_blob is None or right_blob is None:
        return None

    x_left = H_ROI_LEFT[0] + H_ROI_LEFT[2] // 2
    y_left = left_blob.cy()
    x_right = H_ROI_RIGHT[0] + H_ROI_RIGHT[2] // 2
    y_right = right_blob.cy()

    dx = x_right - x_left
    if abs(dx) < 1:
        return None
    slope = (y_right - y_left) / dx
    if abs(slope) > MAX_H_SLOPE:
        return None

    return (x_left, y_left, x_right, y_right, left_blob, right_blob)


def _line_intersection(v_line, h_line):
    x1, y1, x2, y2 = v_line[0], v_line[1], v_line[2], v_line[3]
    x3, y3, x4, y4 = h_line[0], h_line[1], h_line[2], h_line[3]

    v_dx = x2 - x1
    v_dy = y2 - y1
    h_dx = x4 - x3
    h_dy = y4 - y3

    v_len = math.hypot(v_dx, v_dy)
    h_len = math.hypot(h_dx, h_dy)
    if v_len < 1 or h_len < 1:
        return None

    dot = abs(v_dx * h_dx + v_dy * h_dy)
    cos_angle = _clamp(dot / (v_len * h_len), -1.0, 1.0)
    angle_deg = math.degrees(math.acos(cos_angle))
    if angle_deg > 90:
        angle_deg = 180 - angle_deg

    if angle_deg < MIN_CROSS_ANGLE:
        return None

    denom = v_dx * h_dy - v_dy * h_dx
    if abs(denom) < 1e-6:
        return None

    t = ((x3 - x1) * h_dy - (y3 - y1) * h_dx) / denom

    ix = x1 + t * v_dx
    iy = y1 + t * v_dy

    if not (0 <= ix < IMAGE_WIDTH and 0 <= iy < IMAGE_HEIGHT):
        return None

    center_dist = math.hypot(ix - IMAGE_CENTER_X, iy - IMAGE_CENTER_Y)
    if center_dist > MAX_CROSS_CENTER_DIST:
        return None

    return int(round(ix)), int(round(iy)), angle_deg


class CrossroadDetector:
    def __init__(self, line_color=LINE_COLOR, filter_alpha=CV2_FILTER_ALPHA):
        self.line_color = line_color
        self.threshold = line_threshold(line_color)
        self.filter_alpha = filter_alpha
        self.reset()

    def reset(self):
        self.cross_count = 0
        self.lost_count = 0
        self.confirmed = False
        self.expected_x = IMAGE_CENTER_X
        self.expected_y = IMAGE_CENTER_Y
        self.filtered_cross_x = float(IMAGE_CENTER_X)
        self.filtered_cross_y = float(IMAGE_CENTER_Y)
        self.filter_initialized = False

    def process(self, img):
        v_line = _detect_vertical_line(img, self.expected_x, self.threshold)
        h_line = _detect_horizontal_line(img, self.expected_y, self.threshold)
        cross = None
        if v_line is not None and h_line is not None:
            cross = _line_intersection(v_line, h_line)

        if cross is not None:
            self.cross_count += 1
            self.lost_count = 0
            if self.cross_count >= CROSS_CONFIRM_FRAMES:
                self.confirmed = True

            x, y, _ = cross
            if not self.filter_initialized:
                self.filtered_cross_x = float(x)
                self.filtered_cross_y = float(y)
                self.filter_initialized = True
            else:
                self.filtered_cross_x = (
                    self.filter_alpha * x
                    + (1.0 - self.filter_alpha) * self.filtered_cross_x
                )
                self.filtered_cross_y = (
                    self.filter_alpha * y
                    + (1.0 - self.filter_alpha) * self.filtered_cross_y
                )
        else:
            self.lost_count += 1
            self.cross_count = 0
            if self.lost_count >= CROSS_LOST_FRAMES:
                self.confirmed = False

        if v_line is not None:
            self.expected_x = v_line[2]
        if h_line is not None:
            self.expected_y = h_line[1]

        x = int(round(self.filtered_cross_x)) if cross is not None else 0
        y = int(round(self.filtered_cross_y)) if cross is not None else 0
        angle = cross[2] if cross is not None else 0.0
        score = 0.0
        if v_line is not None and h_line is not None:
            score = math.hypot(v_line[2] - v_line[0], v_line[3] - v_line[1])
            score += math.hypot(h_line[2] - h_line[0], h_line[3] - h_line[1])

        return {
            "detected": cross is not None,
            "confirmed": self.confirmed,
            "x": x,
            "y": y,
            "center_x_norm": x / IMAGE_WIDTH if cross is not None else None,
            "angle": angle,
            "score": score,
            "v_line": v_line,
            "h_line": h_line,
            "cross": cross,
            "cross_count": self.cross_count,
        }


# ============================================================================
# yolov8_num_detect.py — YOLOv8 digit detector
# ============================================================================

MODEL_PATH = "/root/models/yolov8_num_detect_v2.mud"
CONFIDENCE_THRESHOLD = 0.4
IOU_THRESHOLD = 0.45


def _parse_room_number(label):
    text = str(label).strip()
    digits = [c for c in text if c.isdigit()]
    if len(digits) != 1:
        return None
    number = int(digits[0])
    return number if 1 <= number <= 8 else None


class DigitDetector:
    def __init__(self, model_path=MODEL_PATH, dual_buff=False):
        self.detector = nn.YOLOv8(model=model_path, dual_buff=dual_buff)

    def input_width(self):
        return self.detector.input_width()

    def input_height(self):
        return self.detector.input_height()

    def input_format(self):
        return self.detector.input_format()

    def detect(self, img, conf_th=CONFIDENCE_THRESHOLD, iou_th=IOU_THRESHOLD):
        width = img.width()
        height = img.height()
        results = []
        objects = self.detector.detect(img, conf_th=conf_th, iou_th=iou_th)
        for obj in objects:
            if obj.class_id < 0 or obj.class_id >= len(self.detector.labels):
                continue
            label = self.detector.labels[obj.class_id]
            number = _parse_room_number(label)
            if number is None:
                continue
            center_x = obj.x + obj.w / 2.0
            center_y = obj.y + obj.h / 2.0
            results.append({
                "number": number,
                "class_id": int(obj.class_id),
                "label": str(label),
                "score": float(obj.score),
                "x": int(obj.x),
                "y": int(obj.y),
                "w": int(obj.w),
                "h": int(obj.h),
                "cx": center_x,
                "cy": center_y,
                "cx_norm": center_x / width,
                "cy_norm": center_y / height,
                "w_norm": obj.w / width,
                "h_norm": obj.h / height,
            })
        return results


def draw_detections(img, detections):
    for d in detections:
        img.draw_rect(d["x"], d["y"], d["w"], d["h"], color=image.COLOR_RED)
        msg = "{}: {:.2f}".format(d["number"], d["score"])
        img.draw_string(d["x"], max(0, d["y"] - 10), msg, color=image.COLOR_GREEN)


# ============================================================================
# Visualization helpers — draw line tracking & crossroad detection on image
# ============================================================================

def _draw_line_tracking(img, line_result):
    """Draw ROIs, fitted line, and error indicator. Coords in 320x240 space."""
    sx = img.width() / IMAGE_WIDTH
    sy = img.height() / IMAGE_HEIGHT

    for roi in ROIS:
        img.draw_rect(
            int(roi[0] * sx), int(roi[1] * sy),
            int(roi[2] * sx), int(roi[3] * sy),
            image.COLOR_BLUE, 1,
        )

    for x, y, _blob in line_result.get("points", []):
        img.draw_cross(int(x * sx), int(y * sy), image.COLOR_RED, 6, 2)

    if not line_result.get("valid"):
        return

    img.draw_line(
        int(line_result["x_top"] * sx),
        int(line_result["line_top_y"] * sy),
        int(line_result["x_bottom"] * sx),
        int(TARGET_Y * sy),
        image.COLOR_GREEN, 3,
    )
    img.draw_line(
        int(IMAGE_CENTER_X * sx),
        int(TARGET_Y * sy),
        int(line_result["x_bottom"] * sx),
        int(TARGET_Y * sy),
        image.COLOR_RED, 2,
    )


def _draw_crossroad(img, cross_result):
    """Draw 4 ROIs, V/H detection lines, and cross point."""
    sx = img.width() / IMAGE_WIDTH
    sy = img.height() / IMAGE_HEIGHT

    for roi, color in [
        (V_ROI_TOP, image.COLOR_BLUE),
        (V_ROI_BOTTOM, image.COLOR_BLUE),
        (H_ROI_LEFT, image.COLOR_GRAY),
        (H_ROI_RIGHT, image.COLOR_GRAY),
    ]:
        img.draw_rect(
            int(roi[0] * sx), int(roi[1] * sy),
            int(roi[2] * sx), int(roi[3] * sy),
            color, 1,
        )

    for line_data, color in [
        (cross_result.get("v_line"), image.COLOR_GREEN),
        (cross_result.get("h_line"), image.COLOR_YELLOW),
    ]:
        if line_data is None:
            continue
        x1, y1, x2, y2 = line_data[0], line_data[1], line_data[2], line_data[3]
        if abs(x2 - x1) > abs(y2 - y1):
            if abs(x2 - x1) < 1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            xs, xe = 0, IMAGE_WIDTH - 1
            ys, ye = y1 + slope * (xs - x1), y1 + slope * (xe - x1)
        else:
            if abs(y2 - y1) < 1:
                continue
            slope = (x2 - x1) / (y2 - y1)
            ys, ye = 0, IMAGE_HEIGHT - 1
            xs, xe = x1 + slope * (ys - y1), x1 + slope * (ye - y1)
        img.draw_line(
            int(_clamp(xs, 0, IMAGE_WIDTH - 1) * sx),
            int(_clamp(ys, 0, IMAGE_HEIGHT - 1) * sy),
            int(_clamp(xe, 0, IMAGE_WIDTH - 1) * sx),
            int(_clamp(ye, 0, IMAGE_HEIGHT - 1) * sy),
            color, 3,
        )

    cross = cross_result.get("cross")
    if cross is not None:
        x, y, _angle = cross
        c = image.COLOR_RED if cross_result.get("confirmed") else image.COLOR_WHITE
        img.draw_cross(int(x * sx), int(y * sy), c, 14, 3)
        img.draw_circle(int(x * sx), int(y * sy), int(18 * min(sx, sy)), c, 2)


# ============================================================================
# delivery_car_test.py — touch-screen test controller
# ============================================================================

CAM_FPS = 30
YOLO_EVERY_N_FRAMES = 2
DISPLAY_EVERY_N_FRAMES = 2
LINE_PACKET_EVERY_N_FRAMES = 2

TARGET_ROI = (0.20, 0.10, 0.80, 0.90)
DEFAULT_CENTER_X_NORM = 0.50

FIXED_ROUTES = {
    1: [LEFT],
    2: [RIGHT],
}

STATE_CODES = {
    CAPTURE_TARGET: 1,
    WAIT_START: 2,
    FOLLOW_LINE: 3,
    DECIDE_DIRECTION: 4,
    WAIT_TURN_DONE: 5,
}

BTN_HEIGHT = 42
BTN_GAP = 4
BTN_MARGIN_BOTTOM = 4
BTN_LABELS = ["RESET", "START", "STOP", "TURN_DONE"]
BTN_COLORS = {
    "RESET": (255, 180, 50),
    "START": (50, 200, 50),
    "STOP": (220, 50, 50),
    "TURN_DONE": (50, 120, 220),
}

_TX_TYPE_NAMES = {
    0x81: "TARGET",
    0x82: "LINE",
    0x83: "TURN",
    0x84: "HOLD",
    0x85: "STATE",
}


def _prepare_vision_image(img):
    rgb = img
    if img.format() != image.Format.FMT_RGB888:
        rgb = img.to_format(image.Format.FMT_RGB888)
    if rgb.width() != IMAGE_WIDTH or rgb.height() != IMAGE_HEIGHT:
        rgb = rgb.resize(IMAGE_WIDTH, IMAGE_HEIGHT)
    return rgb


def _direction_confidence(voter, direction):
    scores, _ = voter.scores()
    total = scores[LEFT] + scores[RIGHT]
    if total <= 0 or direction not in (LEFT, RIGHT):
        return 0
    return round(scores[direction] * 100 / total)


def _draw_target_roi(img):
    x_min, y_min, x_max, y_max = TARGET_ROI
    x = int(x_min * img.width())
    y = int(y_min * img.height())
    w = int((x_max - x_min) * img.width())
    h = int((y_max - y_min) * img.height())
    img.draw_rect(x, y, w, h, image.COLOR_BLUE, 2)


def _draw_status(img, machine, line_result, cross_result, voter, last_event, last_tx):
    scores, counts = voter.scores()
    target_text = "-" if machine.target_number is None else str(machine.target_number)
    line_text = "OK" if line_result.get("valid") else "LOST"
    cross_text = "OK" if cross_result.get("confirmed") else "---"
    lines = [
        "S:{} T:{} DIR:{}".format(machine.state, target_text, machine.last_direction),
        "LINE:{} CROSS:{}".format(line_text, cross_text),
        "V L:{}/{:.1f} R:{}/{:.1f}".format(
            counts[LEFT], scores[LEFT], counts[RIGHT], scores[RIGHT],
        ),
        "EVT:{} TX:{}".format(last_event, last_tx),
    ]
    for i, text in enumerate(lines):
        img.draw_string(2, 2 + i * 16, text, image.COLOR_WHITE)


class DeliveryControllerTest:
    def __init__(self):
        self.digit_detector = DigitDetector(dual_buff=False)
        self.line_tracker = LineTracker(LINE_COLOR)
        self.crossroad_detector = CrossroadDetector(LINE_COLOR)
        self.target_locker = TargetLocker(roi=TARGET_ROI)
        self.direction_voter = DirectionVoter()
        self.machine = DeliveryStateMachine()
        self.cam = camera.Camera(
            self.digit_detector.input_width(),
            self.digit_detector.input_height(),
            self.digit_detector.input_format(),
            fps=CAM_FPS,
        )
        self.disp = display.Display()
        self.ts = touchscreen.TouchScreen()

        self.frame_index = 0
        self.intersection_index = 0
        self.last_event = "NONE"
        self.last_tx = "-"
        self.last_detections = []
        self.last_line_result = {"valid": False}
        self.last_cross_result = {"detected": False, "confirmed": False}
        self.hold_sent = False

        self._btn_rects_disp = {}
        self._btn_rects_computed = False

    def reset(self):
        self.machine.reset()
        self.target_locker.reset()
        self.direction_voter.clear()
        self.line_tracker.reset()
        self.crossroad_detector.reset()
        self.intersection_index = 0
        self.last_detections = []
        self.last_line_result = {"valid": False}
        self.last_cross_result = {"detected": False, "confirmed": False}
        self.hold_sent = False

    def send(self, packet):
        if len(packet) >= 4:
            msg_type = packet[3]
            self.last_tx = _TX_TYPE_NAMES.get(msg_type, "??")
        else:
            self.last_tx = "?"

    def send_state(self):
        self.send(encode_status(STATE_CODES[self.machine.state]))

    def _compute_buttons(self, img_w, img_h):
        n = len(BTN_LABELS)
        total_gap = BTN_GAP * (n + 1)
        btn_w = (img_w - total_gap) // n
        btn_y = img_h - BTN_HEIGHT - BTN_MARGIN_BOTTOM
        for i, label in enumerate(BTN_LABELS):
            btn_x = BTN_GAP + i * (btn_w + BTN_GAP)
            rect_disp = image.resize_map_pos(
                img_w, img_h,
                self.disp.width(), self.disp.height(),
                image.Fit.FIT_CONTAIN,
                btn_x, btn_y, btn_w, BTN_HEIGHT,
            )
            self._btn_rects_disp[label] = rect_disp

    def _draw_buttons(self, img):
        img_w, img_h = img.width(), img.height()
        n = len(BTN_LABELS)
        total_gap = BTN_GAP * (n + 1)
        btn_w = (img_w - total_gap) // n
        btn_y = img_h - BTN_HEIGHT - BTN_MARGIN_BOTTOM

        if not self._btn_rects_computed:
            self._compute_buttons(img_w, img_h)
            self._btn_rects_computed = True

        for i, label in enumerate(BTN_LABELS):
            btn_x = BTN_GAP + i * (btn_w + BTN_GAP)
            r, g, b = BTN_COLORS[label]
            color = image.Color.from_rgb(r, g, b)
            img.draw_rect(btn_x, btn_y, btn_w, BTN_HEIGHT, color, -1)
            img.draw_rect(btn_x, btn_y, btn_w, BTN_HEIGHT, image.COLOR_WHITE, 2)

            text_size = image.string_size(label, scale=1.3)
            text_x = btn_x + (btn_w - text_size.width()) // 2
            text_y = btn_y + (BTN_HEIGHT - text_size.height()) // 2
            img.draw_string(text_x, text_y, label, image.COLOR_WHITE, 1.3)

    def _handle_touch(self, tx, ty):
        for label, (dx, dy, dw, dh) in self._btn_rects_disp.items():
            if dx <= tx <= dx + dw and dy <= ty <= dy + dh:
                self._inject_command(label)
                return

    def _inject_command(self, label):
        if label == "RESET":
            self.last_event = "RESET"
            self.reset()
            self.send_state()
        elif label == "STOP":
            self.last_event = "STOP"
            self.machine.stop()
            self.direction_voter.clear()
            self.hold_sent = False
            self.send_state()
        elif label == "START":
            self.last_event = "START"
            if self.machine.start():
                self.direction_voter.clear()
                self.line_tracker.reset()
                self.crossroad_detector.reset()
                self.hold_sent = False
                self.send_state()
        elif label == "TURN_DONE":
            self.last_event = "TURN_DONE"
            if self.machine.turn_done():
                self.direction_voter.clear()
                self.crossroad_detector.reset()
                self.hold_sent = False
                self.send_state()

    def update_target(self, detections):
        target = self.target_locker.update(detections)
        if target is not None and self.machine.lock_target(target):
            self.send(encode_target_locked(target))
            self.last_event = "TARGET_LOCKED"
            self.send_state()

    def road_center(self, line_result, cross_result):
        if cross_result.get("confirmed") and cross_result.get("center_x_norm") is not None:
            return cross_result["center_x_norm"]
        if line_result.get("valid"):
            return line_result["decision_center_x_norm"]
        return DEFAULT_CENTER_X_NORM

    def update_direction_votes(self, detections, line_result, cross_result):
        if self.machine.target_number in (1, 2):
            return
        center = self.road_center(line_result, cross_result)
        self.direction_voter.add(detections, self.machine.target_number, center)

    def fixed_direction(self):
        return fixed_route_direction(
            self.machine.target_number,
            self.intersection_index,
            FIXED_ROUTES,
        )

    def decide_direction(self):
        if self.machine.target_number in (1, 2):
            direction = self.fixed_direction()
            hold_reason = HOLD_FIXED_ROUTE_MISSING
        else:
            direction = self.direction_voter.decision()
            hold_reason = HOLD_DIRECTION_UNCERTAIN

        if direction in (LEFT, RIGHT) and self.machine.lock_direction(direction):
            confidence = _direction_confidence(self.direction_voter, direction)
            self.send(
                encode_turn_decision(
                    direction,
                    target_number=self.machine.target_number,
                    intersection_index=self.intersection_index,
                    confidence=confidence,
                )
            )
            self.intersection_index += 1
            self.hold_sent = False
            self.send_state()
        elif direction == UNKNOWN and not self.hold_sent:
            self.send(encode_vision_hold(hold_reason))
            self.hold_sent = True

    def process_frame(self):
        model_img = self.cam.read()
        vision_img = _prepare_vision_image(model_img)
        self.frame_index += 1

        run_yolo = (
            self.machine.state in (CAPTURE_TARGET, DECIDE_DIRECTION)
            or self.frame_index % YOLO_EVERY_N_FRAMES == 0
        )
        if run_yolo:
            self.last_detections = self.digit_detector.detect(model_img)

        if self.machine.state == CAPTURE_TARGET:
            self.update_target(self.last_detections)
            line_result = {"valid": False}
            cross_result = {"detected": False, "confirmed": False}
        elif self.machine.state == WAIT_START:
            line_result = {"valid": False}
            cross_result = {"detected": False, "confirmed": False}
        else:
            line_result = self.line_tracker.process(vision_img)
            cross_result = self.crossroad_detector.process(vision_img)
            self.last_line_result = line_result
            self.last_cross_result = cross_result

            if (
                self.machine.state in (FOLLOW_LINE, DECIDE_DIRECTION)
                and self.frame_index % LINE_PACKET_EVERY_N_FRAMES == 0
            ):
                self.send(
                    encode_line_data(
                        line_result["valid"],
                        line_result.get("error", 0.0),
                        line_result.get("angle", 0.0),
                        line_result.get("center_x_norm", DEFAULT_CENTER_X_NORM),
                    )
                )

            if self.machine.state in (FOLLOW_LINE, DECIDE_DIRECTION) and run_yolo:
                self.update_direction_votes(
                    self.last_detections, line_result, cross_result
                )

            self.machine.update_crossroad_visibility(cross_result["detected"])
            if cross_result["confirmed"] and cross_result["detected"]:
                if self.machine.trigger_crossroad():
                    self.send_state()
            if self.machine.state == DECIDE_DIRECTION:
                self.decide_direction()

        if self.frame_index % DISPLAY_EVERY_N_FRAMES == 0:
            draw_detections(model_img, self.last_detections)
            if self.machine.state == CAPTURE_TARGET:
                _draw_target_roi(model_img)
            if self.machine.state in (FOLLOW_LINE, DECIDE_DIRECTION, WAIT_TURN_DONE):
                _draw_line_tracking(model_img, line_result)
                _draw_crossroad(model_img, cross_result)
            self._draw_buttons(model_img)
            _draw_status(
                model_img,
                self.machine,
                line_result,
                cross_result,
                self.direction_voter,
                self.last_event,
                self.last_tx,
            )
            self.disp.show(model_img)

    def run(self):
        self.send_state()
        pressed_already = False
        while not app.need_exit():
            x, y, pressed = self.ts.read()
            if pressed:
                pressed_already = True
            else:
                if pressed_already:
                    pressed_already = False
                    self._handle_touch(x, y)

            self.process_frame()
            time.sleep_ms(1)


def main():
    DeliveryControllerTest().run()


if __name__ == "__main__":
    main()

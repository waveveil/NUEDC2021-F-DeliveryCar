from collections import Counter, deque

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
    def __init__(
        self,
        window_size=7,
        min_votes=5,
        min_score=0.55,
        roi=(0.20, 0.10, 0.80, 0.90),
        min_vote_margin=2,
    ):
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
    def __init__(
        self,
        window_size=8,
        min_samples=4,
        dead_zone=0.07,
        min_winner_score=2.2,
        min_score_margin=0.8,
        min_detection_score=0.45,
    ):
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
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = CAPTURE_TARGET
        self.target_number = None
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
        return True

    def stop(self):
        self.state = WAIT_START if self.target_number is not None else CAPTURE_TARGET
        self.last_direction = UNKNOWN

    def lock_direction(self, direction):
        if self.state not in (FOLLOW_LINE, DECIDE_DIRECTION) or direction not in (LEFT, RIGHT):
            return False
        self.last_direction = direction
        self.state = WAIT_TURN_DONE
        return True

    def turn_done(self):
        if self.state != WAIT_TURN_DONE:
            return False
        self.state = FOLLOW_LINE
        return True

"""MaixCAM 路口检测 V3 —— 色块法，新增 T 字路口识别。

与 V2 的区别：
- V2 要求 4 个 ROI 全部命中才算十字路口 (+)
- V3 允许"一条完整线 + 另一方向单侧命中"即识别为 T 字路口
- 新增 junction_type 字段："crossroad" / "t_left" / "t_right" / "t_up" / "t_down" / "none"

四种 T 字路口：
- t_left:  竖直线 + 仅左侧有水平横线（可左转）
- t_right: 竖直线 + 仅右侧有水平横线（可右转）
- t_up:    水平线 + 仅上方有竖直线段
- t_down:  水平线 + 仅下方有竖直线段（小车最常见：沿竖线走遇到横线）
- crossroad: 竖直线 + 左右均有水平横线（+ 字路口）
"""

import math

from maix import app, camera, display, image, time


# ============================================================
# 通用配置（与 V2 保持一致）
# ============================================================

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_CENTER_X = IMAGE_WIDTH // 2
IMAGE_CENTER_Y = IMAGE_HEIGHT // 2

LINE_COLOR = "black"

BLACK_L_MAX = 30
BLACK_THRESHOLD = [[0, BLACK_L_MAX, -128, 127, -128, 127]]
RED_THRESHOLD = [[0, 80, 40, 80, 0, 80]]

# -------- 竖直方向 ROI --------
V_ROI_TOP    = [110, 50,  100, 30]
V_ROI_BOTTOM = [80,  160, 160, 40]

# -------- 水平方向 ROI --------
H_ROI_LEFT  = [80,  80, 30, 80]
H_ROI_RIGHT = [210, 80, 30, 80]

# 色块筛选
MIN_PIXELS = 20
MIN_AREA   = 20
MAX_BLOB_WIDTH_RATIO  = 0.70
MAX_BLOB_HEIGHT_RATIO = 0.70

# T 字路口单侧检测时，稍微提高像素门槛以减少误检
T_MIN_PIXELS = 30

# 竖直线斜率阈值
MAX_V_SLOPE = 0.577
# 水平线斜率阈值
MAX_H_SLOPE = 0.577

# 两线最小夹角（度）
MIN_CROSS_ANGLE = 80.0

# 交点距画面中心最大距离
MAX_CROSS_CENTER_DIST = 120.0

# 连续帧确认 / 丢失
CROSS_CONFIRM_FRAMES = 3
CROSS_LOST_FRAMES    = 3
T_CONFIRM_FRAMES     = 4   # T 字路口多确认一帧，减少误检
T_LOST_FRAMES        = 3

# 低通滤波
FILTER_ALPHA = 0.4

# -------- 路口类型 --------
JUNCTION_NONE      = "none"
JUNCTION_CROSSROAD = "crossroad"
JUNCTION_T_LEFT    = "t_left"
JUNCTION_T_RIGHT   = "t_right"
JUNCTION_T_UP      = "t_up"
JUNCTION_T_DOWN    = "t_down"


def clamp(value, low, high):
    return max(low, min(high, value))


def line_threshold(line_color):
    if line_color == "black":
        return BLACK_THRESHOLD
    if line_color == "red":
        return RED_THRESHOLD
    raise ValueError("line_color must be 'black' or 'red'")


# ============================================================
# 色块选择
# ============================================================

def choose_blob_v(blobs, expected_x, roi_width):
    """竖直 ROI 中选最佳色块。"""
    best = None
    best_score = -1000000
    max_blob_width = int(roi_width * MAX_BLOB_WIDTH_RATIO)

    for blob in blobs:
        if blob.pixels() < MIN_PIXELS:
            continue
        if blob.w() > max_blob_width:
            continue
        score = blob.pixels() - abs(blob.cx() - expected_x) * 0.8
        if score > best_score:
            best = blob
            best_score = score
    return best


def choose_blob_h(blobs, expected_y, roi_height, min_pixels=MIN_PIXELS):
    """水平 ROI 中选最佳色块。"""
    best = None
    best_score = -1000000
    max_blob_height = int(roi_height * MAX_BLOB_HEIGHT_RATIO)

    for blob in blobs:
        if blob.pixels() < min_pixels:
            continue
        if blob.h() > max_blob_height:
            continue
        score = blob.pixels() - abs(blob.cy() - expected_y) * 0.8
        if score > best_score:
            best = blob
            best_score = score
    return best


# ============================================================
# 线段检测
# ============================================================

def detect_vertical_line(img, expected_x, threshold=BLACK_THRESHOLD):
    """检测竖直线（两个竖直 ROI 色块连线）。"""
    top_blobs = img.find_blobs(
        threshold, roi=V_ROI_TOP,
        area_threshold=MIN_AREA, pixels_threshold=MIN_PIXELS,
        x_stride=2, y_stride=1,
    )
    top_blob = choose_blob_v(top_blobs, expected_x, V_ROI_TOP[2])

    bottom_blobs = img.find_blobs(
        threshold, roi=V_ROI_BOTTOM,
        area_threshold=MIN_AREA, pixels_threshold=MIN_PIXELS,
        x_stride=2, y_stride=1,
    )
    bottom_blob = choose_blob_v(bottom_blobs, expected_x, V_ROI_BOTTOM[2])

    if top_blob is None or bottom_blob is None:
        return None

    x_top    = top_blob.cx()
    y_top    = V_ROI_TOP[1] + V_ROI_TOP[3] // 2
    x_bottom = bottom_blob.cx()
    y_bottom = V_ROI_BOTTOM[1] + V_ROI_BOTTOM[3] // 2

    dy = y_bottom - y_top
    if abs(dy) < 1:
        return None
    slope = (x_bottom - x_top) / dy
    if abs(slope) > MAX_V_SLOPE:
        return None

    return (x_top, y_top, x_bottom, y_bottom, top_blob, bottom_blob)


def _detect_horizontal_segment(img, roi, expected_y, threshold, min_pixels):
    """在单个水平 ROI 中检测色块。

    Returns (x_center, y_center, blob) or None。
    """
    blobs = img.find_blobs(
        threshold, roi=roi,
        area_threshold=MIN_AREA, pixels_threshold=min_pixels,
        x_stride=1, y_stride=2,
    )
    blob = choose_blob_h(blobs, expected_y, roi[3], min_pixels=min_pixels)
    if blob is None:
        return None
    x = roi[0] + roi[2] // 2
    y = blob.cy()
    return (x, y, blob)


def detect_horizontal_full(img, expected_y, threshold=BLACK_THRESHOLD):
    """检测完整水平线（左右 ROI 都有色块）。

    Returns (x_left, y_left, x_right, y_right, left_blob, right_blob) or None。
    """
    left_seg = _detect_horizontal_segment(
        img, H_ROI_LEFT, expected_y, threshold, MIN_PIXELS)
    right_seg = _detect_horizontal_segment(
        img, H_ROI_RIGHT, expected_y, threshold, MIN_PIXELS)

    if left_seg is None or right_seg is None:
        return None

    x_left, y_left, left_blob   = left_seg
    x_right, y_right, right_blob = right_seg

    dx = x_right - x_left
    if abs(dx) < 1:
        return None
    slope = (y_right - y_left) / dx
    if abs(slope) > MAX_H_SLOPE:
        return None

    return (x_left, y_left, x_right, y_right, left_blob, right_blob)


def detect_horizontal_left(img, expected_y, threshold=BLACK_THRESHOLD):
    """仅检测左侧 ROI 是否有水平线段。"""
    return _detect_horizontal_segment(
        img, H_ROI_LEFT, expected_y, threshold, T_MIN_PIXELS)


def detect_horizontal_right(img, expected_y, threshold=BLACK_THRESHOLD):
    """仅检测右侧 ROI 是否有水平线段。"""
    return _detect_horizontal_segment(
        img, H_ROI_RIGHT, expected_y, threshold, T_MIN_PIXELS)


def detect_vertical_top(img, expected_x, threshold=BLACK_THRESHOLD):
    """仅检测上方竖直 ROI 是否有竖直线段。"""
    blobs = img.find_blobs(
        threshold, roi=V_ROI_TOP,
        area_threshold=MIN_AREA, pixels_threshold=T_MIN_PIXELS,
        x_stride=2, y_stride=1,
    )
    blob = choose_blob_v(blobs, expected_x, V_ROI_TOP[2])
    if blob is None:
        return None
    x = blob.cx()
    y = V_ROI_TOP[1] + V_ROI_TOP[3] // 2
    return (x, y, blob)


def detect_vertical_bottom(img, expected_x, threshold=BLACK_THRESHOLD):
    """仅检测下方竖直 ROI 是否有竖直线段。"""
    blobs = img.find_blobs(
        threshold, roi=V_ROI_BOTTOM,
        area_threshold=MIN_AREA, pixels_threshold=T_MIN_PIXELS,
        x_stride=2, y_stride=1,
    )
    blob = choose_blob_v(blobs, expected_x, V_ROI_BOTTOM[2])
    if blob is None:
        return None
    x = blob.cx()
    y = V_ROI_BOTTOM[1] + V_ROI_BOTTOM[3] // 2
    return (x, y, blob)


# ============================================================
# 交点计算
# ============================================================

def line_intersection(v_line, h_line):
    """计算竖直线与水平线的交点及夹角（同 V2）。"""
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
    cos_angle = clamp(dot / (v_len * h_len), -1.0, 1.0)
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


def estimate_t_intersection(v_line, h_seg, side):
    """T 字路口交点估算。

    通过竖直线与经过水平色块的水平线求交点。
    h_seg: (x, y, blob) 单侧水平色块。

    Returns (ix, iy, angle_deg) or None。
    """
    if v_line is None or h_seg is None:
        return None

    x1, y1, x2, y2 = v_line[0], v_line[1], v_line[2], v_line[3]
    hx, hy, _blob = h_seg

    # 构造经过色块的完美水平线
    hx1, hy1 = 0, hy
    hx2, hy2 = IMAGE_WIDTH - 1, hy

    v_dx = x2 - x1
    v_dy = y2 - y1
    h_dx = hx2 - hx1
    h_dy = hy2 - hy1  # = 0

    v_len = math.hypot(v_dx, v_dy)
    if v_len < 1:
        return None

    # 交点：竖直线参数方程在 y=hy 处
    if abs(v_dy) < 1e-6:
        return None
    t = (hy - y1) / v_dy
    ix = x1 + t * v_dx
    iy = hy

    if not (0 <= ix < IMAGE_WIDTH and 0 <= iy < IMAGE_HEIGHT):
        return None

    center_dist = math.hypot(ix - IMAGE_CENTER_X, iy - IMAGE_CENTER_Y)
    if center_dist > MAX_CROSS_CENTER_DIST:
        return None

    # 色块必须在交点的正确一侧
    if side == "left" and hx > ix + 10:
        return None
    if side == "right" and hx < ix - 10:
        return None

    # 竖线与水平线夹角
    dot = abs(v_dx * h_dx)
    cos_angle = clamp(dot / (v_len * max(abs(h_dx), 1)), -1.0, 1.0)
    angle_deg = math.degrees(math.acos(cos_angle))
    if angle_deg > 90:
        angle_deg = 180 - angle_deg

    if angle_deg < MIN_CROSS_ANGLE:
        return None

    return int(round(ix)), int(round(iy)), angle_deg


def estimate_t_intersection_v(h_line, v_seg, side):
    """T 字路口交点估算（竖直单侧版本，用于 t_up / t_down）。

    通过水平线与经过竖直色块的竖直线求交点。
    h_line: 完整水平线 (x_left, y_left, x_right, y_right, ...)
    v_seg:  (x, y, blob) 单侧竖直色块
    side:   "top" 或 "bottom"

    Returns (ix, iy, angle_deg) or None。
    """
    if h_line is None or v_seg is None:
        return None

    x1, y1, x2, y2 = h_line[0], h_line[1], h_line[2], h_line[3]
    vx, vy, _blob = v_seg

    # 构造经过色块的完美竖直线
    vx1, vy1 = vx, 0
    vx2, vy2 = vx, IMAGE_HEIGHT - 1

    h_dx = x2 - x1
    h_dy = y2 - y1
    v_dx = vx2 - vx1  # = 0
    v_dy = vy2 - vy1

    h_len = math.hypot(h_dx, h_dy)
    if h_len < 1:
        return None

    # 交点：水平线参数方程在 x=vx 处
    if abs(h_dx) < 1e-6:
        return None
    t = (vx - x1) / h_dx
    ix = vx
    iy = y1 + t * h_dy

    if not (0 <= ix < IMAGE_WIDTH and 0 <= iy < IMAGE_HEIGHT):
        return None

    center_dist = math.hypot(ix - IMAGE_CENTER_X, iy - IMAGE_CENTER_Y)
    if center_dist > MAX_CROSS_CENTER_DIST:
        return None

    # 色块必须在交点的正确一侧
    if side == "top" and vy > iy + 10:
        return None
    if side == "bottom" and vy < iy - 10:
        return None

    # 水平线与竖直线夹角
    v_len = abs(v_dy)
    dot = abs(h_dy * v_dy)
    cos_angle = clamp(dot / (h_len * max(v_len, 1)), -1.0, 1.0)
    angle_deg = math.degrees(math.acos(cos_angle))
    if angle_deg > 90:
        angle_deg = 180 - angle_deg

    if angle_deg < MIN_CROSS_ANGLE:
        return None

    return int(round(ix)), int(round(iy)), angle_deg


# ============================================================
# 路口检测器 V3
# ============================================================

class CrossroadDetectorV3:
    def __init__(self, line_color=LINE_COLOR, filter_alpha=FILTER_ALPHA):
        self.line_color = line_color
        self.threshold = line_threshold(line_color)
        self.filter_alpha = filter_alpha
        self.reset()

    def reset(self):
        # 十字路口
        self.cross_count = 0
        self.cross_lost  = 0
        self.cross_confirmed = False

        # T 字路口 — 水平单侧
        self.t_left_count  = 0
        self.t_left_lost   = 0
        self.t_left_confirmed  = False
        self.t_right_count = 0
        self.t_right_lost  = 0
        self.t_right_confirmed = False

        # T 字路口 — 竖直单侧
        self.t_up_count   = 0
        self.t_up_lost    = 0
        self.t_up_confirmed   = False
        self.t_down_count = 0
        self.t_down_lost  = 0
        self.t_down_confirmed = False

        # 预测位置
        self.expected_x = IMAGE_CENTER_X
        self.expected_y = IMAGE_CENTER_Y

        # 低通滤波
        self.filtered_cross_x = float(IMAGE_CENTER_X)
        self.filtered_cross_y = float(IMAGE_CENTER_Y)
        self.filter_initialized = False

    # ----------------------------------------------------------
    # 内部：确认 / 丢失计数更新
    # ----------------------------------------------------------

    @staticmethod
    def _update_counters(hit, count, lost, confirm_n, lost_n):
        if hit:
            count += 1
            lost = 0
        else:
            lost += 1
            count = 0
        confirmed = count >= confirm_n
        if lost >= lost_n:
            confirmed = False
        return count, lost, confirmed

    # ----------------------------------------------------------
    # 主处理
    # ----------------------------------------------------------

    def process(self, img):
        # 1. 基础检测（全部 6 种）
        v_line  = detect_vertical_line(img, self.expected_x, self.threshold)
        h_line  = detect_horizontal_full(img, self.expected_y, self.threshold)
        h_left  = detect_horizontal_left(img, self.expected_y, self.threshold)
        h_right = detect_horizontal_right(img, self.expected_y, self.threshold)
        v_top   = detect_vertical_top(img, self.expected_x, self.threshold)
        v_bot   = detect_vertical_bottom(img, self.expected_x, self.threshold)

        # 2. 判断路口类型
        # 十字路口：竖直线 + 水平线都完整
        cross_hit = (v_line is not None and h_line is not None)
        # T 字（水平单侧）：竖直线完整 + 水平仅单侧
        t_left_hit  = (v_line is not None and h_line is None and h_left is not None)
        t_right_hit = (v_line is not None and h_line is None and h_right is not None)
        # T 字（竖直单侧）：水平线完整 + 竖直仅单侧
        t_up_hit   = (h_line is not None and v_line is None and v_top is not None)
        t_down_hit = (h_line is not None and v_line is None and v_bot is not None)

        # 3. 更新各类路口的确认计数
        self.cross_count, self.cross_lost, self.cross_confirmed = \
            self._update_counters(
                cross_hit, self.cross_count, self.cross_lost,
                CROSS_CONFIRM_FRAMES, CROSS_LOST_FRAMES)

        self.t_left_count, self.t_left_lost, self.t_left_confirmed = \
            self._update_counters(
                t_left_hit, self.t_left_count, self.t_left_lost,
                T_CONFIRM_FRAMES, T_LOST_FRAMES)

        self.t_right_count, self.t_right_lost, self.t_right_confirmed = \
            self._update_counters(
                t_right_hit, self.t_right_count, self.t_right_lost,
                T_CONFIRM_FRAMES, T_LOST_FRAMES)

        self.t_up_count, self.t_up_lost, self.t_up_confirmed = \
            self._update_counters(
                t_up_hit, self.t_up_count, self.t_up_lost,
                T_CONFIRM_FRAMES, T_LOST_FRAMES)

        self.t_down_count, self.t_down_lost, self.t_down_confirmed = \
            self._update_counters(
                t_down_hit, self.t_down_count, self.t_down_lost,
                T_CONFIRM_FRAMES, T_LOST_FRAMES)

        # 4. 确定当前帧的路口类型和交点
        junction_type = JUNCTION_NONE
        cross = None
        score = 0.0
        h_line_for_output = h_line  # 输出用的 h_line（完整水平线或 None）

        if cross_hit:
            junction_type = JUNCTION_CROSSROAD
            cross = line_intersection(v_line, h_line)
            if v_line is not None and h_line is not None:
                score = math.hypot(v_line[2] - v_line[0], v_line[3] - v_line[1])
                score += math.hypot(h_line[2] - h_line[0], h_line[3] - h_line[1])
        elif t_left_hit:
            junction_type = JUNCTION_T_LEFT
            cross = estimate_t_intersection(v_line, h_left, "left")
            if v_line is not None:
                score = math.hypot(v_line[2] - v_line[0], v_line[3] - v_line[1])
        elif t_right_hit:
            junction_type = JUNCTION_T_RIGHT
            cross = estimate_t_intersection(v_line, h_right, "right")
            if v_line is not None:
                score = math.hypot(v_line[2] - v_line[0], v_line[3] - v_line[1])
        elif t_up_hit:
            junction_type = JUNCTION_T_UP
            cross = estimate_t_intersection_v(h_line, v_top, "top")
            if h_line is not None:
                score = math.hypot(h_line[2] - h_line[0], h_line[3] - h_line[1])
        elif t_down_hit:
            junction_type = JUNCTION_T_DOWN
            cross = estimate_t_intersection_v(h_line, v_bot, "bottom")
            if h_line is not None:
                score = math.hypot(h_line[2] - h_line[0], h_line[3] - h_line[1])

        # 5. 低通滤波（交点在时更新）
        if cross is not None:
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

        # 6. 更新预测位置
        if v_line is not None:
            self.expected_x = v_line[2]
        elif v_bot is not None:
            self.expected_x = v_bot[0]
        elif v_top is not None:
            self.expected_x = v_top[0]

        if h_line is not None:
            self.expected_y = h_line[1]
        elif h_left is not None:
            self.expected_y = h_left[1]
        elif h_right is not None:
            self.expected_y = h_right[1]

        # 7. 输出
        any_detected = (cross_hit or t_left_hit or t_right_hit
                        or t_up_hit or t_down_hit)
        any_confirmed = (self.cross_confirmed
                         or self.t_left_confirmed
                         or self.t_right_confirmed
                         or self.t_up_confirmed
                         or self.t_down_confirmed)

        x = int(round(self.filtered_cross_x)) if cross is not None else 0
        y = int(round(self.filtered_cross_y)) if cross is not None else 0
        angle = cross[2] if cross is not None else 0.0

        return {
            # ---- 兼容 V2 字段 ----
            "detected":  any_detected,
            "confirmed": any_confirmed,
            "x": x,
            "y": y,
            "center_x_norm": x / IMAGE_WIDTH if cross is not None else None,
            "angle": angle,
            "score": score,
            "v_line": v_line,
            "h_line": h_line_for_output,
            "cross": cross,
            "cross_count": self.cross_count,

            # ---- V3 新增 ----
            "junction_type": junction_type,

            # 各子类型的确认状态
            "crossroad_confirmed": self.cross_confirmed,
            "t_left_confirmed":    self.t_left_confirmed,
            "t_right_confirmed":   self.t_right_confirmed,
            "t_up_confirmed":     self.t_up_confirmed,
            "t_down_confirmed":   self.t_down_confirmed,

            # 单侧色块原始检测（调试/可视化用）
            "h_left":     h_left,
            "h_right":    h_right,
            "v_top":      v_top,
            "v_bot":      v_bot,
            "t_left_hit":  t_left_hit,
            "t_right_hit": t_right_hit,
            "t_up_hit":    t_up_hit,
            "t_down_hit":  t_down_hit,
        }


# ============================================================
# 可视化
# ============================================================

def _draw_extended_line(img, x1, y1, x2, y2, color, thickness):
    """将两点线段延伸至图像边界。"""
    dx = x2 - x1
    dy = y2 - y1
    if abs(dx) < 1 and abs(dy) < 1:
        return
    if abs(dx) >= abs(dy):
        slope = dy / dx if abs(dx) > 0 else 0
        xs, xe = 0, IMAGE_WIDTH - 1
        ys = int(y1 + slope * (xs - x1))
        ye = int(y1 + slope * (xe - x1))
    else:
        slope = dx / dy if abs(dy) > 0 else 0
        ys, ye = 0, IMAGE_HEIGHT - 1
        xs = int(x1 + slope * (ys - y1))
        xe = int(x1 + slope * (ye - y1))
    img.draw_line(
        clamp(xs, 0, IMAGE_WIDTH - 1),
        clamp(ys, 0, IMAGE_HEIGHT - 1),
        clamp(xe, 0, IMAGE_WIDTH - 1),
        clamp(ye, 0, IMAGE_HEIGHT - 1),
        color, thickness,
    )


def draw_result(img, result):
    """在图像上绘制检测结果。"""
    # ROI 框
    for roi, color in [
        (V_ROI_TOP,    image.COLOR_BLUE),
        (V_ROI_BOTTOM, image.COLOR_BLUE),
        (H_ROI_LEFT,   image.COLOR_GRAY),
        (H_ROI_RIGHT,  image.COLOR_GRAY),
    ]:
        img.draw_rect(roi[0], roi[1], roi[2], roi[3], color, 1)

    # 竖直线
    v_line = result["v_line"]
    if v_line is not None:
        x_top, y_top, x_bot, y_bot, t_blob, b_blob = v_line
        for blob, rc in [(t_blob, image.COLOR_GREEN), (b_blob, image.COLOR_GREEN)]:
            img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), rc, 2)
            img.draw_cross(blob.cx(), blob.cy(), image.COLOR_RED, 8, 2)
        _draw_extended_line(img, x_top, y_top, x_bot, y_bot, image.COLOR_GREEN, 3)

    # 完整水平线（十字路口）
    h_line = result["h_line"]
    if h_line is not None:
        xl, yl, xr, yr, l_blob, r_blob = h_line
        for blob, rc in [(l_blob, image.COLOR_YELLOW), (r_blob, image.COLOR_YELLOW)]:
            img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), rc, 2)
            img.draw_cross(blob.cx(), blob.cy(), image.COLOR_RED, 8, 2)
        _draw_extended_line(img, xl, yl, xr, yr, image.COLOR_YELLOW, 3)

    # T 字路口 — 水平单侧色块（v_line 存在但 h_line 为 None 时画）
    if h_line is None and v_line is not None:
        for h_seg, color in [
            (result.get("h_left"),  image.COLOR_YELLOW),
            (result.get("h_right"), image.COLOR_YELLOW),
        ]:
            if h_seg is not None:
                x, y, blob = h_seg
                img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), color, 2)
                img.draw_cross(blob.cx(), blob.cy(), image.COLOR_RED, 8, 2)
                img.draw_line(
                    max(0, x - 40), y,
                    min(IMAGE_WIDTH - 1, x + 40), y,
                    color, 2,
                )

    # T 字路口 — 竖直单侧色块（h_line 存在但 v_line 为 None 时画）
    if v_line is None and h_line is not None:
        for v_seg, color in [
            (result.get("v_top"), image.COLOR_GREEN),
            (result.get("v_bot"), image.COLOR_GREEN),
        ]:
            if v_seg is not None:
                x, y, blob = v_seg
                img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), color, 2)
                img.draw_cross(blob.cx(), blob.cy(), image.COLOR_RED, 8, 2)
                img.draw_line(
                    x, max(0, y - 40),
                    x, min(IMAGE_HEIGHT - 1, y + 40),
                    color, 2,
                )

    # 交点
    cross = result["cross"]
    if cross is not None:
        jtype = result["junction_type"]
        if jtype == JUNCTION_CROSSROAD:
            c_color = image.COLOR_RED if result["crossroad_confirmed"] else image.COLOR_WHITE
            type_str = "+"
        elif jtype == JUNCTION_T_LEFT:
            c_color = image.COLOR_RED if result["t_left_confirmed"] else image.COLOR_WHITE
            type_str = "T-L"
        elif jtype == JUNCTION_T_RIGHT:
            c_color = image.COLOR_RED if result["t_right_confirmed"] else image.COLOR_WHITE
            type_str = "T-R"
        elif jtype == JUNCTION_T_UP:
            c_color = image.COLOR_RED if result["t_up_confirmed"] else image.COLOR_WHITE
            type_str = "T-U"
        elif jtype == JUNCTION_T_DOWN:
            c_color = image.COLOR_RED if result["t_down_confirmed"] else image.COLOR_WHITE
            type_str = "T-D"
        else:
            c_color = image.COLOR_WHITE
            type_str = "?"

        x, y, angle = cross
        img.draw_cross(x, y, c_color, 16, 3)
        img.draw_circle(x, y, 20, c_color, 2)
        img.draw_string(
            0, 0,
            "{} {} {}deg".format(type_str, jtype, int(angle)),
            image.COLOR_RED if result["confirmed"] else image.COLOR_WHITE, 1.2,
        )
        img.draw_string(
            0, 18,
            "({},{})".format(x, y),
            image.COLOR_RED if result["confirmed"] else image.COLOR_WHITE, 1.0,
        )
    else:
        # 无交点时显示各检测器状态
        if v_line is not None:
            v_status = "V:OK"
        elif result.get("v_top") is not None:
            v_status = "VT:OK"
        elif result.get("v_bot") is not None:
            v_status = "VB:OK"
        else:
            v_status = "V:---"

        hl_ok = "HL:OK" if result.get("h_left") is not None else "HL:---"
        hr_ok = "HR:OK" if result.get("h_right") is not None else "HR:---"
        parts = [v_status, hl_ok, hr_ok]
        if result["cross_count"] > 0:
            parts.append("+c{}".format(result["cross_count"]))
        if result.get("t_left_hit"):
            parts.append("TL")
        if result.get("t_right_hit"):
            parts.append("TR")
        if result.get("t_up_hit"):
            parts.append("TU")
        if result.get("t_down_hit"):
            parts.append("TD")
        img.draw_string(
            0, 0,
            "  ".join(parts),
            image.COLOR_WHITE, 1.0,
        )

    # FPS
    fps = time.fps()
    img.draw_string(
        IMAGE_WIDTH - 60, 0,
        "FPS:{:.0f}".format(fps),
        image.COLOR_WHITE, 1.0,
    )


# ============================================================
# 独立测试
# ============================================================

def main():
    cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
    disp = display.Display()
    detector = CrossroadDetectorV3(LINE_COLOR)

    print("Crossroad detect V3 | T-junction enabled | {}x{}".format(
        IMAGE_WIDTH, IMAGE_HEIGHT))

    while not app.need_exit():
        img = cam.read()
        result = detector.process(img)
        draw_result(img, result)
        disp.show(img)


if __name__ == "__main__":
    main()

"""MaixCAM / MaixCAM-Pro 十字路口检测 V2 —— 基于 find_blobs() 色块法。

与 V1（HoughLines 线段配对）不同，本版本借鉴 track_line.py 的色块检测思路：

算法流程：
1. 使用 LAB 阈值 find_blobs() 在 4 个 ROI 中寻找黑色线段的色块；
2. 竖直方向两个 ROI（上、下）的色块中心点连线 → 竖直线；
3. 水平方向两个 ROI（左、右）的色块中心点连线 → 水平线；
4. 计算两线交点，若交点合理且两线接近垂直 → 十字路口；
5. 连续帧确认避免误检，低通滤波平滑输出。
"""

import math
import struct

from maix import camera, display, image, app, uart, pinmap, err, time


# ============================================================
# 用户配置
# ============================================================

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_CENTER_X = IMAGE_WIDTH // 2
IMAGE_CENTER_Y = IMAGE_HEIGHT // 2

LINE_COLOR = "black"

# LAB 阈值，与 track_line.py 保持一致；红线阈值需要现场标定。
BLACK_L_MAX = 30
BLACK_THRESHOLD = [[0, BLACK_L_MAX, -128, 127, -128, 127]]
RED_THRESHOLD = [[0, 80, 40, 80, 0, 80]]

# -------- 竖直方向 ROI：两条水平条带，上下分布 --------
# 上方条带：窄一些，避免引入旁边线条；位于画面中上部。
V_ROI_TOP = [110, 50, 100, 30]
# 下方条带：宽一些，适应近处线条变宽；位于画面下半部。
V_ROI_BOTTOM = [80, 160, 160, 40]

# -------- 水平方向 ROI：两条竖直条带，左右分布 --------
# 左侧条带：窄高，捕获横向道路线的左端。
H_ROI_LEFT = [80, 80, 30, 80]
# 右侧条带：窄高，捕获横向道路线的右端。
H_ROI_RIGHT = [210, 80, 30, 80]

# 色块筛选参数
MIN_PIXELS = 20
MIN_AREA = 20
MAX_BLOB_WIDTH_RATIO = 0.70   # 竖直 ROI 中过滤大面积阴影
MAX_BLOB_HEIGHT_RATIO = 0.70  # 水平 ROI 中过滤大面积阴影

# 竖直线斜率阈值：|dx/dy| < tan(30°) ≈ 0.577，即与竖直方向夹角 ≤ 30°
MAX_V_SLOPE = 0.577

# 水平线斜率阈值：|dy/dx| < tan(30°) ≈ 0.577，即与水平方向夹角 ≤ 30°
MAX_H_SLOPE = 0.577

# 两线最小夹角（度），低于此值认为接近平行，不算十字
MIN_CROSS_ANGLE = 45.0

# 交点距画面中心的最大允许距离
MAX_CROSS_CENTER_DIST = 120.0

# 连续帧确认 / 丢失计数
CROSS_CONFIRM_FRAMES = 3
CROSS_LOST_FRAMES = 3

# 交点低通滤波系数（越小越平滑）
FILTER_ALPHA = 0.4

# 串口
ENABLE_SERIAL = False
UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200


def clamp(value, low, high):
    return max(low, min(high, value))


def line_threshold(line_color):
    if line_color == "black":
        return BLACK_THRESHOLD
    if line_color == "red":
        return RED_THRESHOLD
    raise ValueError("line_color must be 'black' or 'red'")


def setup_uart():
    if not ENABLE_SERIAL:
        return None
    err.check_raise(
        pinmap.set_pin_function("A19", "UART1_TX"),
        "A19 UART1_TX"
    )
    err.check_raise(
        pinmap.set_pin_function("A18", "UART1_RX"),
        "A18 UART1_RX"
    )
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


# ============================================================
# 色块选择
# ============================================================

def choose_blob_v(blobs, expected_x, roi_width):
    """在竖直方向 ROI 中选择最可能是黑线的色块。

    评分：像素数越大越好，距预测位置越近越好。
    过滤：像素太少、色块太宽（可能是阴影/边框）。
    """
    best = None
    best_score = -1000000
    max_blob_width = int(roi_width * MAX_BLOB_WIDTH_RATIO)

    for blob in blobs:
        if blob.pixels() < MIN_PIXELS:
            continue
        if blob.w() > max_blob_width:
            continue

        distance = abs(blob.cx() - expected_x)
        score = blob.pixels() - distance * 0.8

        if score > best_score:
            best = blob
            best_score = score

    return best


def choose_blob_h(blobs, expected_y, roi_height):
    """在水平方向 ROI 中选择最可能是黑线的色块。

    评分：像素数越大越好，距预测位置越近越好。
    过滤：像素太少、色块太高（可能是阴影/边框）。
    """
    best = None
    best_score = -1000000
    max_blob_height = int(roi_height * MAX_BLOB_HEIGHT_RATIO)

    for blob in blobs:
        if blob.pixels() < MIN_PIXELS:
            continue
        if blob.h() > max_blob_height:
            continue

        distance = abs(blob.cy() - expected_y)
        score = blob.pixels() - distance * 0.8

        if score > best_score:
            best = blob
            best_score = score

    return best


# ============================================================
# 线段检测
# ============================================================

def detect_vertical_line(img, expected_x, threshold=BLACK_THRESHOLD):
    """在竖直方向的两个 ROI 中检测色块，返回两点确定的竖直线。

    上方 ROI → 色块中心点 P_top
    下方 ROI → 色块中心点 P_bottom
    两点连线即为竖直线。

    Returns:
        (x_top, y_top, x_bottom, y_bottom, top_blob, bottom_blob) or None
    """
    top_blobs = img.find_blobs(
        threshold,
        roi=V_ROI_TOP,
        area_threshold=MIN_AREA,
        pixels_threshold=MIN_PIXELS,
        x_stride=2,
        y_stride=1,
    )
    top_blob = choose_blob_v(top_blobs, expected_x, V_ROI_TOP[2])

    bottom_blobs = img.find_blobs(
        threshold,
        roi=V_ROI_BOTTOM,
        area_threshold=MIN_AREA,
        pixels_threshold=MIN_PIXELS,
        x_stride=2,
        y_stride=1,
    )
    bottom_blob = choose_blob_v(bottom_blobs, expected_x, V_ROI_BOTTOM[2])

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


def detect_horizontal_line(img, expected_y, threshold=BLACK_THRESHOLD):
    """在水平方向的两个 ROI 中检测色块，返回两点确定的水平线。

    左侧 ROI → 色块中心点 P_left
    右侧 ROI → 色块中心点 P_right
    两点连线即为水平线。

    Returns:
        (x_left, y_left, x_right, y_right, left_blob, right_blob) or None
    """
    left_blobs = img.find_blobs(
        threshold,
        roi=H_ROI_LEFT,
        area_threshold=MIN_AREA,
        pixels_threshold=MIN_PIXELS,
        x_stride=1,
        y_stride=2,
    )
    left_blob = choose_blob_h(left_blobs, expected_y, H_ROI_LEFT[3])

    right_blobs = img.find_blobs(
        threshold,
        roi=H_ROI_RIGHT,
        area_threshold=MIN_AREA,
        pixels_threshold=MIN_PIXELS,
        x_stride=1,
        y_stride=2,
    )
    right_blob = choose_blob_h(right_blobs, expected_y, H_ROI_RIGHT[3])

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


# ============================================================
# 交点计算
# ============================================================

def line_intersection(v_line, h_line):
    """计算竖直线和水平线的交点及夹角。

    v_line: (x1, y1, x2, y2, ...)  竖直线两点
    h_line: (x3, y3, x4, y4, ...)  水平线两点

    Returns:
        (ix, iy, angle_deg) or None
        angle_deg 为两线夹角，0~90 度。
    """
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

    # 两线方向夹角
    dot = abs(v_dx * h_dx + v_dy * h_dy)
    cos_angle = clamp(dot / (v_len * h_len), -1.0, 1.0)
    angle_deg = math.degrees(math.acos(cos_angle))
    if angle_deg > 90:
        angle_deg = 180 - angle_deg

    if angle_deg < MIN_CROSS_ANGLE:
        return None

    # 参数方程求交点：P1 + t * V = P3 + u * H
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


# ============================================================
# 可视化
# ============================================================

def draw_roi(img, roi, color, thickness=1):
    img.draw_rect(roi[0], roi[1], roi[2], roi[3], color, thickness)


def draw_blob(img, blob, center_color, rect_color):
    if blob is not None:
        img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), rect_color, 2)
        img.draw_cross(blob.cx(), blob.cy(), center_color, 8, 2)


def draw_extended_line(img, x1, y1, x2, y2, color, thickness):
    """将两点线段延长至图像边界绘制。"""
    dx = x2 - x1
    dy = y2 - y1

    if abs(dx) < 1 and abs(dy) < 1:
        return

    if abs(dx) >= abs(dy):
        # 以 x 方向延伸
        slope = dy / dx if abs(dx) > 0 else 0
        x_start = 0
        y_start = int(y1 + slope * (x_start - x1))
        x_end = IMAGE_WIDTH - 1
        y_end = int(y1 + slope * (x_end - x1))
    else:
        # 以 y 方向延伸
        slope = dx / dy if abs(dy) > 0 else 0
        y_start = 0
        x_start = int(x1 + slope * (y_start - y1))
        y_end = IMAGE_HEIGHT - 1
        x_end = int(x1 + slope * (y_end - y1))

    img.draw_line(
        clamp(x_start, 0, IMAGE_WIDTH - 1),
        clamp(y_start, 0, IMAGE_HEIGHT - 1),
        clamp(x_end, 0, IMAGE_WIDTH - 1),
        clamp(y_end, 0, IMAGE_HEIGHT - 1),
        color,
        thickness,
    )


class CrossroadDetector:
    def __init__(self, line_color=LINE_COLOR, filter_alpha=FILTER_ALPHA):
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
        v_line = detect_vertical_line(img, self.expected_x, self.threshold)
        h_line = detect_horizontal_line(img, self.expected_y, self.threshold)
        cross = None
        if v_line is not None and h_line is not None:
            cross = line_intersection(v_line, h_line)

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


def send_serial(serial_dev, result):
    if serial_dev is None:
        return
    if result["confirmed"] and result["detected"]:
        payload = struct.pack("<Bhh", 1, result["x"], result["y"])
    else:
        payload = struct.pack("<Bhh", 0, 0, 0)
    frame = bytes((0xAA, 0x55, len(payload))) + payload
    serial_dev.write(frame + bytes((sum(frame) & 0xFF,)))


def draw_result(img, result, fps=None):
    draw_roi(img, V_ROI_TOP, image.COLOR_BLUE, 1)
    draw_roi(img, V_ROI_BOTTOM, image.COLOR_BLUE, 1)
    draw_roi(img, H_ROI_LEFT, image.COLOR_GRAY, 1)
    draw_roi(img, H_ROI_RIGHT, image.COLOR_GRAY, 1)

    v_line = result["v_line"]
    if v_line is not None:
        x_top, y_top, x_bottom, y_bottom, top_blob, bottom_blob = v_line
        draw_blob(img, top_blob, image.COLOR_RED, image.COLOR_GREEN)
        draw_blob(img, bottom_blob, image.COLOR_RED, image.COLOR_GREEN)
        draw_extended_line(
            img, x_top, y_top, x_bottom, y_bottom,
            image.COLOR_GREEN, 3
        )

    h_line = result["h_line"]
    if h_line is not None:
        x_left, y_left, x_right, y_right, left_blob, right_blob = h_line
        draw_blob(img, left_blob, image.COLOR_RED, image.COLOR_YELLOW)
        draw_blob(img, right_blob, image.COLOR_RED, image.COLOR_YELLOW)
        draw_extended_line(
            img, x_left, y_left, x_right, y_right,
            image.COLOR_YELLOW, 3
        )

    cross = result["cross"]
    if cross is not None:
        x, y, angle = cross
        cross_color = image.COLOR_RED if result["confirmed"] else image.COLOR_WHITE
        img.draw_cross(x, y, cross_color, 16, 3)
        img.draw_circle(x, y, 20, cross_color, 2)
        status = "CROSS OK" if result["confirmed"] else "CROSS CHECK"
        img.draw_string(
            0, 0,
            "{} {}deg".format(status, int(angle)),
            image.COLOR_RED, 1.2
        )
        img.draw_string(
            0, 18,
            "({},{})".format(result["x"], result["y"]),
            image.COLOR_RED, 1.0
        )
    else:
        v_status = "V:OK" if v_line is not None else "V:---"
        h_status = "H:OK" if h_line is not None else "H:---"
        img.draw_string(
            0, 0,
            "{}  {}  conf:{}".format(
                v_status, h_status, result["cross_count"]
            ),
            image.COLOR_WHITE, 1.0
        )

    if fps is not None:
        img.draw_string(
            IMAGE_WIDTH - 60, 0,
            "FPS:{:.0f}".format(fps),
            image.COLOR_WHITE, 1.0
        )


def main():
    cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
    disp = display.Display()
    uart_dev = setup_uart()
    detector = CrossroadDetector(LINE_COLOR)

    print(
        "Crossroad detect V2 | color={} | blob-based | {}x{}".format(
            LINE_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT
        )
    )

    while not app.need_exit():
        img = cam.read()
        result = detector.process(img)
        draw_result(img, result, time.fps())
        send_serial(uart_dev, result)
        disp.show(img)


if __name__ == "__main__":
    main()

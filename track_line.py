import math
import struct

from maix import app, camera, display, err, image, pinmap, uart

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_CENTER_X = IMAGE_WIDTH // 2
TARGET_Y = IMAGE_HEIGHT - 20
LINE_COLOR = "black"

BLACK_THRESHOLD = [[0, 30, -128, 127, -128, 127]]
RED_THRESHOLD = [[0, 80, 40, 80, 0, 80]]

ROIS = [
    [80, 105, 160, 35],
    [60, 155, 200, 35],
]

MIN_PIXELS = 25
MIN_AREA = 25
MAX_BLOB_WIDTH_RATIO = 0.70
FILTER_ALPHA = 0.35

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
    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"), "A19 UART1_TX")
    err.check_raise(pinmap.set_pin_function("A18", "UART1_RX"), "A18 UART1_RX")
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def choose_blob(blobs, expected_x, roi_width):
    best = None
    best_score = -1000000
    max_blob_width = int(roi_width * MAX_BLOB_WIDTH_RATIO)

    for blob in blobs:
        if blob.pixels() < MIN_PIXELS or blob.w() > max_blob_width:
            continue
        score = blob.pixels() - abs(blob.cx() - expected_x) * 0.8
        if score > best_score:
            best = blob
            best_score = score
    return best


def detect_line(img, expected_x, threshold):
    points = []
    for roi in ROIS:
        blobs = img.find_blobs(
            threshold,
            roi=roi,
            area_threshold=MIN_AREA,
            pixels_threshold=MIN_PIXELS,
            x_stride=2,
            y_stride=1,
        )
        blob = choose_blob(blobs, expected_x, roi[2])
        if blob is not None:
            points.append((blob.cx(), roi[1] + roi[3] // 2, blob))
    return points


def fit_line(points):
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
    x_top = int(clamp(intercept + slope * line_top_y, 0, IMAGE_WIDTH - 1))
    x_bottom = int(clamp(intercept + slope * TARGET_Y, 0, IMAGE_WIDTH - 1))
    angle = math.degrees(math.atan(slope))
    return x_bottom, angle, x_top, line_top_y


class LineTracker:
    def __init__(self, line_color=LINE_COLOR, filter_alpha=FILTER_ALPHA):
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
        points = detect_line(img, self.last_x_bottom, self.threshold)
        fitted = fit_line(points)
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


def draw_result(img, result):
    for roi in ROIS:
        img.draw_rect(roi[0], roi[1], roi[2], roi[3], image.COLOR_BLUE, 1)
    for x, y, blob in result["points"]:
        img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), image.COLOR_GREEN, 2)
        img.draw_cross(x, y, image.COLOR_RED, 8, 2)

    if result["valid"]:
        img.draw_line(
            result["x_top"],
            result["line_top_y"],
            result["x_bottom"],
            TARGET_Y,
            image.COLOR_GREEN,
            3,
        )
        img.draw_line(
            IMAGE_CENTER_X,
            TARGET_Y,
            result["x_bottom"],
            TARGET_Y,
            image.COLOR_RED,
            2,
        )
        label = "OK E:{:.1f} A:{:.1f}".format(result["error"], result["angle"])
    else:
        label = "LOST"
    img.draw_string(0, 0, label, image.COLOR_RED)


def send_legacy_line_packet(serial_dev, result):
    if serial_dev is None:
        return
    payload = struct.pack(
        "<Bhhh",
        1 if result["valid"] else 0,
        int(clamp(result["error"] * 10, -32768, 32767)),
        int(clamp(result["angle"] * 10, -32768, 32767)),
        int(clamp(result["x_bottom"], -32768, 32767)),
    )
    frame = bytes((0xAA, 0x55, len(payload))) + payload
    serial_dev.write(frame + bytes((sum(frame) & 0xFF,)))


def main():
    tracker = LineTracker(LINE_COLOR)
    serial_dev = setup_uart()
    cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
    disp = display.Display()

    while not app.need_exit():
        img = cam.read()
        result = tracker.process(img)
        draw_result(img, result)
        send_legacy_line_packet(serial_dev, result)
        disp.show(img)


if __name__ == "__main__":
    main()

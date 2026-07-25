from maix import app, camera, display, err, image, pinmap, time, touchscreen, uart

from delivery_logic import (
    CAPTURE_TARGET,
    FOLLOW_LINE,
    LEFT,
    RIGHT,
    UNKNOWN,
    WAIT_START,
    WAIT_TURN_DONE,
    DeliveryStateMachine,
    DirectionVoter,
    TargetLocker,
    fixed_route_direction,
)
from track_line import (
    IMAGE_CENTER_X,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    ROIS,
    TARGET_Y,
    LineTracker,
)
from vision_protocol import (
    FrameParser,
    MessageType,
    encode_line_data,
    encode_status,
    encode_target_locked,
    encode_turn_decision,
)
from yolov8_num_detect import DigitDetector, draw_detections

LINE_COLOR = "black"
CAM_FPS = 30
YOLO_EVERY_N_FRAMES = 2
DISPLAY_EVERY_N_FRAMES = 2
LINE_PACKET_EVERY_N_FRAMES = 2

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

UART_DEVICE = "/dev/ttyS0"
UART_BAUDRATE = 115200
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
    WAIT_TURN_DONE: 5,
}


def setup_uart():
    err.check_raise(pinmap.set_pin_function("A16", "UART0_TX"), "A16 UART0_TX")
    err.check_raise(pinmap.set_pin_function("A17", "UART0_RX"), "A17 UART0_RX")
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def prepare_vision_image(img):
    rgb = img
    if img.format() != image.Format.FMT_RGB888:
        rgb = img.to_format(image.Format.FMT_RGB888)
    if rgb.width() != IMAGE_WIDTH or rgb.height() != IMAGE_HEIGHT:
        rgb = rgb.resize(IMAGE_WIDTH, IMAGE_HEIGHT)
    return rgb


def direction_confidence(voter, direction):
    scores, _ = voter.scores()
    total = scores[LEFT] + scores[RIGHT]
    if total <= 0 or direction not in (LEFT, RIGHT):
        return 0
    return round(scores[direction] * 100 / total)


def draw_target_roi(img):
    x_min, y_min, x_max, y_max = TARGET_ROI
    x = int(x_min * img.width())
    y = int(y_min * img.height())
    width = int((x_max - x_min) * img.width())
    height = int((y_max - y_min) * img.height())
    img.draw_rect(x, y, width, height, image.COLOR_BLUE, 2)


def draw_status(img, machine, line_result, voter, last_event, last_tx):
    scores, counts = voter.scores()
    target_text = "-" if machine.target_number is None else str(machine.target_number)
    line_text = "OK" if line_result.get("valid") else "LOST"
    lines = [
        "S:{} T:{} DIR:{}".format(machine.state, target_text, machine.last_direction),
        "LINE:{}".format(line_text),
        "V L:{}/{:.1f} R:{}/{:.1f}".format(
            counts[LEFT],
            scores[LEFT],
            counts[RIGHT],
            scores[RIGHT],
        ),
        "EVT:{} TX:{}".format(last_event, last_tx),
    ]
    for index, text in enumerate(lines):
        img.draw_string(2, 2 + index * 16, text, image.COLOR_WHITE)


def _draw_line_tracking(img, line_result):
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


class DeliveryController:
    def __init__(self):
        self.digit_detector = DigitDetector(dual_buff=False)
        self.line_tracker = LineTracker(LINE_COLOR)
        self.target_locker = TargetLocker(roi=TARGET_ROI)
        self.direction_voter = DirectionVoter()
        self.machine = DeliveryStateMachine()
        self.parser = FrameParser()
        self.uart_dev = setup_uart()
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
        self.last_uart_event = "NONE"
        self.last_tx = "-"
        self.last_detections = []
        self.last_line_result = {"valid": False}
        self._btn_rects_disp = {}
        self._btn_rects_computed = False

    def reset(self):
        self.machine.reset()
        self.target_locker.reset()
        self.direction_voter.clear()
        self.line_tracker.reset()
        self.parser.clear()
        self.intersection_index = 0
        self.last_detections = []
        self.last_line_result = {"valid": False}

    def send(self, packet):
        self.uart_dev.write(packet)
        if len(packet) >= 4:
            msg_type = packet[3]
            self.last_tx = _TX_TYPE_NAMES.get(msg_type, "??")
        else:
            self.last_tx = "?"

    def send_state(self):
        self.send(encode_status(STATE_CODES[self.machine.state]))

    def receive_uart(self):
        data = self.uart_dev.read()
        for message_type, payload in self.parser.feed(data):
            if message_type == MessageType.RESET:
                self.last_uart_event = "RESET"
                self.reset()
                self.send_state()
            elif message_type == MessageType.STOP:
                self.last_uart_event = "STOP"
                self.machine.stop()
                self.direction_voter.clear()
                self.send_state()
            elif message_type == MessageType.START:
                self.last_uart_event = "START"
                if self.machine.start():
                    self.direction_voter.clear()
                    self.line_tracker.reset()
                    self.send_state()
            elif message_type == MessageType.TURN_DONE:
                self.last_uart_event = "TURN_DONE"
                if self.machine.turn_done():
                    self.direction_voter.clear()
                    self.send_state()

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
            self.last_uart_event = "RESET"
            self.reset()
            self.send_state()
        elif label == "STOP":
            self.last_uart_event = "STOP"
            self.machine.stop()
            self.direction_voter.clear()
            self.send_state()
        elif label == "START":
            self.last_uart_event = "START"
            if self.machine.start():
                self.direction_voter.clear()
                self.line_tracker.reset()
                self.send_state()
        elif label == "TURN_DONE":
            self.last_uart_event = "TURN_DONE"
            if self.machine.turn_done():
                self.direction_voter.clear()
                self.send_state()

    def update_target(self, detections):
        target = self.target_locker.update(detections)
        if target is not None and self.machine.lock_target(target):
            self.last_detections = []
            self.send(encode_target_locked(target))
            self.last_uart_event = "TARGET_LOCKED"
            if target in FIXED_ROUTES:
                direction = fixed_route_direction(target, 0, FIXED_ROUTES)
                if direction in (LEFT, RIGHT):
                    self.machine.last_direction = direction
                    self.send(encode_turn_decision(
                        direction,
                        target_number=target,
                        intersection_index=0,
                        confidence=100,
                    ))
                    self.last_uart_event = "TARGET_LOCKED+DIR"
            self.send_state()

    def _try_direction_decision(self, detections, line_result):
        """For targets 3-8: feed voter and send TURN_DECISION when ready."""
        if self.machine.target_number in (None, 1, 2):
            return

        center = (
            line_result.get("decision_center_x_norm", DEFAULT_CENTER_X_NORM)
            if line_result.get("valid")
            else DEFAULT_CENTER_X_NORM
        )
        self.direction_voter.add(detections, self.machine.target_number, center)
        direction = self.direction_voter.decision()
        if direction in (LEFT, RIGHT) and self.machine.lock_direction(direction):
            self.last_detections = []
            confidence = direction_confidence(self.direction_voter, direction)
            self.send(encode_turn_decision(
                direction,
                target_number=self.machine.target_number,
                intersection_index=self.intersection_index,
                confidence=confidence,
            ))
            self.intersection_index += 1
            self.send_state()

    def process_frame(self):
        self.receive_uart()
        model_img = self.cam.read()
        vision_img = prepare_vision_image(model_img)
        self.frame_index += 1

        state = self.machine.state

        run_yolo = False
        if state == CAPTURE_TARGET:
            run_yolo = True
        elif state == FOLLOW_LINE and self.machine.target_number not in (1, 2):
            run_yolo = self.frame_index % YOLO_EVERY_N_FRAMES == 0

        if run_yolo:
            self.last_detections = self.digit_detector.detect(model_img)

        line_result = {"valid": False}

        if state == CAPTURE_TARGET:
            self.update_target(self.last_detections)
        elif state == WAIT_START:
            pass
        else:
            # FOLLOW_LINE or WAIT_TURN_DONE
            line_result = self.line_tracker.process(vision_img)
            self.last_line_result = line_result

            if (state == FOLLOW_LINE
                    and self.frame_index % LINE_PACKET_EVERY_N_FRAMES == 0):
                self.send(encode_line_data(
                    line_result["valid"],
                    line_result.get("error", 0.0),
                    line_result.get("angle", 0.0),
                    line_result.get("center_x_norm", DEFAULT_CENTER_X_NORM),
                ))

            if state == FOLLOW_LINE and self.machine.target_number not in (1, 2) and run_yolo:
                self._try_direction_decision(self.last_detections, line_result)

        if self.frame_index % DISPLAY_EVERY_N_FRAMES == 0:
            draw_detections(model_img, self.last_detections)
            if state == CAPTURE_TARGET:
                draw_target_roi(model_img)
            if state in (FOLLOW_LINE, WAIT_TURN_DONE):
                _draw_line_tracking(model_img, line_result)
            self._draw_buttons(model_img)
            draw_status(
                model_img,
                self.machine,
                line_result,
                self.direction_voter,
                self.last_uart_event,
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
    DeliveryController().run()


if __name__ == "__main__":
    main()

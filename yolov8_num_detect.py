from maix import app, camera, display, image, nn, time

MODEL_PATH = "/root/models/yolov8_num_detect_v2.mud"
CONFIDENCE_THRESHOLD = 0.4
IOU_THRESHOLD = 0.45


def parse_room_number(label):
    text = str(label).strip()
    digits = [character for character in text if character.isdigit()]
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
            number = parse_room_number(label)
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
    for detection in detections:
        img.draw_rect(
            detection["x"],
            detection["y"],
            detection["w"],
            detection["h"],
            color=image.COLOR_RED,
        )
        message = "{}: {:.2f}".format(detection["number"], detection["score"])
        img.draw_string(
            detection["x"],
            max(0, detection["y"] - 10),
            message,
            color=image.COLOR_GREEN,
        )


def main():
    digit_detector = DigitDetector(dual_buff=True)
    cam = camera.Camera(
        digit_detector.input_width(),
        digit_detector.input_height(),
        digit_detector.input_format(),
    )
    disp = display.Display()

    while not app.need_exit():
        img = cam.read()
        detections = digit_detector.detect(img)
        draw_detections(img, detections)
        img.draw_string(8, 8, "fps: {}".format(time.fps()), color=image.COLOR_WHITE)
        disp.show(img)


if __name__ == "__main__":
    main()

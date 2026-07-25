import struct
from collections import deque

import cv2
import numpy as np
from maix import app, camera, image, display, uart, nn, tensor, pinmap, err


class Config:
    # 摄像头画面
    CAM_W = 320
    CAM_H = 240

    # MaixCAM 上运行的是转换后的 .mud，不是直接运行 .onnx。
    MODEL_PATH = "/root/models/num_detact.mud"

    # 数字所在的固定区域，必须根据实际赛道调整。
    # 当前设置为画面中央区域。
    DIGIT_ROI = (60, 20, 200, 200)  # x, y, w, h

    # ONNX 已解析为 [1, 1, 28, 28]，与 MNIST 输入一致。
    MODEL_SIZE = 28
    MNIST_DIGIT_SIZE = 20

    # 模型训练数据是黑底白字，因此前景是亮区域，不做反色。
    # 如果摄像头实际看到的是白底黑字，改为 True，把输入反色成黑底白字。
    CAMERA_BLACK_ON_WHITE = True
    FOREGROUND_THRESHOLD = 35
    MIN_FOREGROUND_AREA = 20

    # 置信度和时间稳定判断
    CONFIDENCE_THRESHOLD = 0.70
    HISTORY_SIZE = 12
    MIN_VOTE_COUNT = 7

    # 自定义通信使用 UART1，避免 UART0 的系统启动日志。
    UART_DEVICE = "/dev/ttyS1"
    UART_BAUDRATE = 115200


def setup_uart():
    """配置 MaixCAM-Pro UART1：A19=TX，A18=RX。"""
    err.check_raise(
        pinmap.set_pin_function("A19", "UART1_TX"),
        "Failed to set A19 as UART1_TX"
    )
    err.check_raise(
        pinmap.set_pin_function("A18", "UART1_RX"),
        "Failed to set A18 as UART1_RX"
    )
    return uart.UART(Config.UART_DEVICE, Config.UART_BAUDRATE)


def crop_digit_foreground(gray):
    """从黑底白字图像中提取最大亮色前景。"""
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    _, mask = cv2.threshold(
        blurred,
        Config.FOREGROUND_THRESHOLD,
        255,
        cv2.THRESH_BINARY
    )

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8
    )

    if count <= 1:
        return gray, None

    # 忽略背景，选择面积最大的前景连通区域。
    best_label = 1
    best_area = stats[1, cv2.CC_STAT_AREA]
    for label in range(2, count):
        area = stats[label, cv2.CC_STAT_AREA]
        if area > best_area:
            best_label = label
            best_area = area

    if best_area < Config.MIN_FOREGROUND_AREA:
        return gray, None

    x = stats[best_label, cv2.CC_STAT_LEFT]
    y = stats[best_label, cv2.CC_STAT_TOP]
    w = stats[best_label, cv2.CC_STAT_WIDTH]
    h = stats[best_label, cv2.CC_STAT_HEIGHT]

    # 保留原始灰度值，只用 mask 去掉背景，尽量保持 MNIST 的灰度边缘。
    foreground = np.zeros_like(gray)
    foreground[labels == best_label] = gray[labels == best_label]
    return foreground[y:y + h, x:x + w], (x, y, w, h)


def resize_like_mnist(gray):
    """将数字按 MNIST 风格缩放到 20×20 范围并放入 28×28 黑底。"""
    if gray is None or gray.size == 0:
        canvas = np.zeros((Config.MODEL_SIZE, Config.MODEL_SIZE), dtype=np.float32)
        return canvas.reshape(1, 1, Config.MODEL_SIZE, Config.MODEL_SIZE)

    h, w = gray.shape[:2]
    target = Config.MNIST_DIGIT_SIZE
    scale = min(target / max(w, 1), target / max(h, 1))
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))

    resized = cv2.resize(gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((Config.MODEL_SIZE, Config.MODEL_SIZE), dtype=np.uint8)

    x = (Config.MODEL_SIZE - new_w) // 2
    y = (Config.MODEL_SIZE - new_h) // 2
    canvas[y:y + new_h, x:x + new_w] = resized

    # ONNX 图中没有 mean/scale 元信息，模型训练时使用的是 ToTensor()，
    # 因此这里只做 [0, 255] -> [0, 1]，不做额外标准化。
    data = canvas.astype(np.float32) / 255.0
    return data.reshape(1, 1, Config.MODEL_SIZE, Config.MODEL_SIZE)


def preprocess_digit(frame_rgb):
    """裁剪固定 ROI，并返回模型输入和数字框。"""
    x, y, w, h = Config.DIGIT_ROI
    roi = frame_rgb[y:y + h, x:x + w]
    gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
    if Config.CAMERA_BLACK_ON_WHITE:
        gray = 255 - gray
    digit, bbox = crop_digit_foreground(gray)
    return resize_like_mnist(digit), bbox


def softmax(values):
    values = values.astype(np.float32)
    values = values - np.max(values)
    exp_values = np.exp(values)
    return exp_values / np.sum(exp_values)


def check_model(model):
    """检查转换后的 .mud 是否仍与 num_detact.onnx 的结构一致。"""
    inputs = model.inputs_info()
    if len(inputs) != 1:
        raise RuntimeError("数字模型必须只有一个输入")

    info = inputs[0]
    shape = [int(v) for v in info.shape]
    print("model input:", info.name, shape, info.dtype)

    # 允许 batch 维是动态值，但通道和空间尺寸必须正确。
    if len(shape) != 4 or shape[1:] != [1, 28, 28]:
        raise RuntimeError(
            "模型输入不匹配，期望 [N, 1, 28, 28]，实际为 {}".format(shape)
        )

    print("model extra_info:", model.extra_info())


def predict_digit(model, model_input):
    """返回 (数字, 置信度, 概率数组)。"""
    input_tensors = tensor.Tensors()
    input_info = model.inputs_info()[0]
    input_tensor = tensor.tensor_from_numpy_float32(model_input, copy=False)
    input_tensors.add_tensor(
        input_info.name,
        input_tensor,
        False,
        False
    )

    outputs = model.forward(input_tensors, copy_result=False)
    output_tensor = outputs[list(outputs.keys())[0]]
    logits = tensor.tensor_to_numpy_float32(output_tensor, copy=False).flatten()
    probabilities = softmax(logits)

    digit = int(np.argmax(probabilities))
    confidence = float(probabilities[digit])
    return digit, confidence, probabilities


def update_stable_digit(history):
    """在最近若干帧中投票，返回 (数字, 票数) 或 (None, 0)。"""
    valid_digits = [digit for digit in history if digit >= 0]
    if not valid_digits:
        return None, 0

    counts = {}
    for digit in valid_digits:
        counts[digit] = counts.get(digit, 0) + 1

    digit = max(counts, key=counts.get)
    votes = counts[digit]
    if votes >= Config.MIN_VOTE_COUNT:
        return digit, votes
    return None, votes


def send_digit_packet(serial_dev, valid, digit, confidence):
    """发送数字数据帧：AA 55 LEN VALID DIGIT CONF CHECKSUM。"""
    payload = struct.pack(
        "<BBB",
        1 if valid else 0,
        int(digit) if valid else 0xFF,
        int(max(0, min(100, confidence * 100)))
    )
    frame = bytes((0xAA, 0x55, len(payload))) + payload
    checksum = sum(frame) & 0xFF
    serial_dev.write(frame + bytes((checksum,)))


def main():
    cam = camera.Camera(Config.CAM_W, Config.CAM_H)
    disp = display.Display()
    serial_dev = setup_uart()
    model = nn.NN(Config.MODEL_PATH, dual_buff=False)
    check_model(model)

    history = deque(maxlen=Config.HISTORY_SIZE)
    stable_digit = None
    last_confidence = 0.0

    while not app.need_exit():
        img = cam.read()
        frame_rgb = image.image2cv(img, ensure_bgr=False, copy=True)
        model_input, bbox = preprocess_digit(frame_rgb)
        digit, confidence, _ = predict_digit(model, model_input)

        if confidence >= Config.CONFIDENCE_THRESHOLD:
            history.append(digit)
        else:
            history.append(-1)

        candidate, votes = update_stable_digit(history)
        if candidate is not None:
            stable_digit = candidate
            last_confidence = confidence

        x, y, w, h = Config.DIGIT_ROI
        img.draw_rect(x, y, w, h, image.COLOR_BLUE, 2)

        if bbox is not None:
            bx, by, bw, bh = bbox
            img.draw_rect(x + bx, y + by, bw, bh, image.COLOR_GREEN, 2)

        if stable_digit is None:
            label = "raw:{} {:.2f} V:{}/{}".format(
                digit,
                confidence,
                votes,
                Config.MIN_VOTE_COUNT
            )
            valid = False
            output_digit = 0xFF
        else:
            label = "digit:{} conf:{:.2f}".format(
                stable_digit,
                last_confidence
            )
            valid = True
            output_digit = stable_digit

        img.draw_string(5, 5, label, image.COLOR_RED, scale=2)
        disp.show(img)
        send_digit_packet(serial_dev, valid, output_digit, confidence)


if __name__ == "__main__":
    main()

# MaixCAM 电赛常用 API 与实战指南

> 基于项目内 MaixPy v4.12.5 源码、中文文档、测试和官方示例整理。
>
> 适用设备：MaixCAM、MaixCAM-Pro，并标注部分 MaixCAM2 差异。

## 目录

1. [版本、接口风格与使用原则](#1-版本接口风格与使用原则)
2. [最小摄像头与屏幕程序](#2-最小摄像头与屏幕程序)
3. [LAB 阈值与 ROI](#3-lab-阈值与-roi)
4. [巡线核心：get_regression](#4-巡线核心get_regression)
5. [传统视觉常用函数](#5-传统视觉常用函数)
6. [图像处理与绘图](#6-图像处理与绘图)
7. [摄像头配置](#7-摄像头配置)
8. [屏幕配置](#8-屏幕配置)
9. [触摸屏与触摸调参](#9-触摸屏与触摸调参)
10. [完整 LAB 触摸调参程序](#10-完整-lab-触摸调参程序)
11. [UART 与 STM32 通信](#11-uart-与-stm32-通信)
12. [GPIO、PWM、I2C、SPI、ADC、按键和看门狗](#12-gpiopwmi2cspiadc按键和看门狗)
13. [PID 与舵机跟踪](#13-pid-与舵机跟踪)
14. [AI 分类和目标检测](#14-ai-分类和目标检测)
15. [OpenCV、网络与音频](#15-opencv网络与音频)
16. [系统配置、参数保存与开机自启](#16-系统配置参数保存与开机自启)
17. [性能优化与比赛现场检查表](#17-性能优化与比赛现场检查表)
18. [源码和示例索引](#18-源码和示例索引)

---

## 1. 版本、接口风格与使用原则

### 1.1 本手册对应版本

项目内 `maix/version.py` 显示：

```text
MaixPy 4.12.5
```

不同固件版本的枚举、模型支持和少量参数可能变化。比赛前应让开发板固件、MaixVision 和本地参考源码版本保持一致。

### 1.2 现代 MaixPy v4 与 maix.v1

现代 MaixPy v4 推荐写法：

```python
from maix import camera, display, image, app
```

OpenMV 风格兼容写法：

```python
from maix.v1 import lcd, sensor
```

本手册默认使用现代 MaixPy v4 API。`maix.v1` 是兼容层，不是另一套完整实现。旧 OpenMV 教程中的下列函数在当前兼容层会抛出 `ValueError('This operation is not supported')`：

- `cartoon`
- `remove_shadows`
- `chrominvar`
- `illuminvar`
- `get_similarity`
- `find_number`
- `classify_object`
- `find_features`
- `find_eye`
- `find_lbp`
- `find_keypoints`

不要仅凭旧 OpenMV 示例判断 MaixPy v4 是否支持某项功能，应优先查当前版本文档、测试和现代示例。

### 1.3 电赛程序的基本结构

推荐用 `app.need_exit()` 管理退出：

```python
from maix import camera, display, app

cam = camera.Camera(320, 240)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    disp.show(img)
```

相较于 `while True`，这种写法能响应 MaixVision 或系统发出的退出请求，更适合作为正式应用。

### 1.4 常见分辨率选择

| 分辨率 | 典型用途 | 特点 |
|---|---|---|
| `320x240` | 巡线、色块、圆、矩形 | 传统视觉常用，速度和精度平衡较好 |
| `320x224` | NPU 检测、二维码 NPU 检测 | 宽高适合许多模型和硬件约束 |
| `224x224` | 图像分类 | 常见分类模型输入 |
| `640x480` | 小目标、精细定位、拍照 | 清晰，但 CPU、内存和显示开销更大 |

优先使用偶数宽高。AI 模型输入必须以模型返回的尺寸和格式为准。

---

## 2. 最小摄像头与屏幕程序

### 2.1 推荐模板

```python
from maix import camera, display, image, app, time

cam = camera.Camera(
    320,
    240,
    image.Format.FMT_RGB888,
    fps=30,
    buff_num=1
)
disp = display.Display()
cam.skip_frames(30)

while not app.need_exit():
    img = cam.read()
    img.draw_string(4, 4, "MaixCAM", image.COLOR_GREEN)
    disp.show(img, fit=image.Fit.FIT_CONTAIN)
    time.sleep_ms(1)
```

关键点：

- `buff_num=1` 偏向低延迟，但可能降低吞吐或造成丢帧。
- `cam.skip_frames(30)` 让自动曝光和白平衡先稳定。
- 屏幕实际尺寸用 `disp.width()`、`disp.height()` 获取，不要依赖旧示例中的固定数值。
- `FIT_CONTAIN` 保持原图比例并完整显示，可能出现黑边。
- 循环中的 `time.sleep_ms(1)` 能给系统其他任务留出调度机会。

### 2.2 图像适配方式

| 方式 | 行为 | 适用场景 |
|---|---|---|
| `image.Fit.FIT_CONTAIN` | 等比例完整显示，可能留黑边 | 视觉调试、触摸坐标需精确映射 |
| `image.Fit.FIT_COVER` | 等比例铺满，边缘可能被裁剪 | 全屏预览 |
| `image.Fit.FIT_FILL` | 拉伸铺满，可能变形 | 不关心几何比例的 UI |
| `image.Fit.FIT_NONE` | 不自动缩放 | 图像与屏幕尺寸一致 |

几何测量和触摸交互建议使用 `FIT_CONTAIN`，并对触摸坐标做反向映射。

---

## 3. LAB 阈值与 ROI

### 3.1 LAB 阈值格式

RGB 图像的颜色阈值格式为：

```python
thresholds = [
    [L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX]
]
```

各通道意义：

- `L`：亮度，常用范围 `0` 到 `100`。
- `A`：绿色到红色，常用范围 `-128` 到 `127`。
- `B`：蓝色到黄色，常用范围 `-128` 到 `127`。

示例：

```python
thresholds = [[0, 80, -120, -10, 0, 30]]
```

灰度图一般只关心亮度阈值：

```python
thresholds = [[0, 80]]
```

阈值必须在比赛现场光照下重新标定。颜色识别对自动曝光、自动白平衡、灯光频闪和反光非常敏感。

### 3.2 ROI 格式

ROI 格式统一为：

```python
roi = [x, y, width, height]
```

例如只处理图像下半部分：

```python
roi = [0, 120, 320, 120]
```

ROI 的主要作用：

- 减少运算量，提高帧率。
- 排除场地上与任务无关的区域。
- 让巡线算法优先关注车体前方。
- 避免屏幕 UI 区域或固定反光区域进入识别。

### 3.3 阈值调试建议

1. 先固定摄像头曝光和白平衡。
2. 使用较小分辨率和明确 ROI。
3. 先放宽阈值找到目标，再逐步收窄。
4. 用 `binary()` 或 `find_blobs()` 检查阈值是否正确。
5. 同时调整 `pixels_threshold` 和 `area_threshold` 去除噪点。
6. 在实际赛场照明、实际高度和实际镜头姿态下保存最终参数。

---

## 4. 巡线核心：get_regression

### 4.1 作用

`get_regression()` 对符合 LAB 阈值的像素做线性回归，常用于：

- 黑线或彩色线巡线。
- 赛道边线方向估计。
- 激光线、胶带线或杆件角度测量。
- 计算直线相对图像中心的位置误差和角度误差。

现代接口常用形式：

```python
lines = img.get_regression(
    thresholds,
    invert=False,
    roi=[],
    x_stride=2,
    y_stride=1,
    area_threshold=10,
    pixels_threshold=10,
    robust=False
)
```

返回值可迭代，每个 `Line` 对象常用方法：

- `x1()`、`y1()`：起点。
- `x2()`、`y2()`：终点。
- `theta()`：法线角度。
- `rho()`：原点到直线的距离。

### 4.2 最小可运行示例

```python
from maix import camera, display, image, app

cam = camera.Camera(320, 240)
disp = display.Display()
thresholds = [[0, 80, -120, -10, 0, 30]]
roi = [0, 80, 320, 160]

while not app.need_exit():
    img = cam.read()
    lines = img.get_regression(
        thresholds,
        roi=roi,
        area_threshold=100,
        pixels_threshold=100
    )

    img.draw_rect(roi[0], roi[1], roi[2], roi[3], image.COLOR_BLUE, 1)

    for line in lines:
        img.draw_line(
            line.x1(),
            line.y1(),
            line.x2(),
            line.y2(),
            image.COLOR_GREEN,
            2
        )

        theta = line.theta()
        steering_angle = 270 - theta if theta > 90 else 90 - theta
        text = f"angle:{steering_angle:.1f} rho:{line.rho():.1f}"
        img.draw_string(4, 4, text, image.COLOR_GREEN)

    disp.show(img)
```

### 4.3 用直线计算横向误差

比单独使用 `rho()` 更直观的方法，是计算直线在图像底边或指定前视行上的交点。

设线段端点为 `(x1, y1)` 和 `(x2, y2)`，求它在 `target_y` 处的横坐标：

```python
def line_x_at_y(line, target_y):
    x1 = line.x1()
    y1 = line.y1()
    x2 = line.x2()
    y2 = line.y2()

    if y2 == y1:
        return (x1 + x2) // 2

    ratio = (target_y - y1) / (y2 - y1)
    return int(x1 + ratio * (x2 - x1))
```

在主循环中使用：

```python
target_y = img.height() - 1
line_x = line_x_at_y(line, target_y)
center_x = img.width() // 2
position_error = line_x - center_x

img.draw_cross(line_x, target_y, image.COLOR_RED, 8, 2)
img.draw_line(center_x, 0, center_x, img.height(), image.COLOR_BLUE, 1)
```

约定 `position_error > 0` 表示目标在画面右侧。发送给底盘前，应结合摄像头是否镜像、安装方向和电机方向验证符号。

### 4.4 参数如何调

| 参数 | 增大后的典型影响 | 调参建议 |
|---|---|---|
| `x_stride` | 采样更稀疏，速度提高，精度下降 | 从 `2` 开始 |
| `y_stride` | 纵向采样更稀疏 | 从 `1` 开始 |
| `area_threshold` | 过滤更小的连通区域 | 杂点多时增加 |
| `pixels_threshold` | 要求更多有效像素 | 误检多时增加 |
| `robust` | 对离群点更稳健，但可能更耗时 | 反光和局部遮挡明显时测试 |
| `invert` | 选择阈值外像素 | 识别背景而不是目标时使用 |

### 4.5 多 ROI 加权巡线

车辆巡线常把画面分为远、中、近三个 ROI，分别计算色块中心或回归线位置，再加权：

```python
ROIS = [
    (0, 40, 320, 45, 0.15),
    (0, 100, 320, 50, 0.30),
    (0, 170, 320, 70, 0.55)
]
```

近处权重大，转向反应快；远处权重大，能提前预判弯道。具体权重必须结合车速、摄像头俯角和控制周期测试。

---

## 5. 传统视觉常用函数

## 5.1 find_blobs：色块和连通域

基础调用：

```python
blobs = img.find_blobs(
    thresholds,
    invert=False,
    roi=[],
    x_stride=2,
    y_stride=1,
    area_threshold=10,
    pixels_threshold=10,
    merge=False,
    margin=0,
    x_hist_bins_max=0,
    y_hist_bins_max=0
)
```

示例：选择最大色块并画中心：

```python
from maix import camera, display, image, app

cam = camera.Camera(320, 240)
disp = display.Display()
thresholds = [[0, 80, 40, 80, 10, 80]]

while not app.need_exit():
    img = cam.read()
    blobs = img.find_blobs(
        thresholds,
        pixels_threshold=200,
        area_threshold=200
    )

    if blobs:
        target = max(blobs, key=lambda blob: blob.area())
        img.draw_rect(
            target.x(),
            target.y(),
            target.w(),
            target.h(),
            image.COLOR_GREEN,
            2
        )
        img.draw_cross(target.cx(), target.cy(), image.COLOR_RED, 8, 2)
        error_x = target.cx() - img.width() // 2
        img.draw_string(4, 4, f"error_x:{error_x}", image.COLOR_GREEN)

    disp.show(img)
```

`Blob` 常用字段：

| 方法 | 含义 |
|---|---|
| `x()`、`y()`、`w()`、`h()` | 外接矩形 |
| `cx()`、`cy()` | 中心坐标 |
| `rect()` | `[x, y, w, h]` |
| `corners()` | 角点 |
| `area()` | 区域面积 |
| `rotation_deg()` | 旋转角度，单位为度 |
| `pixels()` | 有效像素数，具体可用性以当前绑定为准 |
| `code()` | 命中的阈值编码 |
| `count()` | 合并的色块数量 |
| `perimeter()` | 周长 |
| `roundness()` | 圆度 |
| `elongation()` | 延伸程度 |
| `density()`、`extent()` | 填充程度相关指标 |
| `compactness()`、`solidity()`、`convexity()` | 形状紧致和凸性指标 |
| `major_axis_line()` | 主轴线 |
| `minor_axis_line()` | 次轴线 |
| `enclosing_circle()` | 外接圆 |
| `enclosed_ellipse()` | 拟合椭圆 |
| `x_hist_bins()`、`y_hist_bins()` | 横纵投影直方图 |

`merge=True` 会合并距离接近的色块，`margin` 控制允许的间隔。目标相互靠近时，合并可能把两个独立物体当成一个，应按题目决定。

`maix.v1` 兼容层没有继续传递 `x_hist_bins_max` 和 `y_hist_bins_max`，需要投影直方图时优先使用现代 API。

## 5.2 find_lines：整幅直线

```python
lines = img.find_lines(
    roi=[],
    x_stride=2,
    y_stride=1,
    threshold=2000,
    theta_margin=25,
    rho_margin=25
)

for line in lines:
    img.draw_line(
        line.x1(),
        line.y1(),
        line.x2(),
        line.y2(),
        image.COLOR_GREEN,
        2
    )
```

适合检测明显边缘形成的直线。`threshold` 越大，保留下来的线通常越强；过大会漏检，过小会产生大量杂线。

## 5.3 find_line_segments：线段

```python
segments = img.find_line_segments(
    roi=[],
    merge_distance=10,
    max_theta_difference=15
)

for segment in segments:
    img.draw_line(
        segment.x1(),
        segment.y1(),
        segment.x2(),
        segment.y2(),
        image.COLOR_RED,
        2
    )
    img.draw_string(
        segment.x1(),
        segment.y1(),
        str(segment.length()),
        image.COLOR_RED
    )
```

适合短边、矩形边界、标志物边缘。`merge_distance` 增大后，会尝试连接相邻线段。

## 5.4 find_circles：圆检测

```python
circles = img.find_circles(
    roi=[],
    x_stride=2,
    y_stride=1,
    threshold=3000,
    x_margin=10,
    y_margin=10,
    r_margin=10,
    r_min=10,
    r_max=100,
    r_step=2
)

for circle in circles:
    img.draw_circle(
        circle.x(),
        circle.y(),
        circle.r(),
        image.COLOR_RED,
        2
    )
    img.draw_cross(circle.x(), circle.y(), image.COLOR_GREEN, 8, 2)
```

优化重点：

- 尽量限制 ROI。
- 给出合理的 `r_min` 和 `r_max`。
- 先二值化、边缘化或透视矫正，再做圆检测。
- 多个候选圆可按半径、位置、圆心距离或稳定帧数筛选。

## 5.5 find_rects：矩形检测

```python
rects = img.find_rects(roi=[], threshold=10000)

for rect in rects:
    for corner in rect.corners():
        img.draw_cross(corner[0], corner[1], image.COLOR_RED, 6, 1)
    x, y, w, h = rect.rect()
    img.draw_rect(x, y, w, h, image.COLOR_GREEN, 2)
    img.draw_string(x, y, str(rect.magnitude()), image.COLOR_GREEN)
```

常用方法：

- `corners()`：四个角点。
- `rect()`：外接矩形。
- `magnitude()`：矩形响应强度。

透视角度较大时，检测结果是四边形角点，后续可做透视变换得到正视图。

## 5.6 二维码

传统二维码解码：

```python
qrcodes = img.find_qrcodes()

for code in qrcodes:
    x, y, w, h = code.rect()
    img.draw_rect(x, y, w, h, image.COLOR_GREEN, 2)
    img.draw_string(x, y, code.payload(), image.COLOR_GREEN)
```

当前文档列出的解码器包括：

```python
image.QRCodeDecoderType.QRCODE_DECODER_TYPE_ZBAR
image.QRCodeDecoderType.QRCODE_DECODER_TYPE_QUIRC
```

NPU 加速二维码检测：

```python
from maix import camera, display, app, image

cam = camera.Camera(320, 224)
disp = display.Display()
detector = image.QRCodeDetector()

while not app.need_exit():
    img = cam.read()
    qrcodes = detector.detect(img)

    for code in qrcodes:
        img.draw_string(4, 4, "payload:" + code.payload(), image.COLOR_BLUE)
        for corner in code.corners():
            img.draw_cross(corner[0], corner[1], image.COLOR_RED, 6, 1)

    disp.show(img)
```

`QRCodeDetector` 会使用 NPU。若程序同时运行其他 NPU 模型，应评估资源冲突、显存占用和延迟。

## 5.7 条形码

```python
barcodes = img.find_barcodes()

for code in barcodes:
    x, y, w, h = code.rect()
    img.draw_rect(x, y, w, h, image.COLOR_GREEN, 2)
    img.draw_string(x, y, code.payload(), image.COLOR_GREEN)
```

条形码通常需要较宽且较低的输入区域。条纹方向、运动模糊和曝光过度会显著影响解码。

## 5.8 AprilTag

```python
from maix import camera, display, image, app

cam = camera.Camera(320, 240)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    tags = img.find_apriltags(
        families=image.ApriltagFamilies.TAG36H11
    )

    for tag in tags:
        for corner in tag.corners():
            img.draw_cross(corner[0], corner[1], image.COLOR_RED, 6, 1)

        img.draw_rect(tag.x(), tag.y(), tag.w(), tag.h(), image.COLOR_GREEN, 2)
        text = f"id:{tag.id()} z:{tag.z_translation():.2f}"
        img.draw_string(tag.x(), tag.y(), text, image.COLOR_GREEN)

    disp.show(img)
```

常用信息：

- `id()`：标签编号。
- `family()`：标签族。
- `rotation()`：旋转量。
- `x_translation()`、`y_translation()`、`z_translation()`：标定后的三轴平移估计。
- `corners()`：角点。

测距前必须标定相机参数或根据实际标签尺寸建立经验曲线。缩小图识别可提高速度，但要把角点和矩形坐标映射回原图。

## 5.9 Data Matrix

```python
datamatrices = img.find_datamatrices(
    roi=[],
    effort=200
)

for code in datamatrices:
    x, y, w, h = code.rect()
    img.draw_rect(x, y, w, h, image.COLOR_GREEN, 2)
    img.draw_string(x, y, code.payload(), image.COLOR_GREEN)
```

返回对象还可提供：

- `corners()`
- `rotation()`
- `rows()`、`columns()`
- `capacity()`
- `padding()`

`effort` 增大通常提高搜索强度，也增加耗时。

## 5.10 模板匹配

```python
from maix import camera, display, image, app

cam = camera.Camera(320, 240)
disp = display.Display()
template = image.load("/root/template.png").resize(80, 80)
roi = [0, 0, 320, 240]

while not app.need_exit():
    img = cam.read()
    rect = img.find_template(
        template,
        0.5,
        roi,
        4,
        image.TemplateMatch.SEARCH_EX
    )

    if rect:
        img.draw_rect(rect[0], rect[1], rect[2], rect[3], image.COLOR_GREEN, 2)

    disp.show(img)
```

模板匹配对缩放、旋转、光照和视角变化较敏感。目标姿态变化大时，优先考虑特征检测、颜色形状组合或 AI 检测。

---

## 6. 图像处理与绘图

### 6.1 创建、加载和保存

```python
from maix import image

blank = image.Image(320, 240, image.Format.FMT_RGB888)
loaded = image.load("/root/input.jpg")
loaded.save("/root/output.jpg")
```

常用格式：

- `image.Format.FMT_RGB888`
- `image.Format.FMT_BGR888`
- `image.Format.FMT_GRAYSCALE`
- `image.Format.FMT_YVU420SP`
- `image.Format.FMT_RGBA8888`

### 6.2 几何变换

```python
small = img.resize(160, 120)
roi_img = img.crop(40, 30, 160, 120)
rotated = img.rotate(90)
cloned = img.copy()
gray = img.to_format(image.Format.FMT_GRAYSCALE)
```

`resize()`、`crop()`、`rotate()` 和 `to_format()` 返回新图，不应假设原图被原地修改。连续创建大图会增加内存压力，应复用对象或降低分辨率。

### 6.3 二值化

```python
thresholds = [[0, 100, 20, 80, 10, 80]]
img.binary(thresholds)
```

`binary()` 会修改当前图像。若后续仍需彩色图显示或做其他算法，先复制：

```python
binary_img = img.copy()
binary_img.binary(thresholds)
```

### 6.4 边缘检测

```python
from maix import image
from maix.image import EdgeDetector

edge_img = img.copy()
edge_img.find_edges(
    EdgeDetector.EDGE_CANNY,
    threshold=[50, 100]
)
```

边缘检测适合矩形、圆和轮廓前处理。阈值过低会放大纹理和噪声，过高会丢失弱边。

### 6.5 洪泛填充

```python
filled = img.copy()
filled.flood_fill(
    50,
    50,
    0.05,
    0.05,
    image.COLOR_ORANGE
)
```

洪泛填充可用于填洞、分离背景和构建连通区域。种子点必须落在需要填充的区域内。

### 6.6 直方图与统计量

```python
hist = img.get_histogram()
l_hist = hist["L"]
a_hist = hist["A"]
b_hist = hist["B"]
statistics = img.get_statistics()
```

直方图可用于自动阈值、曝光判断和现场光照监测。统计量适合获取 ROI 的亮度或颜色均值，再构造初始 LAB 阈值。

### 6.7 镜头畸变校正

```python
corrected = img.lens_corr(strength=1.5)
```

广角镜头下的直线会弯曲，影响巡线、矩形和尺寸测量。畸变校正会增加耗时，建议先测量是否确有必要，并限制处理分辨率。

### 6.8 常用绘图函数

```python
img.draw_line(10, 10, 100, 100, image.COLOR_GREEN, 2)
img.draw_rect(20, 20, 80, 60, image.COLOR_RED, 2)
img.draw_circle(160, 120, 30, image.COLOR_BLUE, 2)
img.draw_cross(160, 120, image.COLOR_GREEN, 8, 2)
img.draw_arrow(20, 120, 120, 120, image.COLOR_RED, 2)
img.draw_string(4, 4, "target", image.COLOR_GREEN)
img.draw_image(0, 0, small)
```

每帧绘制大量字符串、半透明图层和复杂图形会降低帧率。正式比赛可只显示关键状态，或提供“调试显示”开关。

---

## 7. 摄像头配置

### 7.1 初始化与格式

```python
from maix import camera, image

cam_rgb = camera.Camera(640, 480, image.Format.FMT_RGB888)
cam_gray = camera.Camera(320, 240, image.Format.FMT_GRAYSCALE)
cam_nv21 = camera.Camera(640, 480, image.Format.FMT_YVU420SP)
```

也可先创建再调整分辨率：

```python
cam = camera.Camera()
cam.set_resolution(width=640, height=480)
```

OpenCV 程序可直接请求 BGR，减少颜色转换：

```python
cam = camera.Camera(320, 240, image.Format.FMT_BGR888)
```

### 7.2 帧率和缓存

```python
cam = camera.Camera(
    320,
    240,
    fps=60,
    buff_num=1
)
```

- `fps` 是请求帧率，实际帧率还受传感器、算法和显示耗时限制。
- `buff_num=1` 偏低延迟。
- 更多缓存有利于吞吐，但会增加内存并可能让取到的画面更旧。
- MaixCAM 框架内部仍可能存在双缓冲，不能把 `buff_num=1` 理解为完全零缓存。

低延迟闭环控制应同时测量“拍摄到控制输出”的端到端延迟，而不只看显示 FPS。

### 7.3 曝光、增益和白平衡

```python
from maix import camera

cam.exposure(1000)
cam.gain(100)
cam.awb_mode(camera.AwbMode.Manual)
cam.set_wb_gain([0.134, 0.0625, 0.0625, 0.1239])
```

调用 `exposure()` 或 `gain()` 后会进入手动曝光模式。恢复自动曝光：

```python
cam.exp_mode(camera.AeMode.Auto)
```

文档给出的手动白平衡参考值：

| 设备 | 参考值 |
|---|---|
| MaixCAM / MaixCAM-Pro | `[0.134, 0.0625, 0.0625, 0.1239]` |
| MaixCAM2 | `[0.0682, 0, 0, 0.04897]` |

这些值只能作为起点。镜头、传感器批次和光源不同都会影响最佳参数。

### 7.4 亮度、对比度和饱和度

```python
cam.luma(50)
cam.constrast(50)
cam.saturation(50)
```

注意当前 API 名为 `constrast()`，这是源码中的实际拼写，不要自行改成 `contrast()`。

### 7.5 镜像和翻转

```python
cam.hmirror(True)
cam.vflip(False)
```

修改后要重新检查：

- 横向误差正负号。
- 舵机或底盘控制方向。
- AprilTag 和二维码坐标。
- 触摸选区映射。

### 7.6 启动稳定和原始图

```python
cam.skip_frames(30)
```

需要传感器原始图时，Python 布尔值必须写为 `True`：

```python
cam = camera.Camera(raw=True)
raw_img = cam.read_raw()
```

不要写成 `raw=true`。

### 7.7 颜色识别的稳定配置流程

1. 相机和灯光预热。
2. 自动曝光运行若干帧。
3. 记录合适曝光和增益，切换手动模式。
4. 固定白平衡。
5. 在多个赛场位置重新测试 LAB 阈值。
6. 对强反光区域使用 ROI 或形态学处理。
7. 保存摄像头参数和视觉阈值，启动时自动加载。

---

## 8. 屏幕配置

### 8.1 基本显示

```python
from maix import display, image

disp = display.Display()
img = image.Image(
    disp.width(),
    disp.height(),
    image.Format.FMT_RGB888
)
img.draw_string(10, 10, "display ready", image.COLOR_GREEN)
disp.show(img)
```

### 8.2 背光

```python
disp.set_backlight(50)
```

最大背光还受 `/boot/board` 中 `disp_max_backlight` 限制。比赛现场可降低背光以减少功耗和发热，但应确保状态信息可读。

### 8.3 发送到 MaixVision

```python
display.send_to_maixvision(img)
```

当物理屏幕和 MaixVision 需要显示不同内容时，可分别调用。

### 8.4 多图层显示

```python
from maix import display, image

main_disp = display.Display()
ui_disp = main_disp.add_channel()

camera_layer = image.Image(
    main_disp.width(),
    main_disp.height(),
    image.Format.FMT_RGB888
)
ui_layer = image.Image(
    main_disp.width(),
    main_disp.height(),
    image.Format.FMT_RGBA8888
)

main_disp.show(camera_layer)
ui_disp.show(ui_layer)
```

多图层适合：

- 摄像头层保持高帧率。
- UI 层低频更新参数、按钮和状态。
- 使用 RGBA 图像绘制透明叠加层。

若使用 `cam.read(block=False)`，返回值可能为 `None`：

```python
img = cam.read(block=False)
if img is not None:
    main_disp.show(img)
```

线程更新 UI 时应避免多个线程同时修改同一个图像对象。

### 8.5 外接屏幕的 pannel 配置

系统根据 `/boot/board` 中的 `pannel` 选择屏幕驱动。当前中文文档列出的常见值：

| 屏幕 | `pannel` 值 |
|---|---|
| MaixCAM 2.3 英寸 | `st7701_hd228001c31` |
| MaixCAM-Pro 2.4 英寸 | `st7701_lct024bsi20` |
| 5 英寸 | `st7701_dxq5d0019_V0` |
| 7 英寸 | `mtd700920b` |
| HDMI 1280x720 60 Hz | `lt9611_1280x720_60hz` |
| HDMI 1024x768 60 Hz | `lt9611_1024x768_60hz` |
| HDMI 640x480 60 Hz | `lt9611_640x480_60hz` |
| HDMI 552x368 60 Hz | `lt9611_552x368_60hz` |

> **硬件警告：** 屏幕型号、时序或电压配置错误可能造成残影，严重时可能损坏屏幕。修改 `/boot/board` 前必须确认硬件型号并备份原配置。修改后需重启设备。

代码中不要硬编码内置屏幕分辨率，始终使用：

```python
screen_width = disp.width()
screen_height = disp.height()
```

---

## 9. 触摸屏与触摸调参

### 9.1 读取触摸

```python
from maix import touchscreen, app, time

ts = touchscreen.TouchScreen()
last_pressed = False

while not app.need_exit():
    x, y, pressed = ts.read()

    if pressed and not last_pressed:
        print("pressed", x, y)

    if not pressed and last_pressed:
        print("released", x, y)

    last_pressed = pressed
    time.sleep_ms(1)
```

`ts.read()` 返回：

```text
(x, y, pressed)
```

其中 `pressed` 为当前是否按下。按下沿适合按钮，持续按下适合滑条，释放沿适合确认一次点击或完成拖框。

### 9.2 按钮命中判断

```python
def point_in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh
```

按钮至少需要处理：

- 按下沿，防止一次按压触发多次。
- 释放沿，确认一次点击完成。
- 适当增大触摸热区。
- 必要时增加时间防抖。

### 9.3 屏幕坐标与图像坐标映射

当 `320x240` 图像以 `FIT_CONTAIN` 显示到另一尺寸的屏幕时，触摸坐标不能直接作为图像坐标。

屏幕坐标映射回图像：

```python
image_x, image_y = image.resize_map_pos_reverse(
    img.width(),
    img.height(),
    disp.width(),
    disp.height(),
    image.Fit.FIT_CONTAIN,
    touch_x,
    touch_y
)
```

图像上的矩形映射到屏幕：

```python
screen_rect = image.resize_map_pos(
    img.width(),
    img.height(),
    disp.width(),
    disp.height(),
    image.Fit.FIT_CONTAIN,
    rect_x,
    rect_y,
    rect_w,
    rect_h
)
```

`FIT_COVER`、`FIT_FILL` 和 `FIT_NONE` 必须使用与 `disp.show()` 相同的适配方式，否则按钮和触摸位置会错位。

### 9.4 触摸拖框

拖框流程：

1. 按下沿记录起点 `(start_x, start_y)`。
2. 持续按下时绘制起点到当前位置的矩形。
3. 释放沿规范化为 `[x, y, w, h]`。
4. 限制最小宽高，防止误触产生零尺寸 ROI。
5. 用 ROI 内统计量或中心像素生成初始 LAB 阈值。

规范化函数：

```python
def normalize_rect(x1, y1, x2, y2):
    left = min(x1, x2)
    top = min(y1, y2)
    width = abs(x2 - x1)
    height = abs(y2 - y1)
    return [left, top, width, height]
```

---

## 10. 完整 LAB 触摸调参程序

下面程序具备：

- 六个 LAB 参数按钮。
- 点击选择参数。
- 按住滑条实时修改参数。
- 实时运行 `get_regression()`。
- `SAVE` 保存到 JSON。
- `RESET` 恢复默认值。
- 启动时自动加载 JSON。
- 屏幕和图像尺寸不一致时自动映射触摸坐标。
- 限制最小值不超过最大值。

```python
from maix import camera, display, image, touchscreen, app, time
import json
import os

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CONFIG_PATH = "/root/lab_tuner.json"
FIT_MODE = image.Fit.FIT_CONTAIN

PARAMETERS = [
    ("L_MIN", 0, 100),
    ("L_MAX", 0, 100),
    ("A_MIN", -128, 127),
    ("A_MAX", -128, 127),
    ("B_MIN", -128, 127),
    ("B_MAX", -128, 127)
]

DEFAULT_THRESHOLD = [0, 80, -120, -10, 0, 30]
PARAM_BUTTONS = [
    [2, 150, 104, 22],
    [108, 150, 104, 22],
    [214, 150, 104, 22],
    [2, 174, 104, 22],
    [108, 174, 104, 22],
    [214, 174, 104, 22]
]

SLIDER_RECT = [8, 210, 176, 14]
SAVE_RECT = [192, 202, 58, 30]
RESET_RECT = [256, 202, 62, 30]


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def point_in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def normalize_threshold(values):
    result = list(DEFAULT_THRESHOLD)

    if isinstance(values, list) and len(values) == 6:
        for index, parameter in enumerate(PARAMETERS):
            minimum = parameter[1]
            maximum = parameter[2]
            result[index] = clamp(int(values[index]), minimum, maximum)

    for minimum_index, maximum_index in ((0, 1), (2, 3), (4, 5)):
        if result[minimum_index] > result[maximum_index]:
            result[minimum_index], result[maximum_index] = (
                result[maximum_index],
                result[minimum_index]
            )

    return result


def load_threshold():
    if not os.path.exists(CONFIG_PATH):
        return list(DEFAULT_THRESHOLD)

    try:
        with open(CONFIG_PATH, "r") as file:
            config = json.load(file)
        return normalize_threshold(config.get("threshold"))
    except Exception as error:
        print("load config failed:", error)
        return list(DEFAULT_THRESHOLD)


def save_threshold(threshold):
    config = {"threshold": threshold}

    with open(CONFIG_PATH, "w") as file:
        json.dump(config, file)
        file.flush()
        os.fsync(file.fileno())


def set_selected_value(threshold, selected, value):
    minimum = PARAMETERS[selected][1]
    maximum = PARAMETERS[selected][2]
    value = clamp(value, minimum, maximum)

    if selected == 0:
        threshold[0] = min(value, threshold[1])
    elif selected == 1:
        threshold[1] = max(value, threshold[0])
    elif selected == 2:
        threshold[2] = min(value, threshold[3])
    elif selected == 3:
        threshold[3] = max(value, threshold[2])
    elif selected == 4:
        threshold[4] = min(value, threshold[5])
    else:
        threshold[5] = max(value, threshold[4])


def update_slider(threshold, selected, x):
    slider_x, slider_y, slider_width, slider_height = SLIDER_RECT
    ratio = clamp((x - slider_x) / slider_width, 0.0, 1.0)
    minimum = PARAMETERS[selected][1]
    maximum = PARAMETERS[selected][2]
    value = round(minimum + ratio * (maximum - minimum))
    set_selected_value(threshold, selected, value)


def draw_filled_rect(img, rect, color):
    img.draw_rect(rect[0], rect[1], rect[2], rect[3], color, -1)


def draw_ui(img, threshold, selected, status):
    draw_filled_rect(img, [0, 146, CAMERA_WIDTH, 94], image.COLOR_BLACK)

    for index, rect in enumerate(PARAM_BUTTONS):
        color = image.COLOR_YELLOW if index == selected else image.COLOR_BLUE
        img.draw_rect(rect[0], rect[1], rect[2], rect[3], color, 2)
        label = f"{PARAMETERS[index][0]}:{threshold[index]}"
        img.draw_string(rect[0] + 3, rect[1] + 3, label, color)

    slider_x, slider_y, slider_width, slider_height = SLIDER_RECT
    minimum = PARAMETERS[selected][1]
    maximum = PARAMETERS[selected][2]
    ratio = (threshold[selected] - minimum) / (maximum - minimum)
    knob_x = int(slider_x + ratio * slider_width)

    img.draw_rect(
        slider_x,
        slider_y,
        slider_width,
        slider_height,
        image.COLOR_WHITE,
        1
    )
    img.draw_line(
        slider_x,
        slider_y + slider_height // 2,
        slider_x + slider_width,
        slider_y + slider_height // 2,
        image.COLOR_WHITE,
        2
    )
    img.draw_circle(
        knob_x,
        slider_y + slider_height // 2,
        6,
        image.COLOR_YELLOW,
        -1
    )

    img.draw_rect(
        SAVE_RECT[0],
        SAVE_RECT[1],
        SAVE_RECT[2],
        SAVE_RECT[3],
        image.COLOR_GREEN,
        2
    )
    img.draw_string(SAVE_RECT[0] + 7, SAVE_RECT[1] + 7, "SAVE", image.COLOR_GREEN)

    img.draw_rect(
        RESET_RECT[0],
        RESET_RECT[1],
        RESET_RECT[2],
        RESET_RECT[3],
        image.COLOR_RED,
        2
    )
    img.draw_string(RESET_RECT[0] + 4, RESET_RECT[1] + 7, "RESET", image.COLOR_RED)

    if status:
        img.draw_string(4, 132, status, image.COLOR_YELLOW)


cam = camera.Camera(
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    image.Format.FMT_RGB888,
    fps=30,
    buff_num=1
)
disp = display.Display()
ts = touchscreen.TouchScreen()

cam.skip_frames(30)
threshold = load_threshold()
selected = 0
last_pressed = False
status = "loaded"
status_started = time.ticks_ms()

while not app.need_exit():
    img = cam.read()
    touch_x, touch_y, pressed = ts.read()

    image_x, image_y = image.resize_map_pos_reverse(
        CAMERA_WIDTH,
        CAMERA_HEIGHT,
        disp.width(),
        disp.height(),
        FIT_MODE,
        touch_x,
        touch_y
    )

    if pressed and not last_pressed:
        for index, rect in enumerate(PARAM_BUTTONS):
            if point_in_rect(image_x, image_y, rect):
                selected = index
                break

        if point_in_rect(image_x, image_y, SAVE_RECT):
            try:
                save_threshold(threshold)
                status = "saved"
            except Exception as error:
                print("save config failed:", error)
                status = "save failed"
            status_started = time.ticks_ms()

        if point_in_rect(image_x, image_y, RESET_RECT):
            threshold = list(DEFAULT_THRESHOLD)
            status = "reset"
            status_started = time.ticks_ms()

    if pressed:
        slider_x, slider_y, slider_width, slider_height = SLIDER_RECT
        slider_touch_rect = [
            slider_x - 8,
            slider_y - 12,
            slider_width + 16,
            slider_height + 24
        ]
        if point_in_rect(image_x, image_y, slider_touch_rect):
            update_slider(threshold, selected, image_x)

    lines = img.get_regression(
        [threshold],
        roi=[0, 0, CAMERA_WIDTH, 145],
        area_threshold=80,
        pixels_threshold=80
    )

    for line in lines:
        img.draw_line(
            line.x1(),
            line.y1(),
            line.x2(),
            line.y2(),
            image.COLOR_GREEN,
            2
        )
        img.draw_string(
            4,
            4,
            f"theta:{line.theta():.1f} rho:{line.rho():.1f}",
            image.COLOR_GREEN
        )

    if time.ticks_diff(status_started) >= 1200:
        status = ""

    draw_ui(img, threshold, selected, status)
    disp.show(img, fit=FIT_MODE)

    last_pressed = pressed
    time.sleep_ms(1)
```

### 10.1 将 get_regression 改成 find_blobs

保留触摸和配置代码，只替换识别部分：

```python
blobs = img.find_blobs(
    [threshold],
    roi=[0, 0, CAMERA_WIDTH, 145],
    area_threshold=80,
    pixels_threshold=80
)

if blobs:
    target = max(blobs, key=lambda blob: blob.area())
    img.draw_rect(
        target.x(),
        target.y(),
        target.w(),
        target.h(),
        image.COLOR_GREEN,
        2
    )
    img.draw_cross(target.cx(), target.cy(), image.COLOR_RED, 8, 2)
```

### 10.2 正式比赛中的改进

- 调参界面和比赛运行界面分开，防止误触。
- 保存时同时记录曝光、增益、白平衡、ROI 和 PID 参数。
- 保存配置后重新读取一次，确认 JSON 可解析。
- 增加物理按键进入调参模式。
- 正式运行时降低 UI 更新频率。
- 若触摸抖动明显，增加 20 到 50 ms 的按键防抖，但滑条拖动不应使用过长防抖。

---

## 11. UART 与 STM32 通信

### 11.1 接线和串口选择

必须满足：

- MaixCAM TX 接 STM32 RX。
- MaixCAM RX 接 STM32 TX。
- 两块板共地。
- MaixCAM IO 为 3.3 V，不可直接接入 5 V 电平。

MaixCAM 的 UART0 常用于启动日志和 Maix Protocol，TX 被外部拉低还可能影响启动。与 STM32 通信优先使用 UART1：

| 信号 | MaixCAM 引脚 | 设备节点 |
|---|---|---|
| UART1 TX | `A19` | `/dev/ttyS1` |
| UART1 RX | `A18` | `/dev/ttyS1` |

### 11.2 初始化

```python
from maix import uart, pinmap, err

err.check_raise(
    pinmap.set_pin_function("A19", "UART1_TX"),
    "set UART1 TX failed"
)
err.check_raise(
    pinmap.set_pin_function("A18", "UART1_RX"),
    "set UART1 RX failed"
)

serial = uart.UART("/dev/ttyS1", 115200)
```

查看当前串口设备：

```python
from maix import uart

print(uart.list_devices())
```

### 11.3 字符协议

调试阶段可以发送文本：

```python
serial.write_str("X=120,Y=85,ANGLE=-12.5\n")
```

优点是串口助手可直接查看；缺点是解析开销和数据长度较大。正式闭环控制建议使用定长二进制帧。

### 11.4 定长二进制协议

定义 9 字节帧：

| 字节 | 含义 |
|---|---|
| 0 | 帧头 `0xAA` |
| 1 | 帧头 `0x55` |
| 2 到 3 | `x`，有符号 16 位，小端 |
| 4 到 5 | `y`，有符号 16 位，小端 |
| 6 到 7 | 角度乘 100，有符号 16 位，小端 |
| 8 | 前 8 字节累加和低 8 位 |

MaixCAM 发送函数：

```python
import struct


def clamp_int16(value):
    return max(-32768, min(32767, int(value)))


def send_target(serial, x, y, angle_deg):
    angle_x100 = round(angle_deg * 100)
    frame_without_checksum = struct.pack(
        "<BBhhh",
        0xAA,
        0x55,
        clamp_int16(x),
        clamp_int16(y),
        clamp_int16(angle_x100)
    )
    checksum = sum(frame_without_checksum) & 0xFF
    serial.write(frame_without_checksum + bytes([checksum]))
```

结合色块识别：

```python
if blobs:
    target = max(blobs, key=lambda blob: blob.area())
    error_x = target.cx() - img.width() // 2
    error_y = target.cy() - img.height() // 2
    send_target(serial, error_x, error_y, target.rotation_deg())
else:
    send_target(serial, 0, 0, 0.0)
```

应在协议中明确“未发现目标”。简单项目可增加状态字节；上例用全零表示未发现目标时，若全零也可能是有效目标，应扩展协议。

### 11.5 读取和回调

同步读取：

```python
data = serial.read()
data_with_timeout = serial.read(len=10, timeout=1000)
```

`read()` 和 `set_received_callback()` 不应混用。接收回调运行在另一个线程，回调中不要直接长时间运行视觉算法或修改未加保护的共享图像对象。

### 11.6 Maix 内置通信协议

MaixPy 提供 `comm.CommProtocol`：

```python
from maix import comm

protocol = comm.CommProtocol(buff_size=1024)
message = protocol.get_msg()

if message:
    protocol.resp_ok(message.cmd, b"received")

protocol.report(0x10, b"target data")
```

该协议可通过 UART 或 TCP 承载，适合需要命令、响应、主动上报和较完整封装的项目。简单 STM32 工程使用自定义定长协议更容易实现；复杂上位机交互可考虑内置协议。

---

## 12. GPIO、PWM、I2C、SPI、ADC、按键和看门狗

## 12.1 引脚复用 pinmap

```python
from maix import pinmap

print(pinmap.get_pins())
print(pinmap.get_pin_functions("A17"))
print(pinmap.get_pin_function("A17"))
pinmap.set_pin_function("A17", "GPIOA17")
```

MaixCAM 和 MaixCAM-Pro 的当前功能查询有一部分依赖软件记录，设置过的引脚结果更可靠；MaixCAM2 支持从硬件读取更多当前状态。

使用引脚前检查它是否与下列功能冲突：

- Wi-Fi。
- 系统状态 LED。
- 系统按键。
- UART0 启动日志。
- 摄像头、屏幕或触摸接口。
- 已启用的 PWM、I2C 或 SPI。

## 12.2 GPIO

```python
from maix import gpio, pinmap, time, sys, err

pin_name = "A6" if sys.device_id() == "maixcam2" else "A14"
gpio_name = "GPIOA6" if sys.device_id() == "maixcam2" else "GPIOA14"

err.check_raise(
    pinmap.set_pin_function(pin_name, gpio_name),
    "set pin failed"
)

output = gpio.GPIO(gpio_name, gpio.Mode.OUT)
output.value(0)

for index in range(10):
    output.toggle()
    time.sleep_ms(200)
```

安全要求：

- GPIO 为 3.3 V 逻辑，不耐受 5 V。
- GPIO 不应直接驱动电机、舵机、大功率 LED、继电器线圈或蜂鸣器功率级。
- 使用 MOSFET、三极管、驱动器或光耦，并根据负载增加续流二极管。
- 大电流负载单独供电，但控制系统必须共地。

## 12.3 PWM 舵机

```python
from maix import pwm, pinmap, err, sys, time

SERVO_FREQ = 50
SERVO_MIN_DUTY = 2.5
SERVO_MAX_DUTY = 12.5

if sys.device_id() == "maixcam2":
    servo_pin = "A31"
    pwm_id = 7
else:
    servo_pin = "A19"
    pwm_id = 7

err.check_raise(
    pinmap.set_pin_function(servo_pin, f"PWM{pwm_id}"),
    "set pinmap failed"
)


def percent_to_duty(percent):
    percent = max(0.0, min(100.0, percent))
    return (
        SERVO_MIN_DUTY
        + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * percent / 100.0
    )


servo = pwm.PWM(
    pwm_id,
    freq=SERVO_FREQ,
    duty=percent_to_duty(50),
    enable=True
)

for percent in (20, 50, 80, 50):
    servo.duty(percent_to_duty(percent))
    time.sleep_ms(500)
```

注意：

- 某些上游舵机示例使用 `sys.device_id()` 却漏导入 `sys`，上例已补齐。
- `0.5 ms` 到 `2.5 ms` 并不适合所有舵机。
- 首次测试应缩小范围，例如 5% 到 10% 占空比。
- 限制机械角度，避免堵转烧毁舵机。
- 舵机不要由 MaixCAM 的 3.3 V 引脚供电。
- `A19` 同时可能用作 UART1 TX，PWM 与 UART1 不能同时占用该引脚。

## 12.4 I2C

MaixCAM 常用软件 I2C5：

| 信号 | 引脚 |
|---|---|
| SCL | `A15` |
| SDA | `A27` |

MaixCAM2 示例使用 I2C6：

| 信号 | 引脚 |
|---|---|
| SCL | `A1` |
| SDA | `A0` |

扫描设备：

```python
from maix import i2c, pinmap, err, sys

if sys.device_id() == "maixcam2":
    i2c_id = 6
    scl_pin = "A1"
    sda_pin = "A0"
else:
    i2c_id = 5
    scl_pin = "A15"
    sda_pin = "A27"

err.check_raise(
    pinmap.set_pin_function(scl_pin, f"I2C{i2c_id}_SCL"),
    "set I2C SCL failed"
)
err.check_raise(
    pinmap.set_pin_function(sda_pin, f"I2C{i2c_id}_SDA"),
    "set I2C SDA failed"
)

bus = i2c.I2C(i2c_id, i2c.Mode.MASTER)
print(bus.scan())
```

I2C 排错顺序：

1. 确认共地和 3.3 V 电平。
2. 确认 SDA、SCL 没有接反。
3. 确认上拉电阻存在。
4. 先降低总线频率。
5. 先运行 `scan()` 检查地址。
6. 确认模块地址是 7 位地址，不要混用包含读写位的 8 位地址。

## 12.5 SPI

MaixCAM SPI4 示例引脚：

| 信号 | 引脚 |
|---|---|
| CS | `A24` |
| MISO | `A23` |
| MOSI | `A25` |
| SCK | `A22` |

MaixCAM2 SPI2 示例引脚：

| 信号 | 引脚 |
|---|---|
| CS1 | `B21` |
| MISO | `B19` |
| MOSI | `B18` |
| SCK | `B20` |

基本回环示例：

```python
from maix import spi, pinmap, err, sys

if sys.device_id() == "maixcam2":
    spi_id = 2
    pin_functions = [
        ("B21", "SPI2_CS1"),
        ("B19", "SPI2_MISO"),
        ("B18", "SPI2_MOSI"),
        ("B20", "SPI2_SCK")
    ]
else:
    spi_id = 4
    pin_functions = [
        ("A24", "SPI4_CS"),
        ("A23", "SPI4_MISO"),
        ("A25", "SPI4_MOSI"),
        ("A22", "SPI4_SCK")
    ]

for pin_name, function_name in pin_functions:
    err.check_raise(
        pinmap.set_pin_function(pin_name, function_name),
        "set SPI pin failed"
    )

spi_dev = spi.SPI(spi_id, spi.Mode.MASTER, 1250000)
tx_data = bytes(range(8))
rx_data = spi_dev.write_read(tx_data, len(tx_data))
print(rx_data)
```

SPI 必须匹配：

- 时钟频率。
- CPOL 和 CPHA 对应的模式。
- 位序。
- 片选有效电平和时序。
- 单工、双工以及 MISO 是否存在。

## 12.6 ADC

当前文档说明 MaixCAM 和 MaixCAM-Pro 支持 ADC，MaixCAM2 不支持该 ADC API。

```python
from maix.peripheral import adc

adc0 = adc.ADC(0, adc.RES_BIT_12)
raw_value = adc0.read()
voltage = adc0.read_vol()

print("raw:", raw_value)
print("voltage:", voltage)
```

12 位原始值范围通常为 `0` 到 `4095`。输入电压不得超过硬件允许范围，传感器输出较高时必须分压，并考虑输入阻抗和参考电压误差。

## 12.7 按键

```python
from maix import key, app, time


def on_key(key_code, state):
    if key_code == key.Keys.KEY_OK:
        if state == key.State.KEY_PRESSED:
            print("pressed")
        elif state == key.State.KEY_RELEASED:
            print("released")
        elif state == key.State.KEY_LONG_PRESSED:
            print("long pressed")


input_key = key.Key(on_key)

while not app.need_exit():
    time.sleep_ms(20)
```

按键回调运行在独立线程。回调应只更新简单状态，不要在回调中执行长时间阻塞操作。对象销毁时还要注意回调闭包造成的循环引用。

## 12.8 看门狗 WDT

```python
from maix import wdt, app, time

watchdog = wdt.WDT(0, 1000)

while not app.need_exit():
    watchdog.feed()
    time.sleep_ms(200)
```

正式程序中，只有当关键视觉、通信和控制流程都正常推进时才喂狗。若无条件在独立线程持续喂狗，主逻辑死锁后看门狗也不会复位系统。

---

## 13. PID 与舵机跟踪

### 13.1 误差定义

色块或检测框中心：

```python
error_x = target_x - image_width / 2
error_y = target_y - image_height / 2
```

可归一化到 `-1` 到 `1`：

```python
normalized_x = error_x / (image_width / 2)
normalized_y = error_y / (image_height / 2)
```

归一化后，同一组 PID 参数在不同分辨率之间更容易迁移。

### 13.2 完整 PID 类

```python
class PID:
    def __init__(self, kp, ki, kd, output_min, output_max, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.last_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.initialized = False

    def update(self, error, dt):
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral)
        )

        derivative = 0.0
        if self.initialized:
            derivative = (error - self.last_error) / dt
        else:
            self.initialized = True

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        output = max(self.output_min, min(self.output_max, output))
        self.last_error = error
        return output
```

### 13.3 色块云台跟踪模板

```python
from maix import camera, display, image, app, time, pwm, pinmap, err


class PID:
    def __init__(self, kp, ki, kd, output_min, output_max, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.last_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.initialized = False

    def update(self, error, dt):
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral)
        )

        derivative = 0.0
        if self.initialized:
            derivative = (error - self.last_error) / dt
        else:
            self.initialized = True

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        self.last_error = error
        return max(self.output_min, min(self.output_max, output))


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
PAN_PWM_ID = 6
TILT_PWM_ID = 7
PAN_PIN = "A18"
TILT_PIN = "A19"
SERVO_MIN_DUTY = 4.0
SERVO_MAX_DUTY = 11.0

err.check_raise(
    pinmap.set_pin_function(PAN_PIN, f"PWM{PAN_PWM_ID}"),
    "set pan PWM failed"
)
err.check_raise(
    pinmap.set_pin_function(TILT_PIN, f"PWM{TILT_PWM_ID}"),
    "set tilt PWM failed"
)

pan_servo = pwm.PWM(PAN_PWM_ID, freq=50, duty=7.5, enable=True)
tilt_servo = pwm.PWM(TILT_PWM_ID, freq=50, duty=7.5, enable=True)

pan_pid = PID(0.8, 0.05, 0.08, -1.0, 1.0, 0.5)
tilt_pid = PID(0.8, 0.05, 0.08, -1.0, 1.0, 0.5)

cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT, buff_num=1)
disp = display.Display()
thresholds = [[0, 80, 40, 80, 10, 80]]

pan_duty = 7.5
tilt_duty = 7.5
last_ticks = time.ticks_ms()

while not app.need_exit():
    img = cam.read()
    dt = time.ticks_diff(last_ticks) / 1000.0
    last_ticks = time.ticks_ms()

    blobs = img.find_blobs(
        thresholds,
        pixels_threshold=200,
        area_threshold=200
    )

    if blobs:
        target = max(blobs, key=lambda blob: blob.area())
        normalized_x = (target.cx() - CAMERA_WIDTH / 2) / (CAMERA_WIDTH / 2)
        normalized_y = (target.cy() - CAMERA_HEIGHT / 2) / (CAMERA_HEIGHT / 2)

        dead_zone = 0.03
        if abs(normalized_x) < dead_zone:
            normalized_x = 0.0
        if abs(normalized_y) < dead_zone:
            normalized_y = 0.0

        pan_duty += pan_pid.update(normalized_x, dt) * dt
        tilt_duty -= tilt_pid.update(normalized_y, dt) * dt

        pan_duty = max(SERVO_MIN_DUTY, min(SERVO_MAX_DUTY, pan_duty))
        tilt_duty = max(SERVO_MIN_DUTY, min(SERVO_MAX_DUTY, tilt_duty))

        pan_servo.duty(pan_duty)
        tilt_servo.duty(tilt_duty)

        img.draw_rect(
            target.x(),
            target.y(),
            target.w(),
            target.h(),
            image.COLOR_GREEN,
            2
        )
        img.draw_cross(target.cx(), target.cy(), image.COLOR_RED, 8, 2)
    else:
        pan_pid.reset()
        tilt_pid.reset()

    disp.show(img)
```

实际运行前必须：

- 根据舵机方向确定 `+=` 或 `-=`。
- 从较小 `Kp` 开始。
- `Ki` 初始设为 0，稳定后只添加少量。
- 目标抖动时增加死区或低通滤波。
- 控制更新使用真实 `dt`，不要假设固定帧率。
- 严格限制占空比和机械角度。
- 检测丢失时重置积分，防止目标重新出现后突然跳动。

### 13.4 低通滤波

```python
filtered_x = alpha * measured_x + (1.0 - alpha) * filtered_x
```

`alpha` 越大越跟手，越小越平滑。可从 `0.2` 到 `0.5` 测试。滤波会引入延迟，快速闭环不应过度平滑。

---

## 14. AI 分类和目标检测

### 14.1 模型输入必须匹配

不要手写模型输入尺寸和格式，应从模型对象读取：

```python
cam = camera.Camera(
    model.input_width(),
    model.input_height(),
    model.input_format()
)
```

### 14.2 图像分类

```python
from maix import camera, display, image, nn, app

classifier = nn.Classifier(
    model="/root/models/classifier.mud",
    dual_buff=True
)

cam = camera.Camera(
    classifier.input_width(),
    classifier.input_height(),
    classifier.input_format()
)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    results = classifier.classify(img)

    if results:
        class_id, probability = results[0]
        label = classifier.labels[class_id]
        text = f"{label}:{probability:.2f}"
        img.draw_string(4, 4, text, image.COLOR_GREEN)

    disp.show(img)
```

分类只告诉整张图属于哪一类，不直接提供目标位置。若题目需要坐标，应使用目标检测或先裁剪固定 ROI。

### 14.3 YOLO11 目标检测

```python
from maix import camera, display, image, nn, app

model = nn.YOLO11(
    model="/root/models/yolo11n.mud",
    dual_buff=True
)

cam = camera.Camera(
    model.input_width(),
    model.input_height(),
    model.input_format()
)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    objects = model.detect(
        img,
        conf_th=0.5,
        iou_th=0.45
    )

    for obj in objects:
        img.draw_rect(
            obj.x,
            obj.y,
            obj.w,
            obj.h,
            image.COLOR_RED,
            2
        )
        label = model.labels[obj.class_id]
        text = f"{label}:{obj.score:.2f}"
        img.draw_string(obj.x, obj.y, text, image.COLOR_RED)

    disp.show(img)
```

本地示例还包含 `nn.YOLOv5` 和 `nn.YOLOv8`，基本流程相同：加载模型、按模型输入创建摄像头、调用 `detect()`、读取目标框。

### 14.4 dual_buff 的取舍

`dual_buff=True`：

- CPU 和 NPU 可以流水并行，吞吐通常更高。
- 返回的是上一帧结果。
- 增加一帧延迟。
- 增加额外内存占用。

适合：

- 只要求较高 FPS 的显示和检测。
- 结果允许落后一帧。

考虑 `dual_buff=False`：

- 激光点跟踪。
- 高速云台闭环。
- 底盘高速避障。
- 对单帧延迟敏感的控制。

### 14.5 检测结果筛选

不要默认使用第一个目标。常见筛选策略：

```python
target = max(objects, key=lambda obj: obj.w * obj.h)
```

按类别和置信度：

```python
candidates = [
    obj
    for obj in objects
    if obj.class_id == wanted_class and obj.score >= 0.65
]
```

还可结合：

- 距离图像中心最近。
- 与上一帧位置最接近。
- 面积范围。
- ROI。
- 连续多帧确认。

### 14.6 MaixCAM2 AI ISP

AI ISP 会占用部分 NPU 资源。需要关闭时：

```python
from maix import app

app.set_sys_config_kv("npu", "ai_isp", "0")
```

修改系统配置前应确认当前设备和固件支持，并重启后验证画质与模型可用内存。

---

## 15. OpenCV、网络与音频

## 15.1 OpenCV 零拷贝转换

```python
from maix import camera, display, image, app
import cv2

cam = camera.Camera(320, 240, image.Format.FMT_BGR888)
disp = display.Display()

while not app.need_exit():
    img = cam.read()
    cv_img = image.image2cv(
        img,
        ensure_bgr=False,
        copy=False
    )

    cv2.line(cv_img, (20, 20), (200, 100), (0, 255, 0), 2)

    display_img = image.cv2image(
        cv_img,
        bgr=True,
        copy=False
    )
    disp.show(display_img)
```

`copy=False` 时，转换后的对象引用原始内存。必须保证底层 `img` 或 `cv_img` 在使用期间仍存在，不要返回悬空引用。

选择原则：

- Maix API 有硬件优化时优先使用 Maix API。
- 透视变换、自适应阈值、复杂轮廓操作可使用 OpenCV。
- 避免一帧内多次 RGB/BGR 转换和内存复制。

## 15.2 Wi-Fi

```python
from maix import network, err

wifi = network.wifi.Wifi()
result = wifi.connect(
    "competition_wifi",
    "password1234",
    wait=True,
    timeout=60
)
err.check_raise(result, "connect wifi failed")
print(wifi.get_ip())
```

正式项目不要把真实密码上传到公开仓库。比赛现场优先保证离线核心功能，网络只用于非关键调试、图传或参数下发。

## 15.3 TCP 服务器骨架

```python
import socket
import threading

HOST = "0.0.0.0"
PORT = 8080


def handle_client(client_socket, address):
    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            client_socket.sendall(b"ACK:" + data)
    finally:
        client_socket.close()
        print("disconnected:", address)


server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(4)

while True:
    client, client_address = server.accept()
    thread = threading.Thread(
        target=handle_client,
        args=(client, client_address),
        daemon=True
    )
    thread.start()
```

网络线程与视觉主循环共享数据时，应通过队列、锁或只交换不可变快照，避免并发修改图像对象。

## 15.4 录音

```python
from maix import audio

recorder = audio.Recorder("/root/test.wav")
recorder.volume(100)
recorder.record(3000)
```

## 15.5 播放音频

```python
from maix import audio

player = audio.Player("/root/test.wav")
player.volume(80)
player.play()
```

音频可用于比赛状态提示。不要在严格控制周期内使用阻塞式长音频操作。

---

## 16. 系统配置、参数保存与开机自启

### 16.1 读取系统配置

```python
from maix import app

locale = app.get_sys_config_kv("language", "locale")
backlight = app.get_sys_config_kv("backlight", "value")

print(locale)
print(backlight)
```

系统配置位于 `/boot/configs`。读取值均为字符串，需要按用途转换：

```python
backlight_value = int(backlight)
```

### 16.2 写入系统配置

```python
from maix import app

app.set_sys_config_kv("npu", "ai_isp", "0")
```

系统配置影响范围比应用 JSON 更大。视觉阈值、ROI 和 PID 参数优先保存在应用自己的 JSON 文件中。

### 16.3 通用 JSON 参数保存

```python
import json
import os

CONFIG_PATH = "/root/competition_config.json"
DEFAULT_CONFIG = {
    "threshold": [0, 80, -120, -10, 0, 30],
    "roi": [0, 80, 320, 160],
    "exposure": 1000,
    "gain": 100,
    "pid": {
        "kp": 0.8,
        "ki": 0.0,
        "kd": 0.08
    }
}


def load_config():
    if not os.path.exists(CONFIG_PATH):
        return dict(DEFAULT_CONFIG)

    try:
        with open(CONFIG_PATH, "r") as file:
            return json.load(file)
    except Exception as error:
        print("load config failed:", error)
        return dict(DEFAULT_CONFIG)


def save_config(config):
    with open(CONFIG_PATH, "w") as file:
        json.dump(config, file)
        file.flush()
        os.fsync(file.fileno())
```

配置来源于用户或外部文件，加载后应检查：

- 列表长度。
- 数值类型。
- LAB 范围。
- ROI 是否超出图像。
- `min` 是否不大于 `max`。
- 舵机和 PID 参数是否处于安全范围。

### 16.4 开机自启

推荐方式是在设备设置中选择需要开机自启的应用。系统也支持在：

```text
/maixapp/auto_start.txt
```

写入应用 ID。

不建议普通摄像头应用直接修改系统启动服务。错误的服务配置可能导致开机后进程持续占用摄像头和屏幕，使 MaixVision 无法正常停止应用。

### 16.5 应用退出

主循环：

```python
while not app.need_exit():
    time.sleep_ms(1)
```

主动请求退出：

```python
app.set_exit_flag(True)
```

释放资源时，可在退出循环后关闭串口、停止 PWM 或将执行器恢复安全位置。具体关闭方法以当前对象接口为准。

---

## 17. 性能优化与比赛现场检查表

## 17.1 测量耗时和 FPS

```python
from maix import time

start = time.ticks_ms()

result = img.find_blobs(
    thresholds,
    pixels_threshold=200,
    area_threshold=200
)

elapsed = time.ticks_diff(start)
print("find_blobs ms:", elapsed)
```

连续 FPS：

```python
from maix import time

time.fps_start()

while True:
    img = cam.read()
    disp.show(img)
    print("fps:", time.fps())
```

不要只测算法函数，应分别测量：

- `cam.read()`。
- 图像预处理。
- 识别或 NPU 推理。
- 绘图。
- `disp.show()`。
- 串口发送。
- 完整控制周期。

## 17.2 优化优先级

1. 缩小 ROI。
2. 降低输入分辨率。
3. 限制圆半径、目标类别或候选范围。
4. 避免每帧保存图片和打印大量日志。
5. 减少图像格式转换和复制。
6. 需要时使用灰度图。
7. 降低 UI 和字符串绘制频率。
8. 传统视觉能稳定解决时，不必强行使用更大的 AI 模型。
9. 吞吐优先时测试 `dual_buff=True`。
10. 闭环延迟优先时测试 `dual_buff=False` 和 `buff_num=1`。

## 17.3 稳定性策略

- 固定供电，舵机和电机使用独立电源。
- 所有控制板共地。
- 使用 WDT，但只在关键任务健康时喂狗。
- 对目标结果做连续帧确认。
- 对偶发丢失设置短暂保持，长时间丢失进入安全状态。
- 限制 PID 积分和执行器输出。
- 配置文件损坏时回退到安全默认值。
- 记录当前配置版本，避免旧配置与新程序字段不一致。
- 串口协议使用帧头、长度或定长结构、校验和、状态字段。
- 不在控制主循环中做阻塞网络连接、长时间文件写入或音频播放。

## 17.4 常见故障排查

| 现象 | 优先检查 |
|---|---|
| 色块时有时无 | 自动曝光、白平衡、LAB 阈值、反光、ROI |
| 巡线方向反了 | 镜像、摄像头安装方向、误差符号、电机方向 |
| 帧率低 | 分辨率、ROI、绘图、格式转换、算法阈值、日志 |
| 显示有黑边 | `FIT_CONTAIN` 的正常行为 |
| 触摸位置偏移 | 显示适配方式与坐标映射方式不一致 |
| 串口乱码 | 波特率、共地、电平、字节序、文本和二进制混用 |
| 板子无法启动 | UART0 TX 被拉低、供电不足、外设占用启动引脚 |
| 舵机抖动 | 供电、共地、检测噪声、PID、PWM 范围、机械负载 |
| I2C 扫不到设备 | 引脚复用、地址、上拉、电压、SDA/SCL、频率 |
| AI 运行内存不足 | 模型大小、`dual_buff`、AI ISP、图像尺寸、多图层 |
| 圆或矩形误检多 | ROI、边缘阈值、尺寸范围、透视矫正、连续帧筛选 |

## 17.5 上场前检查表

### 硬件

- [ ] MaixCAM、STM32、传感器和执行器可靠共地。
- [ ] 所有 MaixCAM IO 都是 3.3 V 安全电平。
- [ ] 舵机和电机不从 GPIO 或弱电源直接取电。
- [ ] PWM 范围不会让舵机撞机械限位。
- [ ] UART0 没有被外部电路拉低。
- [ ] 摄像头、屏幕和触摸排线固定。
- [ ] 电源在电机启动和舵机堵转瞬间仍稳定。

### 视觉

- [ ] 在实际光源下重新标定 LAB 阈值。
- [ ] 曝光、增益和白平衡策略已确定。
- [ ] ROI 与实际机位匹配。
- [ ] 镜像和翻转设置正确。
- [ ] 目标丢失、遮挡和多个目标的策略明确。
- [ ] 检测结果经过连续帧或合理筛选。

### 通信与控制

- [ ] UART 波特率、帧格式、字节序和校验一致。
- [ ] STM32 能区分有效目标和目标丢失。
- [ ] PID 的 `Kp`、`Ki`、`Kd`、死区和输出限制已实车测试。
- [ ] 控制方向和误差正负号已验证。
- [ ] 超时后执行器进入安全状态。

### 软件

- [ ] MaixPy 固件、MaixVision 和模型版本匹配。
- [ ] 配置 JSON 可读取，损坏时有安全默认值。
- [ ] 主循环使用 `app.need_exit()`。
- [ ] 没有每帧写文件或输出大量日志。
- [ ] 已测量端到端延迟，而不只看 FPS。
- [ ] 开机自启应用正确。
- [ ] 断电重启后能独立运行。
- [ ] 保留一份经过验证的参数和程序备份。

---

## 18. 源码和示例索引

以下路径均相对于项目中的：

```text
MaixPy-main/MaixPy-main
```

### 18.1 核心文档

| 内容 | 路径 |
|---|---|
| 巡线与 `get_regression` | `docs/doc/zh/vision/line_tracking.md` |
| 色块识别 | `docs/doc/zh/vision/find_blobs.md` |
| 摄像头 | `docs/doc/zh/vision/camera.md` |
| 屏幕 | `docs/doc/zh/vision/display.md` |
| 触摸屏 | `docs/doc/zh/vision/touchscreen.md` |
| 图像操作 | `docs/doc/zh/vision/image_ops.md` |
| 二维码 | `docs/doc/zh/vision/qrcode.md` |
| 条形码 | `docs/doc/zh/vision/find_barcodes.md` |
| AprilTag | `docs/doc/zh/vision/apriltag.md` |
| OpenCV | `docs/doc/zh/vision/opencv.md` |
| NPU 双缓冲 | `docs/doc/zh/vision/dual_buff.md` |
| 引脚复用 | `docs/doc/zh/peripheral/pinmap.md` |
| GPIO | `docs/doc/zh/peripheral/gpio.md` |
| PWM | `docs/doc/zh/peripheral/pwm.md` |
| UART | `docs/doc/zh/peripheral/uart.md` |
| I2C | `docs/doc/zh/peripheral/i2c.md` |
| SPI | `docs/doc/zh/peripheral/spi.md` |
| ADC | `docs/doc/zh/peripheral/adc.md` |
| 看门狗 | `docs/doc/zh/peripheral/wdt.md` |
| 应用与系统配置 | `docs/doc/zh/basic/app.md` |
| 开机自启 | `docs/doc/zh/basic/auto_start.md` |
| Maix 通信协议 | `docs/doc/zh/comm/maix_protocol.md` |

### 18.2 重点示例

| 内容 | 路径 |
|---|---|
| 现代巡线 | `examples/vision/image_basic/line_tracking.py` |
| v1 回归线 | `examples/maixpy_v1/image/get_regression.py` |
| 色块 | `test/test_image_method/test_find_blobs.py` |
| 直线 | `examples/vision/image_basic/find_lines.py` |
| 线段 | `test/test_image_method/test_find_line_segments.py` |
| 圆 | `examples/vision/image_basic/find_circles.py` |
| 矩形 | `examples/vision/image_basic/find_rects.py` |
| 二维码 | `examples/vision/image_basic/find_qrcodes.py` |
| 快速二维码 | `examples/vision/image_basic/find_qrcodes_faster.py` |
| AprilTag | `examples/vision/image_basic/find_apriltags.py` |
| Data Matrix | `test/test_image_method/test_find_datamatrices.py` |
| 模板匹配 | `test/test_image_method/test_find_template.py` |
| 二值化 | `examples/vision/image_basic/binary.py` |
| 边缘 | `examples/vision/image_basic/find_edges.py` |
| 洪泛填充 | `examples/vision/image_basic/flood_fill.py` |
| 直方图 | `test/test_image_method/test_get_histogram.py` |
| 多图层显示 | `examples/vision/display/display_multi_channel.py` |
| 触摸读取 | `examples/vision/touchscreen/touchscreen_read.py` |
| 触摸绘图 | `examples/vision/touchscreen/touchscreen_draw.py` |
| 触摸坐标映射 | `examples/vision/touchscreen/touchscreen_draw_small_img.py` |
| UART 二进制通信 | `examples/peripheral/uart/comm_uart_binary.py` |
| 舵机 PWM | `examples/peripheral/pwm/pwm_servo.py` |
| I2C 主机 | `examples/peripheral/i2c/i2c_master.py` |
| SPI 回环 | `examples/peripheral/spi/spi_loopback.py` |
| ADC | `examples/peripheral/adc/adc_read.py` |
| YOLO11 | `examples/vision/ai_vision/nn_yolo11_detect.py` |
| YOLOv8 | `examples/vision/ai_vision/nn_yolov8.py` |
| 分类 | `examples/vision/ai_vision/nn_classifier.py` |
| OpenCV 摄像头 | `examples/vision/opencv/opencv_camera.py` |
| 通信协议 | `examples/protocol/comm_protocol.py` |
| Wi-Fi | `examples/network/wifi_connect.py` |
| TCP 服务器 | `examples/network/socket_server.py` |
| 录音 | `examples/audio/audio_record/audio_record_block.py` |
| 播放 | `examples/audio/audio_play/audio_playback_block.py` |
| 时间与 FPS | `examples/basic/demo_time.py` |

### 18.3 电赛综合工程

| 工程 | 重点内容 |
|---|---|
| `projects/demo_diansai_2025_E_circle_track` | YOLO 外框、OpenCV 自适应阈值、洪泛填充、轮廓、四边形、透视变换、圆心、低延迟缓存、手动白平衡、PWM 和 PID |
| `projects/demo_block_tracking` | 触摸拖框、自动取色、LAB 色块跟踪、视场角换算、PID 和云台舵机 |

这两个工程适合学习综合流程，但仍应检查变量作用域、引脚冲突、缺失导入和当前硬件差异后再移植。不要未经验证直接用于比赛。

---

## 结语

电赛视觉系统的可靠性通常不取决于单个函数，而取决于完整链路：

```text
稳定供电
→ 固定或可控成像参数
→ 合理 ROI 和预处理
→ 目标检测与筛选
→ 坐标和角度计算
→ 低延迟通信
→ 有输出限制的控制器
→ 目标丢失和异常安全策略
```

建议先用触摸界面完成现场阈值和参数标定，再关闭大部分调试绘图和日志，最后以端到端延迟、连续运行时间和异常恢复能力作为验收指标。

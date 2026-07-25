# 送药小车视觉系统

全国大学生电子设计竞赛 2021 年 F 题 —— 送药小车（视觉端）

基于 **Sipeed MaixCAM** 平台，通过摄像头 + NPU 实现数字识别与路口检测，配合 STM32 底盘完成自动寻线、路口决策及送药任务。

## 硬件

| 模块 | 型号 |
|------|------|
| 视觉主板 | Sipeed MaixCAM / MaixCAM-Pro |
| 摄像头 | MaixCAM 板载摄像头 |
| 显示屏 | MaixCAM 板载触摸屏 |
| 底盘主控 | STM32 |
| 通信方式 | UART（115200bps） |

## 文件结构

```
.
├── delivery_car_main.py       # 主程序入口（UART: A16 TX / A17 RX）
├── main.py                    # 备用入口（UART: A19 TX / A18 RX）
├── delivery_car_test_bundle.py # 单文件离线测试版（无 UART，纯触摸操作）
├── delivery_logic.py          # 决策逻辑（状态机 / 目标锁定 / 方向投票）
├── track_line.py              # 寻线模块（色块法直线拟合）
├── crossroad_detect_v2.py     # 路口检测（四 ROI 色块法）
├── yolov8_num_detect.py       # YOLOv8 数字检测
├── vision_protocol.py         # UART 通信协议（帧编解码）
├── num_detact.py              # 旧版 MNIST 式数字识别（已废弃，仅供参考）
├── convert.sh                 # ONNX → MLIR 模型转换
├── calibration.sh             # INT8 量化校准
├── model_deploy.sh            # MLIR → cvimodel 部署
├── docs/
│   ├── 项目代码手册.md         # 详细代码手册
│   ├── 送药小车视觉方案.md      # 视觉方案设计文档
│   └── uart_protocol.md       # UART 协议规格说明
└── yolov8_num_detect_v2.cvimodel  # 数字检测模型（NPU）
```

## 系统架构

```
delivery_car_main.py
  ├── DeliveryStateMachine   → 状态机（5 状态）
  ├── TargetLocker           → 多帧投票锁定目标病房号
  ├── DirectionVoter         → 路口方向投票（左/右）
  ├── DigitDetector          → YOLOv8 数字检测（1-8）
  ├── LineTracker            → 黑线循迹
  ├── CrossroadDetector      → 路口检测
  └── FrameParser            → UART 帧解析
```

所有视觉逻辑模块（`delivery_logic.py`）无 MaixPy/OpenCV 依赖，可在 PC 上单独进行单元测试。

## 工作流程

1. **目标捕获** —— 摄像头前手持数字卡片（1-8），多帧投票锁定目标后通过 UART 发送到 STM32
2. **等待出发** —— 收到 STM32 的 START 指令，小车开始寻线
3. **沿线行驶** —— 持续黑线追踪，每 2 帧发送 LINE_DATA 给 STM32 做 PID 控制
4. **路口检测** —— 确认路口后进入方向决策
5. **方向决策** —— 目标 1/2 用预设固定路线，目标 3-8 根据目标数字在路口的相对位置投票决定左/右转
6. **转弯执行** —— 发送 TURN_DECISION，等待 STM32 完成转弯后继续行驶

## UART 通信协议

帧格式：`AA 55 LEN TYPE PAYLOAD CHECKSUM`

| 方向 | 类型 | 说明 |
|------|------|------|
| STM32→Maix | START (0x01) | 开始行驶 |
| STM32→Maix | RESET (0x02) | 复位 |
| STM32→Maix | STOP (0x03) | 停止 |
| STM32→Maix | TURN_DONE (0x04) | 转弯完成 |
| Maix→STM32 | TARGET_LOCKED (0x81) | 目标已锁定 |
| Maix→STM32 | LINE_DATA (0x82) | 线位置数据 |
| Maix→STM32 | TURN_DECISION (0x83) | 转弯决策 |
| Maix→STM32 | VISION_HOLD (0x84) | 视觉暂缓 |

详见 `docs/uart_protocol.md`。

## 模型部署

模型为 YOLOv8，输入尺寸 224×320，经 INT8 量化后部署到 MaixCAM 的 cv181x NPU：

```bash
bash convert.sh        # ONNX → MLIR
bash calibration.sh    # 量化校准（需 200 张校准图片）
bash model_deploy.sh   # MLIR → INT8 cvimodel
```

`.mud` 文件是模型描述符，绑定 `.cvimodel` 与标签、归一化参数等元信息。

## 触摸操作

测试时可使用屏幕上的触摸按钮：

- **RESET** —— 复位，重新捕获目标
- **START** —— 人工触发出发
- **STOP** —— 紧急停止
- **TURN_DONE** —— 模拟 STM32 转弯完成信号

## 已知限制

- 仅支持 LEFT / RIGHT 决策，暂不支持 STRAIGHT
- 未实现到达病房检测、卸药装药流程、掉头与返程
- 路口分类（T 型/十字）尚未区分
- 部分参数需要实地标定

# MaixCam 相机 — 小车 UART 串口通信协议

## 1. 物理连接

| 相机引脚 | 方向 | 小车引脚 |
|---------|------|---------|
| A16 (UART0_TX) | 相机 → 小车 | RX |
| A17 (UART1_RX) | 小车 → 相机 | TX |
| GND | — | GND |

- **波特率**：115200
- **数据位**：8
- **停止位**：1
- **校验位**：无

三根线即可，共地必须接，否则电平不匹配会乱码。

---

## 2. 帧格式（统一）

相机和小车使用同一套帧格式，变长，所有多字节字段为**小端序（little-endian）**。

```
│  Header   │ BodyLen  │ MsgType  │  Payload   │ Checksum │
│  2 bytes  │ 1 byte   │ 1 byte   │ 0~63 bytes │  1 byte  │
│ 0xAA 0x55 │          │          │            │          │
```

| 字段 | 字节数 | 说明 |
|------|--------|------|
| Header | 2 | 固定 `0xAA 0x55`，用于从字节流中定位帧头 |
| BodyLen | 1 | `1 + len(Payload)`，范围 1~64。Payload 为空时 BodyLen = 1 |
| MsgType | 1 | 消息类型编号 |
| Payload | 0~63 | 消息数据体，格式由 MsgType 决定，可能为空 |
| Checksum | 1 | Header + BodyLen + MsgType + Payload 全部字节累加，取低 8 位 |

### 校验和计算

```
sum = Header[0] + Header[1] + BodyLen + MsgType + Payload[0] + ... + Payload[n]
checksum = sum & 0xFF   (sum % 256)
```

---

## 3. 小车 → 相机（上行指令，4 条）

小车发往相机的都是**纯指令，无 Payload，BodyLen = 1**。帧固定 5 字节。

| 指令 | MsgType | 完整帧 (HEX) | 触发时机 |
|------|---------|-------------|---------|
| START | `0x01` | `AA 55 01 01 01` | 小车就绪，可以出发 |
| RESET | `0x02` | `AA 55 01 02 02` | 全局复位 |
| STOP | `0x03` | `AA 55 01 03 03` | 暂停 / 紧急停止 |
| TURN_DONE | `0x04` | `AA 55 01 04 04` | 小车转弯动作已完成 |

### 校验和计算过程

```
START:      0xAA+0x55+0x01+0x01 = 0x101  →  0x01
RESET:      0xAA+0x55+0x01+0x02 = 0x102  →  0x02
STOP:       0xAA+0x55+0x01+0x03 = 0x103  →  0x03
TURN_DONE:  0xAA+0x55+0x01+0x04 = 0x104  →  0x04
```

因为无 Payload，校验和恰好等于 MsgType。小车可以直接硬编码这 4 个字节序列发送。

### C 语言发送示例

```c
// 直接发，无需每次计算
const uint8_t CMD_START[]     = {0xAA, 0x55, 0x01, 0x01, 0x01};
const uint8_t CMD_RESET[]     = {0xAA, 0x55, 0x01, 0x02, 0x02};
const uint8_t CMD_STOP[]      = {0xAA, 0x55, 0x01, 0x03, 0x03};
const uint8_t CMD_TURN_DONE[] = {0xAA, 0x55, 0x01, 0x04, 0x04};

void send_cmd(const uint8_t *cmd) {
    uart_write_bytes(UART_NUM_1, cmd, 5);
}

// 用法
send_cmd(CMD_START);
```

---

## 4. 相机 → 小车（下行消息，5 种）

### 4.1 TARGET_LOCKED — 目标锁定 (0x81)

相机识别到送货目标数字，只发一次。

| 字段 | 偏移 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| target_number | 0 | uint8 | 1 | 1~8，锁定的房号 |

**帧长：6 字节**

```
AA 55  02  81  <target>  <cs>
```

例：锁定 3 号房

```
Payload = 03
BodyLen = 1 + 1 = 2  →  0x02
cs = 0xAA + 0x55 + 0x02 + 0x81 + 0x03 = 0x185 → 0x85
帧 = AA 55 02 81 03 85
```

> **注意**：目标为 1 或 2 时，相机在 TARGET_LOCKED 之后**立即紧跟一条 TURN_DECISION**，小车应将此方向存储起来，后续自动在路口转向。详见 4.3 节和交互流程。

### 4.2 LINE_DATA — 巡线数据 (0x82)

相机持续发送，供小车 PID 循线。**约每 2 帧发一次（约 15Hz）**。

| 字段 | 偏移 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| valid | 0 | uint8 | 1 | 0 = 丢线，1 = 正常 |
| error | 1 | int16 (LE) | 2 | 线段底端与画面中心 X 偏差（像素），正值线偏右 |
| angle | 3 | int16 (LE) | 2 | 线段角度 ×100（度），正值右倾 |
| center_x_norm | 5 | uint16 (LE) | 2 | 线段中心 X 归一化 ×1000，范围 0~1000 |

**帧长：11 字节**

```
AA 55  08  82  <valid:1> <error:2> <angle:2> <center:2>  <cs>
```

例：线正常，右偏 5 像素，角度 2.3° 右倾，归一化中心 0.50

```
valid = 1 → 01
error = 5 → 05 00
angle = 230 (2.3×100) → E6 00
center = 500 (0.50×1000) → F4 01
Payload = 01 05 00 E6 00 F4 01 (7 字节)
BodyLen = 1 + 7 = 8 → 08
cs = 0xAA+0x55+0x08+0x82+0x01+0x05+0x00+0xE6+0x00+0xF4+0x01
   = 170+85+8+130+1+5+0+230+0+244+1 = 874
   = 0x36A & 0xFF = 0x6A
帧 = AA 55 08 82 01 05 00 E6 00 F4 01 6A
```

### 4.3 TURN_DECISION — 路口转向决策 (0x83)

告知小车在**下一个路口**应转向的方向。触发时机取决于目标房号：

- **目标 1~2（固定路线）**：锁定目标后立即发送（在 START 之前），小车存储方向，到路口自动转向。
- **目标 3~8（数字驱动）**：循线过程中，当相机检测到目标数字并完成方向投票后发送。

| 字段 | 偏移 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| direction | 0 | uint8 | 1 | 0 = LEFT（左转），1 = RIGHT（右转） |
| target_number | 1 | uint8 | 1 | 目标房号 1~8 |
| intersection_index | 2 | uint8 | 1 | 当前第几个路口，从 0 开始。1~2 固定为 0 |
| confidence | 3 | uint8 | 1 | 方向置信度 0~100。1~2 固定为 100 |

**帧长：8 字节**

```
AA 55  05  83  <dir:1> <tgt:1> <idx:1> <conf:1>  <cs>
```

例 1：目标 1 号，固定路线左转（锁定后立即发送）

```
Payload = 00 01 00 64
BodyLen = 1 + 4 = 5 → 05
cs = 0xAA+0x55+0x05+0x83+0x00+0x01+0x00+0x64 = 170+85+5+131+0+1+0+100 = 492 = 0x1EC → 0xEC
帧 = AA 55 05 83 00 01 00 64 EC
```

例 2：目标 3 号，在第 1 个路口右转，置信度 85

```
Payload = 01 03 01 55
BodyLen = 5
cs = 0xAA+0x55+0x05+0x83+0x01+0x03+0x01+0x55 = 170+85+5+131+1+3+1+85 = 481 = 0x1E1 → 0xE1
帧 = AA 55 05 83 01 03 01 55 E1
```

### 4.4 VISION_HOLD — 视觉暂缓 (0x84)

相机无法决策或丢线，小车应将底盘**原地保持**（可缓慢刹停或站住不动）。

| 字段 | 偏移 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| reason | 0 | uint8 | 1 | 见下表 |

**reason 取值：**

| 值 | 常量 | 场景 |
|----|------|------|
| 1 | HOLD_NO_TARGET | 未识别到目标数字 |
| 2 | HOLD_DIRECTION_UNCERTAIN | 路口方向投票未达置信 |
| 3 | HOLD_FIXED_ROUTE_MISSING | 固定路线表里没有这个目标 |
| 4 | HOLD_LINE_LOST | 巡线丢失 |

**帧长：6 字节**

```
AA 55  02  84  <reason:1>  <cs>
```

例：方向不确定

```
Payload = 02
BodyLen = 1 + 1 = 2 → 02
cs = 0xAA+0x55+0x02+0x84+0x02 = 0x187 → 0x87
帧 = AA 55 02 84 02 87
```

### 4.5 STATUS — 状态汇报 (0x85)

相机状态机切换时发送。

| 字段 | 偏移 | 类型 | 字节数 | 说明 |
|------|------|------|--------|------|
| state_code | 0 | uint8 | 1 | 见下表 |
| detail | 1 | uint8 | 1 | 预留，当前恒为 0 |

**state_code 取值：**

| 值 | 状态 | 含义 |
|----|------|------|
| 1 | CAPTURE_TARGET | 正在识别目标数字 |
| 2 | WAIT_START | 目标已锁定，等待 START 指令 |
| 3 | FOLLOW_LINE | 循线行驶中 |
| 5 | WAIT_TURN_DONE | 已发出转向决策，等待 TURN_DONE |

> **已废弃**：state_code 4 (DECIDE_DIRECTION) 不再使用。路口方向由数字识别结果直接驱动，不再有独立的路口检测决策阶段。

**帧长：7 字节**

```
AA 55  03  85  <state:1> <detail:1>  <cs>
```

例：进入循线状态

```
Payload = 03 00
BodyLen = 1 + 2 = 3 → 03
cs = 0xAA+0x55+0x03+0x85+0x03+0x00 = 0x18A → 0x8A
帧 = AA 55 03 85 03 00 8A
```

---

## 5. 小车端解析流程

### 5.1 状态机（接收端）

```c
#define BUF_SIZE 128

static uint8_t rx_buf[BUF_SIZE];
static uint8_t rx_idx = 0;

// 在 UART 中断 / 轮询中调用
void parse_byte(uint8_t byte) {
    rx_buf[rx_idx++] = byte;

    // 1. 找帧头
    if (rx_idx < 2) return;
    if (rx_buf[0] != 0xAA || rx_buf[1] != 0x55) {
        // 帧头不对，滑动窗口：丢掉第一个字节
        memmove(rx_buf, rx_buf + 1, --rx_idx);
        return;
    }

    // 2. 等够 4 字节才能读 BodyLen
    if (rx_idx < 4) return;
    uint8_t body_len = rx_buf[2];
    if (body_len < 1 || body_len > 64) {
        // 非法长度，丢掉整个帧头重新找
        memmove(rx_buf, rx_buf + 2, rx_idx - 2);
        rx_idx -= 2;
        return;
    }

    // 3. 帧总长 = Header(2) + BodyLen(1) + MsgType(1) + Payload + Checksum(1)
    //            = 4 + BodyLen
    uint8_t frame_len = 4 + body_len;
    if (rx_idx < frame_len) return;  // 还没收完

    // 4. 校验
    uint8_t cs = 0;
    for (uint8_t i = 0; i < frame_len - 1; i++) {
        cs += rx_buf[i];
    }
    if (cs != rx_buf[frame_len - 1]) {
        // 校验失败，丢掉帧头，重新找
        memmove(rx_buf, rx_buf + 1, --rx_idx);
        return;
    }

    // 5. 解析
    uint8_t msg_type = rx_buf[3];
    uint8_t *payload = rx_buf + 4;
    uint8_t payload_len = body_len - 1;

    handle_message(msg_type, payload, payload_len);

    // 6. 处理完后从 buffer 中移除这一帧，继续找下一帧
    memmove(rx_buf, rx_buf + frame_len, rx_idx - frame_len);
    rx_idx -= frame_len;
}
```

### 5.2 下行消息解析

```c
void handle_message(uint8_t type, uint8_t *data, uint8_t len) {
    switch (type) {
        case 0x81:  // TARGET_LOCKED
            if (len >= 1) {
                uint8_t target = data[0];
                // 记录目标房号
                // 注意：目标为 1 或 2 时，后面会紧跟一条 TURN_DECISION
            }
            break;

        case 0x82:  // LINE_DATA
            if (len >= 7) {
                uint8_t valid  = data[0];
                int16_t error  = *(int16_t *)(data + 1);   // LE
                int16_t angle  = *(int16_t *)(data + 3);   // LE, /100 = 度
                uint16_t center = *(uint16_t *)(data + 5);  // LE, /1000 = 归一化
                if (valid) {
                    // PID 循线：用 error 和 angle 计算转向指令
                } else {
                    // 丢线处理
                }
            }
            break;

        case 0x83:  // TURN_DECISION
            if (len >= 4) {
                uint8_t dir   = data[0];  // 0=LEFT, 1=RIGHT
                uint8_t tgt   = data[1];
                uint8_t idx   = data[2];
                uint8_t conf  = data[3];
                // 目标 1~2：此消息在 START 之前到达，存储方向，在路口自动执行
                // 目标 3~8：此消息在 FOLLOW_LINE 期间到达，在下一个路口执行
                // 执行转弯动作，完成后发 CMD_TURN_DONE
            }
            break;

        case 0x84:  // VISION_HOLD
            if (len >= 1) {
                uint8_t reason = data[0];
                // 停车 / 原地等待
            }
            break;

        case 0x85:  // STATUS
            if (len >= 2) {
                uint8_t state = data[0];
                // 可选：记录 / 指示灯 / 调试
            }
            break;
    }
}
```

### 5.3 Python 发送示例（调试用）

```python
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

CMD_START     = bytes([0xAA, 0x55, 0x01, 0x01, 0x01])
CMD_RESET     = bytes([0xAA, 0x55, 0x01, 0x02, 0x02])
CMD_STOP      = bytes([0xAA, 0x55, 0x01, 0x03, 0x03])
CMD_TURN_DONE = bytes([0xAA, 0x55, 0x01, 0x04, 0x04])

ser.write(CMD_START)
```

---

## 6. 典型交互流程

### 6.1 目标 1~2（固定路线）

小车巡线模块自行检测路口，相机只负责锁定目标并告知方向。

```
小车 (MCU)                             相机 (MaixCam)
    │                                       │
    │── RESET ─────────────────────────────→│  上电复位
    │                                       │
    │←── STATUS (1: CAPTURE_TARGET) ────────│  进入目标捕获状态
    │                                       相机持续跑 YOLO 识别数字牌
    │←── TARGET_LOCKED (房号 1) ────────────│  锁定目标
    │←── TURN_DECISION (LEFT, 房号1, 路口0) │  目标 1=固定左转，立即发送
    │←── STATUS (2: WAIT_START) ────────────│  等待启动
    │                                       │
    │   小车存储方向 = LEFT                   │
    │                                       │
    │── START ─────────────────────────────→│  小车就绪，发车
    │                                       │
    │←── STATUS (3: FOLLOW_LINE) ───────────│  开始循线（不跑 YOLO）
    │←── LINE_DATA ... ─────────────────────│  每 2 帧发一次巡线数据
    │←── LINE_DATA ...                      小车用 error/angle 做 PID
    │←── LINE_DATA ...                      持续跟踪黑线
    │                                       │
    │   小车巡线模块检测到路口                  │
    │   查询存储的方向 = LEFT                  │
    │   执行左转...                            │
    │                                       │
    │── TURN_DONE ─────────────────────────→│  转弯完成
    │                                       │
    │←── STATUS (3: FOLLOW_LINE) ───────────│  继续循线
    │←── LINE_DATA ...                       │
    │    ...到达目标...                        │
```

### 6.2 目标 3~8（数字驱动）

路口识别由小车巡线模块负责，相机通过数字检测来判断在哪个路口转向。

```
小车 (MCU)                             相机 (MaixCam)
    │                                       │
    │── RESET ─────────────────────────────→│  上电复位
    │                                       │
    │←── STATUS (1: CAPTURE_TARGET) ────────│  进入目标捕获状态
    │                                       相机持续跑 YOLO 识别数字牌
    │←── TARGET_LOCKED (房号 3) ────────────│  锁定目标
    │←── STATUS (2: WAIT_START) ────────────│  等待启动
    │                                       │
    │── START ─────────────────────────────→│  小车就绪，发车
    │                                       │
    │←── STATUS (3: FOLLOW_LINE) ───────────│  开始循线 + 数字识别
    │←── LINE_DATA ... ─────────────────────│  每 2 帧巡线数据
    │←── LINE_DATA ...                       │
    │   相机同时检测数字牌                      │
    │   ...小车持续循线...                     │
    │                                       │
    │   小车巡线模块检测到路口                  │
    │   此时若未收到 TURN_DECISION，直行通过    │
    │                                       │
    │←── LINE_DATA ... ─────────────────────│  巡线不中断
    │←── TURN_DECISION (LEFT, 房号3, 路口1)  │  检测到目标数字 3，方向=左转
    │←── STATUS (5: WAIT_TURN_DONE) ────────│  等待小车转弯
    │                                       │
    │   小车在下一个路口执行左转...              │
    │                                       │
    │── TURN_DONE ─────────────────────────→│  转弯完成
    │                                       │
    │←── STATUS (3: FOLLOW_LINE) ───────────│  继续循线 + 恢复数字识别
    │←── LINE_DATA ...                       │
    │    ...循环直到到达目标...                │
```

---

## 7. 注意事项

1. **START 必须等锁定目标后再发**。相机上电后状态为 CAPTURE_TARGET，把数字牌放到相机视野中，等到相机发 TARGET_LOCKED + STATUS(WAIT_START) 之后再发 START，否则 `start()` 返回 false，START 被静默忽略。

2. **目标 1~2 的方向在 START 之前到达**。MCU 收到 TARGET_LOCKED 后，紧接着会收到 TURN_DECISION。此时小车尚未出发（WAIT_START 状态），MCU 应存储此方向，在后续路口自行判断并使用。相机在 FOLLOW_LINE 期间**不再跑数字识别**，不监控路口，只负责巡线。

3. **目标 3~8 的方向在 FOLLOW_LINE 期间到达**。MCU 收到 TURN_DECISION 后，应在**下一个检测到的路口**执行该方向。如果经过路口时未收到 TURN_DECISION，直行通过。

4. **TURN_DONE 后恢复数字识别**。TURN_DONE 发出后，相机回到 FOLLOW_LINE 状态并重新开始扫描数字。下一个路口的转向方向将在再次检测到目标数字时发送。每次转弯方向由当时数字与道路中心的相对位置决定。

5. **WAIT_TURN_DONE 期间无 LINE_DATA**。转弯期间相机视角旋转，巡线数据不可靠，相机暂停发送 LINE_DATA。TURN_DONE 后恢复。

6. **VISION_HOLD 的处理**。收到此消息说明相机不确定，小车应减速/停下等待，不要继续冲。等相机恢复自信后会重新发 LINE_DATA。

7. **发送格式**。串口助手调试时务选 HEX（十六进制）发送，不要选 ASCII。选中 ASCII 会把字符的 ASCII 码发出去，相机完全不解。

8. **校验和必须算对**。校验和不通过，整帧静默丢弃，不会有任何错误提示。调试时如果"发了没反应"，先把原始字节打出来手动核对。

9. **路口检测由小车巡线模块负责**。相机不再检测路口（十字/T 字），路口的识别和通过逻辑完全交给小车端的巡线传感器（如灰度/红外阵列）。相机仅通过 TURN_DECISION 告知"在下一个路口应该往哪转"。

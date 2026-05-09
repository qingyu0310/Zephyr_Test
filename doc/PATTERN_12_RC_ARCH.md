# 点12. 遥控器协议架构：从原始字节到飞控通道

## 整体数据流

```
遥控器 RF 信号
    │
    ▼
接收机（PWM/PPM/SBUS/CRSF...）
    │
    ├── 串口 UART ──→ process_byte() ──┐
    │                                  │
    └── 电平捕获 ICU ─→ process_pulse() ──┴──→ 帧解码器
                                                     │
                                                     ▼
                                            全局数据区（共享状态）
                                              │  g_rc.version++
                                              │  g_rc.ch[]
                                              │
                     ┌────────────────────────┼────────────────────────┐
                     ▼                        ▼                        ▼
              RC_Channels::update()    AP_Mouse::update()     其他消费者
              (检查 version, 读 ch)    (检查 type, 读鼠标)
```

三层结构：

| 层 | 职责 | 谁 | 
|----|------|----|
| 物理层 | UART 收字节 / ICU 捕获电平 | HAL 驱动 |
| 协议层 | 找 sync → 组帧 → 解码 → 写全局数据 | 解码器 |
| 消费者层 | 按自己节奏读数据，判断版本和类型 | RC_Channels、各类驱动 |

## 协议层：解码器设计

### 原则

1. **一个协议一个解码器**（sync 不同、帧格式不同才分开）
2. **解码器不知道谁在用数据**——只写共享状态
3. **解码器可以高频调用**（中断中或主循环中）

### 结构

```cpp
// 解码器（每个协议一个）
static void decode_vt03(const uint8_t* buf) {
    if (buf[0] != 0xA9 || buf[1] != 0x53) return;

    // 解通道 → 写全局
    g_rc.ch[0] = ...;
    g_rc.version++;

    // 有鼠标 → 写鼠标
    g_mouse.x = ...;
    g_mouse.version++;
}

static void decode_generic(const uint8_t* buf) {
    if (buf[0] != 0xAA) return;

    // 只解通道，不解鼠标
    g_rc.ch[0] = ...;
    g_rc.version++;
    // 不碰 g_mouse，鼠标版本不更新
}
```

### 选择解码器的三种方式

**方式 A：用户配置枚举（推荐）**
```cpp
enum RemoteType { REMOTE_VT03, REMOTE_GENERIC, REMOTE_SBUS };
RemoteType g_type = REMOTE_VT03;  // 用户配一次

void on_uart_byte(uint8_t b) {
    // 按枚举走对应解析
    switch (g_type) {
    case REMOTE_VT03:    feed_vt03(b);    break;
    case REMOTE_GENERIC: feed_generic(b); break;
    case REMOTE_SBUS:    feed_sbus(b);    break;
    }
}
```
- 无探测开销
- 换遥控器要改配置

**方式 B：帧头自动探测**
```
收到字节 → 等 sync → sync=0xA9 → VT03
                    → sync=0xAA → Generic
                    → sync=0x0F → SBUS
```
- 零配置，即插即用
- 需要等 sync，第一帧之前的字节丢掉

**方式 C：多后端竞争**
```
收到字节 → 喂给所有后端
         → 谁 frame_count++ 了就是谁
```
- 最适合完全不配置、随时热插拔
- 所有后端都在跑，有额外内存和 CPU 开销

## 消费者层：各子系统读数据

### 消费者通用模式

```cpp
void SomeConsumer::update() {
    // 1. 版本没变 → 没新数据，跳过
    if (g_data.version == _last_ver) return;
    _last_ver = g_data.version;

    // 2. 类型不匹配 → 这个数据我这里不能用，跳过
    if (g_current_type != EXPECTED_TYPE) return;

    // 3. 处理数据
    process(g_data.value);
}
```

### 各子系统

| 子系统 | 订阅数据 | 检查条件 | 频率 |
|--------|---------|---------|------|
| RC_Channels | `g_rc.ch[]` | version | 50~100Hz |
| AP_Mouse | `g_mouse` | type == VT03 | 和主循环一致 |
| AP_Keyboard | `g_keyboard` | type == VT03 | 和主循环一致 |

### 不涉及的数据

Generic 遥控器没有鼠标，`AP_Mouse::update()` 检查 `type != REMOTE_VT03` 后直接 return。不会读到过时数据，也不需要手动清零或超时。

## Pull vs Push

| | Pull（消费者主动读） | Push（解码器主动发） |
|---|---|---|
| 性能 | 快（memcpy + version 检查） | 慢（间接调用 + 上下文切换） |
| 确定性 | 高（消费者控制节奏） | 低（回调不知道在哪被调） |
| 适合 | 状态数据（通道值、鼠标位置） | 事件数据（按钮按下、异常） |
| 实现 | 全局变量 + version | 回调注册 + 函数指针 |

**RC 通道是状态数据，Pull 是正确做法。**

## 不同遥控器数据差异的处理

| 差异 | 处理方式 |
|------|---------|
| 通道数不同 | 每个解码器写 `g_rc.num`，消费者按 `num` 读 |
| 数据格式不同（11bit vs 线性映射） | 各自解各自的，输出到同一个 `g_rc.ch[]` |
| VT03 有鼠标键盘 | VT03 解码器写 `g_mouse` + `g_keyboard`，其他不写 |
| 换遥控器 | 消费者检查 `g_current_type`，类型不对就跳过 |

## 完整代码示例

```cpp
// ============================================================
// 遥控器架构完整示例
// ============================================================

// --- 类型枚举 ---
enum RemoteType : uint8_t {
    REMOTE_VT03,
    REMOTE_GENERIC,
};

// --- 全局数据区 ---
struct {
    uint16_t ch[8];
    uint8_t  num;
    uint32_t version;
} g_rc;

struct {
    int16_t  x, y;
    uint8_t  l, r;
    uint32_t version;
} g_mouse;

static int16_t  g_keyboard;
static uint32_t g_keyboard_ver;
static RemoteType g_current_type;
static uint32_t   g_frame_version;

// --- 解码器（协议层）---
static void decode_vt03(const uint8_t* buf) {
    if (buf[0] != 0xA9 || buf[1] != 0x53) return;
    g_current_type = REMOTE_VT03;
    g_frame_version++;

    g_rc.ch[0] = ...;  // 解通道
    g_rc.version = g_frame_version;

    g_mouse.x = ...;   // 解鼠标
    g_mouse.version = g_frame_version;

    g_keyboard = ...;
    g_keyboard_ver = g_frame_version;
}

static void decode_generic(const uint8_t* buf) {
    if (buf[0] != 0xAA) return;
    g_current_type = REMOTE_GENERIC;
    g_frame_version++;

    for (int i = 0; i < 4; i++)
        g_rc.ch[i] = buf[i+1] * 10 + 1000;
    g_rc.version = g_frame_version;
    // 不写 g_mouse, g_keyboard
}

// --- 消费者（应用层）---
struct RC_Channels {
    void update() {
        if (g_rc.version == _last_ver) return;
        _last_ver = g_rc.version;
        // 使用 g_rc.ch[]
    }
    uint32_t _last_ver = 0;
};

struct AP_Mouse {
    void update() {
        if (g_current_type != REMOTE_VT03) return;
        if (g_mouse.version == _last_ver) return;
        _last_ver = g_mouse.version;
        // 使用 g_mouse
    }
    uint32_t _last_ver = 0;
};
```

## 参考源码

- `libraries/AP_RCProtocol/AP_RCProtocol.cpp` — 前端探测引擎
- `libraries/AP_RCProtocol/AP_RCProtocol_Backend.h` — 后端基类
- `libraries/RC_Channel/RC_Channels.cpp` — 通道消费者
- `libraries/AP_Mouse/` — 鼠标驱动（VT03 等协议使用）
- `libraries/AP_Keyboard/` — 键盘驱动

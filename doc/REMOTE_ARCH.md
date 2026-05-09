# 遥控器架构文档：语义化发布订阅

## 整体架构

```
遥控器 RF 信号
    │
    ▼
接收机 (PWM/PPM/SBUS/DR16/VT13...)
    │
    ▼ UART
UART RX ISR (物理层)
    │
    ▼
Remote::UartRxCpltCallback(buffer, len)
    │
    ▼
DataProcess(buffer, len)  ──→  协议解码器 (协议层)
    │                               │
    │    ┌──────────────────────────┤
    │    │  各协议 namespace         │
    │    │  dr16::dataprocess        │
    │    │  vt13::dataprocess        │
    │    │  vt12::dataprocess        │
    │    │                           │
    │    │  ① 解包原始字节           │
    │    │  ② 归一化通道值           │
    │    │  ③ 键盘 toggle 处理       │
    │    │  ④ 映射到语义字段         │
    │    └──────────────────────────┘
    │
    ▼
zbus_chan_pub(&pub_remote_to, &pub)  ──→   zbus 通道 (传输层)
    │
    ▼
zbus_sub_wait / zbus_chan_read  ←───  消费者 (应用层)
    │
    ├── thread::chassis::ReadRemote()    (底盘)
    ├── thread::gimbal::...              (云台)
    └── ...                              (其他)
```

### 三层职责

| 层 | 职责 | 位置 |
|----|------|------|
| 物理层 | UART 收字节，触发回调 | `Remote` 类 + HAL 驱动 |
| 协议层 | 解包原始字节 → 归一化 → 映射到语义字段 → zbus 发布 | `remote.cpp` 各 protocol namespace |
| 应用层 | 订阅 zbus，读语义字段，用于控制逻辑 | `trd_*.cpp` 各控制线程 |

## 核心设计理念

### 1. 语义接口（topic 层定死）

`topic/remote_to/remote_to.hpp` 定义了消费者看到的接口——语义字段，与具体遥控器协议无关：

```cpp
struct RemoteData {
    float chassisx;           // 底盘前进（[-1, 1]）
    float chassisy;           // 底盘横向（[-1, 1]）
    float yaw;                // 底盘旋转（[-1, 1]）
    float pitch;              // 俯仰（[-1, 1]）
    ChassisMode chassis_mode; // 底盘模式
    StartMode shoot_mode;     // 射击模式
    StartMode reload_mode;    // 换弹模式
    // mouse, keyboard...
};
```

消费者只依赖 `topic::remote_to::RemoteData`，不关心背后是 DR16 还是 VT13。

### 2. 共享类型定义在 topic 层

`Channel`、`Mouse`、`Keyboard` 这些跨协议通用的数据结构定义在 `topic/remote_to/remote_to.hpp` 中：

```cpp
struct Channel {
    float chassisx, chassisy, pitch, yaw;
};

struct Mouse {
    float x, y, z;
    float left, right;
};

union Keyboard {
    uint16_t all;
    struct { uint8_t w : 1; s : 1; a : 1; d : 1; /* ... */ };
};
```

协议解码器中通过 `using` 别名引用：

```cpp
using Channel  = topic::remote_to::Channel;
using Mouse    = topic::remote_to::Mouse;
using Keyboard = topic::remote_to::Keyboard;
```

### 3. 每种协议一个解码器

每个协议在 `remote.cpp` 中有一个独立的 namespace，包含：

- `RawData` — 协议原始数据位域结构（如有）
- `Switch` — 协议特有的开关定义
- `OutputData` — 解码后的中间表示（局部变量 `od`，组合 `Channel`/`Switch`/`Mouse`/`Keyboard`）
- `KeyboardState<Keyboard>` — 键盘 toggle 状态机
- `dataprocess()` — 主解码函数

解码器内部使用协议特有的 `Switch`、`RawData`，配合共享的 `Channel`/`Mouse`/`Keyboard`，只在最后映射到 `RemoteData` 语义字段。

### 4. 统一 zbus 通道

所有协议往同一个 zbus 通道 `pub_remote_to` 发数据，类型都是 `topic::remote_to::RemoteData`。

消费者只需要订阅 `sub_remote_to`，读取 `RemoteData`，按语义字段使用。

```cpp
static topic::remote_to::RemoteData msg{};
static const zbus_channel* chan = nullptr;

zbus_sub_wait(&sub_remote_to, &chan, K_NO_WAIT);
if (chan) {
    zbus_chan_read(chan, &msg, K_NO_WAIT);
    g_vx = msg.chassisx * KMaxMoveVelocity;
    // ...
}
```

### 5. 版本号用于检测新数据

`RemoteData::version` 每次发布递增，消费者可以用它判断是否有新数据到来，避免重复处理。

## 优缺点

### 优点

**上层彻底解耦**
消费者只看 `chassisx`、`chassisy` 等语义字段，完全不关心底层是 DR16、VT13 还是其他协议。更换遥控器协议不需要改任何消费者代码。

**统一数据接口**
所有协议发同一个 `RemoteData` 类型到同一个 zbus 通道。新消费者只需要知道 `topic/remote_to/RemoteData`，不需要了解各协议细节。

**共享类型免重复**
`Channel`、`Mouse`、`Keyboard` 在 topic 层统一定义，所有协议共享，不再各自定义同类结构。

**协议内部隔离**
每个协议的 decoder 在自己的 namespace 内独立实现，内部类型（`RawData`、`Switch`）互不干扰。

**可扩展**
添加新协议只需要加一个 namespace、一个 dataprocess 函数、一个 `GetProcessFunc` case。

**未来可自动探测**
通过 `pub.type` 字段，消费者可以在运行时判断数据来源。当需要同时支持多种协议（同一物理通道不同遥控器热插拔）时，可以在 topic 层添加探测逻辑，消费者通过 type 区分处理。

### 缺点

**违反开闭原则（Open/Closed Principle）**
添加新协议必须修改 `GetProcessFunc()` 的 switch 语句，无法做到"对扩展开放、对修改封闭"。如果未来支持动态注册 decoder，可以解决。

**添加语义字段需要改所有 decoder**
当 `RemoteData` 增加一个新字段（例如 `shoot_mode`），每个已有的协议 decoder 都需要填充它。遗漏会导致该字段在该协议下永远是默认值。

**部分协议无法填充某些字段**
不是所有协议都有鼠标和键盘。对于没有鼠标的协议，`mouse_*` 字段永远是 0。消费者如果需要使用鼠标，必须通过 `pub.type` 判断当前协议是否支持。

**编译期无法校验数据完备性**
消费者无法在编译期断言"我需要的字段一定会被填充"。如果某个协议 decoder 忘记填充 `chassis_mode`，编译器不会报错，运行时 chassis_mode 只是默认值。

**共享通道的类型安全问题**
所有协议共用同一个 `RemoteData` 类型，zbus 通道无法用类型系统区分不同协议的数据。如果两个协议的同一语义字段含义不同，这种差异只能在 decoder 内部解决，类型系统不会帮忙检查。

## 数据流详解

### DR16 协议示例

```
UART buffer: [ch0:11bit][ch1:11bit][ch2:11bit][ch3:11bit][sw:4bit][mouse:6byte][kb:1byte]
    │
    ▼
dr16::dataprocess()
    │
    ├── 通道解包: 11bit × 4 → uint16_t ch0~ch3
    ├── 开关解析: sw1/sw2
    ├── 鼠标解码: dx/dy/dz, left/right
    ├── 归一化: normChannel(ch0, 1024, 1684) → [-1, 1]
    ├── 键盘 toggle: KeyboardState::Process()
    │
    ├── 映射到 RemoteData:
    │   od.ch.chassisx  → pub.chassisx
    │   od.ch.chassisy  → pub.chassisy
    │   od.ch.yaw       → pub.yaw
    │   od.ch.pitch     → pub.pitch
    │   od.sw.sw1       → pub.chassis_mode  (switch 映射)
    │   od.mouse.*      → pub.mouse_*
    │   od.keyboard     → pub.keyboard      (整个 Keyboard union)
    │
    └── zbus_chan_pub(&pub_remote_to, &pub)
```

### 归一化

**通道归一化** (normChannel)：中心对称映射

```
normChannel(v, center, max) = (v - center) / (max - center)
```

- DR16: center=1024, max=1684，输出 [-1, 1]
- 回中时 v=center → 0.0
- 推到边时 v=max → 1.0, v=min → -1.0 (min = center - (max - center))

**鼠标归一化** (normMouse)：

```
normMouse(v, scale) = v * scale * (1/32767), clamp to [-1, 1]
```

### 键盘 Toggle 模式

`KeyboardState<Keyboard>` 模板实现键盘按键的 toggle/非 toggle 模式：

- `keyboard_mode` 的 bit 位标记哪些按键是 toggle 模式（1=toggle, 0=普通）
- toggle 按键：按一次切换状态，再按一次恢复
- 普通按键：按下为 1，松开为 0
- 上升沿检测：`trigger = current_raw.all & (~last_raw_all)`

## 如何添加新的遥控器协议

### 步骤

#### 1. 加枚举值

在 `modules/remotes/remote.hpp` 的 `RemoteType` 中添加新类型：

```cpp
enum class RemoteType
{
    DR16 = 0,
    VT12,
    VT13,
    NewProto,   // 新增
    None,
};
```

#### 2. 实现解码器

在 `modules/remotes/remote.cpp` 中添加新的 namespace。共享类型（`Channel`、`Mouse`、`Keyboard`）已在 topic 层定义，直接用：

```cpp
namespace newproto {

struct Switch { /* ... */ };   // 协议特有（必须自己定义）
// Channel, Mouse, Keyboard 是共享的，不需要定义

struct OutputData { Channel ch; Switch sw; Mouse mouse; Keyboard keyboard; };

static KeyboardState<Keyboard> keyboard_state_{};

static void dataprocess(uint8_t* buffer, uint8_t len)
{
    if (/* 帧头检查 */) return;

    OutputData od{};

    /* 解包 + 归一化 */
    od.ch.chassisx = normChannel(/* ... */);
    od.ch.chassisy = normChannel(/* ... */);
    od.ch.yaw      = normChannel(/* ... */);
    od.ch.pitch    = normChannel(/* ... */);

    /* 开关 + 鼠标 + 键盘 */

    static topic::remote_to::RemoteData pub{};

    pub.chassisx = od.ch.chassisx;
    pub.chassisy = od.ch.chassisy;
    pub.yaw      = od.ch.yaw;
    pub.pitch    = od.ch.pitch;
    pub.chassis_mode = /* 映射 */;
    pub.shoot_mode   = /* 映射 */;
    pub.reload_mode  = /* 映射 */;
    pub.mouse_x = od.mouse.x;
    pub.mouse_y = od.mouse.y;
    pub.mouse_z = od.mouse.z;
    pub.mouse_left  = od.mouse.left;
    pub.mouse_right = od.mouse.right;
    pub.keyboard = od.keyboard;
    pub.type = static_cast<uint8_t>(RemoteType::NewProto);
    pub.version++;
    zbus_chan_pub(&pub_remote_to, &pub, K_MSEC(1));
}

} // namespace newproto
```

#### 3. 注册到工厂

在 `thread::remote::Remote::GetProcessFunc()` 中添加 case：

```cpp
case RemoteType::NewProto:
    DataProcess = newproto::dataprocess;
    break;
```

#### 4. 配置遥控器类型

初始化时指定类型：

```cpp
remote.Init(RemoteType::NewProto);
```

### 新协议模板

```cpp
namespace newproto {

struct Switch { /* ... */ };

struct OutputData { Channel ch; Switch sw; Mouse mouse; Keyboard keyboard; };

static KeyboardState<Keyboard> keyboard_state_{};

static void dataprocess(uint8_t* buffer, uint8_t len)
{
    // if (/* 帧头/长度检查 */) return;

    OutputData od{};

    /* 通道解包 + 归一化 */
    od.ch.chassisx = normChannel(/* ... */);
    od.ch.chassisy = normChannel(/* ... */);
    od.ch.yaw      = normChannel(/* ... */);
    od.ch.pitch    = normChannel(/* ... */);

    /* 开关解析 + 鼠标 + 键盘 */

    static topic::remote_to::RemoteData pub{};
    pub.chassisx = od.ch.chassisx;
    pub.chassisy = od.ch.chassisy;
    pub.yaw      = od.ch.yaw;
    pub.pitch    = od.ch.pitch;
    pub.chassis_mode = /* 按协议开关映射 */;
    pub.mouse_x  = od.mouse.x;
    pub.mouse_y  = od.mouse.y;
    pub.mouse_z  = od.mouse.z;
    pub.mouse_left  = od.mouse.left;
    pub.mouse_right = od.mouse.right;
    pub.keyboard = od.keyboard;
    pub.type = static_cast<uint8_t>(RemoteType::NewProto);
    pub.version++;
    zbus_chan_pub(&pub_remote_to, &pub, K_MSEC(1));
}

} // namespace newproto
```

关键点：
- `Switch` 是协议特有的，必须自己定义
- `Channel`/`Mouse`/`Keyboard` 是共享的，已在 `topic::remote_to` 定义
- `Keyboard` 支持位域访问（`od.keyboard.w`）和原始值访问（`od.keyboard.all`）
- `pub.keyboard` 直接赋整个 union：`pub.keyboard = od.keyboard;`

## 如何修改映射

### 修改摇杆通道 → 语义字段映射

```cpp
// 默认: ch0→chassisx, ch1→chassisy, ch2→yaw, ch3→pitch
pub.chassisx = normChannel(ch1, kCenter, kMax);  // 改为 ch1 控制前进
pub.chassisy = normChannel(ch0, kCenter, kMax);  // ch0 控制横向
```

### 修改开关 → 底盘模式映射

```cpp
switch (od.sw.sw1)
{
    case (SWITCH_UP):
        pub.chassis_mode = topic::remote_to::ChassisMode::Spin;
        break;
    case (SWITCH_MID):
        pub.chassis_mode = topic::remote_to::ChassisMode::Normal;
        break;
    case (SWITCH_DOWN):
        pub.chassis_mode = topic::remote_to::ChassisMode::Follow;
        break;
}
```

可以换成 sw2、或者组合键、或者键盘按键：

```cpp
// 键盘触发模式切换
if (od.keyboard.r) pub.chassis_mode = topic::remote_to::ChassisMode::Spin;
```

### 修改鼠标灵敏度

调整 `kMouseSensX/Y/Z` 常量：

```cpp
constexpr float kMouseSensX = 30.0f;
constexpr float kMouseSensY = 2.0f;
constexpr float kMouseSensZ = 1.0f;
```

### 修改通道归一化参数

```cpp
constexpr int16_t kCenter = 1024;  // 摇杆回中值
constexpr int16_t kMax    = 1684;  // 摇杆最大原始值
```

### 修改键盘按键定义

在 `topic/remote_to/remote_to.hpp` 的 `Keyboard` union 中调整位域：

```cpp
union Keyboard
{
    uint16_t all;
    struct {
        uint8_t w : 1;
        uint8_t s : 1;
        // ...
    };
};
```

Bit 位置对应遥控器键盘矩阵的按键索引。⚠ **Keyboard 是所有协议共享的**，修改会影响所有协议。

### 添加新的语义字段

1. **topic 层**：在 `remote_to.hpp` 的 `RemoteData` 中添加字段
2. **协议 decoder**：在每个协议的 dataprocess 中填充该字段
3. **消费者**：在新字段的消费者线程中读取使用

### 添加新的模式枚举

在 `remote_to.hpp` 中添加：

```cpp
enum class NewMode : uint8_t
{
    OptionA = 0,
    OptionB,
};
```

在 `RemoteData` 中添加成员。每个协议 decoder 根据自己的开关/按键映射填充。

## 文件结构

```
modules/remotes/
├── remote.hpp          Remote 类 + RemoteType 枚举
├── remote.cpp          各协议解码器 (dr16/vt12/vt13)

topic/remote_to/
├── remote_to.hpp       RemoteData + 共享类型 (Channel/Mouse/Keyboard) + 模式枚举 + zbus 声明
├── remote_to.cpp       zbus 通道定义

projects/thread/chassis/
├── trd_chassis.cpp     消费者示例（底盘控制）
```

### 类型归属

| 类型 | 定义位置 | 是否共享 |
|------|---------|---------|
| `RemoteType` | `remote.hpp` | 全局枚举 |
| `RemoteData` | `remote_to.hpp` | 所有协议共用 |
| `Channel` | `remote_to.hpp` | 所有协议共用（`using`） |
| `Mouse` | `remote_to.hpp` | 所有协议共用（`using`） |
| `Keyboard` | `remote_to.hpp` | 所有协议共用（`using`） |
| `RawData` | `remote.cpp` 各 namespace | 协议特有 |
| `Switch` | `remote.cpp` 各 namespace | 协议特有 |
| `KeyboardState<Keyboard>` | `remote.cpp` 匿名 namespace | 模板，各协议实例化 |

## 术语

| 术语 | 说明 |
|------|------|
| 语义字段 | topic 层定义的、与协议无关的控制字段（chassisx、chassisy 等） |
| 协议解码器 | 将原始字节解包、归一化、映射到语义字段的函数 |
| 归一化 | 将原始值（如 0~2047）映射到 [-1, 1] 范围 |
| 键盘 toggle | 按键的切换模式：按一次保持，再按一次释放 |
| 消费者 | 订阅遥控器数据并用于控制的线程 |

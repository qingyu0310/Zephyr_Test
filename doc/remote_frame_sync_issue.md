# 遥控器帧同步问题分析

## 现象

烧录完成后，遥控器解析的值有概率出现错位（看似随机），重启后可能恢复也可能不恢复。

## 数据流

```
UART DMA 接收 → IDLE 中断 → UART_RX_RDY → 环缓冲区 → sem_give
                                                            ↓
                                                  Remote::Task() 唤醒
                                                            ↓
                                                  uart_.Read() → frame_buf_
                                                            ↓
                                                  frame_pos_ >= 18? → dr16::dataprocess()
```

## 根因

**无帧同步机制，定长截断信任 IDLE 边界。**

### 正常情况

```
遥控器发送:  [帧0][空闲>1bit][帧1][空闲>1bit][帧2]...
UART 使能:         ↑ (使能在帧间)
IDLE 触发:              ↑         ↑         ↑
UART_RX_RDY:         [帧0]      [帧1]      [帧2]    ← 每帧边界正确
```

### 启动时错位

```
遥控器发送:  [帧0 字节0..17][帧1 字节0..17][帧2 字节0..17]...
UART 使能:             ↑ (使能在帧中间)
IDLE 触发:                     ↑              ↑
UART_RX_RDY:               [字节8..17]      [字节0..17]   ← 第一段只有10字节
                                         ↓
                                  frame_buf_ = [8..17] | [0..17] = 28字节
                                  取前18: [8..17][0..7] → 错位拼接 → 解析出垃圾值
                                  frame_buf_ 剩余 [0..9] = 10字节
                                  ← 之后每帧都偏移10字节，永久错位
```

### 为什么 IDLE 不能在帧开始处同步

IDLE 触发条件是** RX 线空闲 > 1 bit 时间**。当 UART 在**帧中间**被使能时：

1. DMA 立即开始接收后续字节
2. 当前帧的最后字节到达后，RX 线空闲 → IDLE 触发
3. 驱动 flush DMA，产生 UART_RX_RDY
4. 但这段数据从当前帧的中间开始，不是帧头

硬件只能告诉你"线空闲了"，不能告诉你"这是一帧的开始"。

### 为什么概率性出现

取决于 UART 使能时刻与遥控器发送帧的相位关系：

- **UART 在帧间使能** → 正确
- **UART 在帧中间使能** → 错位

烧录完成、系统初始化的时间每次不同，因此是概率性的。

## 为什么后续不能自恢复

当前 `Task()` 的处理逻辑：

```cpp
if (frame_pos_ >= frame_size_) {
    proto_.func(frame_buf_, frame_size_, pub_);  // 取前 frame_size_ 字节
    memmove(frame_buf_, frame_buf_ + frame_size_, rem);  // 移位剩余
    frame_pos_ = rem;
}
```

每次固定取 `frame_size_`（18）字节，不检查数据合法性。一旦错位，frame_buf_ 的起始偏移永远回不去，除非手动复位 UART DMA。

## 验证方法

在 `Task()` 添加一次性打印，观察第一包数据的字节数和内容：

```cpp
static bool first_frame = true;
...
uint16_t n = uart_.Read(tmp, sizeof(tmp));
if (first_frame && n > 0) {
    first_frame = false;
    printk("first n=%u\n", n);
    for (uint8_t i = 0; i < n; i++) printk("%02x ", tmp[i]);
    printk("\n");
}
```

预期结果：

| 现象 | 第一包长度 | 含义 |
|------|-----------|------|
| 正常 | 18 | UART 在帧间使能，IDLE 边界正确 |
| 错位 | 1-17 | UART 在帧中间使能，第一包是后半帧 |

## 可能的修复方向

### 方案 1：帧内合法性校验 + 滑动窗口

处理前验证数据，如果不可能则逐字节移位直到找到合法帧：

```cpp
while (frame_pos_ >= frame_size_) {
    if (ValidateFrame(frame_buf_, frame_size_)) {
        proto_.func(frame_buf_, frame_size_, pub_);
        memmove(..., frame_size_);
        frame_pos_ -= frame_size_;
    } else {
        // 挪 1 字节继续试
        memmove(..., 1);
        frame_pos_--;
    }
}
```

对于 DR16，验证条件：
- 4 个摇杆通道值均为 11-bit（≤ 0x7FF）
- 开关值只能是 0-3
- 摇杆通道不全部为 2047（全满，极不可能）

### 方案 2：启动时丢弃第一帧

第一次收到数据后，先丢一帧再开始处理，因为第一帧有概率是半帧。

但第一帧可能是完整的（UART 在帧间使能），丢弃浪费一帧（14ms），无大碍。

### 方案 3：First Frame Ignore + Validation

结合方案 1 和 2 最稳妥：启动时标记未同步，找到连续 N 帧合法后再认为已同步。


# 待修改清单

---

## 一、必须改（正确性问题）

### 1.1 遥控器解码在 ISR 上下文执行浮点运算

**位置**: `modules/remotes/remote.cpp` — `UartRxCpltCallback` → `DataProcess` → `dr16::dataprocess` 等

**问题**: UART IRQ callback 里做了协议解析、浮点归一化、键盘状态机、zbus publish。IRQ 上下文中做浮点运算是禁忌：
- 浮点寄存器保存/恢复开销大
- 某些 RISC-V 核在 ISR 中默认禁用 FPU，触发异常
- zbus publish 可能在 ISR 里唤醒高优先级线程，导致非预期的优先级反转

**方案**: IRQ callback 只做 `k_msgq_put` 把原始数据塞到 message queue，解码线程从 msgq 取出再处理。或者利用 UART 的 DMA + 双缓冲，IRQ 只翻转缓冲区所有权。

---

## 二、建议改（设计/可维护性）

### 2.3 zbus 轮询效率低

**位置**: `projects/thread/chassis/trd_chassis.cpp:120`

**问题**: chassis 线程每 1ms 用 `K_NO_WAIT` 轮询遥控器数据，实际遥控器约 50ms 更新一次，99% 轮询空转。

**方案**:
- 方案 A：`zbus_sub_wait` 用 `K_MSEC(50)` 超时
- 方案 B：remote 发布时释放信号量，chassis 事件驱动

---

### 2.4 遥控器断连检测阈值未标注 TODO

**位置**: `modules/remotes/remote.hpp:77` — `k_msleep(50)`

**问题**: 50ms 周期对于遥控器信号丢失检测偏长。

**方案**: 加 TODO 注释，预期实物测试后调至 10-20ms。

---

## 三、可做可不做（风格/细节）

### 3.1 naming 不统一

方法 `CamelCase` / 变量 `snake_case` / 常量 `kCamelCase` / 全局 `g_snake_case`，但实际代码中有混用。涉及面大，可逐步收敛。

### 3.2 头文件内联实现

`lpf.hpp`、`timer.hpp` 等实现在头文件里。模板必须内联，非模板可分离。

### 3.3 doc 目录清理

当前 `doc/` 混合了架构文档和开发日志。建议把 `给ai上的第一课.md` 这类过程记录移到私人目录。

---

## 四、已知但初版合理的（不修）

| 问题 | 原因 |
|------|------|
| PID 参数全是 P-only (ki=0, kd=0) | 无实物可调，初版留框架 |
| CAN bus-off 恢复未实现 | 可后续迭代加入 |
| 功率模型初始值可能不准 | RLS 在线收敛可补偿 |
| 各模块之间缺少 timeout/health check | 单板验证阶段可后续加入 |

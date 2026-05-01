# MotorCtrl — 电机控制框架

基于 Zephyr RTOS 的电机控制框架，目标架构为解耦的三层模型：**中断响应** → **算法控制** → **总线发送**，层间通过 Zephyr **zbus** 发布订阅通信。

---

## 架构总览

```
┌─────────────────────────────────────────────────────────┐
│                       zbus                               │
│  ┌──────────────────┐      ┌──────────────────┐         │
│  │    MotorFB       │      │    MotorCMD      │         │
│  │  (反馈数据通道)   │      │  (控制指令通道)   │         │
│  └───────┬──────────┘      └────────┬─────────┘         │
│          │                         │                     │
│  ┌───────┴──────────┐      ┌───────┴─────────┐          │
│  │   C6xx RX        │      │   Chassis       │          │
│  │  (中断回调)       │      │  (1ms 控制循环)  │          │
│  │  解析 CAN 帧      │      │  PID + PowerCtrl│          │
│  │  发布 MotorFB     │      │  发布 MotorCMD  │          │
│  └──────────────────┘      └───────┬─────────┘          │
│                                    │                     │
│                            ┌───────┴─────────┐          │
│                            │   CAN_TX 线程    │          │
│                            │  订阅 MotorCMD   │          │
│                            │  can_send() 发送  │          │
│                            └─────────────────┘          │
└─────────────────────────────────────────────────────────┘
```

## 数据流

### ① 接收路径（中断上下文）

```
CAN 中断 ──→ mcan4_rx_callback_func
                └── C6xx::CanCpltRxCallback()
                      ├── 解析字节 → 角度 / 转速 / 电流 / 温度
                      └── zbus_pub(&MotorFB, ...)
```

CAN 接收中断中直接调用电机解析回调，解析后的数据通过 zbus 发布到 `MotorFB` 通道，不阻塞中断。

### ② 控制循环（Chassis 线程，1ms 周期）

```
Chassis 线程每 1ms:
  ├── zbus_sub(&MotorFB)       ← 获取各电机当前状态
  ├── 低通滤波电流值
  ├── PID 计算（位置环/速度环）
  ├── PowerCtrl 功率限制
  │   ├── 功率模型预测 P = K1τ² + K2ω² + τω + K3
  │   ├── RLS 在线辨识（可选）
  │   ├── 隶属度功率分配
  │   └── 超限 → 解二次方程求限制电流
  └── zbus_pub(&MotorCMD, ...) ← 发布控制指令
```

### ③ 发送路径（CAN_TX 线程）

```
CAN_TX 线程:
  ├── zbus_sub(&MotorCMD)      ← 等待控制指令
  ├── 组 CAN 帧
  └── can_send()               ← 发送到总线
```

## 线程模型

| 线程 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| **Chassis** | 高 | 1ms | PID + PowerCtrl 控制算法 |
| **CAN_TX** | 中 | 阻塞 | 订阅 MotorCMD → 发送 CAN 帧 |
| **GPIO** (已有) | 低 | 按需 | LED 状态指示 |

CAN 接收不占线程，全在中断回调中完成。

## 发布订阅（zbus）

使用 Zephyr **zbus** 作为层间通信，两个消息通道：

### MotorFB —— 反馈通道

```c
/* 每个电机的反馈数据 */
struct motor_fb {
    float angle;         // 当前角度 (rad)
    float omega;         // 角速度 (rad/s)
    float current;       // 当前电流 (A)
    float velocity;      // 转速 (rpm)
    float temperature;   // 温度 (°C)
};

ZBUS_CHANNEL_DEFINE(MotorFB,                     // 通道名
                    struct motor_fb,             // 消息类型
                    NULL,                        // 初始值
                    NULL,                        // 监听者
                    ZBUS_OBSERVERS_EMPTY);       // 观察者列表
```

发布方：C6xx RX 回调（中断）
订阅方：Chassis 线程

### MotorCMD —— 控制通道

```c
/* 每个电机的控制指令 */
struct motor_cmd {
    float current_set;   // 目标电流 (A)
};

ZBUS_CHANNEL_DEFINE(MotorCMD,
                    struct motor_cmd,
                    NULL,
                    NULL,
                    ZBUS_OBSERVERS_EMPTY);
```

发布方：Chassis 线程
订阅方：CAN_TX 线程

## 算法管线

### PID 控制器

位置式 PID，支持：

- 微分先行（避免设定值跳变引起微分冲击）
- 变速积分（误差大时减弱积分）
- 积分分离（大误差时清零防饱和）
- D 项低通滤波
- 角度模式（-π~π 劣弧处理）

```cpp
alg::pid::Pid pid({.kp = 3.0f, .ki = 0.1f, .kd = 0.05f, .outMax = 60.0f});
float out = pid.Calc(target, now);
```

### 功率控制器

直流电机功率模型，限制底盘总功率不超过预算。

```
P_in = K1·τ² + K2·ω² + τ·ω + K3
```

- **K1** / **K2**：RLS 在线辨识（铜损/铁损系数）
- **K3**：固定常数偏置
- **RLS**：递归最小二乘，遗忘因子 λ = 0.99999
- **隶属度分配**：综合 PID 误差 + 当前功率动态分配电机功率上限
- **受限力矩**：超限时解一元二次方程求限制电流

```cpp
alg::power_ctrl::PowerCtrl ctrl({
    .motorCount = 4,
    .k3 = 3.0f,
    .errUpper = 50.0f,
    .errLower = 0.01f,
});
ctrl.SetMotorData(i, current, velocity, pidErr);
ctrl.Predict();
ctrl.Allocate(powerBudget);
float limited = ctrl.GetLimitedCurrent(i);
```

### 算法管线完整链路

```
原始数据 ─→ 低通滤波 ─→ 功率预测 ─→ RLS 辨识 ─→ 隶属度分配 ─→ 力矩求解
   │           │            │            │            │            │
   │      alg::filter   PowerCtrl   alg::rls   PowerCtrl   PowerCtrl
   │     ::LowPassFilter           ::RLS
   │
   └── PID（位置环/速度环）
       alg::pid::Pid
```

## 电机支持

### DjiC6xx（M3508）

当前实现的电机驱动，基于 CAN 通信：

- 接收：编码器值 → 角度 / 转速 / 电流 / 温度
- 转矩常数：4.577e-5 N·m/A
- 减速比：3591/187（默认）

```cpp
DjiC6xx motor;
motor.Init({.tx_id = 0x200, .rx_id = 0x201});
// 中断回调自动解析
float angle = motor.GetNowAngle();
float current = motor.GetNowCurrent();
```

### 扩展接口

框架预留接口，后续可通过实现统一的 MotorBase 或 duck-typing 方式支持其他电机类型（M2006、DM 系列等）。

## 构建与运行

### 环境要求

- Zephyr SDK 0.16.8
- Zephyr RTOS 3.7.0
- West 1.5.0+
- sdk_glue（板级支持包）
- hpm_sdk（HPMicro SDK）

### 编译

```bash
cd blinky
west build -b hpm6e00evk -p
```

### 烧录

```bash
west flash
```

### Zbus 配置

在 `prj.conf` 中启用 zbus：

```conf
CONFIG_ZBUS=y
```

## 目录结构

```
MotorCtrl/
├── CMakeLists.txt              # 根构建文件
├── Kconfig                     # 根 Kconfig
├── prj.conf                    # 项目公共配置
├── src/
│   └── main.c                  # 主入口
├── projects/                   # 项目应用层
│   ├── apps/
│   │   ├── Init.cpp/h
│   │   └── System_startup.cpp  # 系统初始化 + 线程启动
│   └── thread/
│       ├── thread.hpp
│       └── gpio/               # GPIO 线程
├── drivers/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── communication/
│   │   └── can/
│   │       └── can.hpp         # CAN 总线驱动
│   └── device/
│       └── gpio/               # GPIO 驱动
├── algorithm/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── controller/
│   │   ├── pid/                # PID 控制器
│   │   ├── timer/              # 软件定时器
│   │   └── power_ctrl/         # 功率控制器
│   ├── filter/
│   │   └── lpf/                # 一阶低通滤波器
│   └── identify/
│       └── rls/                # RLS 在线参数辨识
└── modules/
    ├── CMakeLists.txt
    ├── Kconfig
    └── motor/
        └── dji/
            └── dji_c6xx.hpp    # DJI 电机驱动

## 关键头文件

| 路径 | 说明 |
|------|------|
| algorithm/controller/pid/pid.hpp | PID 控制器 |
| algorithm/controller/power_ctrl/power_ctrl.hpp | 功率控制器 |
| algorithm/controller/timer/timer.hpp | 软件定时器 |
| algorithm/filter/lpf/lpf.hpp | 低通滤波器 |
| algorithm/identify/rls/rls.hpp | RLS 在线辨识 |
| modules/motor/dji/dji_c6xx.hpp | DJI 电机驱动 |
| drivers/communication/can/can.hpp | CAN 总线驱动 |
```

## 电源与控制优先级

功率分配策略：**舵向优先 80%，剩余给行进**，按需动态分配。

```c
float steerNeed   = steerCtrl.GetTotalPower();
float steerCap    = totalBudget * 0.8f;
float steerBudget = (steerNeed < steerCap) ? steerNeed : steerCap;
float driveBudget = totalBudget - steerBudget;
```

- 舵向电机要多少给多少，但不超过总预算的 80%
- 剩余功率全部给行进电机
- 舵向不需要功率时（不转向），所有功率自动归行进

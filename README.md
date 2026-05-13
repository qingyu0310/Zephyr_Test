# Blinky — 机器人底盘控制

基于 Zephyr RTOS 的舵轮底盘控制固件。

## 架构

```
Remote (UART DMA) ──→ zbus ──→ Chassis (1ms) ──→ zbus ──→ CAN_TX ──→ 电机
     ↓                          ↓                           ↓
  协议解析                   运动学+PID                   CAN 帧发送
  DR16/VT12/VT13            功率分配                      DJI C6xx
```

## 数据流

### 遥控接收（Remote 线程）

```
UART DMA → 环缓冲区 → k_sem_give()
                        ↓
            Remote::Task() 唤醒
              ↓ uart_.Read()
            frame_buf_ 累积
              ↓ 帧长足够
            dr16::dataprocess()
              ↓ 解包 → 归一化
            zbus_chan_pub(&pub_remote_to)
```

### 底盘控制（Chassis 线程，1ms 周期）

```
zbus_sub(&sub_remote_to)    读取遥控器数据
    ↓
UpdateTarget()              逆向运动学 + 优劣弧
    ↓
ControlCalculate()          PID 串联：角度→力矩 / 速度→力矩
    ↓
PowerAlloc()                功率预测 + 分配（转向组优先）
    ↓
FramePublish()              zbus_chan_pub → can_tx
```

### CAN 发送（CAN_TX 线程）

```
zbus_sub(&sub_chassis_to_can)
    ↓
can1.Send()                 发送电流指令到电机
```

## 线程

| 线程 | 周期 | 职责 |
|------|------|------|
| Remote | 信号量唤醒 | UART 接收 + 遥控协议解析 + zbus 发布 |
| Chassis | 1ms | 运动学 + PID + 功率控制 |
| CAN_TX | 阻塞 | zbus → CAN 帧发送 |
| GPIO | 1s | LED 翻转指示运行状态 |

## 协议支持

| 协议 | 帧长 | 校验 | 说明 |
|------|------|------|------|
| DR16 | 18B | 通道值 11-bit + 开关 0-3 | 主力遥控器 |
| VT12 | 16B | 无 | 预留 |
| VT13 | 21B | 帧头 0xA9 0x53 | 预留 |

帧同步：启动时帧错位 → 逐字节滑动窗口校验 → 自动对齐。

## 功率控制

直流电机功率模型：`P = K₁τ² + K₂ω² + τω + K₃`

- RLS 在线辨识 K₁/K₂（可选，需功率计硬件）
- 隶属度分配：综合 PID 误差 + 当前功率动态分配
- 超限时解二次方程求限制电流

## 构建

```bash
# 项目使用自定义板级配置，通过 BOARD_CFG 指定
west build -b <board> -- -DBOARD_CFG=<config>
```

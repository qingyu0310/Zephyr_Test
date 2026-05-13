# Dm 未完成项

## 1. 掉线检测

```cpp
void AlivePeriodElapsedCallback();
// 需要: flag_, pre_flag_（计数值）
// 每 N 次心跳检查 flag_ == pre_flag_ → 失能；否则保持使能
```

## 2. 周期发送 + 自动状态管理

```cpp
void SendPeriodElapsedCallback();
// now_err_ 判断:
//   Enable   → CtrlData() + CAN发送
//   Disable  → 自动使能
//   其他错误  → 自动清错
```

## 3. 成员变量

| 变量 | 用途 |
|------|------|
| `motor_dm_status_` | 整体状态 ENABLE/DISABLE |
| `flag_`, `pre_flag_` | 掉线检测计数 |
| `alive_heart_` | 心跳计数器 |

## 4. 剩余 Get/Set API

| API | 对应 |
|-----|------|
| `GetAngleMax / GetOmegaMax / GetTorqueMax` | cfg 量程 |
| `GetStatus` | motor_dm_status_ |
| `GetControlMethod` | cfg_.ctrl_met |
| `GetKp / GetKd` | cfg_.kp / cfg_.kd |
| `SetKp / SetKd` | cfg_.kp / cfg_.kd |

注：`GetControlAngle/Omega/Torque` = `GetTargetRad/Vel/Tor`，已在第2轮补齐。

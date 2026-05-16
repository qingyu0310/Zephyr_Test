# NN 底盘控制计划（方案 B · 行进电机）

## Context

用神经网络替代底盘**行进电机**（drive motor）的完整 PID 控制链（`drive_velocity` → `drive_torque` → 电流）。转向电机（steer）保持不变。

底盘现有 8 个 PID（每个轮 4 个），全部 P-only。控制周期 1ms。

---

## 现状

行进电机当前控制流（每个轮）：
```
目标速度 → [velocity PID] → 转矩参考 → [torque PID] → 电流值 → CAN
```

NN 替代这两级 PID，直接输出电流：
```
目标速度 + 当前速度 + 当前转矩 → [NN] → 电流值
```

---

## 网络设计

**输入（3 维）：** `[目标速度, 当前速度, 当前转矩]`
- 目标速度：来自 `chassis_velocity * g_k_factor[wi]`，范围 ~±0.5 m/s
- 当前速度：来自 `snap.velocity`，电机反馈
- 当前转矩：来自 `snap.torque`，电机反馈

**输出（1 维）：** `[电流值]`
- tanh 输出 -1~1，板子上通过 `kCurrentScale = 16384.0 / 20.0` 映射到 CAN 帧

**结构：** 16→16→1，跟 sin 模型一样

---

## 实施步骤

### Step 1: PC 训练脚本

`scripts/train_chassis_nn.py`

在 PC 上仿真电机模型（二阶系统），模拟不同速度→转矩→电流的映射关系。跑不同组合收集数据。

### Step 2: 导出模型

训练 → `.tflite` → C 数组 → `models/chassis_nn/`

### Step 3: Kconfig + CMake 配置

`algorithm/tflm/Kconfig` 加 `TFLM_MODEL_CHASSIS`
`projects/thread/Kconfig` 的 `TRD_CHASSIS` 加 `select TFLM_MODEL_CHASSIS`
`algorithm/tflm/CMakeLists.txt` 加模型源文件

### Step 4: tflm.cpp 加推理函数

在 `tflm.cpp` 加 `chassis_predict(float target_vel, float current_vel, float current_torque)` 返回电流值。

### Step 5: 集成到底盘控制

`trd_chassis.cpp` 的 `ControlCalculate()` 中，把 `drive_velocity` 和 `drive_torque` 两级 PID 替换为 NN 调用。

### Step 6: 验证

1. `python scripts/train_chassis_nn.py` — 训练无报错
2. `west build -b hpm6e00evk` — 编译通过
3. 串口输出 NN 电流值 vs PID 电流值对比

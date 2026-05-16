# PX4 神经网络控制模块使用指南

PX4 提供两个神经网络控制模块，都是实验性的：

| 模块 | 技术栈 | 模型来源 | 调参方式 | 适用场景 |
|------|--------|---------|---------|---------|
| **mc_nn_control** | TensorFlow Lite Micro | 编译进固件 | 改 RPM 参数 | 自己训练 TFLite 模型 |
| **mc_raptor** | RLtools | SD 卡加载 | 换策略文件 | 即开即用，跨平台自适应 |

---

## 一、mc_nn_control（TFLM 端到端控制）

### 1.1 编译

#### SITL 仿真

```bash
make px4_sitl_neural
```

#### 实机（FMU-v6C）

```bash
make px4_fmu-v6c_neural upload
```

#### 其他板子

在 `.px4board` 中添加两行：

```kconfig
CONFIG_LIB_TFLM=y
CONFIG_MODULES_MC_NN_CONTROL=y
```

> 注意：模块增加约 50KB 固件体积，Flash 不够时可以去掉 FW/VTOL/UUV 等非必要模块。

**当前预置板型配置：**

| 板型 | 配置文件 |
|------|---------|
| px4/sitl | `boards/px4/sitl/neural.px4board` |
| px4/fmu-v6c | `boards/px4/fmu-v6c/neural.px4board` |
| mro/pixracerpro | `boards/mro/pixracerpro/neural.px4board` |

### 1.2 启动

模块默认自动启动（`MC_NN_EN=1`），启动后注册一个名为 `"Neural Control"` 的外部飞行模式。

也可在 NuttX shell 手动控制：

```bash
mc_nn_control start
mc_nn_control status    # 查看是否注册成功
mc_nn_control stop
```

### 1.3 切换飞行模式

#### 方法 A：遥控器开关（推荐）

1. 在 QGroundControl 中将遥控器某个开关映射到 **External Mode 1**（或相应编号）
2. PX4 启动后 mc_nn_control 自动注册外部模式，名称显示为 `"Neural Control"`
3. 拨动开关即可切换

> 部分 QGC 版本可能不显示外部模式名，此时直接用 RC 开关（External Mode 1/2/3）切换即可。

#### 方法 B：MAVLink shell 命令

```bash
# 起飞（先切到 Position 模式起飞）
commander takeoff

# 切换到神经网络模式
commander mode ext{MODE_ID}
```

`MODE_ID` 在启动日志中输出，也可通过 `mc_nn_control status` 查看。

### 1.4 参数调优

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MC_NN_EN` | 1 | 是否自动启动 |
| `MC_NN_MAX_RPM` | 22000 | 电机最大 RPM，用于 NN 输出归一化 |
| `MC_NN_MIN_RPM` | 1000 | 电机最小 RPM |
| `MC_NN_THRST_COEF` | 1.2 | 推力系数（内部除以 100000），**越大推力越小** |
| `MC_NN_MANL_CTRL` | 1 | 启用遥控手动控制模式（否则只接受 offboard setpoint） |

**调参建议：**
- 如果感觉 **太冲/太灵敏** → 减小 `MC_NN_THRST_COEF`（推力量更大）
- 如果感觉 **无力/响应慢** → 增大 `MC_NN_THRST_COEF`
- 如果换了不同尺寸的电机 → 调整 `MC_NN_MAX_RPM` / `MC_NN_MIN_RPM`

### 1.5 训练自己的网络

> 训练需要 NVIDIA GPU 和 [Aerial Gym Simulator](https://ntnu-arl.github.io/aerial_gym_simulator/)

**步骤：**

```mermaid
flowchart LR
    A[系统辨识飞行] --> B[获取惯量矩阵]
    B --> C[配置 Aerial Gym]
    C --> D[RL 训练]
    D --> E[PyTorch→TFLite]
    E --> F[xxd 转 C 数组]
    F --> G[替换 control_net.cpp]
    G --> H[重新编译固件]
```

详细步骤：

1. **系统辨识飞行**：使用 DSHOT ESC 遥测记录日志
2. **悬停日志**：读取悬停 RPM
3. **计算惯量**：用电机重量、臂长、总重估算惯量矩阵
4. **填入 Aerial Gym 配置**
5. **RL 训练**：在 Aerial Gym 中用 PyTorch 训练
6. **模型导出**：

```bash
# Aerial Gym 中的转换脚本
cd aerial_gym_simulator/resources/conversion

# PyTorch checkpoint → TFLite
python convert.py --checkpoint checkpoint.pth --output model.tflite

# TFLite → C 数组
xxd -i model.tflite > control_net.cpp
```

7. **替换固件中的模型文件**：[control_net.cpp](../../src/modules/mc_nn_control/control_net.cpp) 和 [control_net.hpp](../../src/modules/mc_nn_control/control_net.hpp)
8. **重编固件**并烧录

### 1.6 日志

NeuralControl 消息记录每次推理的输入/输出/耗时：

```bash
# 确保 SDLOG_PROFILE 包含 debug
param set SDLOG_PROFILE 2  # 或 3（包含 debug）

# 日志中包含 neural_control 话题
topic_listener neural_control
```

---

## 二、mc_raptor（RL 自适应控制）

### 2.1 编译

#### SITL 仿真

```bash
make px4_sitl_raptor gz_x500
```

#### 实机（FMU-v6C）

```bash
make px4_fmu-v6c_raptor upload
```

#### 其他板子

在 `.px4board` 中添加：

```kconfig
CONFIG_LIB_RL_TOOLS=y
CONFIG_MODULES_MC_RAPTOR=y
```

**当前预置板型配置：**

| 板型 | 配置文件 |
|------|---------|
| px4/sitl | `boards/px4/sitl/raptor.px4board` |
| px4/fmu-v6c | `boards/px4/fmu-v6c/raptor.px4board` |

### 2.2 上传策略文件

RAPTOR 的神经网络策略从 SD 卡加载，不编译进固件：

```bash
# 启动 MAVProxy
mavproxy.py --master udp:127.0.0.1:14540   # SITL
# 或
mavproxy.py --master /dev/serial/by-id/usb-...   # 实机

# 创建目录
ftp mkdir /raptor              # SITL
# 或
ftp mkdir /fs/microsd/raptor   # 实机

# 上传策略文件
ftp put src/modules/mc_raptor/blob/policy.tar /raptor/policy.tar
```

**重启飞控**（Ctrl+C 重启 SITL，或实机断电重启）。

> 策略文件也可在 Web 浏览器中测试：[https://raptor.rl.tools](https://raptor.rl.tools)

### 2.3 启动

启动后自动注册外部模式。设置参数：

```bash
# 启用 RAPTOR
param set MC_RAPTOR_ENABLE 1

# 禁用 offboard 模式要求
param set MC_RAPTOR_OFFB 0

# 调整 IMU 速率匹配
param set IMU_GYRO_RATEMAX 250   # SITL
# 实机可以用 400

param save
```

### 2.4 飞行测试

#### 逐步验证法（推荐，防止炸机）

> 官方推荐使用**自锁开关 + 弹簧回中**的方式，默认 Stabilized 模式，按下时切 External Mode 1（RAPTOR），松开立即回到 Stabilized。

```bash
# 先起飞（Stabilized 或 Position 模式）
commander takeoff

# 短暂切到 RAPTOR 观察反应（使用死手开关）
# 如果正常，逐渐延长 RAPTOR 运行时间
```

#### 全自动方式

```bash
# 起飞后直接切换到 RAPTOR
commander mode ext{RAPTOR_MODE_ID}
```

### 2.5 内部参考轨迹

RAPTOR 内置 Lissajous 轨迹生成器，用于不需要外部 setpoint 的基准测试：

```bash
# 启用内部参考模式
param set MC_RAPTOR_INTREF 1

# 重启后
commander takeoff
commander mode ext{MODE_ID}

# 设置 Lissajous 轨迹
mc_raptor intref lissajous <A> <B> <omega_x> <omega_y> <phi> <z> <duration> <num_cycles>
```

参数说明：

| 参数 | 含义 |
|------|------|
| A | X 方向幅度 (m) |
| B | Y 方向幅度 (m) |
| omega_x | X 方向角频率 |
| omega_y | Y 方向角频率 |
| phi | 相位差 |
| z | Z 高度 (m) |
| duration | 持续时间 (s) |
| num_cycles | 循环次数 |

示例（八字轨迹）：

```bash
mc_raptor intref lissajous 0.5 1 0 2 1 1 10 3
```

也可以使用 Web 工具可视化参数：[RAPTOR Trajectory Tool](https://rl-tools.github.io/mc-raptor-trajectory-tool)

### 2.6 系统辨识和训练新策略

如需针对自己的飞行器训练新策略：

1. 参照 [RLtools 文档](https://rl.tools) 设置训练环境
2. 训练新策略导出 `.tar` 文件
3. 通过 MAVLink FTP 上传 SD 卡（无需重编译固件）

### 2.7 调试与日志

```bash
# 创建日志配置
cat > logger_topics.txt << EOF
raptor_status 0
raptor_input 0
trajectory_setpoint 0
vehicle_local_position 0
vehicle_angular_velocity 0
vehicle_attitude 0
vehicle_status 0
actuator_motors 0
EOF

# 上传到飞控
# 实机：
ftp put logger_topics.txt /fs/microsd/etc/logging/logger_topics.txt
# SITL:
ftp put logger_topics.txt etc/logging/logger_topics.txt
```

### 2.8 重要注意事项

- RAPTOR **不需要对每个机型重新训练**，已测试 32g ~ 2.4kg 的 10+ 种飞行器
- 请确保电机布局为标准 PX4 Quadrotor X 布局：1-前右, 2-后左, 3-前左, 4-后右
- 室外测试建议风力 < 5 m/s
- 实测最高直线速度 > 17 m/s
- 除冰保护：`CA_ICE_PERIOD` 可配置 VTOL 电机周期性空转

---

## 三、模块对比与选择

| 需求 | 选哪个 |
|------|--------|
| 不想训练，直接飞 | **mc_raptor** |
| 想在 PyTorch 里训练自己的网络 | **mc_nn_control** |
| 需要遥控器手动控制 | **mc_nn_control**（支持 manual mode） |
| 需要 offboard 轨迹跟踪 | **mc_raptor**（MAVLink setpoint） |
| 追求最小固件体积 | **mc_raptor**（策略在 SD 卡） |
| 需要修改网络结构 | **mc_nn_control**（改 TFLite 模型） |
| 跨平台适应性 | **mc_raptor**（元学习，无需重训） |

---

## 四、常见问题

### 4.1 编译报错

- **Ubuntu 22.04**：不支持 TFLM，需要 Ubuntu 24.04+
- **Flash 不够**：在 `.px4board` 中禁用不需要的模块（`CONFIG_MODULES_FW_*=n` 等）
- **RAM 不够**：mc_nn_control 需要额外 10KB tensor arena

### 4.2 飞行模式不出现

- 更新到最新版 QGC
- 某些飞控不支持外部模式注册，此时只能用 RC 通道切换 External Mode
- 用 `mc_nn_control status` 或 `mc_raptor status` 确认模块已注册成功

### 4.3 飞起来不稳

对于 mc_nn_control：

```bash
# 减小推力系数（更大推力 → 更激进）
param set MC_NN_THRST_COEF 0.8

# 检查电机 RPM 范围
param set MC_NN_MAX_RPM 20000
param set MC_NN_MIN_RPM 1000
```

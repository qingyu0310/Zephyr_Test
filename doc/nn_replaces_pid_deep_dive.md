# mc_nn_control 深度分析：神经网络如何替代 PID 输出

## 概述

PX4 的 `mc_nn_control` 模块用单个前馈神经网络替换了多旋翼飞行控制的**整个级联控制链路**。这篇文档逐行追踪从传感器输入到电机输出的完整数据流。

---

## 一、传统级联 PID 做了什么

传统多旋翼控制链路是 4 级级联：

```
期望位置           期望速度           期望姿态          期望角速率
   ↓                  ↓                  ↓                ↓
[位置PID] ─vel_sp─→ [速度PID] ─att_sp─→ [姿态PID] ─rate_sp─→ [角速率PID]
   ↑                  ↑                  ↑                ↑
  位置反馈            速度反馈            姿态反馈          角速率反馈
                                                                ↓
                                                          [控制分配器]
                                                              ↓
                                                          [混控器 → 电机]
```

每级一个独立的 PID 控制器（各有 P/I/D + 限幅 + 积分），最后通过控制分配矩阵（伪逆）把 torque/thrust 期望值映射到 4/6/8 个电机。

---

## 二、NN 做了什么

NN 模块的配置声明（[mc_nn_control.cpp:160-171](../../src/modules/mc_nn_control/mc_nn_control.cpp#L160-L171)）：

```cpp
config_control_setpoints.flag_multicopter_position_control_enabled = false;  // 跳过位置PID
config_control_setpoints.flag_control_allocation_enabled = false;            // 跳过控制分配
config_control_setpoints.flag_control_position_enabled = false;              // 跳过位置控制
config_control_setpoints.flag_control_climb_rate_enabled = true;             // 只保留爬升率使能
```

NN 控制链路变成：

```
期望位置         + 当前姿态 + 当前速度 + 当前角速度
   ↓
[PopulateInputTensor()]  构建15维输入向量
   ↓
[interpreter->Invoke()]  一次 NN forward（~10KB内存）
   ↓
[RescaleActions()]       物理映射：归一化值 → RPM
   ↓
[PublishOutput()]        直出 actuator_motors
   ↓
电机
```

**5 个传统模块被一个 forward 替代**。

---

## 三、完整数据流逐行拆解

### 步骤 0：触发条件

模块不是定时触发的，而是注册为角速度消息的回调（[mc_nn_control.cpp:79-84](../../src/modules/mc_nn_control/mc_nn_control.cpp#L79-L84)）：

```cpp
bool MulticopterNeuralNetworkControl::init()
{
    if (!_angular_velocity_sub.registerCallback()) {   // 注册回调
        return false;
    }
    return true;
}
```

每次 `vehicle_angular_velocity` uORB 消息发布时，自动触发 `Run()`。这是关键设计：**NN 推理频率 = IMU 角速度采样频率**（通常是 250Hz ~ 1kHz）。

### 步骤 1：读取传感器数据

在 `Run()` 中（[mc_nn_control.cpp:494-511](../../src/modules/mc_nn_control/mc_nn_control.cpp#L494-L511)）：

```cpp
// 角速度更新触发 → 也同步拉取其他传感器最新值
if (_angular_velocity_sub.update(&_angular_velocity)) {
    const float dt = constrain((timestamp - _last_run) * 1e-6f, 0.0002f, 0.02f);
    _last_run = timestamp;

    if (_attitude_sub.updated())       _attitude_sub.copy(&_attitude);
    if (_position_sub.updated())       _position_sub.copy(&_position);
    if (_trajectory_setpoint_sub.updated()) _trajectory_setpoint_sub.copy(&_trajectory_setpoint_temp);
}
```

采集四个最新值：
- `vehicle_angular_velocity` — 角速率（IMU 直接读数）
- `vehicle_attitude` — 姿态四元数（EKF 估计）
- `vehicle_local_position` — 位置/速度（EKF 估计）
- `trajectory_setpoint` — 期望位置（飞控上层下发）

### 步骤 2：构建输入张量

`PopulateInputTensor()` 是**最关键的数据处理函数**（[mc_nn_control.cpp:280-352](../../src/modules/mc_nn_control/mc_nn_control.cpp#L280-L352)），将所有传感器数据拼接成 NN 的 15 维输入：

#### 2.1 坐标系变换

原始数据在 NED（北-东-地）系，NN 的输入坐标系不同：

```cpp
matrix::Dcmf frame_transf;         // ENU 旋转
frame_transf(0,0)=1;  (0,1)=0;  (0,2)=0;    // X ← X
frame_transf(1,0)=0;  (1,1)=-1; (1,2)=0;    // Y ← -Y
frame_transf(2,0)=0;  (2,1)=0;  (2,2)=-1;   // Z ← -Z

matrix::Dcmf frame_transf_2;       // 额外旋转
frame_transf_2(0,0)=0; (0,1)=1; (0,2)=0;    // X ← Y
frame_transf_2(1,0)=-1;(1,1)=0; (1,2)=0;    // Y ← -X
frame_transf_2(2,0)=0; (2,1)=0; (2,2)=1;    // Z ← Z
```

两层矩阵乘法 `frame_transf * frame_transf_2` 做 NED → 局部坐标系的完整变换。所有位置/速度/姿态/角速度都被变换到这个统一坐标系。

#### 2.2 输入向量详解

```
Index  符号          数据来源                        物理含义
───── ──────────    ──────────────────────────       ─────────────────────
  0    input[0]      trajectory_setpoint.x - pos.x    X方向位置误差
  1    input[1]      trajectory_setpoint.y - pos.y    Y方向位置误差
  2    input[2]      trajectory_setpoint.z - pos.z    Z方向位置误差

  3    input[3]      attitude_mat[0][0]               旋转矩阵 R[0][0]
  4    input[4]      attitude_mat[0][1]               旋转矩阵 R[0][1]
  5    input[5]      attitude_mat[0][2]               旋转矩阵 R[0][2]
  6    input[6]      attitude_mat[1][0]               旋转矩阵 R[1][0]
  7    input[7]      attitude_mat[1][1]               旋转矩阵 R[1][1]
  8    input[8]      attitude_mat[1][2]               旋转矩阵 R[1][2]

  9    input[9]      vx                               线速度 X
 10    input[10]     vy                               线速度 Y
 11    input[11]     vz                               线速度 Z

 12    input[12]     angular_vel.x                    角速度 X (roll rate)
 13    input[13]     angular_vel.y                    角速度 Y (pitch rate)
 14    input[14]     angular_vel.z                    角速度 Z (yaw rate)
```

**关键观察**：
- 姿态用**旋转矩阵前两行**（6 个数）表示，不是四元数（4 个）也不是欧拉角（3 个）。旋转矩阵前两行足以确定全部姿态，因为 `R[2] = R[0] × R[1]`。这比四元数更适合 NN，因为值是连续的、无奇异的。
- 输入不包括推力或油门信息——NN 必须仅从位置误差 + 姿态 + 速度 + 角速度推断所需推力。
- 位置误差直接作为输入，不经过 PID 处理。

#### 2.3 NaN 保护

```cpp
_trajectory_setpoint.position[0] = PX4_ISFINITE(val) ? val : 0.0f;
_trajectory_setpoint.position[2] = PX4_ISFINITE(val) ? val : -1.0f;  // Z 默认 -1（高度）
```

### 步骤 3：NN 推理

#### 3.1 模型加载

模型编译时被转换成 C 数组并嵌入固件（[control_net.cpp](../../src/modules/mc_nn_control/control_net.cpp)）：

```cpp
alignas(16) const unsigned char control_net_tflite[] = { 0x1c, 0x00, ... };
constexpr unsigned int control_net_tflite_size = 15088;  // ~15KB
```

启动时由 `InitializeNetwork()` 加载（[mc_nn_control.cpp:87-120](../../src/modules/mc_nn_control/mc_nn_control.cpp#L87-L120)）：

```cpp
const tflite::Model *control_model = ::tflite::GetModel(control_net_tflite);  // 反序列化

// 注册 NN 算子（仅3种）
NNControlOpResolver resolver;      // MicroMutableOpResolver<3>
resolver.AddFullyConnected();      // 全连接层（线性变换 + bias）
resolver.AddRelu();                // ReLU 激活函数
resolver.AddAdd();                 // 加法

// 分配 tensor arena（静态内存，无堆分配）
constexpr int kTensorArenaSize = 10 * 1024;   // 10KB
static uint8_t tensor_arena[kTensorArenaSize];
_interpreter = new tflite::MicroInterpreter(control_model, resolver,
                                            tensor_arena, kTensorArenaSize);
_interpreter->AllocateTensors();
```

**从3种算子推断网络结构**：

| 层 | 类型 | 说明 |
|-----|------|------|
| 输入 | — | 15 维浮点向量 |
| Layer 1 | FullyConnected + ReLU | 15 × H（隐藏层, H?） |
| Layer 2 | FullyConnected + ReLU | H × H（第二隐藏层, 可选） |
| Layer 3 | FullyConnected + Add | H × 4（输出层, 不用 ReLU） |
| 输出 | — | 4 维浮点向量 |

由于只有 3 个 op slots（`MicroMutableOpResolver<3>`），且 FullyConnected 内部不含激活函数（需要显式 Relu），所以网络最多是：**FC → ReLU → FC → ReLU → FC → Add**。至少 2 个隐藏层，因为 Add 单独占一个 slot（用于输出层 bias 累加）。

模型大小 15KB = 15088 字节，其中模型元信息 + FlatBuffers schema 占 ~1-2KB，实际权重约 13KB。如果隐藏层宽度为 H，全连接层参数量为 `(15+1)×H + (H+1)×H + (H+1)×4`（含 bias）。13KB = 3328 个 float32，解得 H ≈ 32：

```
Layer 0: (15+1)×32 = 512 params × 4B = 2048B
Layer 1: (32+1)×32 = 1056 params × 4B = 4224B
Layer 2: (32+1)×4 = 132 params × 4B = 528B
总计: 512 + 1056 + 132 = 1700 params × 4B = 6800B
加上偏置/Buffer开销 ≈ 13KB ✓
```

**所以网络很可能是 15→32→32→4，一个很小的全连接 MLP。**

#### 3.2 推理执行

```cpp
TfLiteStatus invoke_status = _interpreter->Invoke();  // 一次 forward
```

推理时间通过 `NeuralControl.msg` 发布以做性能监控：

```cpp
neural_control.inference_time = inference_time;      // [us]
neural_control.controller_time = full_controller_time; // 总耗时
```

### 步骤 4：输出重映射（核心调参）

`RescaleActions()`（[mc_nn_control.cpp:379-405](../../src/modules/mc_nn_control/mc_nn_control.cpp#L379-L405)）将 NN 输出转换为实际电机命令，实现了**物理域适应**：

```cpp
void MulticopterNeuralNetworkControl::RescaleActions()
{
    const float thrust_coeff = _param_thrust_coeff.get() / 100000.0f;  // ÷100000 归一化
    const float min_rpm = _param_min_rpm.get();    // 例如 1000 RPM
    const float max_rpm = _param_max_rpm.get();    // 例如 22000 RPM
    const float a = 0.8f;      // 非线性映射参数
    const float b = 0.2f;      

    for (int i = 0; i < 4; i++) {
        // ── 第1步：截断到 [-1, 1] ──
        if (_output_tensor->data.f[i] < -1.0f) _output_tensor->data.f[i] = -1.0f;
        else if (_output_tensor->data.f[i] > 1.0f) _output_tensor->data.f[i] = 1.0f;

        // ── 第2步：映射到 [0, 2] 表示推力 ──
        _output_tensor->data.f[i] = _output_tensor->data.f[i] + 1.0f;

        // ── 第3步：推力 → 转速（物理模型） ──
        //  推力 ∝ RPM² ⇒ RPM = √(推力 / 推力系数)
        //  thrust_coeff 是 CT / 100000（实验中测得）
        float rps = _output_tensor->data.f[i] / thrust_coeff;  
        rps = sqrt(rps);                     // RPS = √(T/CT)
        float rpm = rps * 60.0f;             // RPS → RPM

        // ── 第4步：RPM → 归一化电机指令 [-1, 1] ──
        //  在实际电机最小值~最大值之间做线性插值
        _output_tensor->data.f[i] = (rpm * 2.0f - max_rpm - min_rpm) / (max_rpm - min_rpm);

        // ── 第5步：二次非线性挤压 ──
        //  低油门区域更敏感（提升悬停附近精度）
        //  高油门区域更平滑（防止饱和）
        float normalized = (_output_tensor->data.f[i] + 1.0f) / 2.0f;  // [0, 1]
        _output_tensor->data.f[i] = a * ((normalized + b/(2*a))² - (b/(2*a))²);
    }
}
```

**五步映射链条**：

```
NN输出 [-1, 1] → 推力量 [0, 2] → RPS → RPM → 归一化[-1, 1] → 二次弯曲
```

这个映射中包含了物理知识（推力 ∝ RPM²）和实验参数（CT、RPM 范围），NN 本身只需要学习抽象的控制策略，物理边界由外层代码保证。

### 步骤 5：输出发布

直接发布到 `actuator_motors` 话题（[mc_nn_control.cpp:354-375](../../src/modules/mc_nn_control/mc_nn_control.cpp#L354-L375)）：

```cpp
actuator_motors_s actuator_motors;
actuator_motors.control[0] = command_actions[0];   // 电机1
actuator_motors.control[1] = command_actions[1];   // 电机2
actuator_motors.control[2] = command_actions[2];   // 电机3
actuator_motors.control[3] = command_actions[3];   // 电机4
actuator_motors.control[4..11] = -NAN;              // 禁用其余电机通道
_actuator_motors_pub.publish(actuator_motors);
```

注意：输出没有经过控制分配器（`flag_control_allocation_enabled = false`），NN 直接指定每个电机的值，相当于 NN 内部隐式学习了混控矩阵。

---

## 四、完整时序

```
车辆飞行中
    │
    ├─ IMU 产生新的角速度采样
    │      ↓
    ├─ vehicle_angular_velocity uORB 发布
    │      ↓
    ├─ SubscriptionCallback 触发 Run()
    │      ↓
    ├─ Run() 读取同步数据：
    │   • 角速度（触发源）
    │   • 姿态（如有更新）
    │   • 位置/速度（如有更新）
    │   • setpoint（如有更新）
    │      ↓
    ├─ PopulateInputTensor()
    │   • NED → 局部坐标变换
    │   • 计算位置误差
    │   • 打包15维向量
    │      ↓
    ├─ interpreter->Invoke()
    │   • 全连接 → ReLU → 全连接 → ReLU → 全连接 → Bias
    │      ↓
    ├─ RescaleActions()
    │   • 截断 → 推力 → RPM → 归一化 → 二次映射
    │      ↓
    └─ PublishOutput()
        • actuator_motors.control[0..3]
               ↓
        混控器 → ESC → 电机
```

从 IMU 采样到电机输出，端到端延迟只有一次 NN forward 的时间（微秒级）。

---

## 五、与传统 PID 对比

| 方面 | 传统级联 PID | NN 控制 |
|------|-------------|---------|
| **参数数量** | 几十个（P/I/D × 4级 × 3轴 + 限幅） | 4个（RPM范围 + 推力系数） |
| **结构** | 每级独立的线性控制器 | 单个非线性 MLP |
| **物理知识** | 每级输出有明确的物理意义（速度、姿态） | 隐式学习，仅输入-输出端有物理映射 |
| **状态记忆** | 无（无积分时） | 无（纯前馈） |
| **推理硬件** | FPU 乘加运算 | TFLM 解释器（10KB RAM） |
| **调参方式** | 地面站调 P/I/D | 换模型（需重新编译固件） |
| **鲁棒性** | 每级限幅 + anti-windup | 仅输出截断 + RPM clamp |
| **输出维度** | 3轴 torque + 1轴 thrust → 混控 | 4 个电机值直出 |

## 六、关键洞察

1. **NN 替代的不只是一个 PID，而是整个控制栈**。位置环、速度环、姿态环、角速率环、控制分配器，五合一。

2. **物理知识在外层**。NN 输出的是抽象的归一化值，`RescaleActions()` 负责映射到物理世界。这个设计意味着同一个网络可以适配不同硬件，只需调整 `MC_NN_MAX_RPM / MIN_RPM / THRST_COEF` 三个参数。

3. **输入设计是分层级的**。从位置误差（较慢变化）到角速度（最快变化）都在同一输入向量中，网络需要自己学习不同时间尺度的特征。

4. **姿态用旋转矩阵表示**（6 个值而非 4 个四元数或 3 个欧拉角），消除万向节锁定并使姿态误差在欧几里得空间连续，这对网络学习更友好。

5. **推理频率由 IMU 决定**（通过角速度订阅触发），而不固定。这使模块可以适应不同的 IMU 速率（250Hz~1kHz），不依赖于定时器配置。

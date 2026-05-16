"""
从真机数据训练底盘行进电机 NN 模型

数据格式：目标速度,当前速度,转矩,电流（一行一个样本）
"""
import numpy as np
import tensorflow as tf

# ── 1. 读取数据 ──
# 改成你的实际路径
path = r"C:\Users\33527\Desktop\vofa+.csv"
raw = np.genfromtxt(path, delimiter=',', invalid_raise=False)
# 去掉包含 NaN 的行（日志行混入导致）
raw = raw[~np.isnan(raw[:,0])]
print(f"原始数据: {raw.shape[0]} 行")

# ── 2. 系统辨识：从真机数据估算电机参数 ──
Kt = 0.3  # Nm/A (kTorqueK)
dt = 0.001  # 控制周期 1ms

# 速度变化率 ≈ (下一拍速度 - 当前速度) / dt
accel = (raw[1:, 1] - raw[:-1, 1]) / dt
# 用电流和速度拟合：accel = (Kt/J) * current - (B/J) * vel
A = np.column_stack([raw[:-1, 3], -raw[:-1, 1]])  # [current, vel]
coeff, _, _, _ = np.linalg.lstsq(A, accel, rcond=None)
Kt_over_J = coeff[0]
B_over_J = coeff[1]
J = Kt / Kt_over_J
B = B_over_J * J
print(f"辨识结果: J={J:.6f}, B={B:.4f}, Kt/J={Kt_over_J:.2f}")

# ── 3. 用仿真生成训练数据（替代 PID 数据） ──
data = []
for target_vel in np.linspace(-0.5, 0.5, 30):
    vel = 0.0
    for _ in range(200):
        # 用简单的 P 控制算出目标电流，记录 [目标速度, 当前速度, 转矩] → [电流]
        err = target_vel - vel
        current = err * 5.0
        torque = current * Kt
        acc = (torque - B * vel) / J
        vel += acc * dt
        data.append([[target_vel, vel, torque], current])

X = np.array([d[0] for d in data], dtype=np.float32)
Y = np.array([d[1] for d in data], dtype=np.float32).reshape(-1, 1)

max_current = 20.0
Y = Y / max_current
print(f"仿真样本: {len(X)}, 电流范围: ±20A")

# ── 4. 训练 ──
model = tf.keras.Sequential([
    tf.keras.layers.Dense(16, activation='relu', input_shape=(3,)),
    tf.keras.layers.Dense(16, activation='relu'),
    tf.keras.layers.Dense(1, activation='tanh'),
])
model.compile(optimizer='adam', loss='mse')
model.fit(X, Y, epochs=200, verbose=1)

# ── 4. 导出 TFLite ──
import os
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite = converter.convert()
proj = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.makedirs(os.path.join(proj, 'algorithm', 'tflm', 'models', 'chassis'), exist_ok=True)

tflite_path = os.path.join(proj, 'algorithm', 'tflm', 'models', 'chassis', 'chassis_model.tflite')
with open(tflite_path, 'wb') as f:
    f.write(tflite)

h_path = os.path.join(proj, 'algorithm', 'tflm', 'models', 'chassis', 'chassis_model_data.h')
with open(h_path, 'w') as f:
    f.write("#ifndef TFLM_MODELS_CHASSIS_MODEL_DATA_H_\n#define TFLM_MODELS_CHASSIS_MODEL_DATA_H_\n")
    f.write("extern const unsigned char g_chassis_model_data[];\n")
    f.write("extern const unsigned int g_chassis_model_data_size;\n#endif\n")

lines = []
for i in range(0, len(tflite), 12):
    chunk = tflite[i:i+12]
    lines.append('  ' + ', '.join(f'0x{b:02x}' for b in chunk))
c_array = ',\n'.join(lines)

cc_path = os.path.join(proj, 'algorithm', 'tflm', 'models', 'chassis', 'chassis_model_data.cc')
with open(cc_path, 'w') as f:
    f.write(f"#include \"chassis_model_data.h\"\n")
    f.write(f"alignas(16) const unsigned char g_chassis_model_data[] = {{\n{c_array}\n}};\n")
    f.write(f"const unsigned int g_chassis_model_data_size = {len(tflite)};\n")

print(f"导出完成: {len(tflite)} bytes")

# ── 5. 验证（随机抽 20 条） ──
idx = np.random.choice(len(X), 500, replace=False)
pred = model.predict(X[idx])
print("\n随机验证:")
for i in range(20):
    real = Y[idx[i]][0] * max_current
    p = pred[i][0] * max_current
    print(f"目标速度={X[idx[i]][0]:+.3f}, 当前速度={X[idx[i]][1]:+.3f}, "
          f"转矩={X[idx[i]][2]:+.3f}, 预测电流={p:+.3f}, 真实电流={real:+.3f}")

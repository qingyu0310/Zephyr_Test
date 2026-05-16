"""
RL 训练底盘行进电机控制策略
电机模型：J=0.028, B=0.123, Kt=0.3
"""
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO

# ── 1. 电机环境 ──
class MotorEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.J = 0.028
        self.B = 0.123
        self.Kt = 0.3         # 转矩常数
        self.dt = 0.001       # 控制周期
        self.max_current = 20.0
        self.max_vel = 0.5

        # 观测: [目标速度, 当前速度, 转矩]
        self.observation_space = spaces.Box(
            low=-np.array([1, 1, 1]),
            high=np.array([1, 1, 1]),
            dtype=np.float32)
        # 动作: [-1,1] 归一化电流
        self.action_space = spaces.Box(-1, 1, (1,), dtype=np.float32)

        self.vel = 0.0
        self.torque = 0.0
        self.target_vel = 0.0
        self.steps = 0

    def reset(self, seed=None):
        # domain randomization：每次复位随机化电机参数
        self.J = 0.028 * (0.5 + np.random.random())
        self.B = 0.123 * (0.5 + np.random.random())
        self.vel = 0.0
        self.torque = 0.0
        self.last_current = 0.0
        self.target_vel = (np.random.random() - 0.5) * self.max_vel * 2
        self.steps = 0
        return self._obs(), {}

    def _obs(self):
        return np.array([
            self.target_vel / self.max_vel,
            self.vel / self.max_vel,
            self.torque], dtype=np.float32)

    def step(self, action):
        # 电流 = 动作 × 20A
        current = float(action[0]) * self.max_current
        torque = current * self.Kt
        accel = (torque - self.B * self.vel) / self.J
        self.vel += accel * self.dt
        self.torque = torque
        self.steps += 1

        err = abs(self.target_vel - self.vel)
        jerk = abs(current - getattr(self, 'last_current', 0))
        self.last_current = current
        reward = -err - 0.005 * abs(current) - 0.5 * jerk

        # 每 100 步换目标速度
        if self.steps % 100 == 0:
            self.target_vel = (np.random.random() - 0.5) * self.max_vel * 2

        return self._obs(), reward, False, False, {}

# ── 2. 训练 ──
env = MotorEnv()
model = PPO("MlpPolicy", env, verbose=1, n_steps=2048, learning_rate=3e-4)
model.learn(total_timesteps=200000)
model.save("scripts/chassis_rl_policy")

# ── 3. 导出 TFLite ──
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import warnings
warnings.filterwarnings('ignore')
import tensorflow as tf

# 提取 PPO actor 网络权重
policy = model.policy.mlp_extractor.policy_net
action_net = model.policy.action_net
w1, b1 = policy[0].weight.data.numpy().T, policy[0].bias.data.numpy()
w2, b2 = policy[2].weight.data.numpy().T, policy[2].bias.data.numpy()
w3, b3 = action_net.weight.data.numpy().T, action_net.bias.data.numpy()

# 在 Keras 中重建相同网络并设权重
inputs = tf.keras.Input(shape=(3,))
x = tf.keras.layers.Dense(64, activation='tanh')(inputs)
x = tf.keras.layers.Dense(64, activation='tanh')(x)
out = tf.keras.layers.Dense(1, activation='tanh')(x)
keras_model = tf.keras.Model(inputs, out)
keras_model.set_weights([w1, b1, w2, b2, w3, b3])

converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
tflite = converter.convert()
proj = os.path.dirname(os.path.abspath(__file__))
outdir = os.path.join(proj, '..', 'algorithm', 'tflm', 'models', 'chassis')
os.makedirs(outdir, exist_ok=True)

with open(os.path.join(outdir, 'chassis_model.tflite'), 'wb') as f:
    f.write(tflite)
for name in ['h', 'cc']:
    ext = '.' + name
    lines = []
    for i in range(0, len(tflite), 12):
        lines.append('  ' + ', '.join(f'0x{b:02x}' for b in tflite[i:i+12]))
    ca = ',\n'.join(lines)
    with open(os.path.join(outdir, f'chassis_model_data{ext}'), 'w') as f:
        if name == 'h':
            f.write("#ifndef TFLM_MODELS_CHASSIS_MODEL_DATA_H_\n#define TFLM_MODELS_CHASSIS_MODEL_DATA_H_\n")
            f.write("extern const unsigned char g_chassis_model_data[];\n")
            f.write("extern const unsigned int g_chassis_model_data_size;\n#endif\n")
        else:
            f.write('#include "chassis_model_data.h"\n')
            f.write(f'alignas(16) const unsigned char g_chassis_model_data[] = {{\n{ca}\n}};\n')
            f.write(f'const unsigned int g_chassis_model_data_size = {len(tflite)};\n')
print(f"导出完成: {len(tflite)} bytes")

# ── 4. 验证 ──
print("\n验证:")
for _ in range(5):
    obs, _ = env.reset()
    for i in range(100):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, _, _, _ = env.step(action)
        if i % 20 == 0:
            tgt = obs[0] * env.max_vel
            vel = obs[1] * env.max_vel
            curr = float(action[0]) * env.max_current
            print(f"  tgt={tgt:.2f} vel={vel:.2f} curr={curr:.2f} reward={reward:.3f}")

"""
Wheel-Legged Robot RL Training Script

Phase 1: Stand Up (simplest task)
  - State: [roll, pitch, yaw, gyro_x, gyro_y, gyro_z, joint_pos×6, joint_vel×6] = 18D
  - Action: [hip_R, knee_R, hip_L, knee_L, wheel_R, wheel_L] torques = 6D
  - Reward: survive upright + minimize torque usage

Phase 2: Walk (next iteration)
  - Add velocity tracking reward
  - Add gait pattern reward

Dependencies: pip install mujoco stable-baselines3 numpy
"""

import os
import numpy as np
import mujoco
import mujoco.viewer
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback

# ============ Environment ============

class WheelLegEnv:
    """MuJoCo environment for wheel-legged robot."""
    
    MODEL_PATH = os.path.join(os.path.dirname(__file__), "wheel_leg.xml")
    
    # Joint order: left_hip, left_knee, right_hip, right_knee, left_wheel, right_wheel
    JOINT_NAMES = [
        "left_hip_joint", "left_knee_joint",
        "right_hip_joint", "right_knee_joint",
        "left_wheel_joint", "right_wheel_joint",
    ]
    ACTUATOR_NAMES = [
        "left_hip_torque", "left_knee_torque",
        "right_hip_torque", "right_knee_torque",
        "left_wheel_torque", "right_wheel_torque",
    ]
    
    # Task parameters
    MAX_HEIGHT = 0.30
    MIN_HEIGHT = 0.10
    MAX_ROLL = 0.5
    MAX_PITCH = 0.5
    MAX_TORQUE = 10.0  # Nm for joints
    MAX_WHEEL_TORQUE = 18.0  # Nm for wheels
    
    def __init__(self, render=False):
        self.model = mujoco.MjModel.from_xml_path(self.MODEL_PATH)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model) if render else None
        self.step_count = 0
        self.max_steps = 2000
        
    def reset(self):
        """Reset environment to standing pose."""
        mujoco.mj_resetData(self.model, self.data)
        
        # Set initial standing pose
        self.data.key_qpos[:] = self.model.key_qpos[0]
        mujoco.mj_forward(self.model, self.data)
        
        self.step_count = 0
        return self._get_obs()
    
    def step(self, action):
        """Apply torques and step physics."""
        # Clamp torques
        action = np.clip(action, -self.MAX_TORQUE, self.MAX_TORQUE)
        action[4:] = np.clip(action[4:], -self.MAX_WHEEL_TORQUE, self.MAX_WHEEL_TORQUE)
        
        # Apply torques to actuators
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)
        
        self.step_count += 1
        
        # Calculate reward
        reward = self._compute_reward()
        done = self._is_done()
        
        return self._get_obs(), reward, done, {}
    
    def _get_obs(self):
        """Observation: [roll, pitch, yaw, gyro_xyz, joint_pos×6, joint_vel×6] = 18D"""
        # Base orientation (Euler from quaternion)
        quat = self.data.sensordata[10:14]  # base_quat
        roll, pitch, yaw = self._quat_to_euler(quat)
        
        # Gyro
        gyro = self.data.sensordata[0:3]
        
        # Joint positions
        joint_pos = self.data.sensordata[14:20]
        
        # Joint velocities
        joint_vel = self.data.sensordata[20:26]
        
        obs = np.concatenate([
            [roll, pitch, yaw],
            gyro,
            joint_pos,
            joint_vel,
        ])
        
        return obs.astype(np.float32)
    
    def _compute_reward(self):
        """Reward: survive upright, minimize torque."""
        quat = self.data.sensordata[10:14]
        roll, pitch, yaw = self._quat_to_euler(quat)
        
        # Height reward: stay between MIN and MAX_HEIGHT
        z = self.data.sensordata[7]  # base_z position
        height_reward = 1.0 - abs(z - 0.20) * 5.0
        height_reward = np.clip(height_reward, 0, 1)
        
        # Orientation reward: stay upright (roll ~ 0, pitch ~ 0)
        orient_reward = 1.0 - min(abs(roll) + abs(pitch), 2.0)
        orient_reward = np.clip(orient_reward, 0, 1)
        
        # Torque penalty: minimize energy usage
        torque_penalty = np.sum(np.abs(self.data.ctrl[:4])) * 0.001
        
        reward = 1.0 * height_reward + 5.0 * orient_reward - torque_penalty
        
        return reward
    
    def _is_done(self):
        """Episode ends if robot falls over or timeout."""
        quat = self.data.sensordata[10:14]
        roll, pitch, _ = self._quat_to_euler(quat)
        
        # Fall over: |roll| > 60deg or |pitch| > 60deg
        if abs(roll) > 1.05 or abs(pitch) > 1.05:
            return True
        
        # Too low
        if self.data.sensordata[7] < 0.05:
            return True
        
        # Timeout
        if self.step_count >= self.max_steps:
            return True
        
        return False
    
    @staticmethod
    def _quat_to_euler(q):
        """Convert quaternion [w, x, y, z] to [roll, pitch, yaw]."""
        w, x, y, z = q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return roll, pitch, yaw
    
    def close(self):
        if self.renderer:
            self.renderer.close()


# ============ SB3 Compatible Wrapper ============

class StableBaselinesEnv:
    """Gym-compatible wrapper for Stable-Baselines3."""
    
    def __init__(self):
        self.env = WheelLegEnv()
        self.observation_space = None
        self.action_space = None
        self._init_spaces()
        self._obs = self.env.reset()
    
    def _init_spaces(self):
        import gymnasium as gym
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32)
    
    def reset(self, **kwargs):
        self._obs = self.env.reset()
        return self._obs, {}
    
    def step(self, action):
        # Scale actions from [-1,1] to torques
        torques = action.copy()
        torques[:4] *= self.env.MAX_TORQUE
        torques[4:] *= self.env.MAX_WHEEL_TORQUE
        
        obs, reward, done, info = self.env.step(torques)
        self._obs = obs
        truncated = done
        return obs, reward, done, truncated, info
    
    def render(self):
        if self.env.renderer:
            self.env.renderer.update_scene(self.env.data)
            self.env.renderer.render()
    
    def close(self):
        self.env.close()


# ============ Training ============

def train_stand():
    """Train the robot to stand up from a flat surface."""
    print("=" * 60)
    print("Wheel-Legged Robot: Phase 1 - Stand Up Training")
    print("=" * 60)
    
    # Create environment
    env = StableBaselinesEnv()
    
    # Create PPO agent
    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        tensorboard_log="./tb_logs/",
    )
    
    print("\nStarting training...")
    print("State dim: 18 | Action dim: 6 | Max steps: 500000\n")
    
    try:
        model.learn(total_timesteps=500000, progress_bar=True)
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    
    # Save model
    save_path = os.path.join(os.path.dirname(__file__), "rl_stand")
    model.save(save_path)
    print(f"\nModel saved to: {save_path}")
    
    # Export to ONNX for RDK X5 deployment
    export_onnx(model)
    
    env.close()


def export_onnx(model):
    """Export trained policy to ONNX format for RDK X5 deployment."""
    try:
        dummy_input = np.zeros((1, 18), dtype=np.float32)
        torch_model = model.policy.mlp_extractor
        # Placeholder: actual export requires tracing the policy network
        print("ONNX export: Use torch.onnx.export(model.policy, ...)")
        print("See: https://stable-baselines3.readthedocs.io/en/master/guide/export.html")
    except Exception as e:
        print(f"ONNX export skipped: {e}")
        print("Export manually after training completes.")


def evaluate(model_path=None):
    """Evaluate a trained model in the simulator."""
    env = WheelLegEnv(render=True)
    model = PPO.load(model_path or "rl_stand")
    
    obs = env.reset()
    total_reward = 0
    
    try:
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            total_reward += reward
            
            # Render
            env.env.renderer.update_scene(env.env.data)
            env.env.renderer.render()
            
            if done:
                print(f"Episode done. Total reward: {total_reward:.2f}")
                total_reward = 0
                obs = env.reset()
    except KeyboardInterrupt:
        pass
    
    env.close()


# ============ Main ============

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--train", action="store_true", help="Train the robot")
    parser.add_argument("--eval", action="store_true", help="Evaluate trained model")
    parser.add_argument("--model", type=str, default="rl_stand", help="Model path")
    args = parser.parse_args()
    
    if args.train:
        train_stand()
    elif args.eval:
        evaluate(args.model)
    else:
        print("Usage: python train_stand.py --train")
        print("       python train_stand.py --eval --model rl_stand")

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
import os
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback

class CurriculumCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(CurriculumCallback, self).__init__(verbose)
    def _on_step(self) -> bool:
        self.training_env.env_method("update_curriculum_steps", self.num_timesteps)
        return True

class DexRLHandEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, model_path="rlmodel.xml"):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Action Space (13): 0-2: Pos, 3-5: Rot, 6-7: Thumb Mid/Dist, 8-12: Proximal
        self.action_space = spaces.Box(low=-1, high=1, shape=(13,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(58,), dtype=np.float32)
        
        self.proximal_actuator_names = ['act_proxthumb', 'act_proxpoint', 'act_proxmid', 'act_proxring', 'act_proxpinky']
        self.extra_thumb_actuators = ['act_midthumb', 'act_distthumb']
        self.touch_sensor_names = ['thumb_touch', 'point_touch', 'mid_touch', 'ring_touch', 'pinky_touch']

        self.total_steps_taken = 0
        self.max_curriculum_steps = 1_500_000 
        self.min_goal_dist, self.max_goal_dist = 0.05, 0.40

    def update_curriculum_steps(self, steps):
        self.total_steps_taken = steps

    def _euler_to_quat(self, r, p, y):
        cr, cp, cy = np.cos(np.array([r, p, y]) * 0.5)
        sr, sp, sy = np.sin(np.array([r, p, y]) * 0.5)
        return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*sp*sy, cr*cp*sy - sr*sp*cy])

    def _get_obs(self):
        qpos, qvel = self.data.qpos.flat.copy(), self.data.qvel.flat.copy()
        touch_data = np.array([np.tanh(self.data.sensor(n).data[0]) for n in self.touch_sensor_names])
        palm_pos, palm_quat = self.data.body('base').xpos, self.data.body('base').xquat
        obj_pos, goal_pos = self.data.body('object').xpos, self.data.body('goal').xpos
        
        return np.concatenate([
            qpos, qvel, touch_data, 
            goal_pos - obj_pos, obj_pos - palm_pos, palm_quat
        ]).astype(np.float32)

    def step(self, action):
        # 1. RELATIVE MOVEMENT (Joystick Logic)
        # Moves hand up to 3cm per step relative to its current position
        self.data.mocap_pos[0] += action[:3] * 0.03
        
        # Workspace Clipping (Prevents hand from flying away)
        self.data.mocap_pos[0] = np.clip(self.data.mocap_pos[0], [0.3, -1.5, 0.4], [1.0, -0.8, 1.3])

        # 2. Rotation
        self.data.mocap_quat[0] = self._euler_to_quat(action[3]*np.pi, action[4]*np.pi, action[5]*np.pi)

        # 3. Finger Control
        thumb_ctrl = (action[6:8] + 1) / 2 * 1.57
        for i, name in enumerate(self.extra_thumb_actuators):
            self.data.actuator(name).ctrl[0] = thumb_ctrl[i]

        prox_ctrls = (action[8:] + 1) / 2 * 1.57
        for i, name in enumerate(self.proximal_actuator_names):
            self.data.actuator(name).ctrl[0] = prox_ctrls[i]

        mujoco.mj_step(self.model, self.data)
        obs = self._get_obs()
        
        # 4. MAGNET REWARD (Distance Shaping)
        dist_palm_obj = np.linalg.norm(self.data.body('base').xpos - self.data.body('object').xpos)
        dist_obj_goal = np.linalg.norm(self.data.body('object').xpos - self.data.body('goal').xpos)
        
        # Reach Reward (Exponential Magnet)
        reward_reach = 2.0 * np.exp(-4.0 * dist_palm_obj)
        
        # Move Reward (Only active if holding or near ball)
        reward_move = 0.0
        if dist_palm_obj < 0.15:
            reward_move = 5.0 * np.exp(-4.0 * dist_obj_to_goal)

        # Touch Reward
        reward_touch = 0.5 * np.sum(obs[43:48])
        
        reward = reward_reach + reward_move + reward_touch
        
        terminated = dist_obj_goal < 0.04
        if terminated: reward += 1000.0
            
        return obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        
        # Object Randomization
        obj_x, obj_y = self.np_random.uniform(0.6, 0.7), self.np_random.uniform(-1.15, -1.05)
        self.data.body('object').xpos = [obj_x, obj_y, 0.5]
        
        # START HAND NEAR OBJECT (The "Warm Start")
        self.data.mocap_pos[0] = [obj_x, obj_y - 0.1, 0.75]
        self.data.mocap_quat[0] = [1, 0, 0, 0]

        # Curriculum Goal
        progress = min(self.total_steps_taken / self.max_curriculum_steps, 1.0)
        dist = self.np_random.uniform(self.min_goal_dist, self.min_goal_dist + (progress * self.max_goal_dist))
        angle = self.np_random.uniform(0, 2 * np.pi)
        self.data.body('goal').xpos = [obj_x + np.cos(angle)*dist, obj_y + np.sin(angle)*dist, 0.8]
        
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

if __name__ == "__main__":
    os.makedirs("./logs/", exist_ok=True)
    env = DexRLHandEnv(model_path="rlmodel.xml")
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=3e-4, n_steps=4096, batch_size=128, tensorboard_log="./ppo_hand_tensorboard/")
    model.learn(total_timesteps=2_000_000, callback=[CheckpointCallback(50000, './logs/', 'dex'), CurriculumCallback()])
    model.save("dex_hand_final")
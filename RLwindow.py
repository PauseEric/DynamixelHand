import mujoco
import mujoco.viewer
import time
import numpy as np
from stable_baselines3 import PPO
from dexHandRL import DexRLHandEnv # Assumes your env class is in this file

def play():
    # 1. Load the Environment
    # Ensure 'render_mode' is handled if you added it, otherwise we use the viewer manually
    env = DexRLHandEnv(model_path="rlmodel.xml")
    
    # 2. Load the Trained Model
    model_path = "logs/dex_50000_steps.zip"
    try:
        model = PPO.load("logs/dex_50000_steps.zip")
        print(f"Successfully loaded model: {model_path}")
    except:
        print("Model file not found. Make sure the training finished and saved the .zip file.")
        return

    # 3. Launch the Viewer
    # We use the passive viewer so we can step the simulation manually
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        obs, _ = env.reset()
        
        print("Visualizing... Close the window to stop.")
        
        while viewer.is_running():
            step_start = time.time()

            # Use the model to predict the next action
            # deterministic=True is better for testing/evaluation
            action, _states = model.predict(obs, deterministic=True)

            # Step the environment
            obs, reward, terminated, truncated, info = env.step(action)

            # Sync the viewer with the simulation data
            viewer.sync()

            # If the hand succeeds or fails, reset and show a new goal
            if terminated or truncated:
                time.sleep(0.5) # Pause briefly so you can see the success
                obs, _ = env.reset()
                viewer.sync()

            # Maintain real-time playback speed (50 FPS)
            time_until_next_step = env.metadata["render_fps"]**-1 - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    play()
# spot_dance.py updated for full 12-actuator Spot with Correct GIF Export

import mujoco
import mujoco.viewer
import numpy as np
import imageio
import os
import time

# Create media directory if it doesn't exist
os.makedirs('media', exist_ok=True)

# Load the model
model = mujoco.MjModel.from_xml_path('spot_dance_scene.xml')
data = mujoco.MjData(model)

# Prepare GIF frame storage
frames = []

# Define dance steps (target joint positions for 12 actuators)
dance_moves = [
    {'time': 0, 'pose': [0.0]*12},  # Stand still
    {'time': 2, 'pose': [0.5, 0.2, -0.5, -0.5, -0.2, 0.5, 0.5, 0.2, -0.5, -0.5, -0.2, 0.5]},  # Sway left
    {'time': 4, 'pose': [-0.5, -0.2, 0.5, 0.5, 0.2, -0.5, -0.5, -0.2, 0.5, 0.5, 0.2, -0.5]},  # Sway right
    {'time': 6, 'pose': [0.0]*12},  # Reset
    {'time': 8, 'pose': [0.0, 0.3, -0.4, 0.0, 0.3, -0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},  # Bow
    {'time': 10, 'pose': [0.0]*12},  # Reset
    {'time': 12, 'pose': [0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},  # Lift left paw
    {'time': 14, 'pose': [0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},  # Wave
    {'time': 16, 'pose': [0.0]*12}   # End
]

# Control all 12 actuators directly
controlled_joints = [i for i in range(model.nu)]

# Initialize viewer and offscreen renderer
with mujoco.viewer.launch_passive(model, data) as viewer:
    offscreen = mujoco.Renderer(model)
    print("Starting Full Spot Dance Simulation and Recording...")

    t0 = time.time()
    move_idx = 0

    while viewer.is_running():
        mujoco.mj_step(model, data)

        t = time.time() - t0

        # Update dance move if needed
        if move_idx + 1 < len(dance_moves) and t > dance_moves[move_idx+1]['time']:
            move_idx += 1

        # Set joint targets
        if controlled_joints:
            target_pose = dance_moves[move_idx]['pose']
            for idx, joint_id in enumerate(controlled_joints):
                data.ctrl[joint_id] = target_pose[idx]

        # Render offscreen frame
        offscreen.update_scene(data)
        pixels = offscreen.render()

        frames.append(pixels)

        viewer.sync()

    offscreen.close()
    print("Simulation Ended, Saving GIF...")

# Save frames to GIF
output_path = 'media/spot_dance.gif'
imageio.mimsave(output_path, frames, fps=20)
print(f"GIF saved to {output_path}")

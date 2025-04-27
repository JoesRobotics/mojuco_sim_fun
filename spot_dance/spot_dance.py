import mujoco
import mujoco.viewer
import numpy as np
import imageio
import os
import time

# Ensure the media directory exists
os.makedirs('media', exist_ok=True)

# Load the model
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)

# Prepare GIF frame storage
frames = []

# Define dance steps (simple joint position targets)
dance_moves = [
    {'duration': 2.0, 'pose': [0.0]*12},  # Neutral pose
    {'duration': 2.0, 'pose': [0.2, -0.2, 0.2, -0.2]*3},  # Sway left
    {'duration': 2.0, 'pose': [-0.2, 0.2, -0.2, 0.2]*3},  # Sway right
    {'duration': 2.0, 'pose': [0.0]*12},  # Back to neutral
]

# Get joint indices
joint_names = [model.joint(i).name for i in range(model.njnt)]
controlled_joints = [i for i, name in enumerate(joint_names) if 'hip' in name or 'knee' in name]

# Initialize viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting Spot Dance Simulation and Recording...")

    for move in dance_moves:
        start_time = time.time()
        while time.time() - start_time < move['duration']:
            mujoco.mj_step(model, data)
            for idx, joint_id in enumerate(controlled_joints):
                if idx < len(move['pose']):
                    data.ctrl[joint_id] = move['pose'][idx]
            frame = viewer.read_pixels()
            frames.append(frame)
            viewer.sync()

    print("Simulation Ended, Saving GIF...")

# Save frames to GIF
output_path = 'media/spot_dance.gif'
imageio.mimsave(output_path, frames, fps=20)
print(f"GIF saved to {output_path}")


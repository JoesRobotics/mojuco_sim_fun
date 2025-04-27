# spot_dance.py with GIF Export

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

# Define dance steps (target joint positions)
dance_moves = [
    {'time': 0, 'pose': [0.0]*12},                     # Stand still
    {'time': 2, 'pose': [0.2, 0, 0, -0.2, 0, 0, 0.2, 0, 0, -0.2, 0, 0]},  # Head bob (lower front)
    {'time': 4, 'pose': [-0.2, 0, 0, 0.2, 0, 0, -0.2, 0, 0, 0.2, 0, 0]}, # Head bob other way
    {'time': 6, 'pose': [0.0]*12},                     # Reset
    {'time': 8, 'pose': [0.0, 0.2, 0, 0.0, -0.2, 0, 0.0, 0.2, 0, 0.0, -0.2, 0]}, # Twist left
    {'time': 10, 'pose': [0.0, -0.2, 0, 0.0, 0.2, 0, 0.0, -0.2, 0, 0.0, 0.2, 0]}, # Twist right
    {'time': 12, 'pose': [0.0]*12},                    # Reset
    {'time': 14, 'pose': [0.0, 0.0, 0.5, 0.0, 0.0, 0, 0.0, 0.0, 0, 0.0, 0.0, 0]},  # Lift front left leg
    {'time': 16, 'pose': [0.0]*12}                     # End
]

# Get joint indices
joint_names = [model.joint(i).name for i in range(model.njnt)]
controlled_joints = [i for i, name in enumerate(joint_names) if 'knee' in name or 'hip' in name]

# Initialize viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting Spot Dance Simulation and Recording...")

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

        # Capture frame
        frame = viewer.read_pixels()
        frames.append(frame)

        viewer.sync()

    print("Simulation Ended, Saving GIF...")

# Save frames to GIF
output_path = 'media/spot_dance.gif'
imageio.mimsave(output_path, frames, fps=20)
print(f"GIF saved to {output_path}")


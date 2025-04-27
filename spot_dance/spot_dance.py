# spot_dance.py

import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the model
model = mujoco.MjModel.from_xml_path('spot_dance_scene.xml')
data = mujoco.MjData(model)

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
    print("Starting Spot Dance Simulation")

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

        viewer.sync()

    print("Simulation Ended")
# skydio_gate.py with GIF Export

import mujoco
import mujoco.viewer
import numpy as np
import imageio
import os
import time

# Create media directory if it doesn't exist
os.makedirs('media', exist_ok=True)

# Load the model
model = mujoco.MjModel.from_xml_path('gate_world.xml')
data = mujoco.MjData(model)

# Prepare GIF frame storage
frames = []

# Define waypoints at the center of each gate
waypoints = np.array([
    [1.5, 0.0, 1.0],
    [3.0, 1.0, 1.5],
    [4.5, -1.0, 1.2]
])

Kp_pos = 20.0
Kp_ori = 10.0

# Initialize viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting Skydio X2 Gate Navigation Simulation and Recording...")

    waypoint_idx = 0
    target_pos = waypoints[waypoint_idx]

    while viewer.is_running():
        mujoco.mj_step(model, data)

        # Get drone position and velocity
        drone_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'drone')
        drone_pos = data.xpos[drone_body_id]
        drone_vel = data.cvel[drone_body_id][:3]

        # Position control towards target waypoint
        error = target_pos - drone_pos
        force_cmd = Kp_pos * error - 2.0 * drone_vel

        # Apply force to drone
        data.xfrc_applied[drone_body_id][:3] = force_cmd

        # Check if close to waypoint
        if np.linalg.norm(error) < 0.3:
            waypoint_idx += 1
            if waypoint_idx >= len(waypoints):
                print("Gate course completed!")
                break
            target_pos = waypoints[waypoint_idx]

        # Capture frame
        frame = viewer.read_pixels()
        frames.append(frame)

        viewer.sync()

    print("Simulation Ended, Saving GIF...")

# Save frames to GIF
output_path = 'media/skydio_gate.gif'
imageio.mimsave(output_path, frames, fps=20)
print(f"GIF saved to {output_path}")


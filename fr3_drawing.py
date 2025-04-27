# fr3_drawing.py

import mujoco
import mujoco.viewer
import numpy as np
import time

# Load the model
model = mujoco.MjModel.from_xml_path('fr3_drawing_scene.xml')
data = mujoco.MjData(model)

# Define waypoints for the drawing (simple square robot face)
drawing_path = [
    [0.5, 0.0, 0.7],   # Top Left
    [0.7, 0.0, 0.7],   # Top Right
    [0.7, 0.0, 0.5],   # Bottom Right
    [0.5, 0.0, 0.5],   # Bottom Left
    [0.5, 0.0, 0.7]    # Back to Top Left
]

drawing_path = np.array(drawing_path)

# Controller gains
Kp = 5.0

# Initialize viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Starting FR3 Drawing Simulation")
    
    # Sim loop
    waypoint_idx = 0
    target_pos = drawing_path[waypoint_idx]

    while viewer.is_running():
        mujoco.mj_step(model, data)

        # Current end-effector pos (site attached to gripper)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_site')
        current_pos = data.site_xpos[site_id]

        # Position error
        error = target_pos - current_pos

        # Compute joint torques (simple PD)
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, None, site_id)
        torque = jacp.T @ (Kp * error)

        data.ctrl[:7] = torque[:7]

        # Check if close to waypoint
        if np.linalg.norm(error) < 0.01:
            waypoint_idx += 1
            if waypoint_idx >= len(drawing_path):
                print("Drawing complete!")
                break
            target_pos = drawing_path[waypoint_idx]

        viewer.sync()

    print("Simulation Ended")

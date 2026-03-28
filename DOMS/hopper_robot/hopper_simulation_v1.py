import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time

# Load model
model = mujoco.MjModel.from_xml_path("DOMS/hopper_robot/two_link_hopper.xml")
data = mujoco.MjData(model)

# Get joint indices
hip_qadr = model.jnt_qposadr[model.joint("hip").id]
knee_qadr = model.jnt_qposadr[model.joint("knee").id]
z_qadr = model.jnt_qposadr[model.joint("z_slide").id]
hip_vadr = model.jnt_dofadr[model.joint("hip").id]
knee_vadr = model.jnt_dofadr[model.joint("knee").id]

# PD gains
kp = 150
kd = 15

# Set initial crouched position
data.qpos[hip_qadr] = np.radians(-20)
data.qpos[knee_qadr] = np.radians(40)
mujoco.mj_forward(model, data)

# Logging
t_log, z_log, hip_log, knee_log = [], [], [], []

# Run simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    
    while viewer.is_running() and data.time < 10:
        t = data.time
        
        # Same trajectories as working version
        hip_desired = 30 * np.sin(2 * np.pi * 2.0 * t)
        knee_desired = 45 * np.sin(2 * np.pi * 2.0 * t + np.pi)
        
        # PD control
        hip_torque = kp * (np.radians(hip_desired) - data.qpos[hip_qadr]) - kd * data.qvel[hip_vadr]
        knee_torque = kp * (np.radians(knee_desired) - data.qpos[knee_qadr]) - kd * data.qvel[knee_vadr]
        
        # Apply torques
        data.ctrl[0] = hip_torque
        data.ctrl[1] = knee_torque
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Log
        t_log.append(t)
        z_log.append(data.qpos[z_qadr])
        hip_log.append(np.degrees(data.qpos[hip_qadr]))
        knee_log.append(np.degrees(data.qpos[knee_qadr]))
        
        viewer.sync()

# Plot
plt.figure(figsize=(10, 8))

plt.subplot(2,1,1)
plt.plot(t_log, z_log)
plt.title('Base Height')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.grid()

plt.subplot(2,1,2)
plt.plot(t_log, hip_log, label='Hip')
plt.plot(t_log, knee_log, label='Knee')
plt.title('Joint Angles')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

print(f"Max height: {max(z_log):.3f} m")
print(f"Min height: {min(z_log):.3f} m")
print(f"Jump height: {max(z_log)-min(z_log):.3f} m")
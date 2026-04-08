import mujoco
import mujoco_viewer
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time

class TwoLinkHopperSimulator:
    def __init__(self, xml_path="DOMS/hopper_robot/two_link_hopper_2_dof.xml"):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        
        # Data logging
        self.time_history = []
        self.x_history = []  # NEW: Track horizontal position
        self.height_history = []
        self.hip_angle_history = []
        self.knee_angle_history = []
        
        # Control parameters
        self.hip_kp = 150
        self.hip_kd = 15
        self.knee_kp = 150
        self.knee_kd = 15
        
        # Get joint indices properly (CRITICAL for new model)
        self.x_qadr = self.model.jnt_qposadr[self.model.joint("x_slide").id]  # NEW
        self.z_qadr = self.model.jnt_qposadr[self.model.joint("z_slide").id]
        self.hip_qadr = self.model.jnt_qposadr[self.model.joint("hip").id]
        self.knee_qadr = self.model.jnt_qposadr[self.model.joint("knee").id]
        
        self.hip_vadr = self.model.jnt_dofadr[self.model.joint("hip").id]
        self.knee_vadr = self.model.jnt_dofadr[self.model.joint("knee").id]
        
    def reset(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        
    def get_base_height(self):
        """Get height of the base body"""
        return self.data.qpos[self.z_qadr]  # Fixed indexing
    
    def get_base_x(self):  # NEW
        """Get horizontal position of the base body"""
        return self.data.qpos[self.x_qadr]
    
    def get_joint_angles(self):
        """Get current joint angles"""
        return self.data.qpos[self.hip_qadr], self.data.qpos[self.knee_qadr]  # Fixed indexing
    
    def get_joint_velocities(self):  # NEW
        """Get current joint velocities"""
        return self.data.qvel[self.hip_vadr], self.data.qvel[self.knee_vadr]
    
    def compute_hopping_control(self, t, desired_frequency=2.0):
        """Compute control torques for hopping forward"""
        # Desired trajectories - MODIFIED with forward lean
        # Added -10 degree offset to hip to lean forward
        hip_desired = 30 * np.sin(2 * np.pi * desired_frequency * t) - 10  # Lean forward
        knee_desired = 45 * np.sin(2 * np.pi * desired_frequency * t + np.pi) + 10  # Adjust knee
        
        # Convert to radians
        hip_desired_rad = np.radians(hip_desired)
        knee_desired_rad = np.radians(knee_desired)
        
        # Get current states using proper indices
        hip_current, knee_current = self.get_joint_angles()
        hip_vel, knee_vel = self.get_joint_velocities()  # Fixed velocity
        
        # PD control
        hip_torque = self.hip_kp * (hip_desired_rad - hip_current) - self.hip_kd * hip_vel
        knee_torque = self.knee_kp * (knee_desired_rad - knee_current) - self.knee_kd * knee_vel
        
        return hip_torque, knee_torque
    
    def run(self, duration=30, control=True):
        """Run simulation for specified duration"""
        self.reset()
        
        # CRITICAL: Set initial crouched position with forward lean
        self.data.qpos[self.hip_qadr] = np.radians(-30)  # More bent, leaning forward
        self.data.qpos[self.knee_qadr] = np.radians(50)  # More bent knee
        mujoco.mj_forward(self.model, self.data)
        
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
        print(f"Running simulation for {duration} seconds...")
        print(f"Control enabled: {control}")
        print(f"Initial hip angle: {np.degrees(self.data.qpos[self.hip_qadr]):.1f}°")
        print(f"Initial knee angle: {np.degrees(self.data.qpos[self.knee_qadr]):.1f}°")
        
        step = 0
        try:
            while self.viewer.is_alive and self.data.time < duration:
                t = self.data.time
                
                if control:
                    # Apply control
                    hip_torque, knee_torque = self.compute_hopping_control(t)
                    self.data.ctrl[0] = hip_torque
                    self.data.ctrl[1] = knee_torque
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                
                # Log data every 10 steps
                if step % 10 == 0:
                    self.time_history.append(t)
                    self.x_history.append(self.get_base_x())  # NEW: Log x position
                    self.height_history.append(self.get_base_height())
                    hip_angle, knee_angle = self.get_joint_angles()
                    self.hip_angle_history.append(np.degrees(hip_angle))
                    self.knee_angle_history.append(np.degrees(knee_angle))
                
                # Render
                self.viewer.render()
                step += 1
                
        except KeyboardInterrupt:
            print("\nSimulation interrupted")
        finally:
            self.viewer.close()
            
    def plot_results(self):
        """Plot simulation results - MODIFIED to include x position"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        # Height plot
        axes[0, 0].plot(self.time_history, self.height_history)
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Base Height (m)')
        axes[0, 0].set_title('Base Height vs Time')
        axes[0, 0].grid(True)
        
        # NEW: Horizontal position plot
        axes[0, 1].plot(self.time_history, self.x_history, color='green')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('X Position (m)')
        axes[0, 1].set_title('Forward Motion vs Time')
        axes[0, 1].grid(True)
        
        # Hip angle plot
        axes[1, 0].plot(self.time_history, self.hip_angle_history, label='hip')
        axes[1, 0].plot(self.time_history, self.knee_angle_history, label='knee')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Angle (deg)')
        axes[1, 0].set_title('Joint Angles vs Time')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # Trajectory plot (x vs height)
        axes[1, 1].plot(self.x_history, self.height_history)
        axes[1, 1].set_xlabel('X Position (m)')
        axes[1, 1].set_ylabel('Height (m)')
        axes[1, 1].set_title('Trajectory in X-Z Plane')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()

# Main execution
if __name__ == "__main__":
    # Create simulator
    sim = TwoLinkHopperSimulator("DOMS/hopper_robot/two_link_hopper_2_dof.xml")
    
    # Run simulation
    sim.run(duration=5, control=True)
    
    # Plot results
    sim.plot_results()
    
    # Print jump statistics
    if sim.height_history and sim.x_history:
        print(f"\nJump Statistics:")
        print(f"Max height: {max(sim.height_history):.3f} m")
        print(f"Min height: {min(sim.height_history):.3f} m")
        print(f"Jump height: {max(sim.height_history)-min(sim.height_history):.3f} m")
        print(f"Forward distance: {sim.x_history[-1] - sim.x_history[0]:.3f} m")
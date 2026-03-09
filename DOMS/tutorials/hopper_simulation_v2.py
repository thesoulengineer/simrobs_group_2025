import mujoco
import mujoco_viewer
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time

class TwoLinkHopperSimulator:
    def __init__(self, xml_path="DOMS/tutorials/two_link_hopper.xml"):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        
        # Data logging
        self.time_history = []
        self.height_history = []
        self.hip_angle_history = []
        self.knee_angle_history = []
        
        # Control parameters
        self.hip_kp = 150
        self.hip_kd = 15
        self.knee_kp = 150
        self.knee_kd = 15
        
    def reset(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        
    def get_base_height(self):
        """Get height of the base body"""
        return self.data.qpos[0]  # z_slide joint position
    
    def get_joint_angles(self):
        """Get current joint angles"""
        return self.data.qpos[1], self.data.qpos[2]  # hip, knee
    
    def compute_hopping_control(self, t, desired_frequency=2.0):
        """Compute control torques for hopping"""
        # Desired trajectories
        hip_desired = 30 * np.sin(2 * np.pi * desired_frequency * t)
        knee_desired = 45 * np.sin(2 * np.pi * desired_frequency * t + np.pi)
        
        # Convert to radians
        hip_desired_rad = np.radians(hip_desired)
        knee_desired_rad = np.radians(knee_desired)
        
        # Get current states
        hip_current, knee_current = self.get_joint_angles()
        hip_vel = self.data.qvel[1]
        knee_vel = self.data.qvel[2]
        
        # PD control
        hip_torque = self.hip_kp * (hip_desired_rad - hip_current) - self.hip_kd * hip_vel
        knee_torque = self.knee_kp * (knee_desired_rad - knee_current) - self.hip_kd * knee_vel
        
        return hip_torque, knee_torque
    
    def run(self, duration=30, control=True):
        """Run simulation for specified duration"""
        self.reset()
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
        print(f"Running simulation for {duration} seconds...")
        print(f"Control enabled: {control}")
        
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
        """Plot simulation results"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        # Height plot
        axes[0, 0].plot(self.time_history, self.height_history)
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Base Height (m)')
        axes[0, 0].set_title('Base Height vs Time')
        axes[0, 0].grid(True)
        
        # Hip angle plot
        axes[0, 1].plot(self.time_history, self.hip_angle_history)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Hip Angle (deg)')
        axes[0, 1].set_title('Hip Angle vs Time')
        axes[0, 1].grid(True)
        
        # Knee angle plot
        axes[1, 0].plot(self.time_history, self.knee_angle_history)
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Knee Angle (deg)')
        axes[1, 0].set_title('Knee Angle vs Time')
        axes[1, 0].grid(True)
        
        # Trajectory plot (optional)
        axes[1, 1].plot(self.hip_angle_history, self.knee_angle_history)
        axes[1, 1].set_xlabel('Hip Angle (deg)')
        axes[1, 1].set_ylabel('Knee Angle (deg)')
        axes[1, 1].set_title('Joint Space Trajectory')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()

# Main execution
if __name__ == "__main__":
    # Create simulator
    sim = TwoLinkHopperSimulator("DOMS/tutorials/two_link_hopper.xml")
    
    # Run simulation
    sim.run(duration=5, control=True)
    
    # Plot results
    sim.plot_results()
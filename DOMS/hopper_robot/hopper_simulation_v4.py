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
        self.x_history = []  # Track horizontal position
        self.height_history = []
        self.hip_angle_history = []
        self.knee_angle_history = []
        self.work_history = []  # NEW: Track cumulative mechanical work
        
        # Control parameters
        self.hip_kp = 150
        self.hip_kd = 15
        self.knee_kp = 150
        self.knee_kd = 15
        
        # Get joint indices properly
        self.x_qadr = self.model.jnt_qposadr[self.model.joint("x_slide").id]
        self.z_qadr = self.model.jnt_qposadr[self.model.joint("z_slide").id]
        self.hip_qadr = self.model.jnt_qposadr[self.model.joint("hip").id]
        self.knee_qadr = self.model.jnt_qposadr[self.model.joint("knee").id]
        
        self.hip_vadr = self.model.jnt_dofadr[self.model.joint("hip").id]
        self.knee_vadr = self.model.jnt_dofadr[self.model.joint("knee").id]
        
        # NEW: Work tracking variable
        self.cumulative_work = 0.0
        
    def reset(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        
    def get_base_height(self):
        """Get height of the base body"""
        return self.data.qpos[self.z_qadr]
    
    def get_base_x(self):
        """Get horizontal position of the base body"""
        return self.data.qpos[self.x_qadr]
    
    def get_joint_angles(self):
        """Get current joint angles"""
        return self.data.qpos[self.hip_qadr], self.data.qpos[self.knee_qadr]
    
    def get_joint_velocities(self):
        """Get current joint velocities"""
        return self.data.qvel[self.hip_vadr], self.data.qvel[self.knee_vadr]
    
    def compute_hopping_control(self, t, desired_frequency=2.0):
        """Compute control torques for hopping forward"""
        # Desired trajectories with forward lean
        hip_desired = 30 * np.sin(2 * np.pi * desired_frequency * t) + 10  # Lean forward
        knee_desired = 45 * np.sin(2 * np.pi * desired_frequency * t + np.pi) + 10  # Adjust knee
        
        # Convert to radians
        hip_desired_rad = np.radians(hip_desired)
        knee_desired_rad = np.radians(knee_desired)
        
        # Get current states
        hip_current, knee_current = self.get_joint_angles()
        hip_vel, knee_vel = self.get_joint_velocities()
        
        # PD control
        hip_torque = self.hip_kp * (hip_desired_rad - hip_current) - self.hip_kd * hip_vel
        knee_torque = self.knee_kp * (knee_desired_rad - knee_current) - self.knee_kd * knee_vel
        
        return hip_torque, knee_torque
    
    # NEW: Method to compute mechanical cost of transport
    def compute_mechanical_cot(self):
        """
        Compute mechanical Cost of Transport (COT)
        COT = Mechanical Work / (Weight × Distance)
        """
        if len(self.work_history) < 2 or len(self.x_history) < 2:
            return 0.0
        
        total_work = self.work_history[-1]  # Total cumulative work
        distance = self.x_history[-1] - self.x_history[0]  # Total horizontal distance
        
        # Weight = mass * gravity
        total_mass = np.sum(self.model.body_mass)  # Total system mass
        weight = total_mass * 9.81  # Weight in Newtons
        
        if distance > 0 and weight > 0:
            cot = total_work / (weight * distance)
            return cot
        else:
            return 0.0
    
    def run(self, duration=30, control=True):
        """Run simulation for specified duration"""
        self.reset()
        
        # NEW: Reset work tracking
        self.work_history = [0.0]
        self.cumulative_work = 0.0
        
        # Set initial crouched position with forward lean
        self.data.qpos[self.hip_qadr] = np.radians(-30)
        self.data.qpos[self.knee_qadr] = np.radians(50)
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
                
                # NEW: Track mechanical work (power * timestep)
                # Mechanical power = torque * velocity for each joint
                hip_power = abs(self.data.ctrl[0] * self.data.qvel[self.hip_vadr])
                knee_power = abs(self.data.ctrl[1] * self.data.qvel[self.knee_vadr])
                total_power = hip_power + knee_power
                
                # Work = power * time (using simulation timestep)
                work_dt = total_power * self.model.opt.timestep
                self.cumulative_work += work_dt
                
                # Log data every 10 steps
                if step % 10 == 0:
                    self.time_history.append(t)
                    self.x_history.append(self.get_base_x())
                    self.height_history.append(self.get_base_height())
                    hip_angle, knee_angle = self.get_joint_angles()
                    self.hip_angle_history.append(np.degrees(hip_angle))
                    self.knee_angle_history.append(np.degrees(knee_angle))
                    self.work_history.append(self.cumulative_work)  # NEW: Log work
                
                # Render
                self.viewer.render()
                step += 1
                
        except KeyboardInterrupt:
            print("\nSimulation interrupted")
        finally:
            self.viewer.close()
            
            # NEW: Print COT at the end
            if len(self.x_history) > 1:
                distance = self.x_history[-1] - self.x_history[0]
                cot = self.compute_mechanical_cot()
                print(f"\nSimulation Summary:")
                print(f"Total mechanical work: {self.cumulative_work:.2f} J")
                print(f"Total distance: {distance:.3f} m")
                print(f"Mechanical Cost of Transport: {cot:.4f}")
    
    def plot_results(self):
        """Plot simulation results - MODIFIED to include work"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        
        # Height plot
        axes[0, 0].plot(self.time_history, self.height_history)
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Base Height (m)')
        axes[0, 0].set_title('Base Height vs Time')
        axes[0, 0].grid(True)
        
        # Horizontal position plot
        axes[0, 1].plot(self.time_history, self.x_history, color='green')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('X Position (m)')
        axes[0, 1].set_title('Forward Motion vs Time')
        axes[0, 1].grid(True)
        
        # Joint angles plot
        axes[1, 0].plot(self.time_history, self.hip_angle_history, label='hip')
        axes[1, 0].plot(self.time_history, self.knee_angle_history, label='knee')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Angle (deg)')
        axes[1, 0].set_title('Joint Angles vs Time')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # NEW: Mechanical work plot (instead of trajectory)
        if len(self.work_history) > 1:
            work_time = self.time_history[:len(self.work_history)-1]
            axes[1, 1].plot(work_time, self.work_history[1:], color='purple')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Cumulative Work (J)')
            axes[1, 1].set_title('Mechanical Work vs Time')
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
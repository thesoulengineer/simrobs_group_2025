import sys
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from src.motor_driver.canmotorlib import CanMotorController


@dataclass
class MotorState:
    pos: float
    vel: float
    curr: float

    @property
    def state(self):
        return self.pos, self.vel, self.curr

    @state.setter
    def state(self, new_state):
        self.pos = new_state[0]
        self.vel = new_state[1]
        self.curr = new_state[2]
    
    def __str__(self) -> str:
        msg = f"Actual state - pos: {round(self.pos, 3)}, vel: {round(self.vel, 3)}, curr: {round(self.curr, 3)}"
        return msg
    
    # FOR motor AK45_10
joint_controller = CanMotorController(can_socket="can0",
                                        motor_id=1,
                                        motor_type="AK45_10_V1")

print("Enabling Motors..")
pos, vel, curr = joint_controller.enable_motor()
joint_obs = MotorState(pos, vel, curr)
print(f"Actual motor state: {joint_obs}")

joint_obs.state = joint_controller.set_zero_position()
print(f"Actual motor state: {joint_obs}")

sim_time = 10
dt = 1/25
sim_time_line = np.arange(0, sim_time, dt)
sin_wave = np.pi/4 * np.sin(2 * np.pi * sim_time_line)

data_line = []

for i, time_stamp in enumerate(sim_time_line):
    # joint_obs.state = joint_controller.send_rad_command(0, 0, 0, 0, 0)
    # OR
    joint_obs.state = joint_controller.send_rad_command(sin_wave[i], 0, 1, 0, 0)

    print(f"Time: {round(time_stamp, 3)}s, {joint_obs}")

    data_line.append(joint_obs.pos)

    time.sleep(dt)

joint_controller.disable_motor()
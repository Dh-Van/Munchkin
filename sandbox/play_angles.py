from robot import Robot
from control import Controller
import numpy as np
import signal, sys
import pickle
import time

num_dof = 4
arm_lengths = [2, 8, 8, 6]

robot = Robot(num_dof, arm_lengths, enable=True)
controller = Controller(num_dof, arm_lengths)

def estop(sig, frame):
    robot.disable()
    sys.exit(0)

def safe_estop(sig, frame):
    robot.set_theta_list(zero_angles)
    sys.exit(0)

signal.signal(signal.SIGINT, estop)
signal.signal(signal.SIGTSTP, safe_estop)

zero_position = np.array([7.0, 0, 8.8])
zero_angles = controller.numerical_inverse_kinematics(robot.get_theta_list(), zero_position)
robot.set_theta_list(zero_angles)

with open("WORKING_POINTS.pkl", 'rb') as file:
    recorded_angles = pickle.load(file)

x_ms = 20
dt = x_ms / 1000.0
original_dt = 0.008067

last_time = time.perf_counter()
elapsed_traj_time = 0

traj = 0
i = 0

while True:
    now = time.perf_counter()
    loop_elapsed = now - last_time

    if loop_elapsed >= dt:
        last_time = now

        if i > 10 and traj < len(recorded_angles):
            elapsed_traj_time += loop_elapsed
            traj = int(elapsed_traj_time / original_dt)

            if traj < len(recorded_angles):
                traj_i, traj_angle = recorded_angles[traj]
                robot.set_theta_list(traj_angle)

        i += 1

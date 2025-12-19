from robot import Robot
from control import Controller
import numpy as np
import time

robot = Robot(4, [2, 8, 8, 7.35])
controller = Controller(4, [2, 8, 8, 7.35])

# Back to servo mode
for servo in robot.servos:
    servo.servo_mode()
    servo.enable_torque()

# Control loop timing
control_rate = 1  # Hz - fast updates for your control algorithm
dt = 1.0 / control_rate  # Time between commands

# Servo interpolation time - LONGER than dt for smoothness
servo_move_time = 1000  # 200ms - servos take this long to reach target

max_joint_velocity = np.radians(30)  # Safety limit

# Track target position for error calculation
target_theta = robot.get_theta_list()

try:
    while True:
        loop_start = time.time()
        
        # Get current state
        thetalist = robot.get_theta_list()
        
        # Desired end-effector velocity (units/second)
        ee_vel = np.array([0.0, 10.0, 0.0])
        
        # Calculate joint velocities
        thetadot = controller.calc_damped_inverse_jacobian(thetalist) @ ee_vel
        
        # Apply safety limits
        # thetadot = np.clip(thetadot, -max_joint_velocity, max_joint_velocity)
        
        # Calculate target position for next timestep
        new_thetalist = thetalist + thetadot * dt
        target_theta = new_thetalist
        
        # Calculate tracking error
        error = target_theta - thetalist
        
        # Print diagnostics
        print(f"\nJoint velocities (deg/s): {np.degrees(thetadot)}")
        print(f"Current angles (deg):     {np.degrees(thetalist)}")
        print(f"Target angles (deg):      {np.degrees(target_theta)}")
        print(f"Tracking error (deg):     {np.degrees(error)}")
        
        # Command servos to reach this position in servo_move_time ms
        robot.set_theta_list(new_thetalist, move_time=servo_move_time)
        
        # Sleep for the remainder of the control period
        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    robot.disable()
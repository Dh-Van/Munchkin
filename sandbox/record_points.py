from robot import Robot
from control import Controller
import numpy as np
import signal, sys
import pickle
    

num_dof = 4
arm_lengths = [2, 8, 8, 6]

robot = Robot(num_dof, arm_lengths, enable=True)
controller = Controller(num_dof, arm_lengths)

def estop(sig, frame):
    robot.disable()
    sys.exit(0)
    
zero_position = np.array([7.0,   0,    8.8])
zero_angles = controller.numerical_inverse_kinematics(robot.get_theta_list(), zero_position)
    
def safe_estop(sig, frame):
    robot.set_theta_list(zero_angles)
    sys.exit(0)
    
# signal.signal(signal.SIGINT, estop)
# signal.signal(signal.SIGTSTP, safe_estop)
    
robot.set_theta_list(zero_angles)


recorded_angles = []
i = 0
try:
    while True:
        # if(i < 15):
        #     robot.set_theta_list(zero_angles)
        if(i > 15 and i < 20):
            robot.disable()
        if(i > 40):
            print("RECORDING")
            angles = robot.get_theta_list()
            position = controller.forward_kinematics(angles)
            
            recorded_angles.append((i, angles))
        i += 1
except KeyboardInterrupt:
    with open("points.pkl", 'wb') as file:
        pickle.dump(recorded_angles, file)
        print("recorded all angles")
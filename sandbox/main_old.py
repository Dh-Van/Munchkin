from robot import Robot
from control import Controller
import numpy as np

num_dof = 4
arm_lengths = [2, 8, 8, 6]

robot = Robot(num_dof, arm_lengths)
controller = Controller(num_dof, arm_lengths)

test_pos = np.array([15.05, 0, 9.6])
test_pos = np.array([4.28, -0.02, 19.03])

# test_pos = np.array([15.37, 0, 1.86])
# init_angles = robot.get_theta_list()
# init_pos = controller.forward_kinematics(init_angles)[:3]
# final_pos = np.array([8, 15, 6], dtype=np.float64)

# robot.set_theta_list(controller.numerical_inverse_kinematics(init_angles, init_pos))

# points = controller.create_trajectory(init_pos, final_pos)
# trajectory_list = controller.trajectory_to_theta_list(init_angles, points)
# robot.set_theta_list(np.array([-0.184, 0.72, -2.341, 0.695]))

i = 0
while True:
    # if(i < len(trajectory_list)):
    #     theta_list = trajectory_list[i]
    #     robot.set_theta_list(theta_list)
    angles = robot.get_theta_list(False, False)
    print(angles)
    curr_pos = controller.forward_kinematics(angles)
    # print(curr_pos)
    new_pos = [curr_pos[0], curr_pos[1], curr_pos[2] + 0.1]
    new_angles = controller.numerical_inverse_kinematics(angles, new_pos)
    # # print(new_angles - angles)
    # robot.set_theta_list(new_angles)
        
    i += 1

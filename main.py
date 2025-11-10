from arm import Arm

arm = Arm(4)

while True:
    theta_list = arm.get_theta_list()
    # print(theta_list)
    print(arm.forward_kinematics(theta_list))
from pylx16a.lx16a import *
import constants
import numpy as np

class Arm:
    
    def __init__(self, num_dof):
        self.num_dof = num_dof
        self.servos: list[LX16A] = []
        
        LX16A.initialize("/dev/ttyUSB0")
        for i in range(num_dof):
            servo = LX16A(i)
            # servo.enable_torque()
            servo.disable_torque()
            self.servos.append(servo)
            
    def generate_dh_table(self, theta_list):
        DH = np.zeros(shape = (4, 4))
        
        # theta, d, alpha, a
        DH[0] = [theta_list[0], constants.PLATE[2], np.pi / 2, constants.PLATE[0]]
        DH[1] = [theta_list[1] - 1.4535102, 0, 0, constants.ARM_LENGTHS[0]]
        DH[2] = [theta_list[2] - 1.31946891, 0, 0, constants.ARM_LENGTHS[1]]
        DH[3] = [theta_list[3] - 1.95197624, 0, 0, constants.ARM_LENGTHS[2]]
        
        return DH
    
    def dh_to_transformation_matrix(self, dh_table):
        theta, d, alpha, a = dh_table
        
        T = np.zeros(shape = (4, 4))
        T[0] = [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)]
        T[1] = [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)]
        T[2] = [0, np.sin(alpha), np.cos(alpha), d]
        T[3] = [0, 0, 0, 1]
        
        return T
    
    def forward_kinematics(self, theta_list, radians = False):
        DH = self.generate_dh_table(theta_list)
        
        T = np.zeros(shape = (4, 4, 4))
        for i in range(4):
            T[i] = self.dh_to_transformation_matrix(DH[i])
            
        return T[0] @ T[1] @ T[2] @ T[3] @ np.array([0, 0, 0, 1])
    
    def get_theta_list(self):
        theta_list = np.zeros(self.num_dof)
        # offsets = [0, 1.4535102, 1.31946891, 1.95197624]
        offsets = [0, 0, 0, 0]
        for i in range(self.num_dof):
            angle = np.radians(self.servos[i].get_physical_angle()) - offsets[i]
            # angle = self.servos[i].get_physical_angle()
            theta_list[i] = np.round(angle, decimals = 2)
            
        return theta_list
            
    def configure_servos(self, servo_func_name, arg=None):
        for servo in self.servos:
            servo_func = getattr(servo, servo_func_name)
            servo_func(arg)
            
    def independent_control_setpoint(self, setpoint):
        for i, servo in enumerate(self.servos):
            servo.move(setpoint[i])
import numpy as np
from pylx16a.lx16a import *

class Robot:
    
    def __init__(self, num_dof, arm_lengths, enable = True):
        self.num_dof = num_dof
        self.arm_lengths = arm_lengths
        
        self.servos: list[LX16A] = []
        self.errors = []
        self.gains = [0, 0, 0]
        
        # self.offsets = np.array([2.237, 1.495, 2.857, 1.939])
        self.offsets = np.array([2.932, 1.454, 2.874, 1.914])

        
        LX16A.initialize('/dev/ttyUSB0')
        for i in range(num_dof):
            servo = LX16A(i)
            if enable: 
                servo.enable_torque()
            else: 
                servo.disable_torque()
            
            self.servos.append(servo)
            
            
    def get_theta_list(self, radians=True, offset=True):
        theta_list = []
        offsets = self.offsets if offset else [0, 0, 0, 0]
        
        for i, servo in enumerate(self.servos):
            angle = np.radians(servo.get_physical_angle()) if radians else servo.get_physical_angle()
            angle -= offsets[i] if radians else np.degrees(offsets[i])
            theta_list.append(angle)
            
        return np.round(np.array(theta_list), 3)
    
    def set_theta_list(self, theta_list, radians=True, time=0):
        theta_list = theta_list if radians else np.radians(theta_list)
        theta_list = theta_list + self.offsets
                
        for i, angle in enumerate(theta_list):
            angle = np.degrees(angle)
            clipped_angle = self.get_clipped_angle(angle)
            
            self.servos[i].move(clipped_angle, int(time))
            
        # for servo in self.servos:
        #     servo.move_start()
                
    def get_clipped_angle(self, angle, max_angle = 240):
        angle = np.mod(angle, 360)
        gap = (max_angle + 360) / 2
        
        clipped_angle = np.where(
            (angle > max_angle) & (angle > gap),
            0,
            angle
        )
        
        clipped_angle = np.where(
            (clipped_angle > max_angle) & (clipped_angle <= gap),
            max_angle,
            clipped_angle
        )
        
        return clipped_angle
        
    def disable(self):
        for servo in self.servos:
            servo.disable_torque()

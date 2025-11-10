from pylx16a.lx16a import *

class Arm:
    
    def __init__(self, num_dof):
        self.num_dof = num_dof
        
        LX16A.initialize("/dev/ttyUSB0")
        for i in range(num_dof):
            servo = LX16A(i)
            servo.enable_torque()
            self.servos.append(servo)
            
            
    def configure_servos(self, servo_func_name, arg=None):
        for servo in self.servos:
            servo_func = getattr(servo, servo_func_name)
            servo_func(arg)
            
    def independent_control_setpoint(self, setpoint):
        for i, servo in self.servos:
            servo.move(setpoint[i])
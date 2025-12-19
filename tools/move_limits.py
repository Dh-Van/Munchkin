from pylx16a.lx16a import *

servos: list[LX16A] = []

LX16A.initialize('/dev/ttyUSB0')
for i in range(4):
    servo = LX16A(i)
    servo.enable_torque()
    
    servos.append(servo)
    
    
servo_index = 0
servo = servos[servo_index]
servo.disable_torque()

while True:
    print(servo.get_physical_angle())

# init_angle = servo.get_physical_angle()

# for i in range(init_angle, 240 - init_angle):
#     servo.move(i)
    
# init_angle = servo.get_physical_angle()
# for i in range(init_angle, 240 - init_angle, -1):
#     servo.move(i)
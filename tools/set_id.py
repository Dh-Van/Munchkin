from pylx16a.lx16a import *
import time
import sys

LX16A.initialize("/dev/ttyUSB0")

for i in range(3):
    servo = LX16A(i)
    servo.enable_torque()

def_servo = LX16A(2)

# def_servo.move(def_servo.get_physical_angle() - 30, time=400)

print([method for method in dir(def_servo) if not method.startswith('_')])

# def_servo.led_power_on()

# NEW_ID = sys.argv[1]
# NEW_ID = int(NEW_ID)

# print(f"Intended servo id: {NEW_ID}")

# def_servo.set_id(NEW_ID)
# time.sleep(0.1)

# print(f"New servo id: {NEW_ID}")
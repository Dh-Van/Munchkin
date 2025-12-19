import redis, json, pickle, signal, sys
from robot import Robot
from control import Controller
from enum import Enum
from time import time, sleep
import numpy as np

NUM_DOF = 4
ARM_LENGTHS = [2, 8, 8, 7.35]

# How fast the loop runs, in Hz
LOOP_FREQUENCY = 200
PERIOD = 1.0 / LOOP_FREQUENCY

kP = 0.1

class ARM_STATE(Enum):
    DISABLED = 0
    IDLE = 1
    HOME = 2
    SCOOPING = 3
    MOUTH_TRACKING = 4
    RETRACTING = 5
    TRUE_VALUES = 6
    RECORDING = 7
    VEL_KIN = 8
    
    
def main(inital_arm_state = ARM_STATE.HOME):
    robot = Robot(NUM_DOF, ARM_LENGTHS)
    
    controller = Controller(NUM_DOF, ARM_LENGTHS)
    camera_queue = redis.Redis(host='localhost', port=6379, decode_responses=True)
    
    arm_state: ARM_STATE = inital_arm_state
    recorded_angles = []
    i = 0
    
    load_fname = "RECORDED_ANGLES"
    loaded_angles = []
    with(open(f'recordings/{load_fname}.pkl', 'rb') as f):
        loaded_angles = pickle.load(f)
    
    try:
        while True:
            
            start_time = time()
                    
            match(arm_state):
                case ARM_STATE.DISABLED:
                    robot.disable()
                case ARM_STATE.IDLE:
                    pass
                case ARM_STATE.HOME:
                    robot.set_theta_list([ 0.067, 0.599, -2.279, 0.26])
                case ARM_STATE.SCOOPING:
                    if(i > len(loaded_theta_list)):
                        arm_state = ARM_STATE.MOUTH_TRACKING
                    loaded_theta_list = loaded_angles[i]
                    robot.set_theta_list(loaded_theta_list)
                case ARM_STATE.MOUTH_TRACKING:
                    data = json.loads(camera_queue.get("mouth_position"))
                    if(data['tracking'] and data['mouth_open']):

                        raw_x = (-data['x']) - 53
                        raw_y = (-data['y']) + 175

                        print(raw_x, raw_y)
                                                
                        ee_vel = np.array([
                            2.0,
                            2.0 * raw_x,
                            3.0 * raw_y
                        ])
                        
                        theta = robot.get_theta_list()
                        thetadot = controller.calc_damped_inverse_jacobian(theta) @ ee_vel

                        robot.set_theta_list(theta + thetadot * PERIOD)
                        
                case ARM_STATE.RETRACTING:
                    pass
                case ARM_STATE.TRUE_VALUES:
                    robot.disable()
                    print(robot.get_theta_list(True, True))
                case ARM_STATE.RECORDING:
                    robot.disable()
                    angles = robot.get_theta_list()
                    recorded_angles.append(angles)
                case ARM_STATE.VEL_KIN:
                    ee_vel = np.array([0.0, -1, 0.0])
                    
                    theta = robot.get_theta_list()
                    thetadot = controller.calc_damped_inverse_jacobian(theta) @ ee_vel
                    
                    gain = 100

                    robot.set_theta_list(theta + gain * (thetadot * PERIOD))
                    
                    print(gain * thetadot * PERIOD)

                    
            if(arm_state != ARM_STATE.SCOOPING):
                i = 0
                
                
            sleep_time = PERIOD - time() + start_time
            if(sleep_time > 0):
                sleep(sleep_time)
                
            i += 1
                
    except KeyboardInterrupt:
        robot.disable()
        if(len(recorded_angles) > 0):
            fname = "RECORDED_ANGLES"
            with open(f'recordings/{fname}.pkl', 'wb') as f:
                pickle.dump(recorded_angles, f)
        sys.exit(0)

if __name__ == "__main__":
    # main(ARM_STATE.VEL_KIN)
    main(ARM_STATE.DISABLED)
    # main(ARM_STATE.MOUTH_TRACKING)
    # main(ARM_STATE.V)

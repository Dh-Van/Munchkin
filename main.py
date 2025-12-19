import time, json, redis, sys
import numpy as np
from robot import Robot
from control import Controller

NUM_DOF = 4
ARM_LENGTHS = [2, 8, 8, 7.35]
HZ = 200
DT = 1.0 / HZ

ee_vel = np.array([0.0, 0.0, 0.0])

def main():
    robot = Robot(NUM_DOF, ARM_LENGTHS, enable=True)
    controller = Controller(NUM_DOF, ARM_LENGTHS)
    camera_queue = redis.Redis(host='localhost', port=6379, decode_responses=True)

    theta = robot.get_theta_list()
    
    ee_vel = np.array([0.0, 0.0, 0.0])

    try:
        while True:
            data = json.loads(camera_queue.get("mouth_position"))

            if(data['tracking']):
                cam_x = data['x']
                cam_y = data['y']
                cam_depth = data['z']
                
                print(cam_depth)
                
                if(cam_depth > 20):
                    ee_vel[0] = 1.0
                    ee_vel[1] = -cam_x * 0.1
                    ee_vel[2] = -cam_y * 0.1
            else:
                ee_vel = np.array([0.0, 0.0, 0.0])
                

            theta_next = controller.velocity_kinematics(theta, ee_vel * DT)

            robot.set_theta_list(theta_next)

            theta = theta_next

            time.sleep(DT)
    except KeyboardInterrupt:
        robot.disable()
        sys.exit(0)

if __name__ == "__main__":
    main()

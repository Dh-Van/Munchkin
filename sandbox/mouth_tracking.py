import numpy as np
import redis
import json
import sys
from time import sleep, time
from pylx16a.lx16a import *

# ==================================
#            CONFIGURATION
# ==================================

# SERVO CONFIGURATION
# Set the correct port for your LX-16A bus controller
SERVO_PORT = '/dev/ttyUSB0'
SERVO_IDS = [0, 1, 2, 3]  # Servos 0, 1, 2, and 3
NUM_DOF = len(SERVO_IDS)

# CONTROL LOOP PARAMETERS
LOOP_FREQUENCY = 120      # Target control loop frequency
PERIOD = 1.0 / LOOP_FREQUENCY

# GRADIENT DESCENT PARAMETERS
# EPS_SPEED: The change in motor speed (velocity) used to probe the gradient
EPS_SPEED = 50.0          # Range is -1000 to 1000. 50 is a small, measurable perturbation.
STEP = 0.0035             # Learning rate (how quickly speed changes based on error loss)
MAX_STEP_LIMIT = 200.0    # Absolute limit for the motor speed (max velocity magnitude)
SMOOTH = 0.85             # Smoothing factor for the speed update (momentum)

# MOUTH TRACKING OFFSETS
# These are your configured setpoints for X and Y when the arm is correctly positioned.
OFFSET_X = 0
OFFSET_Y = 0

# MANUAL SIGN ADJUSTMENT VECTOR (CRITICAL FOR DIRECTION)
# Set to 1 for correct direction, -1 to invert the direction of movement for a servo.
# You must tune this based on which way your servos physically move the arm.
# Example: If servo 1 moves the arm away from the mouth, change the second element to -1.
VELOCITY_SIGNS = np.array([1, 1, 1, 1], dtype=float) 


# ==================================
#           SERVO CONTROL CLASS
# ==================================

class ServoController:
    """Manages the LX-16A servos for the tracking arm using Motor Mode."""
    def __init__(self, port, ids):
        try:
            # Initialize the bus communication
            LX16A.initialize(port)
        except ServoError as e:
            print(f"Failed to initialize LX16A bus on port {port}: {e}")
            sys.exit(1)

        self.servos = [LX16A(id_) for id_ in ids]
        self.num_dof = len(ids)
        
        # Internal storage for current motor speeds (the control variable, omega)
        self.motor_speeds = np.zeros(self.num_dof) 

        print(f"Initialized {self.num_dof} servos on port {port}.")
        
        # Enable torque and switch to motor mode
        try:
            for servo in self.servos:
                servo.enable_torque()
                # Set initial speed to 0, which also switches mode to motor mode
                servo.motor_mode(0) 
        except ServoError as e:
            print(f"Servo initialization failed: {e}")
            print("Ensure servos are connected, torque is enabled, and IDs are correct.")
            sys.exit(1)

    def set_motor_speeds(self, speeds: np.ndarray):
        """Applies an array of speeds to the servos, clamping to the hardware limits."""
        for i, servo in enumerate(self.servos):
            # Clamp the speed to the valid range for motor_mode: -1000 to 1000
            speed_val = int(np.clip(speeds[i], -1000, 1000))
            # The motor_mode function sets the speed AND keeps it in motor mode
            servo.motor_mode(speed_val)

    def disable(self):
        """Disables torque and stops the servos."""
        print("Disabling servos...")
        for servo in self.servos:
            try:
                # Stop rotation and disable torque
                servo.motor_mode(0) 
                servo.disable_torque()
            except ServoError as e:
                print(f"Warning: Could not disable servo {servo.id_}. {e}")
        print("Servos disabled.")
        
# ==================================
#       TRACKING & GD FUNCTIONS
# ==================================

def get_mouth_error(data):
    """
    Calculates the 3D error vector (Target - Current) from Redis data.
    """
    return np.array([
        (data['x']) - OFFSET_X,       # left/right error (X)
        (data['y']) + OFFSET_Y,      # up/down error (Y)
        data['z']                     # depth error (Z)
    ], dtype=float)


def wait_for_fresh_frame(camera, old_raw):
    """Waits for a new frame of mouth position data from Redis."""
    # Wait time must allow a new camera frame to be generated after the speed perturbation.
    for _ in range(3): # Check up to 3 times (approx 30ms total)
        raw = camera.get("mouth_position")
        if raw != old_raw:
            return raw
        sleep(0.01)
    return old_raw


def gradient_descent_step(servo_controller: ServoController, camera: redis.Redis):
    """
    Performs one step of velocity-based gradient descent.
    """
    
    # 1. GET INITIAL STATE AND LOSS
    raw0 = camera.get("mouth_position")
    if not raw0:
        return

    data = json.loads(raw0)

    # Halt if not tracking or mouth is closed
    if not (data.get("tracking") and data.get("mouth_open")):
        servo_controller.set_motor_speeds(np.zeros(NUM_DOF))
        servo_controller.motor_speeds = np.zeros(NUM_DOF)
        return

    err = get_mouth_error(data)
    
    # Loss function: squared Euclidean norm of the error vector
    loss = err @ err

    current_speeds = servo_controller.motor_speeds.copy()
    grad = np.zeros(NUM_DOF)

    # 2. FINITE DIFFERENCE GRADIENT ESTIMATION (dL/d(speed_j))
    for j in range(NUM_DOF):
        
        # --- Perturbation Step ---
        servo = servo_controller.servos[j]
        
        original_speed_j = current_speeds[j]
        perturbed_speed_j = original_speed_j + EPS_SPEED

        # Apply speed perturbation
        try:
            servo.motor_mode(int(perturbed_speed_j))
        except ServoError:
            grad[j] = 0
            continue # Skip this servo if command fails

        # Wait for NEW camera frame/displacement
        raw_new = wait_for_fresh_frame(camera, raw0)
        
        # Calculate new loss
        if raw_new:
            data_new = json.loads(raw_new)
            err2 = get_mouth_error(data_new)
            loss2 = err2 @ err2
        else:
            # If no new data, assume no change in loss
            loss2 = loss

        # Calculate gradient component: (Change in Loss) / (Change in Speed)
        grad[j] = (loss2 - loss) / EPS_SPEED
        
        # Restore original speed immediately
        try:
            servo.motor_mode(int(original_speed_j))
        except ServoError:
            pass # Ignore restoration failure; rely on the next loop iteration

    # 3. GRADIENT DESCENT UPDATE
    
    # delta_speed = -LearningRate * Gradient
    delta_speed = -STEP * grad
    
    # Apply Smoothing (Momentum): new_speeds = SMOOTH * old_speeds + (1 - SMOOTH) * (old_speeds + delta_speed)
    new_speeds = SMOOTH * current_speeds + (1 - SMOOTH) * (current_speeds + delta_speed)
    
    # CRITICAL CORRECTION: APPLY MANUAL SIGN ADJUSTMENT
    new_speeds *= VELOCITY_SIGNS
    
    # Clamp the speed to the max limit
    new_speeds = np.clip(new_speeds, -MAX_STEP_LIMIT, MAX_STEP_LIMIT)

    # Update the internal state and send commands to the servos
    servo_controller.motor_speeds = new_speeds.copy()
    servo_controller.set_motor_speeds(new_speeds)
    
    print(f"  Loss: {loss:.4f}, Speeds: {new_speeds.round(2)}")


# ==================================
#             MAIN LOOP
# ==================================

def main():
    """Main function to run the velocity-based mouth tracking loop."""
    
    # Initialize Redis connection
    camera = redis.Redis(host="localhost", port=6379, decode_responses=True)
    
    try:
        # Initialize servo controller
        servo_controller = ServoController(SERVO_PORT, SERVO_IDS)
        
        print("\n--- Mouth Tracking Started ---")
        print(f"Tuning needed: Adjust VELOCITY_SIGNS in CONFIGURATION.")
        print(f"Stop with Ctrl+C.\n")

        while True:
            start = time()

            # Perform the core tracking logic
            gradient_descent_step(servo_controller, camera)

            # Control loop frequency limiter
            dt = PERIOD - (time() - start)
            if dt > 0:
                sleep(dt)

    except KeyboardInterrupt:
        pass  # Handled in the finally block
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        
    finally:
        # Ensure servos are stopped and disabled on exit
        if 'servo_controller' in locals():
            servo_controller.disable()
        sys.exit(0)


if __name__ == "__main__":
    main()
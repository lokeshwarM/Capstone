import math
# pyrefly: ignore [missing-import]
from controller import Supervisor

# Initialize the Webots Robot API
robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())

# Initialize Keyboard
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# Get and initialize motors
front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(1.0)

# Get sensors
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)

imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

print(f"[{robot.getName()}] Flight controller started. Click inside the 3D window to fly!")
print("Controls: W/A/S/D (Move) | Q/E (Rotate) | Up/Down Arrows (Altitude)")

# Wait one step for sensors to initialize
robot.step(TIME_STEP)
target_altitude = gps.getValues()[2]
if math.isnan(target_altitude) or target_altitude < 0:
    target_altitude = 5.0 # Fallback

# Force the drone to climb 2 meters at start so we know it's alive!
target_altitude += 2.0

while robot.step(TIME_STEP) != -1:
    # Constants for PID
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    
    # Sensor readings
    roll, pitch, yaw = imu.getRollPitchYaw()
    
    roll_velocity, pitch_velocity, yaw_velocity = gyro.getValues()
    altitude = gps.getValues()[2]
    
    # Read Keyboard Input
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    
    key = keyboard.getKey()
    
    # Check if this drone is currently selected by the user
    is_selected = False
    try:
        selected_node = robot.getSelected()
        my_self = robot.getSelf()
        if selected_node is not None and my_self is not None:
            current_node = selected_node
            while current_node is not None:
                if current_node.getId() == my_self.getId():
                    is_selected = True
                    break
                current_node = current_node.getParentNode()
    except Exception as e:
        print(f"[{robot.getName()}] Selection error: {e}")
        
    while key > 0:
        # TEMP: Apply to ALL drones to verify keyboard is working
        if key == ord('W'):
            pitch_disturbance = -2.0 # Pitch Forward
        elif key == ord('S'):
            pitch_disturbance = 2.0  # Pitch Backward
            
        elif key == ord('A'):
            roll_disturbance = -2.0  # Roll Left
        elif key == ord('D'):
            roll_disturbance = 2.0   # Roll Right
            
        elif key == ord('Q'):
            yaw_disturbance = 1.3    # Yaw Left
        elif key == ord('E'):
            yaw_disturbance = -1.3   # Yaw Right
            
        elif key == keyboard.UP:
            target_altitude += 0.05
        elif key == keyboard.DOWN:
            target_altitude -= 0.05
            
        key = keyboard.getKey()

    # Compute motor velocities (PID for stability)
    # The signs for pitch and roll velocity might need inversion depending on Webots axis
    roll_input = K_ROLL_P * max(min(roll, 1.0), -1.0) + roll_velocity + roll_disturbance
    pitch_input = K_PITCH_P * max(min(pitch, 1.0), -1.0) - pitch_velocity + pitch_disturbance
    yaw_input = yaw_disturbance
    
    # Altitude control
    climb_input = K_VERTICAL_P * (target_altitude - altitude)

    # Base hover velocity for DJI Mavic in Webots
    base_velocity = 68.5 + climb_input

    # Apply mixing to motors
    # Front Left: +pitch, -roll, +yaw
    fl = base_velocity + pitch_input - roll_input + yaw_input
    # Front Right: +pitch, +roll, -yaw
    fr = base_velocity + pitch_input + roll_input - yaw_input
    # Rear Left: -pitch, -roll, -yaw
    rl = base_velocity - pitch_input - roll_input - yaw_input
    # Rear Right: -pitch, +roll, +yaw
    rr = base_velocity - pitch_input + roll_input + yaw_input

    # Cap velocities to physical limits
    motors[0].setVelocity(max(min(fl, 600.0), -600.0))
    motors[1].setVelocity(max(min(fr, 600.0), -600.0))
    motors[2].setVelocity(max(min(rl, 600.0), -600.0))
    motors[3].setVelocity(max(min(rr, 600.0), -600.0))

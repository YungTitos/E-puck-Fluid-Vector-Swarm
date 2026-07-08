import random
from controller import Robot

# SWARM TUNING PARAMETERS 
# Vector Weights
WEIGHT_LIGHT = 0.001   # The Pull
WEIGHT_PROX = 0.0000001    # The Push (Repulsion)
WEIGHT_SLIDE = 0.010   # The Flow (Tangential)
WEIGHT_NOISE = 3.0     # The Wander (Jitter)

# Thresholds & Speeds
BASE_SPEED_FRAC = 0.5        # Percentage of MAX_SPEED to drive forward continuously
SLIDE_THRESHOLD = 300        # Proximity reading required to trigger wall-sliding
LIGHT_STOP_THRESHOLD = 25000  # Total light reading below which the robot stops

# Swarm State Variables
my_slide_dir = random.choice([-1, 1])
noise_timer = 0
noise_left = 0
noise_right = 0
is_finished = False

# 2. INITIALIZATION
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28  # Maximum wheel speed for an e-puck in rad/s


# 3. MOTOR SETUP

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 4. SENSOR SETUP
light_sensors = [robot.getDevice(f'ls{i}') for i in range(8)]
prox_sensors = [robot.getDevice(f'ps{i}') for i in range(8)]

for i in range(8):
    light_sensors[i].enable(TIME_STEP)
    prox_sensors[i].enable(TIME_STEP)

print("Motors, Light, and Proximity Sensors ready.")

# 5. MAIN CONTROL LOOP
while robot.step(TIME_STEP) != -1:
    
    # Read all sensors once per tick to save compute
    light_vals = [ls.getValue() for ls in light_sensors]
    prox_vals = [ps.getValue() for ps in prox_sensors]
    
    total_light = sum(light_vals)
    
    # --- STOP CONDITION ---
    front_prox = prox_vals[0] + prox_vals[7]

    if total_light > 25000:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        if not is_finished:
            print("Robot successfully reached the target zone!")
            is_finished = True
            
        continue
    
    # --- Vector 1: THE PULL (Light) ---
    right_light = light_vals[0] + light_vals[1] + light_vals[2] + light_vals[3]
    left_light = light_vals[7] + light_vals[6] + light_vals[5] + light_vals[4]
    light_difference = left_light - right_light 

    # --- Vector 2: THE PUSH (Repulsion) ---
    right_prox = prox_vals[0] + prox_vals[1] + prox_vals[2]
    left_prox = prox_vals[7] + prox_vals[6] + prox_vals[5]
    prox_difference = right_prox - left_prox

    # --- Vector 3: THE FLOW (Sliding) ---
    front_prox = prox_vals[0] + prox_vals[7]

    # --- Vector 4: THE WANDER (Burst Noise) ---
    noise_timer -= 1
    if noise_timer <= 0:
        noise_left = random.uniform(-2.0, 2.0)
        noise_right = random.uniform(-2.0, 2.0)
        noise_timer = 20

    # --- THE MERGE ---
    left_speed = MAX_SPEED * BASE_SPEED_FRAC
    right_speed = MAX_SPEED * BASE_SPEED_FRAC
    
    # 1. Apply Light Vector (Always active)
    left_speed -= (light_difference * WEIGHT_LIGHT)
    right_speed += (light_difference * WEIGHT_LIGHT)
    
    # Repulsion Vector (ALWAYS active to prevent crashing into walls/peers)
    if abs(prox_difference) > 150:
        sign = 1 if prox_difference > 0 else -1
        prox_difference_squared = (prox_difference ** 2) * sign
        left_speed -= (prox_difference_squared * WEIGHT_PROX)
        right_speed += (prox_difference_squared * WEIGHT_PROX)
    
    # 3. Apply Sliding Vector (Always active when near a frontal wall)
    if front_prox > 200:
        left_speed += (front_prox * WEIGHT_SLIDE * my_slide_dir)
        right_speed -= (front_prox * WEIGHT_SLIDE * my_slide_dir)
    
    # 4. Apply Noise Vector 
    left_speed += (noise_left * WEIGHT_NOISE)
    right_speed += (noise_right * WEIGHT_NOISE)
    
    # Clamp speeds to physical limits
    left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    
    # Execute motor commands
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
import random
from controller import Robot

# INITIALIZATION
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28  # The maximum wheel speed for an e-puck in radians/sec


# MOTOR SETUP
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Start with motors stopped
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# SENSOR SETUP
light_sensors = [robot.getDevice(f'ls{i}') for i in range(8)]
prox_sensors = [robot.getDevice(f'ps{i}') for i in range(8)]

for i in range(8):
    light_sensors[i].enable(TIME_STEP)
    prox_sensors[i].enable(TIME_STEP)

print("Motors, Light and Proximity Sensors ready.")

while robot.step(TIME_STEP) != -1:
    
    # Light Vector: Steer TOWARD the light
    right_light = light_sensors[0].getValue() + light_sensors[1].getValue() + light_sensors[2].getValue() + light_sensors[3].getValue()
    left_light = light_sensors[7].getValue() + light_sensors[6].getValue() + light_sensors[5].getValue() + light_sensors[4].getValue()
    light_difference = left_light - right_light 
    turn_weight_light = 0.001

    # Repulsion Vector: Steer AWAY from the obstacle
    right_prox = prox_sensors[0].getValue() + prox_sensors[1].getValue() + prox_sensors[2].getValue()
    left_prox = prox_sensors[7].getValue() + prox_sensors[6].getValue() + prox_sensors[5].getValue()
    prox_difference = right_prox - left_prox
    turn_weight_prox = 0.002

    # FLOW Vector: Steer ALONG the wall (slide along it)
    front_prox = prox_sensors[0].getValue() + prox_sensors[7].getValue()
    
    # If the wall is directly in front, generate a strong rotational force.
    # 300 is our threshold; below that, it's just empty space or a glancing angle.
    slide_force = 0
    if front_prox > 300: 
        slide_force = front_prox 
    turn_weight_slide = 0.005 

    # Wandering Vector: Add some randomness to the movement to avoid getting stuck in local minima
    noise_left = random.uniform(-1.0, 1.0)
    noise_right = random.uniform(-1.0, 1.0)
    turn_weight_noise = 1.0 

    left_speed = MAX_SPEED * 0.5
    right_speed = MAX_SPEED * 0.5
    
    # Apply the Light Vector (Steer TOWARD the light)
    left_speed += (light_difference * turn_weight_light)
    right_speed -= (light_difference * turn_weight_light)
    
    # Apply the Repulsion Vector (Steer AWAY from the obstacle)
    left_speed -= (prox_difference * turn_weight_prox)
    right_speed += (prox_difference * turn_weight_prox)

    # Apply Sliding Vector (Spin horizontally if blocked frontally)
    left_speed += (slide_force * turn_weight_slide)
    right_speed -= (slide_force * turn_weight_slide)
    
    # Apply Noise Vector (Add the jitter)
    left_speed += (noise_left * turn_weight_noise)
    right_speed += (noise_right * turn_weight_noise)
    
    left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
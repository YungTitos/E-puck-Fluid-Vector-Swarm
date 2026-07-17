import random
import logging
from dataclasses import dataclass
from typing import List, Tuple
from controller import Robot, Motor, DistanceSensor, LightSensor

# CONFIGURATION

@dataclass
class SwarmWeights:
    """Dataclass to hold and easily pass tuning parameters."""
    light: float = 0.00035064081371168326
    prox: float = 0.00007652504295632288
    slide: float = 0.0012960190610422663
    noise: float = 1.6244235549045687

class RobotConfig:
    """Static configuration for physical limits and thresholds."""
    MAX_SPEED: float = 6.28 
    BASE_SPEED_FRAC: float = 0.5 
    
    # Thresholds
    SLIDE_THRESHOLD: int = 200
    PROX_REPULSION_THRESHOLD: int = 150
    TARGET_REACHED_LIGHT_THRESHOLD: int = 25000 
    NOISE_RESET_TICKS: int = 20
    NUM_SENSORS: int = 8

# CONTROLLER CLASS

class SwarmRobotController:
    def __init__(self):
        # Core Initialization
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Subsystem Setup
        self._setup_logging()
        self.weights = self._load_tuning_parameters()
        
        # State Variables
        self.slide_dir: int = random.choice([-1, 1])
        self.noise_timer: int = 0
        self.noise_left: float = 0.0
        self.noise_right: float = 0.0
        self.is_finished: bool = False
        
        # Device Initialization
        self._init_motors()
        self._init_sensors()
        
        self.logger.info("Initialization complete. Motors, Light, and Proximity Sensors ready.")

    def _setup_logging(self) -> None:
        """Configures standard logging instead of using print statements."""
        logging.basicConfig(
            level=logging.INFO,
            format='%(levelname)s [%(name)s] %(message)s'
        )
        self.logger = logging.getLogger(self.robot.getName())

    def _load_tuning_parameters(self) -> SwarmWeights:
        """Parses Optuna parameters from Webots customData safely."""
        weights = SwarmWeights()
        custom_data = self.robot.getCustomData()
        
        if custom_data:
            try:
                params = [float(p.strip()) for p in custom_data.split(',')]
                if len(params) >= 4:
                    weights.light = params[0]
                    weights.prox = params[1]
                    weights.slide = params[2]
                    weights.noise = params[3]
                    self.logger.info("Successfully loaded custom tuning parameters.")
                else:
                    self.logger.warning("Custom data contains too few parameters. Using defaults.")
            except ValueError as e:
                self.logger.error(f"Failed to parse customData (expected floats): {e}. Using defaults.")
            except Exception as e:
                self.logger.error(f"Unexpected error parsing customData: {e}. Using defaults.")
                
        return weights

    def _init_motors(self) -> None:
        """Initializes and configures the wheel motors."""
        self.left_motor: Motor = self.robot.getDevice('left wheel motor')
        self.right_motor: Motor = self.robot.getDevice('right wheel motor')
        
        # Set to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.stop_motors()

    def _init_sensors(self) -> None:
        """Initializes and enables distance and light sensors."""
        self.light_sensors: List[LightSensor] = []
        self.prox_sensors: List[DistanceSensor] = []
        
        for i in range(RobotConfig.NUM_SENSORS):
            ls = self.robot.getDevice(f'ls{i}')
            ps = self.robot.getDevice(f'ps{i}')
            
            ls.enable(self.time_step)
            ps.enable(self.time_step)
            
            self.light_sensors.append(ls)
            self.prox_sensors.append(ps)

    def stop_motors(self) -> None:
        """Halts the robot."""
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def _calculate_speeds(self, light_vals: List[float], prox_vals: List[float]) -> Tuple[float, float]:
        """Core Braitenberg algorithm computing motor velocities based on sensor inputs."""
        
        # Base Speeds
        left_speed = RobotConfig.MAX_SPEED * RobotConfig.BASE_SPEED_FRAC
        right_speed = RobotConfig.MAX_SPEED * RobotConfig.BASE_SPEED_FRAC
        
        # Vector 1: THE PULL (Light)
        # Right sensors: 0-3, Left sensors: 4-7
        right_light = sum(light_vals[:4])
        left_light = sum(light_vals[4:8])
        light_diff = left_light - right_light 
        
        left_speed += (light_diff * self.weights.light)
        right_speed -= (light_diff * self.weights.light)
        
        # Vector 2: THE PUSH (Repulsion)
        # Front-right: 0-2, Front-left: 5-7
        right_prox = sum(prox_vals[:3])
        left_prox = sum(prox_vals[5:8])
        prox_diff = right_prox - left_prox
        
        if abs(prox_diff) > RobotConfig.PROX_REPULSION_THRESHOLD:
            sign = 1 if prox_diff > 0 else -1
            prox_diff_squared = (prox_diff ** 2) * sign
            left_speed -= (prox_diff_squared * self.weights.prox)
            right_speed += (prox_diff_squared * self.weights.prox)
            
        # Vector 3: THE FLOW (Sliding)
        front_prox = prox_vals[0] + prox_vals[7]
        if front_prox > RobotConfig.SLIDE_THRESHOLD:
            left_speed += (front_prox * self.weights.slide * self.slide_dir)
            right_speed -= (front_prox * self.weights.slide * self.slide_dir)
            
        # Vector 4: THE WANDER (Burst Noise)
        self.noise_timer -= 1
        if self.noise_timer <= 0:
            self.noise_left = random.uniform(-2.0, 2.0)
            self.noise_right = random.uniform(-2.0, 2.0)
            self.noise_timer = RobotConfig.NOISE_RESET_TICKS
            
        left_speed += (self.noise_left * self.weights.noise)
        right_speed += (self.noise_right * self.weights.noise)
        
        # Clamp Speeds
        left_speed = max(min(left_speed, RobotConfig.MAX_SPEED), -RobotConfig.MAX_SPEED)
        right_speed = max(min(right_speed, RobotConfig.MAX_SPEED), -RobotConfig.MAX_SPEED)
        
        return left_speed, right_speed

    def run(self) -> None:
        """Main control loop."""
        while self.robot.step(self.time_step) != -1:
            
            # Read all sensors
            light_vals = [ls.getValue() for ls in self.light_sensors]
            prox_vals = [ps.getValue() for ps in self.prox_sensors]
            
            total_light = sum(light_vals)
            
            # Stop Condition
            if total_light > RobotConfig.TARGET_REACHED_LIGHT_THRESHOLD:
                self.stop_motors()
                
                if not self.is_finished:
                    self.is_finished = True
                continue
                
            # Move
            left_cmd, right_cmd = self._calculate_speeds(light_vals, prox_vals)
            
            self.left_motor.setVelocity(left_cmd)
            self.right_motor.setVelocity(right_cmd)

# ENTRY POINT
if __name__ == '__main__':
    controller = SwarmRobotController()
    controller.run()
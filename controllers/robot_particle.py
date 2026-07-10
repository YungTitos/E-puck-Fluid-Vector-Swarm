from enum import Enum
import math
import random


from controller import Robot, Motor, DistanceSensor, LightSensor

class Side(Enum):
    LEFT = "left"
    RIGHT = "right"

class Strength(Enum):
    HIGH = 2.0
    NORMAL = 1.0

class RobotParticle:
    """
    Constructor for the RobotParticle object
    Args: 
    - radius (float): The radius of the free space around the robot, used for separation
    - margin (float): The extra area outside the separation area, used to monitor swarm direction
    - w_alignment (float):
    - w_separation (float):
    - w_cohesion (float):
    - w_noise (float):
    """
    def __init__(
            self, 
            radius: float = 1.0,
            margin: float = 0.2,
            w_alignment: float = 1.0, 
            w_separation: float = 1.0, 
            w_cohesion: float = 1.0, 
            w_noise: float = 1.0
        ):
        self.controller: Robot = Robot()
        self.left_motor: Motor = self.controller.getMotor("left weel motor")
        self.right_motor: Motor = self.controller.getMotor("right weel motor")
        self.light_sensors_array: list[LightSensor] = []
        self.proximity_sensors_array: list[DistanceSensor] = []
        self.time_step: float = self.controller.getBasicTimeStep()
        self.radius: float = radius
        self.w_alignment = w_alignment
        self.w_separation = w_separation
        self.w_cohesion = w_cohesion
        self.w_noise = w_noise

        for i in range(8):
            self.light_sensors_array.append(
                self.controller.getLightSensor(f"ls{i}")
            )
            self.proximity_sensors_array.append(
                self.controller.getDistanceSensor(f"ps{i}")
            )

        self.__sensor_setup()

    def __sensor_setup(self):
        # TODO: understand what it does
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        for i in range(8):
            # TODO: understand the purpose, solve the type clash self.time_step is float, 
            # .enable() whant an int, are we using the correct logic?
            self.light_sensors_array[i].enable(int(self.time_step))
            self.proximity_sensors_array[i].enable(int(self.time_step))

    def step(self) -> bool:
        # TODO: understand the purpose, solve the type clash self.time_step is float, 
        # .step() whant an int, are we using the correct logic?
        if self.controller.step(int(self.time_step)) == -1:
            return False

        alignment_vector: tuple[float, float] = self.__get_alignment_vector()
        separation_vector: tuple[float, float] = self.__get_separation_vector()
        cohesion_vector: tuple[float, float] = self.__get_cohesion_vector()
        noise_vector: tuple[float, float] = self.__get_noise_vector()

        direction_vector: tuple[float, float] = (
            self.w_alignment * alignment_vector[0] + 
            self.w_separation * separation_vector[0] +
            self.w_cohesion * cohesion_vector[0] +
            self.w_noise * noise_vector[0],

            self.w_alignment * alignment_vector[1] + 
            self.w_separation * separation_vector[1] +
            self.w_cohesion * cohesion_vector[1] +
            self.w_noise * noise_vector[1],
        )

        # Direction vector decoding
        side: Side = Side.LEFT if direction_vector[1] >= 0 else Side.RIGHT
        strength: Strength = Strength.NORMAL if direction_vector[0] >= 0 else Strength.HIGH

        return True

    def __get_alignment_vector(self) -> tuple[float, float]:
        front_left: float = (self.light_sensors_array[7].getValue() + self.light_sensors_array[6].getValue()) / 2
        back_left: float = (self.light_sensors_array[5].getValue() + self.light_sensors_array[4].getValue()) / 2
        back_right: float = (self.light_sensors_array[3].getValue() + self.light_sensors_array[2].getValue()) / 2
        front_right: float = (self.light_sensors_array[1].getValue() + self.light_sensors_array[0].getValue()) / 2
        
        if front_left <= back_left and front_left <= back_right and front_left <= front_right:
            return (1, 1)
        elif back_left <= front_left and back_left <= back_right and back_left <= front_right:
            return (-1, 1)
        elif back_right <= front_left and back_right <= back_left and back_right <= front_right:
            return (-1, -1)
        else:
            return (1, -1)

    def __get_separation_vector(self) -> tuple[float, float]:
        proximity_readings: list[tuple[float, float]] = []

        for i in range(8):
            # TODO: verify that the angles actually match the sensors direction
            # TODO: verify that the module arithmetic is correct (suppose to 
            # handle the edges of the sensor array)
            angle: float = 2 * math.pi * i / ((i + 1) % 8)
            value: float = self.proximity_sensors_array[7 - i].getValue() + self.proximity_sensors_array[7 - i - 1].getValue()
            if value < self.radius:
                proximity_readings.append((value * math.cos(angle), value * math.sin(angle)))

        result: list[float] = [0.0, 0.0]
        # Sum all the distance vector, to get the resulting one
        for reading in proximity_readings:
            result[0] += reading[0]
            result[1] += reading[1]

        # Normalise the resulting vector
        norm: float = math.sqrt(result[0]**2 + result[1]**2)
        result[0] /= norm
        result[1] /= norm

        return (result[0], result[1])

    def __get_cohesion_vector(self) -> tuple[float, float]:
        proximity_readings: list[tuple[float, float]] = []

        for i in range(8):
            # TODO: verify that the angles actually match the sensors direction
            # TODO: verify that the module arithmetic is correct (suppose to 
            # handle the edges of the sensor array)
            angle: float = 2 * math.pi * i / ((i + 1) % 8)
            value: float = self.proximity_sensors_array[7 - i].getValue() + self.proximity_sensors_array[7 - i - 1].getValue()
            if self.radius <= value and value <= self.radius:
                proximity_readings.append((value * math.cos(angle), value * math.sin(angle)))

        result: list[float] = [0.0, 0.0]
        # Sum all the distance vector, to get the resulting one
        for reading in proximity_readings:
            result[0] += reading[0]
            result[1] += reading[1]

        # Normalise the resulting vector
        norm: float = math.sqrt(result[0]**2 + result[1]**2)
        result[0] /= norm
        result[1] /= norm

        return (result[0], result[1])

    def __get_noise_vector(self) -> tuple[float, float]:
        # Generate a random vector (point) in the unit circle
        theta: float = random.random() * 2 * math.pi
        return (math.cos(theta), math.sin(theta))


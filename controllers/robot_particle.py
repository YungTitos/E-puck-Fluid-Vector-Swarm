import random

from controller import Robot, Motor, DistanceSensor, LightSensor


class RobotParticle:
    def __init__(
            self, 
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

        resulting_direction: tuple[float, float] = (
            self.w_alignment * alignment_vector[0] + 
            self.w_separation * separation_vector[0] +
            self.w_cohesion * cohesion_vector[0] +
            self.w_noise * noise_vector[0],

            self.w_alignment * alignment_vector[1] + 
            self.w_separation * separation_vector[1] +
            self.w_cohesion * cohesion_vector[1] +
            self.w_noise * noise_vector[1],
        )

        # TODO: 

        return True

    def __get_alignment_vector(self) -> tuple[float, float]:
        # Get the light readings
        # Group the readings and compute average
        # Derive a direction vecotor
        # TODO: implement
        ...

    def __get_separation_vector(self) -> tuple[float, float]:
        # Get the proximity readings
        # Group the readings and compute average
        # Derive a direction vecotor
        # TODO: implement
        ...

    def __get_cohesion_vector(self) -> tuple[float, float]:
        # Get the proximity readings
        # Group the readings and compute average
        # Derive a direction vecotor
        # TODO: implement
        ...

    def __get_noise_vector(self) -> tuple[float, float]:
        # Generate a random vector in the unit circle
        # Generate a random angle [0, 2pi]
        # Compute sin, cos
        # TODO: implement
        ...
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class Gearbox:
    def __init__(self, shifter: Motor, output: Motor, gears = 4, gear_distance=1510):
        """Creates a gear shift using 2 motors."""
        self.shifter = shifter
        self.output = output
        self.gears = gears
        self.gear_distance = gear_distance

        self.shift_speed = 720
        self.stall_threshold = 100

    def reset(self):
        self.shifter.dc(-90)
        while self.shifter.load() < self.stall_threshold:
            pass
        self.shifter.hold()
        self.shifter.reset_angle(0)

    def settings(self):
        ...

    def shift_to(self, gear: int):
        if gear not in range(1, self.gears + 1):
            raise ValueError("Invalid gear number")
        
        gear = gear - 1 # Change to zero-based index
        self.shifter.run_target(self.shift_speed, self.gear_distance * gear)
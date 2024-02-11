from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class Gearbox:
    def __init__(self, shifter: Motor, output: Motor, gears = 4, gear_distance=1470):
        """Creates a gear shift using 2 motors."""
        self.shifter = shifter
        self.output = output
        self.gears = gears
        self.gear_distance = gear_distance

        self.shift_speed = 720
        self.stall_threshold = 75

    def reset(self):
        self.shifter.dc(-90)
        while self.shifter.load() < self.stall_threshold and self.shifter.angle() > 100:
            pass
        self.shifter.hold()
        self.shifter.run_angle(self.shift_speed, 540)
        self.shifter.reset_angle(0)

    def settings(self):
        ...

    def shift_to(self, gear: int):
        if gear not in range(1, self.gears + 1):
            raise ValueError("Invalid gear number")
        
        gear = gear - 1 # Change to zero-based index
        self.shifter.run_target(self.shift_speed, self.gear_distance * gear)

hub = PrimeHub()
hub.imu.reset_heading(0)

changeM = Motor(Port.A) #blue
outputM = Motor(Port.C) #purple
gear_box = Gearbox(changeM, outputM)

left_wheel = Motor(Port.B) #green
right_wheel = Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE) #aqua
wheels = DriveBase(left_wheel, right_wheel, 62, 129)
# wheels.use_gyro(True)

right_sensor = ColorSensor(Port.F) #yellow
left_sensor = ColorSensor(Port.D) #red

# wheels.settings(straight_speed=200)
# wheels.straight(30000)

gear_box.reset()
gear_box.output.run_time(730, 5000)
gear_box.shift_to(4)
gear_box.output.run_time(730, 5000)
gear_box.shift_to(2)
gear_box.output.run_time(730, 5000)
gear_box.reset()
gear_box.output.run_time(730, 5000)
gear_box.shift_to(3)
gear_box.output.run_time(730, 5000)

# right_wheel.run_time(500, 1000)
# left_wheel.run_time(500, 1000)
# wheels.drive(500, 0)
# wait(1000)
# print(right_sensor.reflection())
# print(left_sensor.reflection())


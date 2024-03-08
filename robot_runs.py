from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.tools import hub_menu


class Gearbox:
    def __init__(self, shifter: Motor, output: Motor, gears = 4, gear_distance=1380):
        """Creates a gear shift using 2 motors."""
        self.shifter = shifter
        self.output = output
        self.gears = gears
        self.gear_distance = gear_distance

        self.shift_speed = 720 
        self.stall_threshold = 110

    def reset(self):
        self.shifter.dc(-90)
        while self.shifter.load() < self.stall_threshold or self.shifter.angle() > 100:
            pass
        self.shifter.hold()
        self.shifter.run_angle(self.shift_speed, 400)
        self.shifter.reset_angle(0)

    def settings(self):
        ...

    def shift_to(self, gear: int, wait=True):
        if gear not in range(1, self.gears + 1):
            raise ValueError("Invalid gear number")
        
        gear = gear - 1 # Change to zero-based index
        self.shifter.run_target(self.shift_speed, self.gear_distance * gear, wait=wait)

    def wait_for_gear(self):
        while not self.shifter.done():
            pass   

############ techniacal stuff ############
hub = PrimeHub()
hub.imu.reset_heading(0)

changeM = Motor(Port.B) #blue
outputM = Motor(Port.D) #purple
gear_box = Gearbox(changeM, outputM)

left_wheel = Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE) #green
right_wheel = Motor(Port.F) #aqua
wheels = DriveBase(left_wheel, right_wheel, 62, 129)
wheels.use_gyro(True)

right_sensor = ColorSensor(Port.A) #yellow
left_sensor = ColorSensor(Port.C) #red

BLACK = 15
WHITE = 99
TARGET = 57

WHEEL_DIAMETER = 62.4
WHEEL_PRIMETER = WHEEL_DIAMETER * 3.14159265358979323846 #20 digits of pi


timer = StopWatch()

############# functions ##############
def deg_to_mm(degrees):
    return (degrees / 360) * WHEEL_PRIMETER

def until_black(speed, sensor, brake = True):
    wheels.drive(speed, 0)
    while sensor.reflection() > 25:
        print(sensor.reflection())
    if brake:
        wheels.stop()

def until_white(speed, sensor, brake = True):
    wheels.drive(speed, 0)
    while sensor.reflection() < 85:
        print(sensor.reflection())
    if brake:
        wheels.stop()

def follow_line(speed, distance, sensor, side, kp, brake = True):
    if side == "R":
        direction = 1
    else:
        direction = -1
    left_wheel.reset_angle()
    while deg_to_mm(left_wheel.angle()) < distance:
        change = (TARGET - sensor.reflection()) * kp * direction
        left_wheel.dc(speed + change)
        right_wheel.dc(speed - change)

def follow_line_until_black(speed, detect_sensor, follow_sensor, side, kp, brake = True):
    if side == "R":
        direction = 1
    else:
        direction = -1
    while detect_sensor.reflection() > 25:
        change = (TARGET - follow_sensor.reflection()) * kp * direction
        left_wheel.dc(speed + change)
        right_wheel.dc(speed - change)
    if brake:
        wheels.stop()

def follow_line_time(speed, seconds, sensor, side, kp, brake):
    if side == "R":
        direction = 1
    else:
        direction = -1
    timer.reset()
    while timer.time() * 1000 < seconds:
        change = (TARGET - sensor.reflection()) * kp * direction
        left_wheel.dc(speed + change)
        right_wheel.dc(speed - change)
    if brake:
        wheels.stop()

def gyro_abs(target, speed):
    while not (target > hub.imu.heading() - 0.005) or not (target < hub.imu.heading() + 0.005):
        deg = (target - hub.imu.heading() + 360) % 360
        if deg > 180:
            left_wheel.dc(-speed)
            right_wheel.dc(speed)
        else:
            left_wheel.dc(speed)
            right_wheel.dc(-speed)
    wheels.stop()
    print(hub.imu.heading())

def gyro_turn(target, speed, clockwise = True, brake = Stop.HOLD):
    target = (target + 360) % 360
    current = (hub.imu.heading()) % 360
    short_way = target - current
    long_way = (360 - abs(short_way)) * (-1 * short_way / abs(short_way))
    wheels.settings(turn_rate=speed, turn_acceleration=30)
    if clockwise:
        wheels.turn(max(short_way, long_way), then=brake)
    else:
        wheels.turn(min(short_way, long_way), then=brake)
    print(hub.imu.heading())
    

def straight_untill_black(speed, sensor, wait1):
    wheels.settings(straight_speed=speed)
    wheels.straight(1000, wait=wait1)
    while sensor.reflection() > 20:
        pass
    wheels.stop()
  
def straight_time(speed, seconds):
    wheels.settings(straight_speed=speed)
    wheels.straight(1000, wait=False)
    timer.reset()
    while timer.time() < seconds * 1000:
        pass
    wheels.stop()

############ fun functions ######
    


############ runs ###############
gear_box.reset()

def run1(): 
    gear_box.shift_to(4, False)
    wheels.settings(straight_speed=300)
    wheels.straight(-365, then=Stop.NONE)
    wheels.settings(straight_speed=100)
    wheels.straight(-70)
    wheels.settings(turn_rate=17)
    wheels.turn(30, then= Stop.NONE)
    gyro_turn(160, speed=500, brake= Stop.HOLD)
    wheels.straight(-70)
    wheels.settings(straight_speed=200)
    wheels.straight(70)
    right_wheel.dc(75)
    left_wheel.dc(15)
    while right_sensor.reflection() < 95:
        pass
    wheels.straight(30)
    gear_box.output.run_time(-1000, 1500, wait=False)
    gyro_abs(140, 40)
    follow_line_until_black(40, left_sensor, right_sensor, 'L', 0.7)
    wheels.stop()
    gyro_abs(92, 40) 
    straight_time(250, 1.5)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    gear_box.output.run_time(1000, 1800)
    wheels.settings(200)
    wheels.straight(-100)
    gear_box.output.run_angle(1000, 360, wait=False)
    wheels.curve(50, 50, then=Stop.NONE)
    wheels.settings(1000)
    wheels.straight(-1500)
    



    


def run2(): 
    wheels.settings(straight_speed=400)
    wheels.straight(650, wait= False)
    gear_box.shift_to(2)
    gear_box.wait_for_gear()
    straight_untill_black(170,left_sensor, False)
    gear_box.output.run_time(-1000000000, 2000)
    wait(500)
    gear_box.output.run_time(1000000000, 2000)
    wait(1200)
    wheels.turn(10)
    wheels.settings(straight_speed=400)
    wheels.straight(1100)
def run3(): 
     wheels.settings(straight_speed=500)
     wheels.straight(320, wait= False)
     gear_box.shift_to(4)
     gear_box.output.run_time(-10000000, 2500)
     wheels.straight(200)
     gear_box.output.run_time(5000, 3000)
     wheels.straight(-550)


def run4(): 
     wheels.settings(straight_speed=300)
     wheels.straight(-850)
     wheels.straight(1050)
     gear_box.output.run_angle(360,1000)
     gear_box.output.run_angle(-500,7000)

def run5(): 
    gear_box.reset()
    until_black(400, left_sensor, False)
    wheels.straight(85)
    wheels.settings(turn_rate=200)
    wheels.turn(-45)
    follow_line(35, 650, left_sensor, "R", 0.9)


def run9(): 
     gear_box.output.run_time(730, 5000)

def tester():
    gear_box.reset()
    while Button.LEFT not in hub.buttons.pressed():
        gear = hub_menu("1", "2", "3", "4") if hub != 3 else 2.5
        gear_box.shift_to(int(gear))
        gear_box.output.dc(-100)
        while Button.RIGHT not in hub.buttons.pressed():
            pass
        gear_box.output.hold()
        gear_box.reset()

    




# wheels.settings(straight_speed=200)
# wheels.straight(30000)


# right_wheel.run_time(500, 1000)
# left_wheel.run_time(500, 1000)
# wheels.drive(500, 0)
# wait(1000)
# print(right_sensor.reflection())
# print(left_sensor.reflection())
while True:
    selected= hub_menu("1","2","3","4","5","9", "T")

    if selected == "1": 
        run1()
    elif selected == "2":
        run2()
    elif selected == "3":
        run3()
    elif selected == "4":
        run4()
    elif selected == "5":
        run5()
    elif selected == "9":
        run9() 
    elif selected == "T":
        tester()

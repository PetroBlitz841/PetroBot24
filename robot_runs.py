from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.tools import hub_menu


class Gearbox:
    def __init__(self, shifter: Motor, output: Motor, gears = 4, gear_distance=1380):
        """Creates a gear shift using 2 motors."""
        self.shifter = shifter # the shifting motor
        self.output = output # the motor who power the gearbox
        self.gears = gears # the amount of gears in the gearbox
        self.gear_distance = gear_distance # the distance between one gear

        self.shift_speed = 720 # shifting speed
        self.stall_threshold = 110 # the load the motor feels at the edge of the gearbox

    def reset(self):
        self.shifter.dc(-90) # moving the shift gear towards the edge of the gearbox
        while self.shifter.load() < self.stall_threshold or self.shifter.angle() > 100:
            # checking by the load if we got to the edge of the gearbox
            # and checking if the motor angle counting says we close to the edge
            pass
        self.shifter.hold()
        self.shifter.run_angle(self.shift_speed, 400) # fixing the gap created by pushing the edge
        self.shifter.reset_angle(0) 

    def shift_to(self, gear: int, wait=True):
        if gear not in range(1, self.gears + 1): # checking if the gear number is valid
            raise ValueError("Invalid gear number")
        
        gear = gear - 1 # Change to zero-based index
        self.shifter.run_target(self.shift_speed, self.gear_distance * gear, wait=wait) 
        

    def wait_for_shift(self):
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
COLOR_LIST = [
    Color.WHITE,
    Color.BLACK,
    Color.GREEN,
    Color.RED,
    Color.BLUE,
    Color.YELLOW,
    Color.VIOLET,
    Color(h=49, s=17, v=100),
]
right_sensor.detectable_colors(COLOR_LIST)
left_sensor.detectable_colors(COLOR_LIST)


timer = StopWatch()

############# functions ##############
def deg_to_mm(degrees):
    return (degrees / 360) * WHEEL_PRIMETER

def until_black(speed, sensor: ColorSensor):
    wheels.settings(straight_speed=speed)
    wheels.straight(1000, wait=False)
    while sensor.reflection() > 20:
        print(sensor.reflection())
    wheels.brake()
def until_white(speed, sensor: ColorSensor, brake = True):
    wheels.drive(speed, 0)
    while sensor.reflection() < 85:
        print(sensor.reflection())
    if brake:
        wheels.stop()

def follow_line(speed, distance, sensor: ColorSensor, side, kp):
    if side == "R": 
        direction = 1
    else: 
        direction = -1 # changing which wheel will add or subtruct the change
    left_wheel.reset_angle() # reseting wheel angle for counting the distance we crossed
    while deg_to_mm(left_wheel.angle()) < distance: # converting the angle of the wheel to mm and checking if we crossed the distance
        change = (TARGET - sensor.reflection()) * kp * direction # calculating the change needed to add or subtruct from the wheels
        left_wheel.dc(speed + change)
        right_wheel.dc(speed - change)

def follow_line_until_black(speed, detect_sensor: ColorSensor, follow_sensor: ColorSensor, side, kp, brake = True):
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

def follow_line_time(speed, seconds, sensor: ColorSensor, side, kp, brake):
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
    while not (target > hub.imu.heading() - 0.01) or not (target < hub.imu.heading() + 0.01): # the stoping range
        direction = (target - (hub.imu.heading() % 360)) % 360 # calculating the direction of the turn
        if direction > 180:
            left_wheel.dc(-speed)
            right_wheel.dc(speed)
        else:
            left_wheel.dc(speed)
            right_wheel.dc(-speed)
    wheels.stop()

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
    

def straight_untill_black(speed, sensor: ColorSensor):
    wheels.settings(straight_speed=speed)
    wheels.straight(1000, wait=False)
    while sensor.reflection() > 20: # checking if the sensor see black
        pass
    wheels.brake()

def straight_untill_white(speed, sensor: ColorSensor):
    wheels.settings(straight_speed=speed)
    wheels.straight(1000, wait=False)
    while sensor.reflection() < 95: # checking if the sensor see black
        pass
    wheels.brake()
  
def straight_time(speed, seconds, direction = 1):
    wheels.settings(straight_speed=speed)
    wheels.straight(1000 * direction, wait=False)
    timer.reset()
    while timer.time() < seconds * 1000: # checking if the time passed
        pass
    wheels.stop()

############ fun functions ######
    
def smile():
    hub.display.icon(
        [
            [0, 100, 0, 100, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [100, 0, 0, 0, 100],
            [0, 100, 100, 100, 0]
        ]
    )

############ runs ###############
gear_box.reset()

def run1(): 
    smile() 
    gear_box.shift_to(4, False)
    wheels.settings(straight_speed=300)
    wheels.straight(-365, then=Stop.NONE)
    wheels.settings(straight_speed=100)
    wheels.straight(-70)
    wheels.settings(turn_rate=35)
    wheels.turn(30, then= Stop.NONE)
    gyro_turn(160, speed=500, brake= Stop.HOLD) # turning clockwise so the attachment will be in home
    # leaving the attachment
    wheels.straight(-70)
    wheels.settings(straight_speed=200)
    wheels.straight(70)
    # getting to the black line
    right_wheel.dc(75)
    left_wheel.dc(15)
    while right_sensor.reflection() < 95:
        pass
    wheels.straight(30)
    gear_box.output.run_time(-1000, 1900, wait=False) # moving the gripper to his possition while moving
    gyro_abs(140, 40) # turning to the line
    # getting to the M02
    follow_line_until_black(40, left_sensor, right_sensor, 'L', 0.7)
    wheels.stop()
    gyro_abs(92, 40) 
    # completing M02
    straight_time(250, 1.5)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    gear_box.output.run_time(1000, 1800) # moving the gripper to secure sam the stage manager
    #getting home
    wheels.settings(200)
    wheels.straight(-100)
    gear_box.output.run_angle(1000, 360, wait=False) # continue moving the gripper while moving
    wheels.curve(60, 50, then=Stop.NONE)
    wheels.settings(straight_speed=700)
    while Button.LEFT not in hub.buttons.pressed() and Button.RIGHT not in hub.buttons.pressed():
        # allowing us finishing the run without leaving the menu so we can shift gears between runs
        wheels.straight(-1000, wait=False)
    gear_box.shift_to(2, wait=False) # shifting to gear 2
    wheels.stop()     


def run2(): 
    smile()
    gear_box.shift_to(2, False)
    gear_box.wait_for_shift()
    straight_untill_black(170,left_sensor)
    gear_box.output.run_time(-1000, 2000)
    wait(500)
    gear_box.output.run_time(1000, 2000)
    wait(1200)
    wheels.settings(straight_speed=400)
    wheels.straight(1100)
    while Button.LEFT not in hub.buttons.pressed() and Button.RIGHT not in hub.buttons.pressed():
        wheels.straight(1100, wait=False)
    gear_box.shift_to(4, wait=False)
    wheels.stop()  

def run3():
    smile()
    hub.imu.reset_heading(0)
    wheels.reset()
    wheels.drive(250, -3)
    gear_box.shift_to(4, False)
    while wheels.distance() < 320:
        pass
    wheels.brake()
    gear_box.wait_for_shift()
    gear_box.output.dc(-100)
    wait(4000)
    gear_box.output.dc(100)
    wheels.drive(250, -2)
    wheels.reset()
    while wheels.distance() < 320:
        pass
    wheels.brake()
    left_wheel.dc(40)
    wait(12000)
    wheels.drive(-400, 3)
    gear_box.output.stop()
    while Button.LEFT not in hub.buttons.pressed() and Button.RIGHT not in hub.buttons.pressed():
        pass
    gear_box.shift_to(1, wait=False)
    wheels.brake()


def run4(): 
    smile()
    gear_box.shift_to(1)
    wheels.settings(straight_speed=300)
    wheels.straight(-850)
    wheels.drive(-400, 40)
    wait(800)
    wheels.brake()
    wait(500)
    while Button.LEFT not in hub.buttons.pressed() and Button.RIGHT not in hub.buttons.pressed():
        wheels.straight(1050, wait=False)
    wheels.stop()


def run5(): 
    smile()
    gear_box.reset()
    hub.imu.reset_heading(0)
    straight_untill_black(250, left_sensor)
    follow_line(50, 720,left_sensor, "R" , 1.4)
    # wheels.straight(50)
    wheels.brake()
    gyro_abs(-48  ,   45)

    # gyro_turn(319, 35, clockwise = True, brake = Stop.HOLD)
    straight_untill_white(250, right_sensor)
    wheels.straight(65)
    gear_box.output.run_time(720,1000)

    gyro_abs(0, 45)
    gyro_abs(-40, 45)
    gear_box.output.run_time(720,1000)

    wheels.straight(-150)
    gyro_abs(-48, 45)
    gear_box.output.run_time(-700,900)
    gear_box.output.run_time(700,1600)

    gear_box.shift_to(2, False)

    wheels.straight(200)
    wheels.straight(1000, wait=False)
    while not right_sensor.color() == Color(h=49, s=17, v=100):
        pass
    wheels.brake()
    wheels.straight(70)
    gyro_abs(39, 45)
    straight_time(350, 1.5, -1)

    gear_box.wait_for_shift()
    gear_box.output.dc(-100)
    wait(4850) 
    gear_box.output.hold()

    gear_box.shift_to(3)
    gear_box.wait_for_shift()
    gear_box.output.dc(-100)
    wait(1100)
    #gear_box.output.run_time(-820, 1500, wait=False)
    wheels.drive(480, 6)
    wait(2000)
    wheels.stop()
    gear_box.output.hold()

    gear_box.shift_to(4, False)

    wheels.straight(-155)
    gyro_abs(-48, 45)
    wheels.settings(straight_speed=250)
    wheels.straight(395)
    wheels.brake()
    gyro_abs(-10, 45)
    wheels.straight(160)
    # gyro_abs(-40, 45)
    # straight_untill_white(150, left_sensor)
    # straight_untill_black(150, left_sensor)
    # gyro_abs(-98, 35)
    # wheels.settings(straight_speed=180)
    # wheels.straight(195)












def run9(): 
     print(right_sensor.hsv())
    #  wheels.drive(300,0)

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

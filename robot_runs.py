from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.tools import hub_menu

############ techniacal stuff ############
hub = PrimeHub()
hub.speaker.beep()
hub.imu.reset_heading(0)

right_arm = Motor(Port.F)  # green
left_arm = Motor(Port.E)  # yellow

right_wheel = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)  # blue
left_wheel = Motor(Port.A)  # red


right_sensor = ColorSensor(Port.D) # aqua
left_sensor = ColorSensor(Port.C) # magenta

BLACK = 15
WHITE = 99
TARGET = 57

WHEEL_DIAMETER = 62.4
WHEEL_PRIMETER = WHEEL_DIAMETER * 3.14159265358979323846  # 20 digits of pi
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
# right_sensor.detectable_colors(COLOR_LIST)
# left_sensor.detectable_colors(COLOR_LIST)

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


def straight_time(speed, seconds, direction=1):
    wheels.settings(straight_speed=speed)
    wheels.straight(1000 * direction, wait=False)
    timer.reset()
    while timer.time() < seconds * 1000:  # checking if the time passed
        pass
    wheels.stop()


# ############ runs ###############


def run3():
    wheels = DriveBase(left_wheel, right_wheel, wheel_diameter=62, axle_track=120)
    wheels.use_gyro(True)
    wheels.settings(straight_acceleration=300)
    left_arm.run_time(-200, 1800, wait=False)
    straight_time(550, 2)
    wheels.drive(500, 15)
    wait(900)
    wheels.stop()
    right_arm.run_time(-800, 700)
    wheels.straight(-300)
    wheels.drive(-300, -15)
    wait(1000)
    left_arm.run_time(200, 1800, wait=False)
    wheels.drive(-300, -30)
    wait(30000)



def run1():
    wheels = DriveBase(left_wheel, right_wheel, wheel_diameter=62, axle_track=120)
    wheels.use_gyro(True)
    wheels.settings(straight_acceleration=200)
    left_wheel.reset_angle(0)
    right_wheel.reset_angle(0)
    wheels.straight(350,wait=False)
    right_arm.run_angle(1500,-1900 )
    left_arm.run_angle(800,640,wait= False)
    right_arm.run_angle(1500,1700 )
    left_arm.run_angle(800,-630)
    right_arm.run_angle(1500,-1000)
    wheels.settings(straight_acceleration=100)
    wheels.straight(-450,wait=False)
    right_arm.run_angle(1500,1000 ,wait= False)

def run2():
    
    # hub.imu.reset_heading(20)
    # straight_untill_black(600, right_sensor)
    # gyro_abs(0, 30)
    # -----------------------------------------------------------
    wheels = DriveBase(left_wheel, right_wheel, wheel_diameter=62, axle_track=120)
    wheels.use_gyro(True)
    wheels.settings(straight_acceleration=300)
    wheels.settings(straight_speed=950)
    wheels.straight(615) # getting out of home
    right_wheel.run_angle(250, 97, wait=False) # turning
    left_wheel.run_angle(250, -97)
    wheels.straight(960) # getting to meshutefet
    left_arm.run_angle(500, 280) # picking sam
    wheels.straight(-290)
    wheels.curve(270, 94) # turning to mufa orot
    wheels.straight(1080) 
    right_arm.run_time(900, 2200) # puting amir child at mofa orot circle
    right_arm.run_time(900, 1400)
    right_arm.reset_angle(0)
    left_arm.run_time(-900, 2400) # 
    right_arm.run_time(-900, 2900, wait=False) # pushing elevator to havaya otefet
    # angle = hub.imu.heading()
    # while not right_arm.done():
    #     if hub.imu.heading() > 1+angle:
    #         # redo the thing
    #         right_arm.run_target(900, 0)
    #         wheels.straight(60)
    #         right_arm.run_time(-900, 2900, wait=False) # pushing elevator to havaya otefet
    #     angle = hub.imu.heading()
    #     wait(50)
    wheels.straight(-20)
    wheels.straight(710, wait=False)
    wait(700)
    left_arm.run_angle(900, -3200) # put stuff down in the museum
    left_arm.run_angle(900, 3200, wait=False)
    wheels.straight(110)
    wheels.settings(straight_speed=380)
    wheels.straight(100, wait=False)
    right_arm.run_time(900, 3800) # do flower thingy
    wheels.straight(35)
    right_arm.run_time(-900, 2300, wait=False)
    wheels.straight(150)

def run4():
    wheels = DriveBase(left_wheel, right_wheel, wheel_diameter=62, axle_track=120)
    wheels.use_gyro(True)
    wheels.settings(straight_acceleration=1000)
    wheels.straight(600,wait=False)
    left_arm.run_angle(1500, -4000)
    wheels.straight(-210,wait=False)
    left_arm.run_angle(1000, 720,wait=False)
    wheels.straight(-130)
    wheels.straight(580)
    right_arm.run_angle(700, -60)
    wheels.straight(200)
    right_arm.run_angle(400, -80)
    # wheels.straight(650)
    # wheels.settings(straight_speed=450)
    # wheels.straight(450, wait=False)
    # right_arm.run_time(900, 1500)
    # wait(350)
    # right_arm.run_time(900, 1500)








while True:
    selected = hub_menu("1", "2", "3", "4")
    if selected == "1":
        run1()
    elif selected == "2":
        run2()
    elif selected == "3":
        run3()
    elif selected == "4":
        run4()


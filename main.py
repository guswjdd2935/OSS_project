#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
import re
from pybricks.iodevices import UARTDevice

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
ser=UARTDevice(Port.S2,baudrate=115200)
#현진
# Write your program here.
# ev3.speaker.beep()
# left_motor=Motor(Port.A)
# right_motor=Motor(Port.D)
# grab_motor=Motor(Port.B)
# shooting_motor=Motor(Port.C)
# robot=DriveBase(left_motor,right_motor,wheel_diameter=56,axle_track=115)



# shooting_motor.run_until_stalled(-100,Stop.COAST,duty_limit=50)
# grab_motor.run_until_stalled(100,Stop.COAST,duty_limit=50)
# grab_motor.reset_angle(0)

# grab_motor.run_target(100,-100)
# robot.straight(100)

# grab_motor.run_until_stalled(200,Stop.COAST,duty_limit=50)

# grab_motor.run_until_stalled(-200,Stop.COAST,duty_limit=50)
# shooting_motor.run(2000)
# time.sleep(0,25)
# shooting_motor.stop()


ser=UARTDevice(Port.S2,baudrate=115200)

while True:
    data=ser.read_all()
    print(data)
#    INT = re.findall(r'\d+', data)
#    print(INT)
    time.sleep_ms(1000)
    pattern = "([0-9]+),([0-9]+)"
    result = re.search(pattern, data)
    print(result)

# left_motor=Motor(Port.A)
# right_motor=Motor(Port.D)
# wheel_diameter=5.6
# axle_track=115
# robot=DriveBase(left_motor, right_motor, wheel_diameter,axle_track)

# def data_filter(data):
#     decoded_data=data.decoded().strip()
#     if decoded_data.isdigit():
#         return int(decoded_Data)
#     return None

# def p_control(cam_data, kp, power):
#     error=threshold-cam_data
#     output=error*kp
#     robot.drive(power,output)

# ev3.speaker.beep()
# threshold=200
# print("start")
# while True:
#     data=ser.read_all()
#     if data:
#         filtered_data=data_filter(data)
#         print("hello")
#         if filtered_data is not None:
#             print(filtered_data)
#             p_control(filtered_data,kp=0.5,power=100)
#     wait(10)




#슈팅
# from pybricks.robotics import DriveBase

# left_motor=Motor(Port.A)
# right_motor=Motor(Port.D)
# grab_motor=Motor(Port.B)
# Shooting_arm=Motor(Port.C)
# giro=GyroSensor(Port.S1)

# wheel_diameter=5.6
# axle_track=115

# robot=DriveBase(left_motor, right_motor, wheel_diameter,axle_track)


# grab_motor.run_until_stalled(-360,Stop.COAST,duty_limit=80)
# robot.straight(10)
# grab_motor.reset_angle(0)
# grab_motor.run_until_stalled(-360,Stop.COAST,duty_limit=50)

# grab_motor.run_target(100,-90)


# Shooting_arm.run_until_stalled(50,Stop.COAST,duty_limit=50)
# Shooting_arm.reset_angle(0)
# print(1)
# Shooting_arm.run(2000)
# time.sleep(0.25)
# Shooting_arm.stop()

    time.sleep_ms(1000)
left_motor=Motor(Port.A)
right_motor=Motor(Port.D)
robot=DriveBase(left_motor,right_motor,wheel_diameter=56,axle_track=115)

def data_filter(data):
    decoded_data=data.decode().strip()
    if decoded_data.isdigit():
        return int(decoded_data)
    return None

def p_control(cam_data,kp,power):
    error=threshold-cam_data
    output=error*kp
    robot.drive(power,output)

ev3.speaker.beep()
threshold=200
while True:
    data=ser.read_all()
    if data:
        filtered_data=data_filter(data)
        if filtered_data is not None:
            print(filtered_data)
            p_control(filtered_data,kp=0.5,power=100)
    wait(10)


def pd_control(cam_Data,kp,kd,power):
    global previous_error
    error=threshold-cam_data
    derivative=error-previous_error
    output=(kp*error)+(kd*derivative)
    robot.drive(power,output)
    previous_error=error

    ev3.speaker.beep()
    threshold=200
    previous_error=0

    while True:
        data=ser.read_all()
        if data:
            filtered_data=data_filter(data)
            if filtered_data is not None:
                print(filtered_data)
                pd_control(filtered_data,kp=0.5,kd=0.1,power=100)
        wait(10)


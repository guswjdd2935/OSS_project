#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import UARTDevice
import time
import re

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

"""
    객체 정의
    left_motor
    right_motor
    grab_motor
    shooting_motor
    robot
    
"""
# Create your objects here.
ev3 = EV3Brick()
ser=UARTDevice(Port.S2,baudrate=115200)
left_motor=Motor(Port.A)
right_motor=Motor(Port.D)
grab_motor=Motor(Port.B)
shooting_motor=Motor(Port.C)
robot=DriveBase(left_motor,right_motor,wheel_diameter=56,axle_track=115)


"""
    프로그램 작성
    
    wheel_diameter= 5.6
    axle_track = 115
    
"""
# Write your program here.
ev3.speaker.beep()

wheel_diameter= 5.6
axle_track = 115

"""
grap_arm, shooting_arm 동작

1. ball을 찾아 돌아다닐 때, grap_arm 중간 높이
2. ball 발견 시, grap_arm 내림 (reset_angle)
3. grap_arm 올림
4. shooting_motor 내림(reset_angle)
5. shooting 

"""

# shooting_motor.run_until_stalled(-100,Stop.COAST,duty_limit=50) # shooting_motor 내리기
# grab_motor.run_until_stalled(100,Stop.COAST,duty_limit=50) #  grab_motor 올리기
# grab_motor.reset_angle(0) 

# grab_motor.run_target(100,-100)
# robot.straight(100)

# grab_motor.run_until_stalled(200,Stop.COAST,duty_limit=50)

# grab_motor.run_until_stalled(-200,Stop.COAST,duty_limit=50)
# shooting_motor.run(2000)
# time.sleep(0.25)
# shooting_motor.stop()

#공을 잡고 슈팅을 한 후 슈팅모터를 제자리에 위치하는 함수
def shoot():
    shooting_motor.run_until_stalled(-100,Stop.COAST,duty_limit=50) # shooting_motor 내리기
    grab_motor.run_until_stalled(100,Stop.COAST,duty_limit=50) #  grab_motor 올리기
    grab_motor.reset_angle(0) 
    shooting_motor.reset_angle(0)

    grab_motor.run_target(100,-100)
    robot.straight(100)

    grab_motor.run_until_stalled(200,Stop.COAST,duty_limit=50)

    grab_motor.run_until_stalled(-200,Stop.COAST,duty_limit=50)
    shooting_motor.run(2000)
    time.sleep(0.25)
    shooting_motor.stop()
    shooting_motor.run_target(100, 0)


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


"""
    ball tracking 함수
    data_filter : ...
    p_control :
    pd_control :
    
"""

def data_filter(data):
    decoded_data=data.decoded().strip()
    if decoded_data.isdigit():
        return int(decoded_Data)
    return None

def p_control(cam_data, kp, power):
    error=threshold-cam_data
    output=error*kp
    robot.drive(power,output)

def pd_control(cam_Data,kp,kd,power):
    global previous_error
    error=threshold-cam_data
    derivative=error-previous_error
    output=(kp*error)+(kd*derivative)
    robot.drive(power,output)
    previous_error=error

    """
    main
    """
    ev3.speaker.beep()
    threshold=200
    previous_error=0
    
    while True:
        data=ser.read_all()
        if data:
            filtered_data=data_filter(data)
            if filtered_data is not None:
                print(filtered_data)
                pd_control(filter_data,kp=0.5,kd=0.1,power=100)
        wait(10)
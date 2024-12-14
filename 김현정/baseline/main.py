#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
import random

#==========[Initialize]==========
#==========[sensors]==========
ev3 = EV3Brick()
gyro = GyroSensor(Port.S3)
ser = UARTDevice(Port.S2, baudrate=115200)
color_sensor = ColorSensor(Port.S4)
#ultrasonic_sensor = UltrasonicSensor(Port.S4)
#touch_sensor = TouchSensor(Port.S4)
#==========[motors]==========
grab_motor = Motor(Port.B)
shooting_motor = Motor(Port.C)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

#==========[target_angle turn(gyro)]==========
def turn(target_angle, power):
    """
    로봇을 target_angle 방향으로 회전시키는 함수.
    
    Parameters:
    - target_angle: 목표 각도 (양수는 시계 방향, 음수는 반시계 방향).
    - power: 기본 회전 속도 (양수).
    """
    while True:
        # 현재 자이로 각도 읽기
        angle = gyro.angle()
        ev3.screen.clear()  # 화면 초기화
        ev3.screen.print("Angle:", angle)  # 현재 각도 출력
        error = target_angle - angle  # 목표 각도와 현재 각도의 차이 계산
        
        # 방향에 따라 회전
        turn_power = power if error > 0 else -power
        robot.drive(0, turn_power)  # 로봇 회전: 0은 전진/후진 속도, turn_power는 회전 속도
        
        # 목표 각도에 근접하면 멈춤
        if abs(error) <= 2 or abs(error) >= 363 :  # 오차가 3도 이하로 작아지면 정지
            robot.stop()
            break
#=============================================

def turn_to_zero(power):
    """
    로봇을 각도 0으로 회전시키는 함수.
    
    Parameters:
    - power: 기본 회전 속도 (양수).
    """
    while True:
        # 현재 자이로 각도 읽기
        angle = gyro.angle()
        ev3.screen.print("Angle :", gyro.angle())
        ev3.screen.clear()  # 화면 초기화
        ev3.screen.print("Angle:", angle)  # 현재 각도 출력
        error = -angle  # 목표 각도 (0)와 현재 각도의 차이 계산
        # 방향에 따라 회전
        turn_power = power if error > 0 else -power
        robot.drive(0, turn_power)  # 로봇 회전: 0은 전진/후진 속도, turn_power는 회전 속도
        
        # 목표 각도에 근접하면 멈춤
        if abs(error) <= 2:  # 오차가 3도 이하로 작아지면 정지
            robot.stop()
            break


#==========[camera_chase]==========
def process_uart_data(data):
    try:
        # 데이터를 문자열로 디코드 (키워드 인자 제거)
        data_str = data.decode().strip()
        if not data_str:
            pass

        # 문자열에서 리스트 파싱
        data_str = data_str.strip("[]")
        parsed_list = [int(value.strip()) for value in data_str.split(",")]

        # 파싱된 결과 반환
        return parsed_list
    except:
        # 에러 처리
        return [-1,-1] # -1이 나오면 무시하는 코드 사용

def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output)
    previous_error = error

#==========[shooting positions]==========
def grab(command):
    if command == 'motion3':
        #close
        grab_motor.run_until_stalled(700,Stop.COAST,duty_limit=50)
        #set_zero point
        grab_motor.reset_angle(0)
    elif command == 'motion1':
        
        #open1
        grab_motor.run_until_stalled(-700,Stop.COAST,duty_limit=70)
    elif command == 'motion2':

        #open2
        grab_motor.run_target(500,-180)

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run_until_stalled(-500,Stop.COAST,duty_limit=70)
    elif command == 'shoot':
        #shooting
        shooting_motor.run(3500)
        time.sleep(0.25)
        shooting_motor.stop()



#==========[setup]==========
ev3.speaker.beep()
threshold = 60
previous_error = 0
gyro.reset_angle(0)  # 각도 초기화
ev3.screen.print("Angle 0 :", gyro.angle())  # 현재 각도 출력
#==========[zero set position setting]==========
shoot('zero') #shoot 모터가 안쪽이고,
grab('motion3') #grab 모터가 바깥쪽이므로 shoot먼저 세팅 후 grab을 세팅해야한다
time.sleep(1)
grab('motion2') #공을 잡기 위한 높이로 열기

print("Zero set postion completed")

#==========[main loop]==========
while True:
    data = ser.read_all()
    try:
        detected_color = color_sensor.color()
        print(detected_color)
        if detected_color == Color.GREEN:  # 초록색을 감지하면
            ev3.speaker.beep()  # 비프음으로 알림
            robot.stop()  # 정지
            robot.straight(-200)  # 200mm 후진
            robot.stop()  # 후진 후 멈춤
        filter_result = process_uart_data(data)
        if filter_result[0] == -1:
            random_angle=random.randint(-20, 20)
            
        if filter_result[0] != -1 and filter_result[1] != -1:
            if filter_result[1] > 90:  # 공이 가까워진 경우
                robot.straight(300)  # 공 쪽으로 전진
                grab('motion3')
                robot.straight(200)# 공을 잡음
                robot.straight(-100)
                ev3.screen.print("Angle 1:", gyro.angle())  # 현재 각도 출력
                time.sleep(0.5)# 딜레이
                turn(0,100)  # 정면(각도 0)으로 회전
                time.sleep(0.5)      # 딜레이
                grab('motion1')      # 슛 준비
                time.sleep(0.5)      # 딜레이
                shoot('shoot')       # 공 날리기
                time.sleep(0.5)      # 딜레이
                shoot('zero')        # 슛 모터 초기화
                grab('motion2')      # 공 잡기 준비
                turn_to_zero(100)
                robot.straight(-300)  # 설정된 속도로 600mm 후진

            else:  # 공이 멀리 있는 경우 추적
                pd_control(filter_result[0], kp=0.6, kd=0.4, power=200)

    except Exception as e:
        print("Error:", e)
        pass


while True:
    try:
        data = ser.read_all()
        filter_result = process_uart_data(data)
        if filter_result[0]!= -1 and filter_result[1]!= -1:
            print(filter_result)
            pd_control(filter_result[0], kp=0.7, kd=0.3, power=100)
        wait(10)
    except:
        pass
        


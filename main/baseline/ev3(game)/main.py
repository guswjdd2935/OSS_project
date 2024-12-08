#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

#==========[Initialize]==========
#==========[sensors]==========
ev3 = EV3Brick()
gyro = GyroSensor(Port.S1)
ser = UARTDevice(Port.S2, baudrate=115200)
ultrasonic_sensor = UltrasonicSensor(Port.S3)
touch_sensor = TouchSensor(Port.S4)

#==========[motors]==========
grab_motor = Motor(Port.B)
shooting_motor = Motor(Port.C)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

#==========[target_angle turn(gyro)]==========
def turn(target_angle, power): 
    current_angle = gyro.angle()
    while abs(current_angle - target_angle) > 5:
        current_angle = gyro.angle()
        if current_angle < target_angle:
            left_motor.run(power)
            right_motor.run(-power)
        elif current_angle > target_angle:
            left_motor.run(-power)
            right_motor.run(power)

    # 목표 각도에 도달하면 모터 정지
    left_motor.stop()
    right_motor.stop()


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
        grab_motor.run_until_stalled(400,Stop.COAST,duty_limit=50)
        #set_zero point
        grab_motor.reset_angle(0)
    elif command == 'motion1':
        
        #open1
        grab_motor.run_until_stalled(-400,Stop.COAST,duty_limit=50)
    elif command == 'motion2':

        #open2
        grab_motor.run_target(400,-70)

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run_until_stalled(-400,Stop.COAST,duty_limit=50)
    elif command == 'shoot':
        #shooting
        shooting_motor.run(2000)
        time.sleep(0.25)
        shooting_motor.stop()



#==========[setup]==========
ev3.speaker.beep()
threshold = 60
previous_error = 0
gyro.reset_angle(0)
#==========[zero set position setting]==========
shoot('zero') #shoot 모터가 안쪽이고,
grab('motion3') #grab 모터가 바깥쪽이므로 shoot먼저 세팅 후 grab을 세팅해야한다
time.sleep(1)
grab('motion2') #공을 잡기 위한 높이로 열기

print("Zero set postion completed")

#==========[main loop]==========
while True:
    data = ser.read_all()
    # 데이터 처리 및 결과 필터링
    try:
        filter_result = process_uart_data(data)
        #filter_result[0] : x, filter_result[1] : y
        if filter_result[0]!= -1 and filter_result[1]!= -1:
        # if filter_result[0]!= -1 and filter_result[1]!= -1:
            if filter_result[1] > 110: #공이 카메라 화면 기준으로 아래에 위치 = 로봇에 가까워졌다
                robot.straight(200) #강제로 앞으로 이동
                grab('motion3') #공을 잡기
                time.sleep(0.5) #동작간 딜레이
                turn(0,100) #정면(상대방 진영)바라보기
                time.sleep(0.5) #동작간 딜레이
                grab('motion1') #슛을 위한 열기
                time.sleep(0.5) #동작간 딜레이
                shoot('shoot') #공 날리기
                turn(0,-100)
                time.sleep(0.5) #동작간 딜레이
                shoot('zero')
                grab('motion2') 
            else: #공이 카메라 화면 기준 멀리 위치해 있으면 chase한다
                pd_control(filter_result[0], kp=0.6, kd=0.4, power=150)
        else: # 센서가 공을 보지 못했을 경우의 움직임.
            
            """
            터치 센서가 눌리면 후진
            """
            # if touch_sensor.pressed():
            #     robot.straight(-200)
            #     time.sleep(0.5)
            #     continue 
            
            """
            초음파 센서 5cm 이하 시 후진
            """
            distance = ultrasonic_sensor.distance()  # 센서 값은 mm 단위로 반환됨
            if distance <= 50:  # 5cm (50mm) 이하일 때
                print(f"장애물 감지! 거리: {distance}mm. 후진 시작")
                robot.straight(-200)  # 200mm 후진
                time.sleep(0.5)  # 딜레이 추가
                continue  # 후진 후 다음 루프로 이동
            
            turn(-30, 100) 
            time.sleep(0.5)
            turn(60, 100)
            time.sleep(0.5)
            turn(0, 100)
            time.sleep(0.5)
            robot.straight(200) 
            time.sleep(0.5)
        time.sleep_ms(50)
    except:
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
        


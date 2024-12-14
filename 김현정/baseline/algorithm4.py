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
ultrasonic_sensor = UltrasonicSensor(Port.S4)

#==========[motors]==========
grab_motor = Motor(Port.B)
shooting_motor = Motor(Port.C)
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

#==========[target_angle turn(gyro)]==========
def turn(target_angle, power):
    while True:
        angle = gyro.angle()
        error = target_angle - angle
        turn_power = power if error > 0 else -power
        robot.drive(0, turn_power)
        
        if abs(error) <= 5:  # 5도 이내로 근접
            robot.stop()
            wait(10)
            if abs(gyro.angle() - target_angle) <= 3:  # 미세 조정
                break
#=============================================

def return_to_start():
    while True:
        current_angle = gyro.angle()
        current_distance = ultrasonic.value()
        
        # 시작 위치와 현재 위치의 거리와 각도 계산
        distance_error = start_distance - current_distance
        angle_error = start_angle - current_angle

        # 각도 조정
        if abs(angle_error) > 5:
            turn_power = 100 if angle_error > 0 else -100
            left_motor.run_forever(speed_sp=-turn_power)
            right_motor.run_forever(speed_sp=turn_power)
        else:
            # 각도 조정이 완료되면 거리 조정
            if abs(distance_error) > 50:
                move_power = 200 if distance_error > 0 else -200
                left_motor.run_forever(speed_sp=move_power)
                right_motor.run_forever(speed_sp=move_power)
            else:
                # 시작 위치에 도달하면 정지
                left_motor.stop(stop_action="brake")
                right_motor.stop(stop_action="brake")
                break

        time.sleep(0.1)


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
start_distance = ultrasonic.value()
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
        filter_result = process_uart_data(data)
        
        if filter_result[0] != -1 and filter_result[1] != -1:
            if filter_result[1] > 100:  # 공이 가까워진 경우
                robot.straight(200)  # 공 쪽으로 전진
                grab('motion3')      # 공을 잡음
                time.sleep(0.5)      # 딜레이
                
                turn(0, 100)         # 정면(각도 0)으로 회전
                print("turn")
                time.sleep(0.5)      # 딜레이
                
                # 파란색을 감지할 때까지 이동
                while True:
                    robot.drive(100, 0)  # 전진 (속도 100, 회전 0)
                    if color_sensor.color() == Color.BLUE:  # 파란색 감지
                        robot.stop()      # 멈춤
                        break             # 루프 탈출
                
                grab('motion1')      # 슛 준비
                time.sleep(0.5)      # 딜레이
                shoot('shoot')       # 공 날리기
                time.sleep(0.5)      # 딜레이
                shoot('zero')        # 슛 모터 초기화
                grab('motion2')      # 공 잡기 준비
                left_motor.run_until_stalled(speed_sp=backward_speed, stop_action="brake", duty_limit=50) 
                right_motor.run_until_stalled(speed_sp=backward_speed, stop_action="brake",duty_limit=50)
            else:  # 공이 멀리 있는 경우 추적
                pd_control(filter_result[0], kp=0.6, kd=0.4, power=200)

    except Exception as e:
        print("Error:", e)
        pass



# while True:
#     try:
#         data = ser.read_all()
#         filter_result = process_uart_data(data)
#         if filter_result[0]!= -1 and filter_result[1]!= -1:
#             print(filter_result)
#             pd_control(filter_result[0], kp=0.7, kd=0.3, power=100)
#         wait(10)
#     except:
#         pass
        


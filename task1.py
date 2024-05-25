#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
import dubins
from xycar_msgs.msg import xycar_motor
#=============================================
# 모터 토픽을 발행할 것임을 선언
#=============================================
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
rx, ry = [], []

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
P_MID = ((P_ENTRY[0]+P_END[0])/2 , (P_ENTRY[1]+P_END[1])/2) # 주차라인 중점좌표
GOOD_PARKING_DISTANCE = abs(np.sqrt(pow((AR[0]-P_MID[0]) , 2) + pow((AR[1]-P_MID[1]) , 2))) # 주차라인 중점 좌표와 AR태그의 거리
wheelbase = 84 # 차량 휠베이스

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    # 시뮬레이터에서 제공하는 정보
    max_steering_angle = np.pi / 9  # 최대 조향 각도
    turning_radius = wheelbase / max_steering_angle  # 최대 회전 반경

    # 시작 위치와 목표 위치
    start_pose = (sx, sy, math.radians(syaw+90))  # (x, y, yaw), 여러번 시도하여 yaw값 계산
    goal_pose = (P_ENTRY[0], P_ENTRY[1], (7/4)*np.pi)  # (x, y, yaw), 여러번 시도하여 yaw값 계산

    # Dubins 경로 계산
    path = dubins.shortest_path(start_pose, goal_pose, turning_radius) # Dubins 경로 계산
    configurations, _ = path.sample_many(dt) # dt 간격으로 sampling 하여 configurations 변수에 저장

    for config in configurations:
        x, y, yaw = config # config에서 x, y, yaw(헤딩) 값을 추출하여 각각 x, y, yaw 변수에 할당
        rx.append(x) # rx 리스트에 x값 저장
        ry.append(y) # ry 리스트에 y값 저장

    # 목표 위치(goal_pose)에서 P_END 까지의 직선 경로 계산
    dx = P_END[0] - rx[-1] # x좌표 차이 계산
    dy = P_END[1] - ry[-1] # y좌표 차이 계산
    distance = np.sqrt(pow(dx,2) + pow(dy,2)) # 거리 계산
    num_points = int(distance / dt)  # dt 단위로 이동하는 경로 포인트의 수

    for i in range(1, num_points + 1):
        rx.append(rx[-1] + dx / num_points) # rx 리스트에 포인트 추가
        ry.append(ry[-1] + dy / num_points) # ry 리스트에 포인트 추가

    # x와 y 좌표 리스트 반환
    return rx, ry


#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================

# PurePusuitController class 생성
class PurePursuitController:
    def __init__(self, look_ahead):
        self.look_ahead = look_ahead

    def control(self, x, y, yaw, velocity, rx, ry, screen):
        # 후륜 축 중점 기준 가장 가까운 경로 포인트 찾기
        dx = [x - icx for icx in rx]
        dy = [y - icy for icy in ry]
        d = [abs(np.sqrt(pow(idx , 2) + pow(idy , 2))) for (idx, idy) in zip(dx, dy)]

        if not d:
            raise ValueError(
                "The path is empty. Please make sure the planning function is called correctly and the generated path is valid.")
        # 가장 가까운 포인트 설정
        closest_index = d.index(min(d))

        # look ahead 포인트 찾기
        # 후륜 축 중점을 기준으로 찾은 가장 가까운 점(closest_index)을 시작으로 하여 인덱스를 늘려가며 look_ahead_distance 만큼의 거리의 인덱스 찾기
        look_ahead_index = closest_index # 일단 가장 가까운 인덱스를 look_ahead_index 로 지정
        while look_ahead_index < len(rx) and d[look_ahead_index] < self.look_ahead: # rx 리스트 내에 있거나 look_ahead 거리가 넘지 않는 선에서 계속 인덱스를 더해가며 look_ahead_index 를 찾는다
            look_ahead_index += 1

        if look_ahead_index >= len(rx): # 만약 look_ahead_index 가 rx 리스트 크기보다 크거나 같다면, look_ahead_index 는 마지막 index로 설정
            look_ahead_index = len(rx) - 1

        # 화면에 look ahead index에 해당하는 점 표시
        point_radius = 5
        point_color = (255, 0, 0)
        pygame.draw.circle(screen, point_color, (int(rx[look_ahead_index]), int(ry[look_ahead_index])), point_radius)

        # 조향 각도 계산
        dx = rx[look_ahead_index] - x # 현재 x좌표와 look_ahead_index의 x좌표 차이 계산
        dy = -ry[look_ahead_index] + y # 현재 y좌표와 look_ahead_index의 y좌표 차이 계산
        target_angle = np.arctan2(dy, dx) # target_angle 계산

        alpha = target_angle - yaw # target_angle 과 현재 yaw값의 차이 계산
        angle = -np.arctan2(2 * wheelbase * math.sin(alpha), self.look_ahead) # 조향각 계산

        if angle > np.pi/9: # 최댓값 설정
            angle = np.pi/9
        elif angle < -np.pi/9: # 최솟값 설정
            angle = -np.pi/9

        return angle


def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    # AR태그와의 거리 계산
    distance_from_AR = abs(np.sqrt(pow((x-AR[0]) , 2) + pow((y-AR[1]) , 2)))

    # AR태그와의 거리가 앞서 정한 GOOD_PARKING_DISTANCE 에서 1만큼 더한 값보다 작거나 같다면 정지(즉, 최적 주차 위치에 가까우면 정지)
    if distance_from_AR <= GOOD_PARKING_DISTANCE+1:
        drive(0,0)

    # 차량 후륜 축 중점 기준의 좌표 계로 변환
    yaw = math.radians(yaw)
    x -= (wheelbase/2) * math.cos(yaw)
    y += (wheelbase/2) * math.sin(yaw)

    # Pure Pursuit 컨트롤러 생성, look_ahead 거리는 140으로 설정
    controller = PurePursuitController(look_ahead=140)

    # 조향 각도 계산
    steering_angle = controller.control(x, y, yaw, velocity, rx, ry, screen)
    # 조향 각도를 degree 단위로 변환, 가중치 2.5를 통해 더 적극적인 steering angle 조절
    steering_angle = 2.5*np.degrees(steering_angle)

    # 주행 속도 결정 (가장 빠르게 주행하도록 함)
    speed = 50

    # 모터 제어 함수 호출
    drive(steering_angle, speed)

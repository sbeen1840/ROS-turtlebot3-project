#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Int8
import threading


class MazeEscape:

#! #############################################
# ! 초기 생성자 함수

    def __init__(self):
        rospy.init_node('maze_escape')

# * < 콜백 메세지 초기화 >
        self.scan = []
        self.color = 0 # 원의 색상 플래그
        self.circle = 320 # 카메라 x축 정중앙 좌표값
        self.grasp = 0 # 동작 여부 플래그

# * < 발행 설정 >
        # /cmd_vel 발행
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # /action 발행
        self.action_pub = rospy.Publisher("/action", Int8, queue_size=1)

# * < 구독 설정 >
        # /scan 구독
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # /color 구독
        self.color_sub = rospy.Subscriber("/color", Int8, self.color_callback)

        # /grasp 구독
        self.grasp_sub = rospy.Subscriber("/grasp", Int8, self.grasp_callback)

        # /circle 구독
        self.circle_sub = rospy.Subscriber("/circle",  Int16, self.circle_callback)

# * < 메인 쓰레드 설정>
        self.main_thread = threading.Thread(target=self.main)
        self.main_thread.start()


#! #############################################
# ! 콜백함수 모음

    def scan_callback(self, scan):
        # if len(scan.ranges) == 360:
        self.scan = scan.ranges

    # 라이다 보정을 위한 평균값 사용
        # 전방
        self.front = self.scan[0:5]+self.scan[355:360] # 10
        self.front_avg = sum(self.front)/10

        # 전방 좌측
        self.side_left = self.scan[5:35] # 30
        self.side_left_avg = sum(self.side_left)/10

        # 전방 우측
        self.side_right = self.scan[325:355] # 30
        self.side_right_avg = sum(self.side_right)/10

        # 좌측
        self.left = self.scan[80:100] # 20
        self.left_avg = sum(self.left)/20

        # 우측
        self.right = self.scan[260:280] # 20
        self.right_avg = sum(self.right)/20


    def color_callback(self, msg):
        self.color = msg.data

    def circle_callback(self, msg):
        self.circle = msg.data

    def grasp_callback(self, msg):
        self.grasp = msg.data


# ! #############################################
# ! 라이다 특정방향이 임계치보다 큰 값인지 리턴하는 함수

    def front_ok(self):

        self.front = self.scan[0:5]+self.scan[355:360] # 10
        self.front_avg = sum(self.front)/10
        return self.front_avg > 0.25# 0.3

    def side_left_ok(self):
        self.side_left = self.scan[5:35] # 30
        self.side_left_avg = sum(self.side_left)/10
        return self.side_left_avg > 1.0 # 0.4

    def side_right_ok(self):
        self.side_right = self.scan[325:355] # 30
        self.side_right_avg = sum(self.side_right)/10
        return self.side_right_avg > 1.0 # 0.4

    def left_ok(self):
        self.left = self.scan[80:100] # 20
        self.left_avg = sum(self.left)/20
        return self.left_avg > 0.5#0.5

    def right_ok(self):
        self.right = self.scan[260:280] # 20
        self.right_avg = sum(self.right)/20
        return self.right_avg > 0.5#0.5


#! #############################################
# ! 모터 동작 함수

    def move_go(self): # 직진
        move = Twist()
        move.linear.x = 0.03 # 0.02
        move.angular.z = 0.0
        print ("\n\t -- move go -- 직진")
        print ("\t -- move go -- linear.x: {:.2f}".format(move.linear.x))

        self.cmd_pub.publish(move)

    def move_back(self): # back
        move = Twist()
        move.linear.x = -0.02 # 0.02
        move.angular.z = 0.0
        print ("\n\t -- move back -- 후진")
        print ("\n\t -- move back -- angular.z: {:.2f}".format(move.angular.z))

        self.cmd_pub.publish(move)

    def move_stop(self): # 정지
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = 0.0
        print ("\n\t -- move stop -- 정지")
        print ("\t -- move stop -- linear.x: {:.2f}".format(move.linear.x))

        self.cmd_pub.publish(move)

    def move_turn_right(self): # 완전 우회전
        move = Twist()
        move.linear.x = 0.02
        move.angular.z = -0.5
        print ("\n\t -- move_turn_right -- 우회전")
        print ("\t -- move_turn_right -- angular.z: {:.2f}".format(move.angular.z))

        self.cmd_pub.publish(move)

# ! #############################################
# ! 미로 자율주행 모드 함수

    def move_auto(self): # 미로 자율주행 모드
        move = Twist()

# * 1. 앞에 장애물 없을 때
        if self.front_ok() : # 앞에 장애물 없음

## * 1-1. 전방 우측 가까울 때
            if self.side_right_avg< 1.3:
                move.linear.x = 0.03 #0.02잘됨 # 직진
                move.angular.z = 0.4 #0.35
                print("\n\t -- move auto -- 앞에 장애물 없음, 가만히 좌회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t side_left : {:.2f}".format(self.side_left_avg))
                print ("\t side_right : {:.2f}".format(self.side_right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)

## * 1-2. 전방 좌측 가까울 때
            if self.side_left_avg < 1.3 :
                move.linear.x = 0.03 #0.02잘됨 # 직진
                move.angular.z = -0.4
                print("\n\t -- move auto -- 앞에 장애물 없음, 가만히 우회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t side_left : {:.2f}".format(self.side_left_avg))
                print ("\t side_right : {:.2f}".format(self.side_right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)

## * 1-3. 좌측 가까울 때
            if self.left_avg < 0.24:
                move.linear.x = 0.03 #0.02잘됨 # 직진
                move.angular.z = -0.15
                print("\n\t -- move auto -- 앞에 장애물 없음, 왼쪽벽 위험 살짝 우회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t left : {:.2f}".format(self.left_avg))
                print ("\t right : {:.2f}".format(self.right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)

## * 1-4. 우측 가까울 때
            if self.right_avg < 0.24:
                move.linear.x = 0.03 #0.02잘됨 # 직진
                move.angular.z = 0.15
                print("\n\t -- move auto -- 앞에 장애물 없음, 오른벽 위험 살짝 좌회전")
                print ("\t front : {:.2f}".format(self.front_avg))
                print ("\t left : {:.2f}".format(self.left_avg))
                print ("\t right : {:.2f}".format(self.right_avg))
                self.cmd_pub.publish(move)
                rospy.sleep(1.0)

## * 1-5. 아무것도 아닐 때
            move.linear.x = 0.03 #0.02잘됨 # 직진
            move.angular.z = 0.0
            print("\n\t -- move auto -- 앞에 장애물 없음, 직진")
            print ("\t front : {:.2f}".format(self.front_avg))
            print ("\t side_left : {:.2f}".format(self.side_left_avg))
            print ("\t side_right : {:.2f}".format(self.side_right_avg))
            print ("\t left : {:.2f}".format(self.left_avg))
            print ("\t right : {:.2f}".format(self.right_avg))

# * 2. 전방 장애물 있을 때
        else:

## * 2-1. 전방 우측이 좌측보다 괜찮을 때
            if self.right_ok() and self.left_ok() and self.side_right_avg > self.side_left_avg: # 장애물 있는데 오른쪽 벽 괜찮음
                move.linear.x = 0.0 # 우회전
                move.angular.z = -0.45
                rospy.sleep(1.0)
                print("\n\t -- move auto -- 앞에 장애물있고 오른쪽 여유있음, 조금 우회전")
                print ("\t right : {:.2f}".format(self.right_avg))

## * 2-2. 전방 좌측이 우측보다 괜찮을 때
            elif self.right_ok() and self.left_ok() and self.side_right_avg < self.side_left_avg: # 장애물 있는데 오른쪽 벽 괜찮음
                move.linear.x = 0.0
                move.angular.z = 0.45
                print("\n\t -- move auto -- 앞에 장애물있고 왼쪽 여유있음, 조금 좌회전")
                print ("\t left : {:.2f}".format(self.left_avg))

## * 2-3. 전방 좌우측 모두 안괜찮을 때
            else: # 장애물 있는데 양쪽 벽 모두 시원치 않음

##### * 2-3-1. 우측이 좌측보다 괜찮을 때
                if self.left_avg < self.right_avg: # 왼쪽벽이 너무 가까이 있을 때
                    move.linear.x = -0.02
                    move.angular.z = -0.2 # 왼쪽벽에서 멀어지는 후진
                    self.cmd_pub.publish(move)
                    rospy.sleep(1.0)

                    move.linear.x = -0.02
                    move.angular.z = 0.0
                    print("\n\t -- move auto -- 앞과 양쪽 여유 없고 왼쪽벽 너무 가까움, 후진")
                    print ("\t right : {:.2f}".format(self.right_avg))
                    print ("\t left : {:.2f}".format(self.left_avg))

##### * 2-3-2. 좌측이 우측보다 괜찮을 떄
                else:                       # 오른쪽 벽이 너무 가까이 있을 때
                    move.linear.x = -0.02
                    move.angular.z = 0.2 # 오른쪽벽에서 멀어지는 후진
                    self.cmd_pub.publish(move)
                    rospy.sleep(1.0)

                    move.linear.x = -0.02
                    move.angular.z = 0.0
                    print("\n\t -- move auto -- 앞과 양쪽 여유 없고 오른쪽벽 너무 가까움, 후진")
                    print ("\t right : {:.2f}".format(self.right_avg))
                    print ("\t left : {:.2f}".format(self.left_avg))


        self.cmd_pub.publish(move)

# ! #############################################
# ! 센터트랙킹 이동 함수

    def move_circle(self):
        move = Twist()

        mid_x = 160
        delta_x	= self.circle - mid_x
        norm = 15

        if delta_x > norm:
            print("\n\t -- circle tracking -- delX: {:.2f}. 우회전".format(delta_x))
            move.linear.x = 0.0
            move.angular.z = -0.025

        elif delta_x < -norm:
            print("\n\t -- circle tracking -- delX: {:.2f}. 좌회전".format(delta_x))
            move.linear.x = 0.0
            move.angular.z = 0.025

        else:
            print("\n\t -- circle tracking -- delX: {:.2f}. 직진".format(delta_x))
            move.linear.x = 0.02
            move.angular.z = 0.0

        self.cmd_pub.publish(move)

# ! #############################################
# ! 매니퓰레이터 동작 함수

    def action_1(self): # 동작1
        action = Int8()
        action.data = 1
        print("\n\t -- [동작1] -- 초록원 발견해서 집기 동작을 시행합니다. ")

        self.action_pub.publish(action)


    def action_2(self): # 동작2
        action = Int8()
        action.data = 2
        print("\n\t -- [동작2] -- 파랑원 발견해서 집기 동작을 시행합니다.")

        self.action_pub.publish(action)


    def action_3(self):
        action = Int8()
        action.data = 3
        print("\n\t -- [동작3] -- 초기동작을 시행합니다.")

        self.action_pub.publish(action)


# ! #############################################
# ! 메인쓰레드 함수

    def main(self):
        while not rospy.is_shutdown():

# * 1. 라이다데이터 360개 쌓이면
            if (len(self.scan)==360):

                print('\n')
                print('|------------------ front: {:.2f} -------------------|'.format(self.front_avg))
                print('|side_left : {:.2f} -------------------- side_right : {:.2f}|'.format(self.side_left_avg, self.side_right_avg))
                print('|left: {:.2f} --------------------------------- right : {:.2f}|'.format(self.left_avg, self.right_avg))
                print('\n')

                print('\n')
                print('|------------------ front_ok: {} -------------------|'.format(self.front_ok()))
                print('|side_left_ok : {} -------------------- side_right_ok : {:.2f}|'.format(self.side_left_ok(), self.side_right_ok()))
                print('|left_ok: {} --------------------------------- right_ok : {:.2f}|'.format(self.left_ok(), self.right_ok()))
                print('\n')

######## * 1-1. 빨간색 3 발견하면 (정지)
                if self.color == 3 :
                    self.move_stop() # 정지
                    rospy.sleep(1.0)

######## * 1-2. 초록원 1 발견하면 (4발견하면 동작1하고 직진)
                elif self.color == 1 or self.color == 4:

                    while self.color !=4:  # 초록원이 충분히 가까울 때까지
                        self.move_circle() # 센터 트랙킹
                        rospy.sleep(1.0)

                    else: # 초록원 가까이 옴
                        self.move_stop() # 정지
                        rospy.sleep(0.5)
                        print("초록 원 가까워져서 정지")

                        self.action_1() # 동작1
                        rospy.sleep(4.0)

                    while self.grasp!=1: # 동작1 완료 될 때까지

                        self.move_stop() # 정지
                        rospy.sleep(0.5)

                    else: # 동작1 완료됨
                        self.grasp = 0 # 초기화

######## * 1-3. 파란원 2 발견하면 (5발견하면 동작2하고 우회전 및 직진)
                elif self.color == 2 or self.color == 5 : # 파란원 발견하면

                    while self.color != 5 : # 파란원이 충분히 가까울 때까지,,,,,
                        self.move_circle() # 센터 트랙킹
                        rospy.sleep(1.0)

                    else: # 파란원 가까이 오면
                        self.move_stop() # 정지
                        rospy.sleep(0.5)

                        self.action_2() # 동작B
                        rospy.sleep(5.0)
                    while self.grasp != 2: # 동작2 완료될 때까지,,,,,

                        self.move_stop() # 정지
                        rospy.sleep(1.0)

                    else: # 동작2 완료됨
                        self.grasp = 0 # 초기화

                        self.move_back() # 뒤로 좀 후진했다가
                        rospy.sleep(2.0)

                        self.move_turn_right() # 완전우회전
                        rospy.sleep(3.4)

######## * 1-4. 무색 0 발견하면 (자율주행)
                else:
                    self.move_auto() # 직진
                    rospy.sleep(0.5)

# * 2. 라이다 360개 다 안채워졌으면
            else :
                print("\n\t -- 라이다가 데이터 쌓이는 중 --")
                self.action_3() # 매뉴퓰레이터 동작 초기화

if __name__ == "__main__":
    maze_escape = MazeEscape()
    try:
        rospy.spin()
    except Exception as e:
        print("Exception occurred:", str(e))
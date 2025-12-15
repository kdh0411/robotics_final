#!/usr/bin/env python3
# 위 한 줄은 "이 파일을 실행 파일처럼 직접 실행할 때" 사용되는 shebang이다.
# (예: chmod +x 후 ./cmd_vel_to_wheels.py 로 실행 가능)

import rclpy
# rclpy는 ROS2 파이썬 클라이언트 라이브러리(노드 생성, 토픽 pub/sub 등)

from rclpy.node import Node
# Node는 ROS2 노드(프로세스 안의 통신 단위)를 만들기 위한 베이스 클래스

from geometry_msgs.msg import Twist
# cmd_vel 표준 메시지 타입: 선속도(linear) + 각속도(angular)

from std_msgs.msg import Float64
# 저수준 바퀴 명령에서 사용하는 단일 실수 메시지 타입


class CmdVelToWheels(Node):
    # CmdVelToWheels는 "고수준 cmd_vel을 받아서 좌/우 바퀴 속도 명령으로 바꾸는 노드" 클래스

    def __init__(self):
        # 생성자: 노드가 시작될 때 한 번 실행됨
        super().__init__("cmd_vel_to_wheels")
        # 노드 이름을 cmd_vel_to_wheels로 지정

        # -------------------------------
        # 1) 파라미터 선언(기본값 포함)
        # -------------------------------
        # wheel_radius: 바퀴 반지름(m). (확정값: 0.0175)
        self.declare_parameter("wheel_radius", 0.0175)

        # wheel_separation: 좌/우 바퀴 중심 간 거리(m). (확정값: 0.10)
        self.declare_parameter("wheel_separation", 0.10)

        # cmd_timeout: cmd_vel이 끊겼을 때 몇 초 후 정지(0 명령)시킬지
        # Isaac/컨트롤러가 마지막 명령을 유지하는 문제를 막기 위한 안전장치
        self.declare_parameter("cmd_timeout", 0.3)

        # max_wheel_omega: 바퀴 각속도(rad/s) 상한
        # 너무 큰 값이 들어가면 시뮬레이션에서 튀거나 미끄러지거나 불안정해질 수 있어 클램프함
        self.declare_parameter("max_wheel_omega", 50.0)

        # -------------------------------
        # 2) 파라미터 값 읽기
        # -------------------------------
        # 파라미터를 실제 계산에 사용할 수 있도록 float로 꺼내옴
        self.r = float(self.get_parameter("wheel_radius").value)
        # r = 바퀴 반지름

        self.L = float(self.get_parameter("wheel_separation").value)
        # L = 바퀴 간격(좌/우 바퀴 중심 거리)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        # cmd_timeout = cmd_vel이 안 들어오면 정지시키는 시간(초)

        self.max_omega = float(self.get_parameter("max_wheel_omega").value)
        # max_omega = 바퀴 각속도 상한(rad/s)

        # -------------------------------
        # 3) 퍼블리셔/서브스크라이버 생성
        # -------------------------------
        # 왼쪽 바퀴 저수준 명령 퍼블리셔
        # (Isaac Sim OmniGraph의 ROS2 Subscriber 노드가 이 토픽을 받아 바퀴 조인트에 적용)
        self.pub_l = self.create_publisher(Float64, "/duckie/wheel_left_cmd", 10)

        # 오른쪽 바퀴 저수준 명령 퍼블리셔
        self.pub_r = self.create_publisher(Float64, "/duckie/wheel_right_cmd", 10)

        # 고수준 cmd_vel(입력) 서브스크라이버
        # geometry_msgs/Twist의 linear.x(전진속도 v), angular.z(회전속도 w)를 받을 것
        self.sub = self.create_subscription(Twist, "/duckie/cmd_vel", self.cb_cmd, 10)

        # -------------------------------
        # 4) 타임아웃(Watchdog)용 상태/타이머
        # -------------------------------
        # 마지막으로 cmd_vel을 받은 시간을 저장
        self.last_cmd_time = self.get_clock().now()

        # 0.05초(=20Hz)마다 watchdog() 실행해서 "명령 끊김"을 감지
        self.timer = self.create_timer(0.05, self.watchdog)

        # 노드 시작 로그 출력(지금 설정된 파라미터 확인용)
        self.get_logger().info(
            f"[cmd_vel_to_wheels] start  r={self.r:.4f}m  L={self.L:.4f}m  timeout={self.cmd_timeout:.2f}s  max_omega={self.max_omega:.1f}rad/s"
        )

    def clamp(self, x, lim):
        # x 값을 -lim ~ +lim 범위로 제한(클램프)
        # 너무 큰 바퀴 속도를 보내면 시뮬레이션이 불안정해질 수 있어서 안전장치로 사용
        return max(-lim, min(lim, x))

    def publish_wheels(self, omega_l, omega_r):
        # 왼쪽/오른쪽 바퀴 각속도(rad/s)를 토픽으로 발행하는 함수

        msg_l = Float64()              # 왼쪽 토픽 메시지 생성
        msg_r = Float64()              # 오른쪽 토픽 메시지 생성

        msg_l.data = float(omega_l)    # 메시지에 실수 값 넣기
        msg_r.data = float(omega_r)

        self.pub_l.publish(msg_l)      # 왼쪽 바퀴 명령 발행
        self.pub_r.publish(msg_r)      # 오른쪽 바퀴 명령 발행

    def cb_cmd(self, msg: Twist):
        # /duckie/cmd_vel 토픽을 받으면 호출되는 콜백 함수

        # 전진 속도 v (m/s) : Twist.linear.x
        v = msg.linear.x

        # 회전 속도 w (rad/s) : Twist.angular.z
        w = msg.angular.z

        # -------------------------------
        # 5) diff-drive 변환(핵심)
        # -------------------------------
        # 좌/우 바퀴 선속도:
        # v_left  = v - w*(L/2)
        # v_right = v + w*(L/2)
        #
        # 바퀴 각속도(rad/s)로 바꾸려면:
        # omega = v_wheel / r
        #
        # 따라서:
        omega_l = (v - w * (self.L / 2.0)) / self.r
        omega_r = (v + w * (self.L / 2.0)) / self.r

        # 너무 큰 값 방지용 클램프
        omega_l = self.clamp(omega_l, self.max_omega)
        omega_r = self.clamp(omega_r, self.max_omega)

        # 최종 바퀴 명령 발행
        self.publish_wheels(omega_l, omega_r)

        # 마지막 cmd_vel 수신 시간 갱신(Watchdog가 정지시키지 않게)
        self.last_cmd_time = self.get_clock().now()

    def watchdog(self):
        # cmd_vel이 끊겼는지 감시하는 함수
        # Isaac Sim/컨트롤러는 마지막 setpoint를 계속 유지하는 경우가 있어
        # cmd_vel이 멈추면 바퀴도 자동으로 멈추게(0 보내기) 만들어줌

        # 현재 시간 - 마지막 cmd_vel 수신 시간 = 경과 시간(dt)
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

        # 경과 시간이 cmd_timeout보다 크면 "명령이 끊겼다"고 판단
        if dt > self.cmd_timeout:
            # 안전하게 좌/우 바퀴 속도를 0으로 보내서 정지
            self.publish_wheels(0.0, 0.0)


def main():
    # ROS2 통신 초기화
    rclpy.init()

    # 노드 객체 생성
    node = CmdVelToWheels()

    try:
        # 노드를 계속 실행(콜백/타이머가 돌게 됨)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C로 종료할 때 예외가 나는데, 그걸 정상 종료로 처리
        pass

    # 종료 직전에 혹시 모를 잔류 명령을 막기 위해 정지 명령 한 번 더 발행
    node.publish_wheels(0.0, 0.0)

    # 노드 정리
    node.destroy_node()

    # ROS2 종료
    rclpy.shutdown()


if __name__ == "__main__":
    # 이 파일이 직접 실행될 때 main() 호출
    main()

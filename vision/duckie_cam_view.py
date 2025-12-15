import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class DuckieCamView(Node):
    def __init__(self):
        super().__init__("duckie_cam_view")

        # ROS Image ↔ OpenCV 이미지 변환을 위한 브리지
        self.bridge = CvBridge()

        # Isaac Sim에서 퍼블리시되는 카메라 토픽 구독
        self.sub = self.create_subscription(
            Image,
            "/duckie/camera/image_raw",
            self.cb,
            10
        )
        # Isaac Sim에서 퍼블리시 되는 cmd_vel 구독
        self.cmd_pub = self.create_publisher(
            Twist,
            "/duckie/cmd_vel",
            10
        )

        self.get_logger().info("Camera image subscriber started")
    

    def cb(self, msg: Image): # callback
        # 기본값 초기화
        cx = None
        cy = None
        area = None
        # =================================
        # 1. 빨간 큐브 masking
        # =================================
        # -------------------------------
        # 1) ROS Image → OpenCV (RGB)
        # -------------------------------
        frame_rgb = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

        # -------------------------------
        # 2) RGB → BGR (OpenCV 기본)
        # -------------------------------
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # -------------------------------
        # 3) BGR → HSV
        # -------------------------------
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # -------------------------------
        # 4) 빨간색 HSV 범위 정의
        # -------------------------------
        # 빨간색 영역 1 (Hue 낮은 쪽)
        lower_red1 = (0, 120, 70)
        upper_red1 = (10, 255, 255)

        # 빨간색 영역 2 (Hue 높은 쪽)
        lower_red2 = (170, 120, 70)
        upper_red2 = (180, 255, 255)

        # -------------------------------
        # 5) HSV Thresholding → mask
        # -------------------------------
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # 두 구간 합치기
        red_mask = mask1 + mask2

        # =================================
        # 2. 중심 좌표 계산
        # =================================

        # -------------------------------
        # 1) Contour 검출
        # -------------------------------
        # 이진 영상(red_mask)에서 객체의 외곽선(contour)을 검출
        # RETR_EXTERNAL: 가장 바깥쪽 윤곽선만 추출
        # CHAIN_APPROX_SIMPLE: 윤곽선을 구성하는 점 수를 최소화
        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 검출된 contour가 하나 이상 존재하는 경우에만 처리
        if contours:
            # 면적이 가장 큰 contour를 타겟(빨간 큐브)으로 선택
            c = max(contours, key=cv2.contourArea)
            # -------------------------------
            # 2) Bounding Box
            # -------------------------------
            # 선택된 contour를 감싸는 최소 사각형의 좌표 계산
            # x, y : 좌상단 좌표
            # w, h : 너비와 높이
            x, y, w, h = cv2.boundingRect(c)

            # 원본 영상(frame_bgr)에 초록색 사각형으로 bounding box 시각화
            cv2.rectangle(
                frame_bgr, (x, y), (x + w, y + h),
                (0, 255, 0), 2
            )

            # -------------------------------
            # 3) Centroid 계산
            # -------------------------------
            # contour의 공간 모멘트(moment) 계산
            M = cv2.moments(c)

            # m00이 0이 아닐 때만 중심 좌표 계산 가능
            # (0이면 나눗셈 오류 발생)
            if M["m00"] != 0:
                # 중심 좌표 (centroid) 계산 공식
                # cx = m10 / m00, cy = m01 / m00
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # 계산된 중심 좌표를 파란색 점으로 표시
                cv2.circle(frame_bgr, (cx, cy), 5, (255, 0, 0), -1)

                # 중심 좌표 값을 텍스트로 화면에 출력
                cv2.putText(
                    frame_bgr, f"({cx},{cy})",
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 0), 1
                )

        # ===============================
        # 3. 자율 주행 제어
        # ===============================
        frame_width = frame_bgr.shape[1]
        center_x = frame_width // 2

        center_margin = 50     # 중앙 허용 오차(px)
        stop_area = 200 * 200  # 전진 허용 기준 -> 바운딩 박스 크기로

        cmd = Twist()
        state = "NONE"

        # bbox 면적(거리 근사)
        if (cx is not None) and (cy is not None):
            area = w * h
        else:
            area = None

        # 주행 로직
        # cx가 계산되지 않았으면 → 큐브 안 보임으로 처리
        if cx is None:
            # -------------------------------
            # (1) 큐브 안 보임 → 둥글게 탐색
            # -------------------------------
            state = "SEARCH"
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0

        else:
            # bounding box 면적 계산 (거리 추정용)
            area = w * h

            # 정지 조건
            # 바운딩 박스 크기 기준
            if area is not None and area >=stop_area:
                # -------------------------------
                # (2) 충분히 가까움 + 정렬 완료 → 정지
                # -------------------------------
                state = "STOP"
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            else:
                # -------------------------------
                # (3) 위치 기반 추적
                # -------------------------------
                if cx < center_x - center_margin:
                    # 왼쪽에 있음 → 좌회전
                    state = "TURN_LEFT"
                    cmd.linear.x = 0.00
                    cmd.angular.z = 1.0

                elif cx > center_x + center_margin:
                    # 오른쪽에 있음 → 우회전
                    state = "TURN_RIGHT"
                    cmd.linear.x = 0.00
                    cmd.angular.z = -1.0

                else:
                    # 중앙에 있음 → 전진
                    state = "FORWARD"
                    cmd.linear.x = 0.1
                    cmd.angular.z = 0.0

        # 퍼블리시
        self.cmd_pub.publish(cmd)

        # 디버그 표시 (영상에 상태/면적 출력)
        cv2.putText(
            frame_bgr,
            f"{state} area={area if area is not None else 'N/A'}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        # 상태 로그 출력 (확인용)
        # self.get_logger().info(
        #     f"State: {state}, cx: {cx}, area: {area if cx is not None else 'N/A'}"
        # )



        # -------------------------------
        # 결과 확인
        # -------------------------------
        # 화면 표시용으로만 크기 축소 (예: 640x360)
        frame_show = cv2.resize(frame_bgr, (640, 360))
        mask_show  = cv2.resize(red_mask, (640, 360))

        cv2.imshow("duckie_camera", frame_show)
        #cv2.imshow("red_mask", mask_show)
        cv2.waitKey(1)

def main():
    # ROS2 초기화
    rclpy.init()

    # 노드 생성
    node = DuckieCamView()

    # 콜백 루프 실행
    rclpy.spin(node)

    # 종료 처리
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# ROS2 Python 클라이언트 라이브러리(rclpy) import
# - ROS2 노드 생성, 서비스/토픽 pub/sub, spin 루프 등에 사용
import rclpy
from rclpy.node import Node  # ROS2의 기본 Node 클래스

# 우리가 만든 서비스 타입 (duckie_interfaces 패키지 안에 srv/LedControl.srv)
# - 요청(Request): string color
# - 응답(Response): bool success, string message
from duckie_interfaces.srv import LedControl

# Isaac Sim Action Graph가 구독하는 토픽 메시지 타입
# - std_msgs/String 메시지에는 data라는 문자열 필드가 하나 있음
from std_msgs.msg import String


class DuckieLedService(Node):
    """
    역할:
    1) /duckie_led_control 서비스(Server)를 열어 '색상' 요청을 받는다.
    2) 요청받은 색상을 /duckie/led_color 토픽(std_msgs/String)으로 Publish한다.
       → Isaac Sim Action Graph(ROS2 Subscriber)가 이 토픽을 받아 LED 색을 변경함.

    즉, '서비스 → 토픽 변환 브릿지 노드'이다.
    """

    def __init__(self):
        # Node 이름을 "duckie_led_service"로 설정 (ros2 node list에 이 이름으로 보임)
        super().__init__("duckie_led_service")

        # ✅ Publisher 생성
        # - 토픽 이름: /duckie/led_color
        # - 메시지 타입: std_msgs/String
        # - queue_size: 10 (퍼블리시가 순간 몰릴 때 버퍼 역할)
        #
        # Isaac Sim Action Graph에서 ROS2 Subscriber가 이 토픽을 구독하고 있음.
        self.pub = self.create_publisher(
            String,               # 퍼블리시할 메시지 타입
            "/duckie/led_color",   # 토픽 이름 (Action Graph와 반드시 동일해야 함)
            10                    # QoS depth (간단히: 버퍼 크기)
        )

        # ✅ Service 생성
        # - 서비스 이름: /duckie_led_control
        # - 서비스 타입: duckie_interfaces/srv/LedControl
        # - 콜백 함수: handle_led_request (요청 들어올 때 실행되는 함수)
        #
        # 터미널에서 다음처럼 호출하면 이 콜백이 실행됨:
        # ros2 service call /duckie_led_control duckie_interfaces/srv/LedControl "{color: 'red'}"
        self.srv = self.create_service(
            LedControl,               # 서비스 타입
            "/duckie_led_control",    # 서비스 이름
            self.handle_led_request   # 요청 처리 콜백
        )

        # 실행 확인용 로그 (노드가 정상적으로 떠 있는지 확인)
        self.get_logger().info("Duckiebot LED service ready")

    def handle_led_request(self, request, response):
        """
        서비스 요청이 들어올 때 실행되는 함수(콜백)

        request.color: 사용자가 원하는 색상 문자열 (예: 'red', 'green')
        response.success / response.message: 서비스 응답으로 돌려줄 값
        """

        # 1) 토픽으로 publish할 std_msgs/String 메시지 생성
        msg = String()

        # 2) 서비스 요청에서 받은 색상 문자열을 msg.data에 담기
        #    (Isaac Sim Action Graph는 이 data 값을 읽어서 LED 색을 바꿈)
        msg.data = request.color

        # 3) /duckie/led_color 토픽으로 publish
        #    - publish는 "던져놓고 끝" (비동기)
        #    - 실제로 Isaac Sim이 받아서 반영하는 건 Action Graph 쪽 로직
        self.pub.publish(msg)

        # 4) 서비스 호출자에게 성공 응답 반환
        response.success = True
        response.message = f"LED set to {request.color}"

        # 반드시 response를 return 해야 서비스 응답이 전송됨
        return response


def main():
    # ROS2 통신 초기화 (노드 생성 전에 1번 필수)
    rclpy.init()

    # 위에서 정의한 서비스 노드 생성
    node = DuckieLedService()

    # 노드를 계속 살려두면서 콜백을 처리하는 이벤트 루프
    # - 서비스 요청이 들어오면 handle_led_request가 실행됨
    rclpy.spin(node)

    # spin이 끝나면 (Ctrl+C로 종료 시) ROS2 종료 처리
    rclpy.shutdown()


# 파이썬 파일을 직접 실행했을 때만 main() 실행
# (import 해서 쓸 경우엔 자동 실행 방지)
if __name__ == "__main__":
    main()

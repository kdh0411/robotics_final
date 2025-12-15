# robotics_final
Isaac Sim 기반 Duckiebot 모델링 및 자율주행 프로젝트

# 1. 모델링 소개 
1. 구조
<img width="583" height="292" alt="Image" src="https://github.com/user-attachments/assets/911b8d77-2fdd-42c3-a644-5c6053dd970a" />


2. 로봇 모델 이미지
   
<img width="565" height="289" alt="Image" src="https://github.com/user-attachments/assets/2605f528-f45f-4dbc-b7a9-d3095be2c7f0" />
# 2. Duckiebot LED Control

본 프로젝트에서는 ROS2 기반 서비스 호출을 통해 Duckiebot의 LED 색상을 제어하는 기능을 구현하였습니다.  
ROS2 서비스 요청은 토픽 메시지로 변환되어 Isaac Sim 내부 Action Graph를 통해 LED 메시에 반영됩니다.

---

## 2.1 시스템 구조
ROS2 Service Call
↓
duckie_led_service.py (ROS2, rclpy)
↓
/duckie/led_color (std_msgs/String)
↓
Isaac Sim ROS2 Bridge
↓
Action Graph + Script Node
↓
Duckiebot LED 색상 변경


---

## 2.2 빌드 방법

### 1) ROS2 워크스페이스 빌드
humble 빌드
cd ~/robotics_final/duckiebot_ws
colcon build
source ~/robotics_final/duckiebot_ws/install/local_setup.bash

## 2.3 실행 방법
1) Isaac Sim 실행
Duckiebot USD 파일 로드
LED 제어용 Action Graph 활성화
Simulation Play 상태 유지
2) LED 서비스 노드 실행
python3 ~/robotics_final/led/duckie_led_service.py
3) LED 색상 변경 서비스 호출
ros2 service call /duckie_led_control duckie_interfaces/srv/LedControl "{color: 'blue'}"
##2.4 지원 색상
red/green/blue/white/off(black)

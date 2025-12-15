# robotics_final
Isaac Sim 기반 Duckiebot 모델링 및 자율주행 프로젝트

# 1. 모델링 소개 
1. 구조
<img width="583" height="292" alt="Image" src="https://github.com/user-attachments/assets/911b8d77-2fdd-42c3-a644-5c6053dd970a" />


2. 로봇 모델 이미지
<img width="565" height="289" alt="Image" src="https://github.com/user-attachments/assets/2605f528-f45f-4dbc-b7a9-d3095be2c7f0" />

3. 로봇 usd file : usd_file/duckiebot.usd

4. 빨간 큐브 usd fild : usd_file/target_cube.usd

# 2. Duckiebot LED Control

본 프로젝트에서는 ROS2 기반 서비스 호출을 통해 Duckiebot의 LED 색상을 제어하는 기능을 구현하였습니다.  
ROS2 서비스 요청은 토픽 메시지로 변환되어 Isaac Sim 내부 Action Graph를 통해 LED 메시에 반영됩니다.

---

## 2.1 시스템 구조
ROS2 Service Call-> duckie_led_service.py (ROS2, rclpy)-> /duckie/led_color (std_msgs/String) ->Isaac Sim ROS2 Bridge
->Action Graph + Script Node ->Duckiebot LED 색상 변경


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

## 2.4 지원 색상
red/green/blue/white/off(black)

# 3. ROS 기반 모터 제어 구현

본 프로젝트에서는 Duckiebot의 주행을 제어하기 위해 ROS2 기반 모터 제어를 구현하였습니다.  
모터 제어는 **저수준(Low-level)** 과 **고수준(High-level)** 제어로 구성됩니다.

---

## 3.1 저수준(Low-level) 모터 제어

저수준 제어는 좌·우 바퀴에 직접 각속도(rad/s)를 전달하여 로봇을 구동하는 방식입니다.

### 사용 토픽
- `/duckie/wheel_left_cmd`  (`std_msgs/Float64`)
- `/duckie/wheel_right_cmd` (`std_msgs/Float64`)

Isaac Sim Action Graph에서 Articulation Controller를 통해
`wheel_joint_left`, `wheel_joint_right` 조인트에 각속도를 전달합니다.

---

### 빌드 방법
humble 빌드
source ~/robotics_final/duckiebot_ws/install/local_setup.bash (ws 빌드)


### 실행 방법
1. Isaac Sim 실행
- Duckiebot USD 로드
- 저수준 모터 제어용 Action Graph 활성화
- Simulation Play 상태 유지
2. 바퀴 각속도 명령 전송 (값은 변경 가능)
왼쪽 바퀴
ros2 topic pub /duckie/wheel_left_cmd std_msgs/msg/Float64 "{data: 5.0}"
오른쪽 바퀴
ros2 topic pub /duckie/wheel_right_cmd std_msgs/msg/Float64 "{data: 5.0}"

## 3.2 고수준(High-level) 모터 제어
고수준 제어는 선속도와 회전 속도를 입력으로 받아,
좌·우 바퀴 각속도로 변환하는 방식입니다.

### 실행 방법
한 터미널에서 cmd_vel_to_wheels.py 실행
또 다른 터미널에서 제어 명령 실행 (값은 조정가능)
(1) 직진
ros2 topic pub -r 10 /duckie/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.0}}"
x = 0.1 : 전진
z = 0.0 : 회전 없음
(2) 제자리 회전 (좌회전)
ros2 topic pub -r 10 /duckie/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 1.0}}"
 (3) 제자리 회전 (우회전)
ros2 topic pub -r 10 /duckie/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: -1.0}}"
 (4) 곡선 주행
ros2 topic pub -r 10 /duckie/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1}, angular: {z: 0.7}}"
 (5) 멈추기 (명시적)
ros2 topic pub -1 /duckie/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
추가적으로 teleop으로 키보드 수동 조작이 가능합니다.
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -r /cmd_vel:=/duckie/cmd_vel

i : 전진

, : 후진

j : 좌회전

l : 우회전

k : 정지

# 4. 카메라 신호 획득 및 ROS Publish
## /duckie/camera/image_raw 실행 방법
1. Isaac sim play 후 Viewport2 띄우기 (camera_front)
2. 터미널에서 ros2 run rqt_image_view rqt_image_view 실행
3. rqt에서 새로고침 후 /duckie/camera/image_raw 를 클릭
### 해상도 변경
Isaac sim Viewport2 의 해상도를 변경
### 프레임 변경
Isaac sim  ROS2Camera Helper Node 프로퍼티중 frameSkipCount로 조절

## 압축 이미지 사용 가능: /image_raw/compressed
1. image_raw와 동일하게 실행
2. 컴프레스 변환 노드를 실행
ros2 run image_transport republish raw compressed \
--ros-args \
-r in:=/duckie/camera/image_raw \
-r out/compressed:=/duckie/camera/image_raw/compressed
3. rqt에서 새로고침 후 /duckie/camera/image_raw/compressed 클릭


# 5. 영상처리 후 Duckiebot 행동 제어

### 실행 방법
humble 
ws 빌드
1. 아이작심에서 더키봇 스폰
2. 타겟 큐브 스폰 - Translate z -> 0.05 고정 x 및 y 는 +1~-1 사이 --> 환경마다 다를 수 있으므로 벽의 위치를 보고 조정
3. cmd_vel_to_wheels.py 실행 
4. duckie_cam_view.py 실행

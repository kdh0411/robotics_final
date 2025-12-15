# robotics_final
Isaac Sim 기반 Duckiebot 모델링 및 자율주행 프로젝트

# 1. 모델링 소개 
1. 구조
duckiebot (Xform + ArticulationRoot) 
 ├── chassis (Xform) rigid body
 │     ├── body                     (Mesh:Cube)        ← Duckiebot 바디
 │     ├── led_1                    (Mesh:Cube)
 │     ├── led_2                    (Mesh:Cube)
 │     ├── led_3                    (Mesh:Cube)
 │     ├── caster_ball              (Mesh:Sphere)      ← 보조바퀴
 │     ├── camera_front             (Camera)           ← 전방 카메라
 │
 ├── left_wheel (Xform)
 │     ├── wheel_mesh               (Mesh:Cylinder)
 │     
 │
 ├── right_wheel (Xform)
 │     ├── wheel_mesh               (Mesh:Cylinder)
 │     
 ├── Joints (Scope)
 ├     ├── wheel_joint_left               (PhysicsRevolute)
 ├     ├── wheel_joint_right              (PhysicsRevolute)

2. 로봇 모델 이미지
   

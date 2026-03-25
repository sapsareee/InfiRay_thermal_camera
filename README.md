# infiray thermal camera AT300 / AT313X

CmakeList 보면 

project(infiray_ros2)는 패키지의 이름

"add_executable(thermal_camera_node src/infiray_with_ros2_fixed_fast.cpp)"는 node와 실행할 코드의 이름

colcon build에 성공했다면


ros2 run infiray_ros2 thermal_camera_node

로 열화상 카메라를 시작할 수 있음

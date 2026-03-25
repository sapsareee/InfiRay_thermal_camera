# infiray thermal camera AT300 / AT313X

CmakeList 보면 "add_executable(thermal_camera_node src/infiray_with_ros2_fixed_fast.cpp)" 내용이 있음

해당 패키지 안에서, src 폴더 안에 실행할 cpp파일 이름을 적고 설정한 node이름으로

ros2 run infiray_ros2 thermal_camera_node

로 열화상 카메라를 시작할 수 있음

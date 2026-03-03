cmake_minimum_required(VERSION 3.8)
project(infiray_ros2)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3 -march=native -DNDEBUG -Wall -DLINUX -D_LINUX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(INFIRAY_SDK_DIR "/home/hyun/dev/sdks/infiray_sdk/IRT_InfraredTemp_SDK_Linux_X64_V1010/x64")

# add_executable(thermal_camera_node src/infiray_with_ros2.cpp)
add_executable(thermal_camera_node src/infiray_with_ros2_fixed_fast.cpp)

# 헤더 포함 (이제 충돌할 SDK 내 opencv 폴더가 없으므로 순서 상관 없음)
target_include_directories(thermal_camera_node PUBLIC ${INFIRAY_SDK_DIR}/include)

# ROS2 및 OpenCV 의존성을 아주 깔끔하게 주입
ament_target_dependencies(thermal_camera_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

# InfiRay 전용 라이브러리만 링크
target_link_libraries(thermal_camera_node
  ${INFIRAY_SDK_DIR}/libs/libInfraredTempSDK.so
  ${INFIRAY_SDK_DIR}/libs/libIRNetClient.so
  ${INFIRAY_SDK_DIR}/libs/libhyvstream.so
  ${INFIRAY_SDK_DIR}/libs/libhttpclient.so
  curl ssl crypto pthread z lzma
)

set_target_properties(thermal_camera_node PROPERTIES 
  BUILD_WITH_INSTALL_RPATH TRUE 
  INSTALL_RPATH "${INFIRAY_SDK_DIR}/libs"
)

install(TARGETS thermal_camera_node DESTINATION lib/${PROJECT_NAME})
ament_package()
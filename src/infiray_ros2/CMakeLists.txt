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
find_package(fieldscale_cpp REQUIRED)

set(INFIRAY_SDK_DIR "/home/hyun/dev/sdks/infiray_sdk/IRT_InfraredTemp_SDK_Linux_X64_V1010/x64")

add_executable(thermal_camera_fire_detect src/infiray_fire_detect.cpp)
add_executable(thermal_camera_fire_detect_raw16 src/infiray_fire_detect_raw16.cpp)


# 헤더 포함 (이제 충돌할 SDK 내 opencv 폴더가 없으므로 순서 상관 없음)
target_include_directories(thermal_camera_fire_detect PUBLIC ${INFIRAY_SDK_DIR}/include)
target_include_directories(thermal_camera_fire_detect_raw16 PUBLIC ${INFIRAY_SDK_DIR}/include)

# ROS2 및 OpenCV 의존성을 아주 깔끔하게 주입
ament_target_dependencies(
  thermal_camera_fire_detect
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)
ament_target_dependencies(
  thermal_camera_fire_detect_raw16
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  fieldscale_cpp
)

# InfiRay 전용 라이브러리만 링크
foreach(target_name
    thermal_camera_fire_detect
    thermal_camera_fire_detect_raw16)
  target_link_libraries(${target_name}
    ${INFIRAY_SDK_DIR}/libs/libInfraredTempSDK.so
    ${INFIRAY_SDK_DIR}/libs/libIRNetClient.so
    ${INFIRAY_SDK_DIR}/libs/libhyvstream.so
    ${INFIRAY_SDK_DIR}/libs/libhttpclient.so
    curl ssl crypto pthread z lzma
  )

  set_target_properties(${target_name} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    # DT_RPATH를 사용해 전역 /usr/lib/TmSDK보다 시스템 런타임을 먼저 선택한다.
    # SDK에만 있는 라이브러리는 마지막 경로에서 계속 찾는다.
    INSTALL_RPATH "/lib/x86_64-linux-gnu;/usr/lib/x86_64-linux-gnu;${INFIRAY_SDK_DIR}/libs"
  )
  target_link_options(${target_name} PRIVATE "-Wl,--disable-new-dtags")
endforeach()

install(
  TARGETS
    thermal_camera_fire_detect
    thermal_camera_fire_detect_raw16
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

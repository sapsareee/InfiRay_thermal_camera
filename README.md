# InfiRay 열화상 카메라 ROS 2 워크스페이스

## 역할

이 워크스페이스는 InfiRay AT300/AT313X 열화상 카메라의 영상과 온도 데이터를 받아 ROS 2 토픽으로 발행합니다.

`thermal_camera_fire_detect` 노드는 다음 작업을 수행합니다.

- 열화상 영상 수신 및 `/thermal/image` 발행
- 최고 온도와 온도 변화 추세 계산
- 설정 온도와 지속 시간을 이용한 화재 감지
- 대시보드와 `web_video_server`에서 사용할 영상·상태 토픽 제공
- 지연을 줄이기 위해 UDP RTSP와 최신 1프레임 버퍼 사용

## 준비 사항

- ROS 2 Humble
- OpenCV, `cv_bridge`, `rclcpp`, `sensor_msgs`, `std_msgs`
- InfiRay SDK
- 카메라와 PC가 같은 네트워크에 연결되어 있어야 함

기본 카메라 설정은 다음과 같음.

- IP: `192.168.1.123`
- 사용자 이름/비밀번호: `XXX` / `XXX`
- 영상 전송: UDP
- ROS 영상 발행 주기: 10 Hz

InfiRay SDK 경로는 [CMakeLists.txt](src/infiray_ros2/CMakeLists.txt)에 설정되어 있으므로 다른 PC에서 사용할 때는 `INFIRAY_SDK_DIR`을 실제 설치 경로로 변경해야 합니다.

## 빌드

```bash
cd /home/hyun/dev/repos/infiray_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select infiray_ros2
source install/setup.bash
```

새 터미널을 열었다면 실행 전에 다시 환경을 불러옵니다.

```bash
source /opt/ros/humble/setup.bash
source /home/hyun/dev/repos/infiray_ws/install/setup.bash
```

## 실행

기본 실행:

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect
```

로컬 OpenCV 화면을 함께 표시하려면:

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect --ros-args -p show_display:=true
```

카메라 IP 또는 ROS 영상 발행 주기를 변경하려면:

```bash
ros2 run infiray_ros2 thermal_camera_fire_detect --ros-args \
  -p camera_ip:=192.168.1.123 \
  -p target_image_fps:=10.0
```

종료할 때는 `Ctrl+C`를 누릅니다.

## 주요 ROS 2 토픽

| 토픽 | 형식 | 설명 |
| --- | --- | --- |
| `/thermal/image` | `sensor_msgs/msg/Image` | 열화상 영상 |
| `/thermal/max_temperature` | `std_msgs/msg/Float32` | 감지 영역의 최고 온도 |
| `/thermal/temperature_trend` | `std_msgs/msg/Float32` | 초당 온도 변화량 |
| `/thermal/fire_detected` | `std_msgs/msg/Bool` | 화재 감지 결과 |

토픽 동작 확인:

```bash
ros2 topic hz /thermal/image
ros2 topic echo /thermal/max_temperature
ros2 topic echo /thermal/fire_detected
```

## 대시보드 영상 연동

`FireRobotDashboard` 전체 기능을 사용하려면 별도 터미널에서 ROS Bridge와 `web_video_server`를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/hyun/dev/repos/infiray_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
source /opt/ros/humble/setup.bash
source /home/hyun/dev/repos/infiray_ws/install/setup.bash
ros2 run web_video_server web_video_server
```

대시보드는 `http://<로봇 IP>:8080/stream?topic=/thermal/image` 형태로 열화상 영상을 받아 표시합니다.

## 참고

- 실행 직후 RTSP 연결과 디코더 초기화에 몇 초가 걸릴 수 있습니다.
- 로그의 `drop`은 지연을 방지하기 위해 사용하지 않은 이전 입력 프레임 수이며 오류가 아닙니다.
- `unknown frame type` 또는 종료 시 `NetFramework` 메시지는 벤더 라이브러리 내부 경고로, 영상·온도 토픽과 정상 종료가 유지된다면 치명적인 오류가 아닙니다.

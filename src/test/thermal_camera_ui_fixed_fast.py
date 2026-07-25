import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class ROS2Signals(QObject):
    # 영상 시그널은 제거하고, 가벼운 온도/화재 상태 시그널만 유지
    temp_signal = pyqtSignal(float)
    fire_signal = pyqtSignal(bool)

class ThermalUINode(Node):
    def __init__(self, signals):
        super().__init__('thermal_ui_node')
        self.signals = signals
        self.bridge = CvBridge()
        
        # [핵심 변경] 최신 이미지를 저장할 변수와 스레드 락(Lock) 생성
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        self.img_sub = self.create_subscription(
            Image, '/thermal/image', self.image_callback, qos_profile_sensor_data)
        self.temp_sub = self.create_subscription(
            Float32, '/thermal/max_temp', self.temp_callback, qos_profile_sensor_data)
        self.fire_sub = self.create_subscription(
            Bool, '/thermal/fire_detected', self.fire_callback, qos_profile_sensor_data)

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 시그널을 쏘지 않고, 변수에 최신 이미지만 덮어씌움 (이벤트 큐 포화 방지)
        with self.image_lock:
            self.latest_image = cv_img

    def temp_callback(self, msg):
        self.signals.temp_signal.emit(msg.data)

    def fire_callback(self, msg):
        self.signals.fire_signal.emit(msg.data)

class MainWindow(QMainWindow):
    def __init__(self, ros_node): # 노드 객체를 받아오도록 수정
        super().__init__()
        self.ros_node = ros_node
        self.initUI()
        
        # [핵심 변경] 화면 렌더링 전용 타이머 설정 (약 30 FPS 기준 33ms 주기)
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.render_latest_image)
        self.render_timer.start(33) 
        
    def initUI(self):
        self.setWindowTitle('EV Fire-Fighting Robot Control Center')
        self.setGeometry(100, 100, 1000, 700)
        self.setStyleSheet("background-color: #2c3e50; color: white;")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.status_label = QLabel("SYSTEM READY")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 20pt; font-weight: bold; background-color: #34495e; padding: 10px;")
        layout.addWidget(self.status_label)

        content_layout = QHBoxLayout()
        
        self.video_label = QLabel()
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("border: 2px solid #7f8c8d; background-color: black;")
        self.video_label.setScaledContents(True) 
        
        content_layout.addWidget(self.video_label)

        info_layout = QVBoxLayout()
        
        temp_group = QGroupBox("Thermal Info")
        temp_vbox = QVBoxLayout()
        self.temp_display = QLabel("0.0 °C")
        self.temp_display.setStyleSheet("font-size: 40pt; color: #e74c3c; font-family: 'Consolas';")
        temp_vbox.addWidget(self.temp_display)
        temp_group.setLayout(temp_vbox)
        info_layout.addWidget(temp_group)

        state_group = QGroupBox("Mission Status")
        state_vbox = QVBoxLayout()
        self.state_list = ["1. Searching", "2. Fire Detected", "3. Aligning", "4. Deploying Cover", "5. Retreating"]
        self.state_labels = []
        for s in self.state_list:
            lbl = QLabel(s)
            lbl.setStyleSheet("font-size: 12pt; color: #95a5a6;")
            state_vbox.addWidget(lbl)
            self.state_labels.append(lbl)
        state_group.setLayout(state_vbox)
        info_layout.addWidget(state_group)

        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet("background-color: #c0392b; font-weight: bold; height: 50px;")
        info_layout.addWidget(self.stop_btn)

        content_layout.addLayout(info_layout)
        layout.addLayout(content_layout)

    def render_latest_image(self):
        # 타이머가 돌 때마다 공유 변수에서 '가장 최신' 이미지만 낚아채서 렌더링
        with self.ros_node.image_lock:
            if self.ros_node.latest_image is None:
                return
            # 화면에 그리는 동안 원본 이미지가 바뀌지 않도록 복사(copy)
            img_to_show = self.ros_node.latest_image.copy()

        qt_img = self.convert_cv_to_qt(img_to_show)
        self.video_label.setPixmap(qt_img)

    def convert_cv_to_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format)

    @pyqtSlot(float)
    def update_temp(self, temp):
        self.temp_display.setText(f"{temp:.1f} °C")
        if temp > 60.0:  
            self.temp_display.setStyleSheet("font-size: 40pt; color: #ff0000; font-weight: bold;")
        else:
            self.temp_display.setStyleSheet("font-size: 40pt; color: #e74c3c;")

    @pyqtSlot(bool)
    def update_fire_status(self, is_fire):
        if is_fire:
            self.status_label.setText("🔥 FIRE DETECTED! 🔥")
            self.status_label.setStyleSheet("font-size: 20pt; font-weight: bold; background-color: #c0392b; color: yellow;")
        else:
            self.status_label.setText("MONITORING...")
            self.status_label.setStyleSheet("font-size: 20pt; font-weight: bold; background-color: #34495e; color: white;")

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    ros_signals = ROS2Signals()
    node = ThermalUINode(ros_signals)
    
    # 노드 객체를 MainWindow에 넘겨주어 타이머가 최신 이미지를 꺼내갈 수 있게 함
    main_win = MainWindow(node)
    
    ros_signals.temp_signal.connect(main_win.update_temp)
    ros_signals.fire_signal.connect(main_win.update_fire_status)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    main_win.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
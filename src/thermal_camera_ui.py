import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class ThermalUINode(Node):
    def __init__(self, ui_app):
        super().__init__('thermal_ui_node')
        self.ui_app = ui_app
        # 영상 및 온도 데이터 구독
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(Image, '/thermal/image', self.image_callback, 10)
        self.temp_sub = self.create_subscription(Float32, '/thermal/max_temp', self.temp_callback, 10)
        self.fire_sub = self.create_subscription(Bool, '/thermal/fire_detected', self.fire_callback, 10)

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.ui_app.update_image(cv_img)

    def temp_callback(self, msg):
        self.ui_app.update_temp(msg.data)

    def fire_callback(self, msg):
        self.ui_app.update_fire_status(msg.data)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('EV Fire-Fighting Robot Control Center')
        self.setGeometry(100, 100, 1000, 700)
        self.setStyleSheet("background-color: #2c3e50; color: white;")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 상단 타이틀 및 상태
        self.status_label = QLabel("SYSTEM READY")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 20pt; font-weight: bold; background-color: #34495e; padding: 10px;")
        layout.addWidget(self.status_label)

        # 메인 콘텐츠 (영상 + 우측 정보창)
        content_layout = QHBoxLayout()
        
        # 1. 영상 표시 영역
        self.video_label = QLabel()
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("border: 2px solid #7f8c8d; background-color: black;")
        content_layout.addWidget(self.video_label)

        # 2. 정보 및 제어창
        info_layout = QVBoxLayout()
        
        # 온도 표시
        temp_group = QGroupBox("Thermal Info")
        temp_vbox = QVBoxLayout()
        self.temp_display = QLabel("0.0 °C")
        self.temp_display.setStyleSheet("font-size: 40pt; color: #e74c3c; font-family: 'Consolas';")
        temp_vbox.addWidget(self.temp_display)
        temp_group.setLayout(temp_vbox)
        info_layout.addWidget(temp_group)

        # 로봇 상태 (시나리오 단계)
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

        # 긴급 정지 버튼
        self.stop_btn = QPushButton("EMERGENCY STOP")
        self.stop_btn.setStyleSheet("background-color: #c0392b; font-weight: bold; height: 50px;")
        info_layout.addWidget(self.stop_btn)

        content_layout.addLayout(info_layout)
        layout.addLayout(content_layout)

    def update_image(self, cv_img):
        qt_img = self.convert_cv_to_qt(cv_img)
        self.video_label.setPixmap(qt_img)

    def convert_cv_to_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format)

    def update_temp(self, temp):
        self.temp_display.setText(f"{temp:.1f} °C")
        if temp > 60.0:  # 임계값 예시
            self.temp_display.setStyleSheet("font-size: 40pt; color: #ff0000; font-weight: bold;")
        else:
            self.temp_display.setStyleSheet("font-size: 40pt; color: #e74c3c;")

    def update_fire_status(self, is_fire):
        if is_fire:
            self.status_label.setText("🔥 FIRE DETECTED! 🔥")
            self.status_label.setStyleSheet("font-size: 20pt; font-weight: bold; background-color: #c0392b; color: yellow;")
        else:
            self.status_label.setText("MONITORING...")
            self.status_label.setStyleSheet("font-size: 20pt; font-weight: bold; background-color: #34495e; color: white;")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    main_win = MainWindow()
    node = ThermalUINode(main_win)

    # ROS2 스핀을 별도 스레드에서 실행하거나 타이머로 처리
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    main_win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
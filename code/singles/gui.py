import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer

from cv_bridge import CvBridge
import cv2

import signal
import sys

class CameraViewer(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.bridge = CvBridge()
        self.image_label = QLabel("Waiting for image...")

        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        self.setLayout(layout)

        self.sub = self.node.create_subscription(
            Image, "/image_raw", self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)



        # Draw detected markers on original image
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_image))


def main():
    rclpy.init()
    node = rclpy.create_node('camera_viewer_node')

    app = QApplication(sys.argv)
    viewer = CameraViewer(node)

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    viewer.setWindowTitle("ROS 2 Camera Feed")
    viewer.resize(640, 480)
    viewer.show()

    signal.signal(signal.SIGINT, lambda *args: app.quit())

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

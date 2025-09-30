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

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

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
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.camera_matrix = np.array([
            [1000.0,    0.0, 640.0],   # fx,  cx
            [   0.0, 1000.0, 360.0],   #     fy, cy
            [   0.0,    0.0,   1.0]])
        self.dist_coeffs = np.zeros((5,))
        self.marker_length = 0.05

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Convert rotation to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvec)
                quat = R.from_matrix(rot_matrix).as_quat()

                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = "camera_frame"
                t.child_frame_id = f"aruco_marker_{int(marker_id)}"
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.tf_broadcaster.sendTransform(t)

        # Show the image
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

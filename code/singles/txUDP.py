# ros2_sender.py
import socket
import rclpy
from array import array
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class UDPSender(Node):
    def __init__(self):
        super().__init__('udp_sender')
        self.sub = self.create_subscription(LaserScan, '/scan', self.send_data, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_ip = "192.168.0.102"  # External Pi's IP
        self.udp_port = 5005

    def send_data(self, msg):
        data = array('f', msg.ranges).tobytes()
        self.sock.sendto(data, (self.udp_ip, self.udp_port))

def main(args=None):
    rclpy.init(args=args)
    node = UDPSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

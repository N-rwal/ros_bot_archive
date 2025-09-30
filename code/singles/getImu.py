import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
#from tf_transformations import euler_from_quaternion
import math

#----------------------------------------SETUP

print("script starting")

def euler_from_quaternion(quat):
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class IMUYawReader(Node):
    def __init__(self):
        super().__init__('imu_yaw_reader')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.yaw = None  # Store latest yaw

        # Add a timer that calls self.print_yaw at 10 Hz
        self.create_timer(0.1, self.print_yaw)

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.yaw = yaw

    def print_yaw(self):
        if self.yaw is not None:
            print(f"Current Yaw (deg): {math.degrees(self.yaw):.2f}")


def main(args=None):
    print("reached main")
    rclpy.init(args=args)
    node = IMUYawReader()
    try:
        rclpy.spin(node)
        #----------------------------------------LOOP
    except KeyboardInterrupt:
        print("exit one")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

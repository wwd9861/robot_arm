import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import JointState

import time
from math import *

l1 = 116
l2 = 105
l3 = 105
l4 = 70


class Drawer(Node):
    def __init__(self):
        super().__init__("drawer")
        self.data = JointState()
        self.data.name = ["revolute1", "revolute2", "revolute3", "revolute4"]
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self.timer_callback)

        self.t = 0

    def timer_callback(self):
        y = 0

        self.t = self.t + 0.1

        x = 50 * sin(self.t) + 150
        z = 50 * cos(self.t)

        phi = -pi / 2
        theta1 = atan2(z, x)
        c3 = ((sqrt(x**2 + z**2) - l4 * cos(phi)) ** 2 + (y - l1 - l4 * sin(phi)) ** 2 - l2**2 - l3**2) / (2 * l2 * l3)
        s3 = -sqrt(1 - c3**2)
        k1 = l2 + l3 * c3
        k2 = l3 * s3
        theta3 = atan2(s3, c3)
        theta2 = atan2(y - l1 - l4 * sin(phi), sqrt(x**2 + z**2) - l4 * cos(phi)) - atan2(k2, k1)
        theta4 = phi - theta2 - theta3
        theta1 = atan2(z, x)

        self.data.position = [-theta1, theta2 - pi / 2, -theta3, theta4]
        if (
            abs(self.data.position[0]) > pi / 2
            or abs(self.data.position[1] > pi / 2)
            or self.data.position[2] > pi
            or self.data.position[2] < 0
            or abs(self.data.position[3]) > pi / 2
        ):
            rp.shutdown()
        self.data.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.data)


def main():
    rp.init()
    drawer = Drawer()
    rp.spin(drawer)

    drawer.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()

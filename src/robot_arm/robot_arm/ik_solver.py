import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from robot_arm_msg.srv import Coordinate

from math import *

l1 = 116
l2 = 105
l3 = 105
l4 = 70


class IKSolver(Node):

    def __init__(self):
        super().__init__("ik_solver")
        self.srv = self.create_service(Coordinate, "ik", self.ik_callback)
        self.data = JointState()
        self.data.name = ["revolute1", "revolute2", "revolute3", "revolute4"]
        self.timer_period = 0.1
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

    def ik_callback(self, request, response):
        theta1 = atan2(request.z, request.x)
        c3 = (
            (sqrt(request.x**2 + request.z**2) - l4 * cos(request.phi)) ** 2 + (request.y - l1 - l4 * sin(request.phi)) ** 2 - l2**2 - l3**2
        ) / (2 * l2 * l3)
        s3 = -sqrt(1 - c3**2)
        k1 = l2 + l3 * c3
        k2 = l3 * s3
        theta3 = atan2(s3, c3)
        theta2 = atan2(request.y - l1 - l4 * sin(request.phi), sqrt(request.x**2 + request.z**2) - l4 * cos(request.phi)) - atan2(k2, k1)
        theta4 = request.phi - theta2 - theta3
        theta1 = atan2(request.z, request.x)
        joint_solution = [-theta1, theta2 - pi / 2, -theta3, theta4]
        self.data.header.stamp = self.get_clock().now().to_msg()
        self.data.position = joint_solution
        self.publisher.publish(self.data)

        return response


def main():
    rclpy.init()
    ik_solver = IKSolver()
    rclpy.spin(ik_solver)

    ik_solver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

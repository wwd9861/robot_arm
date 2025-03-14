import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        self.data = JointTrajectory()
        self.data.header.frame_id = "world"
        self.data.joint_names = ["revolute1", "revolute2", "revolute3", "revolute4"]
        self.points = JointTrajectoryPoint()
        self.points.time_from_start.sec = 1

        self.publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

    def ik_callback(self, request, response):
        c3 = (
            (sqrt(request.x**2 + request.y**2 - 50**2) - l4 * cos(request.phi)) ** 2
            + (request.z - l1 - l4 * sin(request.phi)) ** 2
            - l2**2
            - l3**2
        ) / (2 * l2 * l3)
        s3 = -sqrt(1 - c3**2)
        k1 = l2 + l3 * c3
        k2 = l3 * s3
        theta3 = atan2(s3, c3)
        theta2 = atan2(
            request.z - l1 - l4 * sin(request.phi), sqrt(request.x**2 + request.y**2 - 50**2) - l4 * cos(request.phi)
        ) - atan2(k2, k1)
        theta4 = request.phi - theta2 - theta3
        theta1 = atan2(request.y, request.x) - atan2(50, sqrt(request.x**2 + request.y**2 - 50**2))
        self.points.positions = [theta1, theta2 - pi / 2, theta3, theta4]
        self.data.points = [self.points]
        self.publisher.publish(self.data)
        print(sqrt(request.x**2 + request.y**2 - 50**2))
        return response


def main():
    rclpy.init()
    ik_solver = IKSolver()
    rclpy.spin(ik_solver)

    ik_solver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

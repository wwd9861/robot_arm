import sys

from robot_arm_msg.srv import Coordinate
import rclpy
from rclpy.node import Node
from math import pi

from time import sleep


class ClientAsync(Node):

    def __init__(self):
        super().__init__("client")
        self.cli = self.create_client(Coordinate, "/ik")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = Coordinate.Request()

    def send_request(self, x, y, z, phi):
        self.req.x = x
        self.req.y = y
        self.req.z = z
        self.req.phi = phi
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    client = ClientAsync()
    move1 = client.send_request(-100.0, 100.0, 200.0, 0.0)
    rclpy.spin_until_future_complete(client, move1)
    sleep(2)
    move2 = client.send_request(100.0, -100.0, 300.0, 0.0)
    rclpy.spin_until_future_complete(client, move1)
    sleep(2)
    move3 = client.send_request(100.0, 100.0, 200.0, 0.0)
    rclpy.spin_until_future_complete(client, move1)
    sleep(2)
    move1 = client.send_request(-200.0, 0.0, 100.0, 0.0)
    rclpy.spin_until_future_complete(client, move1)
    sleep(2)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

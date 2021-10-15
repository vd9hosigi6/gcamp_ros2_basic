#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.srv import TurningControl
import time



# 노드 생성하기
class ExampleServerNode(Node):
        # 생성자
        def __init__(self):
                super().__init__("example_server_node")

                # Service Server 생성하기
                self.server = self.create_service(TurningControl, "example_service", self.run_callback)

                # publisher 생성하기
                self.publisher = self.create_publisher(Twist, "skidbot/cmd_vel", 10)

                self.get_logger().info("example_server start")

        # move_robot
        def move_robot(self, duration=1, linear_vel_x=0.0, angular_vel_z=0.0):
                self.get_logger().info("move_robot")

                twist_msg = Twist()
                twist_msg.linear.x = linear_vel_x
                twist_msg.angular.z = angular_vel_z

                start_time = self.get_clock().now().to_msg().sec
                end_time = self.get_clock().now().to_msg().sec

                while (end_time - start_time) < duration:
                        self.publisher.publish(twist_msg)
                        end_time = self.get_clock().now().to_msg().sec
                        time.sleep(0.01)

        # stop_robot
        def stop_robot(self):
                self.get_logger().info("stop_robot")

                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

                self.publisher.publish(twist_msg)

        # callback
        def run_callback(self, request, response):
                self.get_logger().info("run_callback")

                # control robot
                self.move_robot(request.time_duration, request.linear_vel_x, request.angular_vel_z)
                self.stop_robot()

                response.success = True
                return response

# main()
def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        # Node 생성하기
        example_server_node = ExampleServerNode()

        # 실행하기
        rclpy.spin(example_server_node)

        # 종료하기
        example_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
        main()


#! /usr/bin/env python3

# 모듈 불러오기
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Node 클래스 정의하기
class LaserNode(Node):
        # 쌩성자
        def __init__(self):
                # Node 생성자를 호출해 노드의 이름 설정하기
                super().__init__("laser_node")

                # subscriber 객체 생성하기
                self.subscriber = self.create_subscription(
                        LaserScan, "skidbot/scan", self.subscribe_callback, 10
                )

        # subscribe callback
        def subscribe_callback(self, msg):
                self.get_logger().info(f"front object distance : {msg.ranges[360]}")

# main
def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        # subscriber 깩체 생성하기
        laser_node = LaserNode()
        
        # 수집하기
        rclpy.spin(laser_node)

        # 종료하기
        laser_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
        main()

